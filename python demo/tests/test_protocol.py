import json
import socket
import struct
import threading
import time
import unittest

from protocol import (MAX_PDU, MAX_TEXT_BYTES, ConnectionClosed, ProtocolError, encode_pdu,
                      make_message, make_receipt, recv_pdu, send_pdu,
                      validate_message, validate_receipt)


class StructureTests(unittest.TestCase):
    def test_message_and_matching_receipt(self):
        """Message format | Accept valid text and its matching receipt."""
        message = make_message("Hello rover — 地面站")
        validate_message(message)
        validate_receipt(make_receipt(message), message)

    def test_invalid_message_structure(self):
        """Message format | Reject invalid fields, IDs, versions, and text."""
        message = make_message("hello")
        invalid = [dict(message, v=True), dict(message, v=2), dict(message, type="command"),
                   dict(message, id="invalid"), dict(message, id=1), dict(message, extra=1),
                   dict(message, text=1), dict(message, text=""), dict(message, text="   "),
                   dict(message, text="é" * (MAX_TEXT_BYTES // 2 + 1)), dict(message, text="\ud800"), {}]
        for value in invalid:
            with self.subTest(value=value), self.assertRaises(ProtocolError):
                validate_message(value)

    def test_receipt_must_match_id_text_and_status(self):
        """Receipts | Reject a wrong ID, changed text, or invalid receipt status."""
        message = make_message("hello")
        receipt = make_receipt(message)
        invalid = [dict(receipt, id=make_message("other")["id"]), dict(receipt, text="other"),
                   dict(receipt, status="action completed"), dict(receipt, type="message"),
                   dict(receipt, extra=1), {}]
        for value in invalid:
            with self.subTest(value=value), self.assertRaises(ProtocolError):
                validate_receipt(value, message)

    def test_pdu_header_counts_utf8_bytes_and_includes_header(self):
        """Framing | Count UTF-8 bytes and include the two-byte header in the length."""
        message = make_message("café")
        packet = encode_pdu(message)
        self.assertEqual(struct.unpack("!H", packet[:2])[0], len(packet))
        self.assertEqual(json.loads(packet[2:]), message)

    def test_maximum_text_fits_both_request_and_receipt(self):
        """Size limits | Fit maximum-length text into both a message and a receipt."""
        for text in ("x" * MAX_TEXT_BYTES, "é" * (MAX_TEXT_BYTES // 2),
                     "x" + "\x00" * (MAX_TEXT_BYTES - 1)):
            with self.subTest(text_kind=repr(text[:2])):
                message = make_message(text)
                self.assertLessEqual(len(encode_pdu(message)), MAX_PDU)
                self.assertLessEqual(len(encode_pdu(make_receipt(message))), MAX_PDU)

    def test_oversized_or_non_json_outgoing_payload(self):
        """Size limits | Reject oversized frames and data that cannot be encoded as JSON."""
        for value in ({"x":"a" * MAX_PDU}, {"x":float("nan")}, {"x":object()}, []):
            with self.subTest(value=type(value)), self.assertRaises(ProtocolError):
                encode_pdu(value)


class FramingTests(unittest.TestCase):
    def setUp(self):
        self.sender, self.receiver = socket.socketpair()
        self.addCleanup(self.sender.close)
        self.addCleanup(self.receiver.close)

    def test_fragmented_header_and_body(self):
        """Framing | Reassemble a message arriving in several pieces."""
        message = make_message("fragmented")
        packet = encode_pdu(message)
        def send_fragments():
            for piece in (packet[:1], packet[1:2], packet[2:10], packet[10:]):
                self.sender.sendall(piece)
                time.sleep(0.01)
        thread = threading.Thread(target=send_fragments)
        thread.start()
        try:
            self.assertEqual(recv_pdu(self.receiver, timeout=1), message)
        finally:
            thread.join(timeout=1)
            self.assertFalse(thread.is_alive())

    def test_coalesced_pdus_and_sendall(self):
        """Framing | Read combined messages separately and send a complete frame."""
        first, second = make_message("one"), make_message("two")
        self.sender.sendall(encode_pdu(first) + encode_pdu(second))
        self.assertEqual(recv_pdu(self.receiver), first)
        self.assertEqual(recv_pdu(self.receiver), second)
        send_pdu(self.sender, first)
        self.assertEqual(recv_pdu(self.receiver), first)

    def test_clean_eof(self):
        """Disconnects | Treat a disconnect between messages as normal."""
        self.sender.close()
        self.assertIsNone(recv_pdu(self.receiver))

    def test_eof_mid_header_or_payload(self):
        """Disconnects | Detect a disconnect halfway through a header or payload."""
        for data in (b"\x00", b"\x00\x10{}"):
            with self.subTest(data=data):
                sender, receiver = socket.socketpair()
                with sender, receiver:
                    sender.sendall(data)
                    sender.shutdown(socket.SHUT_WR)
                    with self.assertRaises(ConnectionClosed):
                        recv_pdu(receiver)

    def test_invalid_lengths_rejected_without_waiting_for_payload(self):
        """Framing | Reject invalid frame lengths before reading the payload."""
        for length in (0, 1, 2, MAX_PDU + 1, 65535):
            with self.subTest(length=length):
                self.sender.sendall(struct.pack("!H", length))
                with self.assertRaises(ProtocolError):
                    recv_pdu(self.receiver, timeout=0.2)

    def test_invalid_json_and_duplicate_keys(self):
        """Message format | Reject invalid JSON, UTF-8, duplicate keys, and NaN."""
        for payload in (b"invalid", b"[]", b"\xff", b'{"x":1,"x":2}', b'{"x":NaN}'):
            with self.subTest(payload=payload):
                self.sender.sendall(struct.pack("!H", len(payload) + 2) + payload)
                with self.assertRaises(ProtocolError):
                    recv_pdu(self.receiver)

    def test_read_timeout_restores_original_socket_timeout(self):
        """Timeouts | Stop a stalled read and restore the previous socket timeout."""
        self.receiver.settimeout(2)
        with self.assertRaisesRegex(TimeoutError, "reading"):
            recv_pdu(self.receiver, timeout=0.05)
        self.assertEqual(self.receiver.gettimeout(), 2)

    def test_header_and_payload_share_one_deadline(self):
        """Timeouts | Use one read deadline even when bytes arrive slowly."""
        def trickle():
            self.sender.sendall(b"\x00")
            time.sleep(0.04)
            self.sender.sendall(b"\x04")
            time.sleep(0.04)
            self.sender.sendall(b"{")
        thread = threading.Thread(target=trickle)
        thread.start()
        try:
            with self.assertRaises(TimeoutError):
                recv_pdu(self.receiver, timeout=0.06)
        finally:
            thread.join(timeout=1)

    def test_write_timeout_with_nonreading_peer(self):
        """Timeouts | Stop writing when the other endpoint is not reading."""
        # Fill a small socket buffer using valid-sized PDUs until sendall times out.
        self.sender.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
        packet = {"data":"x" * 3000}
        with self.assertRaisesRegex(TimeoutError, "writing"):
            for _ in range(1000):
                send_pdu(self.sender, packet, timeout=0.02)


if __name__ == "__main__":
    unittest.main()
