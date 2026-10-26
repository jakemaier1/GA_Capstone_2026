"""Small JSON messages inside Program 2's two-byte, total-length PDU frame."""

import json
import socket
import struct
import time
import uuid

MAX_PDU = 4096                 # Header included; same length convention as pdu.c.
MAX_TEXT_BYTES = 512           # Even maximally escaped text fits a request and receipt.
CONNECT_TIMEOUT = 3.0
HANDSHAKE_TIMEOUT = 5.0
IO_TIMEOUT = 5.0
READ_TIMEOUT = 1800.0    # 30 mins -> 60.0 for 1 min        # Receiver idle/full-frame timeout, without heartbeats.


class ProtocolError(ValueError):
    """A message does not satisfy our framing or structure rules."""


class ConnectionClosed(ConnectionError):
    """The peer closed before a complete message or response arrived."""


def _check_fields(message, expected):
    if type(message) is not dict or set(message) != expected:
        raise ProtocolError("message has missing or unexpected fields")
    if type(message["v"]) is not int or message["v"] != 1:
        raise ProtocolError("only integer protocol version 1 is supported")
    identifier = message["id"]
    try:
        if type(identifier) is not str or str(uuid.UUID(identifier)) != identifier:
            raise ValueError
    except (ValueError, AttributeError):
        raise ProtocolError("message id must be a canonical UUID") from None
    text = message["text"]
    if type(text) is not str or not text.strip():
        raise ProtocolError("text must be a nonempty string")
    try:
        size = len(text.encode("utf-8"))
    except UnicodeError:
        raise ProtocolError("text must be valid UTF-8") from None
    if size > MAX_TEXT_BYTES:
        raise ProtocolError(f"text exceeds {MAX_TEXT_BYTES} UTF-8 bytes")


def validate_message(message):
    _check_fields(message, {"v", "type", "id", "text"})
    if message["type"] != "message":
        raise ProtocolError("expected type 'message'")


def make_message(text):
    message = {"v": 1, "type": "message", "id": str(uuid.uuid4()), "text": text}
    validate_message(message)
    return message


def make_receipt(message):
    return {"v": 1, "type": "receipt", "id": message["id"],
            "status": "received", "text": message["text"]}


def validate_receipt(receipt, sent):
    _check_fields(receipt, {"v", "type", "id", "status", "text"})
    if receipt["type"] != "receipt" or receipt["status"] != "received":
        raise ProtocolError("expected a 'received' receipt")
    if receipt["id"] != sent["id"] or receipt["text"] != sent["text"]:
        raise ProtocolError("receipt does not match the sent message")


def encode_pdu(message):
    if type(message) is not dict:
        raise ProtocolError("JSON payload must be an object")
    try:
        payload = json.dumps(message, ensure_ascii=False, separators=(",", ":"),
                             allow_nan=False).encode("utf-8")
    except (ValueError, TypeError, RecursionError) as error:
        raise ProtocolError("message cannot be encoded as JSON") from error
    total = len(payload) + 2
    if not 3 <= total <= MAX_PDU:
        raise ProtocolError(f"total PDU length must be 3-{MAX_PDU} bytes")
    return struct.pack("!H", total) + payload


def send_pdu(connection, message, timeout=IO_TIMEOUT):
    packet = encode_pdu(message)
    previous_timeout = connection.gettimeout()
    connection.settimeout(timeout)
    try:
        connection.sendall(packet)      # Sends all frame bytes or raises an error.
    except socket.timeout:
        raise TimeoutError("timed out writing a PDU") from None
    finally:
        connection.settimeout(previous_timeout)


def _recv_exact(connection, count, deadline, allow_clean_eof=False):
    data = bytearray()
    while len(data) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("timed out reading a complete PDU")
        connection.settimeout(remaining)
        try:
            chunk = connection.recv(count - len(data))
        except socket.timeout:
            raise TimeoutError("timed out reading a complete PDU") from None
        if not chunk:
            if allow_clean_eof and not data:
                return None
            raise ConnectionClosed("peer disconnected partway through a PDU")
        data.extend(chunk)
    return bytes(data)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError("duplicate JSON field")
        result[key] = value
    return result


def _reject_constant(value):
    raise ProtocolError(f"non-finite JSON number: {value}")


def recv_pdu(connection, timeout=IO_TIMEOUT):
    # One deadline covers header and payload; trickled bytes cannot reset it.
    previous_timeout = connection.gettimeout()
    deadline = time.monotonic() + timeout
    try:
        header = _recv_exact(connection, 2, deadline, allow_clean_eof=True)
        if header is None:
            return None                # Normal disconnect between messages.
        total = struct.unpack("!H", header)[0]
        if not 3 <= total <= MAX_PDU:
            raise ProtocolError(f"total PDU length must be 3-{MAX_PDU} bytes")
        payload = _recv_exact(connection, total - 2, deadline)
    finally:
        connection.settimeout(previous_timeout)
    try:
        message = json.loads(payload.decode("utf-8"), object_pairs_hook=_unique_object,
                             parse_constant=_reject_constant)
    except (ValueError, RecursionError) as error:
        raise ProtocolError(f"invalid JSON payload: {error}") from error
    if type(message) is not dict:
        raise ProtocolError("JSON payload must be an object")
    return message
