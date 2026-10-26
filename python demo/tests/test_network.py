from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from pathlib import Path
import queue
import signal
import socket
import ssl
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

from connection import Connection
from generate_certs import generate, openssl
from ground_station import Tower
from protocol import (MAX_PDU, MAX_TEXT_BYTES, ConnectionClosed, ProtocolError,
                      encode_pdu, make_message, make_receipt, recv_pdu, send_pdu)
from rover_client import connect
from security import client_context, server_context


class Process:
    """Read CLI output in a thread so a failed startup cannot hang a test."""

    def __init__(self, *args):
        self.process = subprocess.Popen([sys.executable, *map(str, args)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, bufsize=1)
        self.lines, self.queue = [], queue.Queue()

        def read():
            for line in self.process.stdout:
                self.lines.append(line)
                self.queue.put(line)
        self.thread = threading.Thread(target=read, daemon=True)
        self.thread.start()

    def expect(self, text, timeout=5):
        deadline = time.monotonic() + timeout
        while text not in "".join(self.lines):
            try:
                self.queue.get(timeout=max(0, deadline - time.monotonic()))
            except queue.Empty:
                raise AssertionError(f"CLI did not print {text!r}: {''.join(self.lines)}") from None

    def send(self, text):
        self.process.stdin.write(text + "\n")
        self.process.stdin.flush()

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=3)
        self.thread.join(timeout=1)
        self.process.stdin.close()
        self.process.stdout.close()


class TLSTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="secure-comms-tests-")
        cls.addClassCleanup(cls.directory.cleanup)
        folder = Path(cls.directory.name)
        cls.certs, cls.rogue = folder / "trusted", folder / "rogue"
        generate(cls.certs, rovers=4)
        generate(cls.rogue)
        # Trusted CA, correct key usage, but unauthorized application identities.
        for name, purpose, san, key in (
            ("other-rover", "clientAuth", "URI:urn:airport-rover:other", "rover-1"),
            ("other-tower", "serverAuth", "DNS:tower.local,URI:urn:airport-rover:other", "tower"),
        ):
            csr, ext = folder / f"{name}.csr", folder / f"{name}.ext"
            openssl("req", "-new", "-key", cls.certs / f"{key}.key", "-subj", f"/CN={name}", "-out", csr)
            ext.write_text(f'''basicConstraints=critical,CA:FALSE
keyUsage=critical,digitalSignature,keyEncipherment
extendedKeyUsage={purpose}
subjectKeyIdentifier=hash
authorityKeyIdentifier=keyid:always
subjectAltName={san}
''')
            openssl("x509", "-req", "-in", csr, "-CA", cls.certs / "ca.crt", "-CAkey", cls.certs / "ca.key",
                    "-set_serial", "20" if name == "other-rover" else "21", "-days", "1", "-sha256",
                    "-extfile", ext, "-out", folder / f"{name}.crt")
        cls.folder = folder

    def setUp(self):
        self.messages, self.logs, self.sessions = [], [], []
        self.tower = Tower(self.certs, port=0, read_timeout=3,
                           handler=lambda identity, msg: self.messages.append((identity, msg)),
                           log=self.logs.append).start()
        self.addCleanup(lambda: self.tower.stop())

    def wait_for(self, predicate, timeout=2):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                self.fail(f"Condition did not become true; logs: {self.logs}")
            threading.Event().wait(0.005)

    def raw_client(self, identity="rover-1", context=None, name="tower.local"):
        return connect("127.0.0.1", self.tower.port,
                       context or client_context(self.certs, identity), name)

    def rover(self, identity="rover-1", handler=None):
        session = Connection(self.raw_client(identity), "tower", handler or (lambda msg: None),
                             read_timeout=3, log=self.logs.append).start()
        self.sessions.append(session)
        self.addCleanup(session.join)
        self.addCleanup(session.close)
        self.wait_for(lambda: identity in self.tower.rovers())
        return session

    @contextmanager
    def peer(self, responder, context=None):
        """A one-connection TLS tower for deliberate bad-receipt scenarios."""
        errors = []
        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            listener.settimeout(2)
            context = context or server_context(self.certs)

            def worker():
                try:
                    raw, _ = listener.accept()
                    with raw:
                        raw.settimeout(1)
                        with context.wrap_socket(raw, server_side=True) as tls:
                            responder(tls)
                except Exception as error:
                    errors.append(error)
            thread = threading.Thread(target=worker, daemon=True)
            thread.start()
            try:
                yield listener.getsockname()[1], errors
            finally:
                thread.join(timeout=2)
                self.assertFalse(thread.is_alive(), "test peer did not finish")

    def test_two_and_four_rovers_communicate_concurrently(self):
        """Multiple rovers | Exchange messages concurrently with two, then four authenticated rovers."""
        rovers = []
        for count in (2, 4):
            while len(rovers) < count:
                rovers.append(self.rover(f"rover-{len(rovers) + 1}"))
            barrier = threading.Barrier(count)

            def send(index):
                barrier.wait(timeout=2)
                return rovers[index].send(f"Hello from rover-{index + 1} — 地面站")
            with ThreadPoolExecutor(max_workers=count) as pool:
                receipts = list(pool.map(send, range(count)))
            self.assertEqual(len({receipt['id'] for receipt in receipts}), count)
            for index, receipt in enumerate(receipts):
                self.assertEqual(receipt['text'], f"Hello from rover-{index + 1} — 地面站")
                self.assertEqual(receipt['status'], 'received')
            self.assertEqual(len(self.tower.rovers()), count)
        self.assertEqual(len(self.messages), 6)
        for identity, msg in self.messages:
            self.assertIn(identity, msg['text'])

    def test_targeted_tower_message_reaches_only_selected_rover(self):
        """Routing | Deliver tower text only to the rover selected by its certificate identity."""
        first, second = [], []
        self.rover("rover-1", first.append)
        self.rover("rover-2", second.append)
        receipt = self.tower.send("rover-2", "Hello selected rover")
        self.assertEqual(first, [])
        self.assertEqual(len(second), 1)
        self.assertEqual(second[0]['text'], 'Hello selected rover')
        self.assertEqual(receipt['id'], second[0]['id'])
        with self.assertRaisesRegex(ValueError, "not connected"):
            self.tower.send("rover-99", "hello")

    def test_simultaneous_two_way_messages_do_not_mix_with_receipts(self):
        """Two-way traffic | Send in both directions together without mixing messages or receipts."""
        received = []
        rover = self.rover(handler=received.append)
        with ThreadPoolExecutor(max_workers=2) as pool:
            for number in range(20):
                barrier = threading.Barrier(2)

                def upward():
                    barrier.wait(timeout=2)
                    return rover.send(f"up {number}")

                def downward():
                    barrier.wait(timeout=2)
                    return self.tower.send("rover-1", f"down {number}")
                up, down = pool.submit(upward), pool.submit(downward)
                self.assertEqual(up.result(timeout=3)['text'], f"up {number}")
                self.assertEqual(down.result(timeout=3)['text'], f"down {number}")
        self.assertEqual(len(received), 20)
        self.assertEqual(len(self.messages), 20)

    def test_disconnect_removes_rover_and_others_continue(self):
        """Disconnects | Remove one rover, keep another working, and allow the first to reconnect."""
        first, second = self.rover("rover-1"), self.rover("rover-2")
        first.close()
        self.wait_for(lambda: self.tower.rovers() == ["rover-2"])
        self.assertEqual(second.send("still connected")['text'], 'still connected')
        replacement = self.rover("rover-1")
        self.assertEqual(replacement.send("back again")['text'], 'back again')

    def test_duplicate_active_identity_is_rejected(self):
        """Authorization | Reject a duplicate rover identity while preserving the original connection."""
        original = self.rover()
        with self.raw_client() as duplicate:
            self.assertIsNone(recv_pdu(duplicate, timeout=1))
        self.wait_for(lambda: any("duplicate active rover identity" in line for line in self.logs))
        self.assertEqual(self.tower.rovers(), ["rover-1"])
        self.assertEqual(original.send("original survives")['text'], 'original survives')

    def test_stalled_handshake_does_not_block_existing_or_new_rovers(self):
        """Handshakes | Keep existing traffic and new rover connections working during a stalled handshake."""
        first = self.rover()
        with mock.patch('ground_station.HANDSHAKE_TIMEOUT', 1):
            with socket.create_connection(("127.0.0.1", self.tower.port)):
                with ThreadPoolExecutor(max_workers=2) as pool:
                    active = pool.submit(first.send, "still working")
                    joining = pool.submit(self.rover, "rover-2")
                    self.assertEqual(active.result(timeout=0.7)['text'], 'still working')
                    second = joining.result(timeout=0.7)
                    self.assertEqual(second.send("joined during handshake")['text'], 'joined during handshake')
                self.wait_for(lambda: any("handshake" in line and "timed out" in line for line in self.logs))

    def test_connection_limit_includes_pending_handshakes(self):
        """Connection limit | Bound established sessions and stalled handshakes without dropping existing rovers."""
        self.tower.stop()
        self.tower = Tower(self.certs, port=0, max_clients=2, log=self.logs.append,
                           handler=lambda identity, msg: self.messages.append((identity, msg))).start()
        original = self.rover()
        with socket.create_connection(("127.0.0.1", self.tower.port)):
            self.wait_for(lambda: len(self.tower.sockets) == 2)
            with self.assertRaises(OSError):
                self.raw_client("rover-2")
            self.assertEqual(original.send("capacity reached")['text'], 'capacity reached')
        self.assertTrue(any("connection limit" in line for line in self.logs))

    def test_missing_client_certificate(self):
        """Certificates | Reject a rover that presents no client certificate."""
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.load_verify_locations(cafile=self.certs / 'ca.crt')
        with self.assertRaises(OSError):
            with self.raw_client(context=context) as tls:
                send_pdu(tls, make_message('unauthorized'))
                recv_pdu(tls)
        self.assertEqual(self.messages, [])

    def test_untrusted_client_certificate(self):
        """Certificates | Reject a rover certificate issued by another CA."""
        context = client_context(self.rogue, 'rover-1')
        context.load_verify_locations(cafile=self.certs / 'ca.crt')
        with self.assertRaises(OSError):
            with self.raw_client(context=context) as tls:
                send_pdu(tls, make_message('unauthorized'))
                recv_pdu(tls)
        self.assertEqual(self.messages, [])

    def test_trusted_certificate_with_wrong_rover_identity(self):
        """Authorization | Reject a trusted client certificate without an authorized rover SAN identity."""
        context = client_context(self.certs, 'rover-1')
        context.load_cert_chain(self.folder / 'other-rover.crt', self.certs / 'rover-1.key')
        with self.raw_client(context=context) as tls:
            self.assertIsNone(recv_pdu(tls, timeout=1))
        self.wait_for(lambda: any('authorized rover SAN' in line for line in self.logs))
        self.assertEqual(self.messages, [])

    def test_incorrect_tower_hostname(self):
        """Certificates | Reject a tower certificate that does not match the expected hostname."""
        with self.assertRaises(ssl.SSLCertVerificationError):
            self.raw_client(name='wrong-tower.local')

    def test_untrusted_tower_certificate(self):
        """Certificates | Reject a tower certificate issued by an untrusted CA."""
        with self.assertRaises(ssl.SSLCertVerificationError):
            self.raw_client(context=client_context(self.rogue, 'rover-1'))

    def test_trusted_tower_with_wrong_station_identity(self):
        """Authorization | Verify the tower SAN identity as well as its trusted hostname."""
        context = server_context(self.certs)
        context.load_cert_chain(self.folder / 'other-tower.crt', self.certs / 'tower.key')
        def rejected(tls):
            try:
                tls.recv(1)
            except ConnectionResetError:
                pass  # The rover deliberately aborts after rejecting the identity.
        with self.peer(rejected, context) as (port, errors):
            with self.assertRaisesRegex(PermissionError, 'authorized tower SAN'):
                connect('127.0.0.1', port, client_context(self.certs, 'rover-1'))
        self.assertEqual(errors, [])

    def test_malformed_messages_never_reach_handler_and_other_rover_survives(self):
        """TLS input | Reject malformed/oversized messages without interrupting another rover."""
        other = self.rover('rover-2')
        invalid = make_message('hello') | {'extra': 1}
        oversized = make_message('hello') | {'text': 'x' * (MAX_TEXT_BYTES + 1)}
        packets = [struct.pack('!H', 9) + b'invalid', struct.pack('!H', MAX_PDU + 1),
                   struct.pack('!H', 1), encode_pdu(invalid), encode_pdu(oversized)]
        for packet in packets:
            with self.subTest(packet=packet[:12]):
                with self.raw_client() as tls:
                    tls.sendall(packet)
                    self.assertIsNone(recv_pdu(tls, timeout=1))
                self.wait_for(lambda: 'rover-1' not in self.tower.rovers())
        self.assertEqual(self.messages, [])
        self.assertEqual(other.send('valid after failures')['text'], 'valid after failures')

    def test_disconnect_mid_frame(self):
        """Disconnects | Detect a rover disconnect halfway through a TLS frame."""
        with self.raw_client() as tls:
            send_pdu(tls, make_message('connection established'))
            self.assertIsNotNone(recv_pdu(tls))
            tls.sendall(b'\x00')
            tls.shutdown(socket.SHUT_WR)
            # Keep the read side open until the tower sees EOF; shutdown disables TLS.
            self.wait_for(lambda: any('partway' in line for line in self.logs))
        self.assertEqual(len(self.messages), 1)

    def test_receiver_idle_and_partial_frame_timeouts(self):
        """Timeouts | Close both idle sessions and sessions stalled partway through a frame."""
        self.tower.read_timeout = 0.08
        for packet in (b'', b'\x00\x10{'):
            with self.subTest(packet=packet), self.raw_client() as tls:
                if packet:
                    tls.sendall(packet)
                self.assertIsNone(recv_pdu(tls, timeout=1))
            self.wait_for(lambda: 'rover-1' not in self.tower.rovers())
        self.assertEqual(self.messages, [])
        self.assertTrue(any('timed out' in line for line in self.logs))

    def test_invalid_or_missing_receipts_close_sender(self):
        """Receipts | Close on wrong IDs, text, status, malformed JSON, disconnects, or missing receipts."""
        for case in ('wrong_id', 'wrong_text', 'wrong_status', 'invalid_json', 'disconnect', 'timeout'):
            with self.subTest(case=case):
                def respond(tls):
                    sent = recv_pdu(tls)
                    if case == 'disconnect':
                        return
                    if case == 'timeout':
                        recv_pdu(tls, timeout=1)
                        return
                    if case == 'invalid_json':
                        tls.sendall(b'\x00\x05bad')
                        return
                    receipt = make_receipt(sent)
                    if case == 'wrong_id':
                        receipt['id'] = make_message('other')['id']
                    elif case == 'wrong_text':
                        receipt['text'] = 'changed'
                    else:
                        receipt['status'] = 'action completed'
                    send_pdu(tls, receipt)
                    recv_pdu(tls, timeout=1)
                with self.peer(respond) as (port, errors):
                    tls = connect('127.0.0.1', port, client_context(self.certs, 'rover-1'))
                    session = Connection(tls, 'tower', lambda msg: None, log=self.logs.append).start()
                    try:
                        with self.assertRaisesRegex(ConnectionClosed, 'unconfirmed'):
                            session.send('hello', timeout=0.08)
                        self.assertTrue(session.closed.is_set())
                    finally:
                        session.close()
                        session.join()
                self.assertEqual(errors, [])

    def test_unrelated_incoming_messages_do_not_extend_receipt_deadline(self):
        """Receipts | Keep the receipt deadline even while unrelated incoming messages are acknowledged."""
        incoming = []

        def respond(tls):
            recv_pdu(tls)  # Intentionally never acknowledge the outgoing message.
            try:
                for number in range(100):
                    send_pdu(tls, make_message(f'unrelated {number}'))
                    if recv_pdu(tls, timeout=1) is None:
                        break
                    threading.Event().wait(0.005)
            except OSError:
                pass
        with self.peer(respond) as (port, errors):
            tls = connect('127.0.0.1', port, client_context(self.certs, 'rover-1'))
            session = Connection(tls, 'tower', incoming.append, log=self.logs.append).start()
            try:
                started = time.monotonic()
                with self.assertRaisesRegex(ConnectionClosed, 'unconfirmed'):
                    session.send('needs its own receipt', timeout=0.1)
                self.assertLess(time.monotonic() - started, 0.7)
                self.assertGreater(len(incoming), 1)
            finally:
                session.close()
                session.join()
        self.assertEqual(errors, [])

    def test_unsolicited_receipt_closes_connection(self):
        """Receipts | Reject a receipt when no message is awaiting confirmation."""
        with self.raw_client() as tls:
            send_pdu(tls, make_receipt(make_message('unsolicited')))
            self.assertIsNone(recv_pdu(tls, timeout=1))
        self.wait_for(lambda: any('unexpected receipt' in line for line in self.logs))

    def test_connection_refused(self):
        """Connections | Report failure when no tower is listening."""
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        with self.assertRaises(OSError):
            connect('127.0.0.1', port, client_context(self.certs, 'rover-1'))

    def test_tls_write_timeout_with_nonreading_peer(self):
        """Timeouts | Bound TLS writes when a peer stops reading, using the shared connection helper."""
        release = threading.Event()
        with self.peer(lambda tls: release.wait(timeout=2)) as (port, errors):
            tls = connect('127.0.0.1', port, client_context(self.certs, 'rover-1'))
            tls.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
            session = Connection(tls, 'tower', lambda msg: None,
                                 io_timeout=0.03, log=self.logs.append)
            try:
                with self.assertRaisesRegex(TimeoutError, 'writing'):
                    for _ in range(1000):
                        session._write({'data': 'x' * 3000})
            finally:
                session.close()
                release.set()
        self.assertEqual(errors, [])

    def test_tower_shutdown_releases_sessions_and_pending_handshakes(self):
        """Shutdown | Close established sessions and pending handshakes and finish their workers."""
        rover = self.rover()
        with socket.create_connection(('127.0.0.1', self.tower.port)):
            self.wait_for(lambda: len(self.tower.sockets) == 2)
            with self.tower.lock:
                workers = list(self.tower.workers)
            self.tower.stop()
            self.assertTrue(rover.closed.wait(timeout=1))
            self.assertEqual(self.tower.rovers(), [])
            self.assertFalse(self.tower.thread.is_alive())
            self.assertTrue(all(not worker.is_alive() for worker in workers))

    def test_bad_console_text_does_not_close_a_healthy_session(self):
        """Input | Reject empty/oversized operator text locally and keep the connection usable."""
        rover = self.rover()
        for text in ('', '   ', 'x' * (MAX_TEXT_BYTES + 1)):
            with self.subTest(text_size=len(text)), self.assertRaises(ProtocolError):
                rover.send(text)
        self.assertFalse(rover.closed.is_set())
        self.assertEqual(rover.send('valid text')['text'], 'valid text')

    def test_client_handshake_timeout(self):
        """Timeouts | Stop the rover handshake when a TCP server does not speak TLS."""
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            listener.listen(1)
            with mock.patch('rover_client.HANDSHAKE_TIMEOUT', 0.05):
                with self.assertRaises(TimeoutError):
                    connect('127.0.0.1', listener.getsockname()[1], client_context(self.certs, 'rover-1'))

    def test_connection_timeout_parameter(self):
        """Timeouts | Keep the rover's three-second TCP connection deadline."""
        with mock.patch('rover_client.socket.create_connection', side_effect=socket.timeout('connect timed out')) as create:
            with self.assertRaises(TimeoutError):
                connect('127.0.0.1', 8443, client_context(self.certs, 'rover-1'))
            self.assertEqual(create.call_args.kwargs['timeout'], 3)

    def test_generator_preserves_existing_credentials_and_private_key_permissions(self):
        """Credentials | Refuse overwrites, validate rover counts, and restrict private-key permissions."""
        with self.assertRaisesRegex(ValueError, 'refusing to overwrite'):
            generate(self.certs)
        with self.assertRaisesRegex(ValueError, 'positive integer'):
            generate(self.folder / 'invalid', rovers=0)
        for key in self.certs.glob('*.key'):
            self.assertEqual(key.stat().st_mode & 0o777, 0o600)
        self.assertNotEqual((self.certs / 'rover-1.key').read_bytes(), (self.certs / 'rover-2.key').read_bytes())

    def test_actual_tower_and_rover_clis(self):
        """CLI demo | List two rovers, send targeted tower text, receive rover text, and run single-message mode."""
        root = Path(__file__).resolve().parent.parent
        tower = Process(root / 'ground_station.py', '--port', '0', '--certs', self.certs)
        self.addCleanup(tower.close)
        tower.expect('Tower listening on')
        startup = ''.join(tower.lines)
        port = startup.split('127.0.0.1:')[1].split()[0]
        clients = []
        for identity in ('rover-1', 'rover-2'):
            client = Process(root / 'rover_client.py', '--port', port, '--certs', self.certs,
                             '--rover-id', identity)
            self.addCleanup(client.close)
            clients.append(client)
            tower.expect(f'CONNECTED {identity}')
            client.expect('Type a message')
        tower.send('/rovers')
        tower.expect('Connected rovers: rover-1, rover-2')
        tower.send('/send rover-2 Hello selected rover')
        clients[1].expect("text='Hello selected rover'")
        tower.expect("RECEIVED by rover-2")
        self.assertNotIn('Hello selected rover', ''.join(clients[0].lines))
        clients[0].send('Rover telemetry placeholder')
        tower.expect("text='Rover telemetry placeholder'")
        clients[0].expect('RECEIVED by tower')
        single = subprocess.run([sys.executable, str(root / 'rover_client.py'), '--port', port,
                                 '--certs', str(self.certs), '--rover-id', 'rover-3', '--message', 'one shot'],
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(single.returncode, 0, single.stderr)
        self.assertIn("echo='one shot'", single.stdout)
        clients[0].send('/quit')
        self.assertEqual(clients[0].process.wait(timeout=3), 0)
        tower.expect('rover-1: connection closed')
        wrong = subprocess.run([sys.executable, str(root / 'rover_client.py'), '--port', port,
                                '--certs', str(self.certs), '--rover-id', 'rover-4',
                                '--server-name', 'wrong.local', '--message', 'hello'],
                               capture_output=True, text=True, timeout=5)
        self.assertEqual(wrong.returncode, 1)
        self.assertIn('CERTIFICATE_VERIFY_FAILED', wrong.stderr)
        tower.send('/send rover-2 Still working')
        clients[1].expect("text='Still working'")
        tower.send('/quit')
        self.assertEqual(tower.process.wait(timeout=3), 0)


if __name__ == '__main__':
    unittest.main()
