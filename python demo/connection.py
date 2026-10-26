"""One reader, serialized writes, and matching receipts for a two-way TLS link."""

import select
import socket
import ssl
import threading
import time

from protocol import (IO_TIMEOUT, READ_TIMEOUT, ConnectionClosed, ProtocolError,
                      make_message, make_receipt, recv_pdu, send_pdu,
                      validate_message, validate_receipt)


def log_event(text):
    print(text, flush=True)


class TLSStream:
    """Keep deadlines local to each thread and serialize individual SSL calls.

    The socket stays nonblocking. Waiting outside the SSL lock lets the reader
    wait for input without blocking a writer or changing its socket timeout.
    Existing protocol helpers can keep using their socket-like interface.
    """

    def __init__(self, connection):
        self.socket = connection
        connection.setblocking(False)
        self.ssl_lock = threading.Lock()
        self.local = threading.local()

    def gettimeout(self):
        return getattr(self.local, "timeout", None)

    def settimeout(self, timeout):
        self.local.timeout = timeout

    def _call(self, operation, deadline):
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError("TLS I/O deadline expired")
            try:
                with self.ssl_lock:
                    return operation()
            except (ssl.SSLWantReadError, ssl.SSLWantWriteError) as error:
                reading = isinstance(error, ssl.SSLWantReadError)
            remaining = None if deadline is None else deadline - time.monotonic()
            if remaining is not None and remaining <= 0:
                raise TimeoutError("TLS I/O deadline expired")
            ready = select.select([self.socket] if reading else [],
                                  [] if reading else [self.socket], [], remaining)
            if not ready[0] and not ready[1]:
                raise TimeoutError("TLS I/O deadline expired")

    def _deadline(self):
        timeout = self.gettimeout()
        return None if timeout is None else time.monotonic() + timeout

    def recv(self, count):
        return self._call(lambda: self.socket.recv(count), self._deadline())

    def sendall(self, packet):
        deadline = self._deadline()
        remaining = memoryview(packet)
        while remaining:
            sent = self._call(lambda: self.socket.send(remaining), deadline)
            if not sent:
                raise ConnectionClosed("peer disconnected during write")
            remaining = remaining[sent:]

    def close(self):
        with self.ssl_lock:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.socket.close()


class Connection:
    """A background reader serves messages while the console waits for receipts."""

    def __init__(self, connection, peer, handler, *, log=log_event,
                 read_timeout=READ_TIMEOUT, io_timeout=IO_TIMEOUT):
        self.stream = TLSStream(connection)
        self.peer, self.handler, self.log = peer, handler, log
        self.read_timeout, self.io_timeout = read_timeout, io_timeout
        self.write_lock = threading.Lock()
        self.request_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.closed = threading.Event()
        self.received = threading.Event()
        self.pending = self.receipt = None
        self.reason = "disconnected"
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.run, name=f"reader-{self.peer}", daemon=True)
        self.thread.start()
        return self

    def _write(self, packet):
        deadline = time.monotonic() + self.io_timeout
        if not self.write_lock.acquire(timeout=self.io_timeout):
            raise TimeoutError("timed out waiting to write a frame")
        try:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("timed out writing a frame")
            send_pdu(self.stream, packet, timeout=remaining)
        finally:
            self.write_lock.release()

    def send(self, text, timeout=IO_TIMEOUT):
        message = make_message(text)
        if not self.request_lock.acquire(blocking=False):
            raise ProtocolError("a message to this peer is still awaiting its receipt")
        try:
            with self.state_lock:
                if self.closed.is_set():
                    raise ConnectionClosed(self.reason)
                self.pending, self.receipt = message, None
                self.received.clear()
            self._write(message)
            if not self.received.wait(timeout):
                raise TimeoutError("timed out waiting for the matching receipt")
            with self.state_lock:
                if self.receipt is None:
                    raise ConnectionClosed(self.reason)
                return self.receipt
        except (OSError, ValueError) as error:
            self.close(str(error))
            raise ConnectionClosed(f"message receipt is unconfirmed: {error}") from error
        finally:
            with self.state_lock:
                self.pending = None
            self.request_lock.release()

    def run(self):
        try:
            while not self.closed.is_set():
                packet = recv_pdu(self.stream, timeout=self.read_timeout)
                if packet is None:
                    return
                if packet.get("type") == "message":
                    validate_message(packet)
                    self.handler(packet)
                    self._write(make_receipt(packet))
                else:
                    with self.state_lock:
                        if self.pending is None or self.receipt is not None:
                            raise ProtocolError("unexpected receipt or message type")
                        validate_receipt(packet, self.pending)
                        self.receipt = packet
                        self.received.set()
        except (OSError, ValueError) as error:
            self.close(str(error))
        finally:
            self.close()

    def close(self, reason="peer disconnected"):
        with self.state_lock:
            if self.closed.is_set():
                return
            self.reason = reason
            self.closed.set()
            self.received.set()
        self.stream.close()
        self.log(f"{self.peer}: connection closed ({reason}).")

    def join(self):
        if self.thread is not None:
            self.thread.join(timeout=2)
