"""Tower server: accept authenticated rovers and send text to a selected rover."""

import argparse
import math
from pathlib import Path
import socket
import threading
import time

from connection import Connection, log_event
from protocol import HANDSHAKE_TIMEOUT, READ_TIMEOUT
from security import rover_identity, server_context


def handle_message(rover_id, message):
    """CPE INTEGRATION POINT: hand validated rover input to your application."""
    print(f"FROM {rover_id} id={message['id']} text={message['text']!r}", flush=True)


class Tower:
    def __init__(self, certs, host="127.0.0.1", port=8443, *, max_clients=16,
                 read_timeout=READ_TIMEOUT, handler=handle_message, log=log_event):
        if max_clients < 1 or not math.isfinite(read_timeout) or read_timeout <= 0:
            raise ValueError("max-clients and read-timeout must be positive and finite")
        self.context = server_context(certs)
        self.host, self.port = host, port
        self.read_timeout, self.handler, self.log = read_timeout, handler, log
        self.lock = threading.Lock()
        self.slots = threading.BoundedSemaphore(max_clients)
        self.stopping = threading.Event()
        self.clients, self.sockets, self.workers = {}, set(), set()
        self.listener = self.thread = None

    def start(self):
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.listener.bind((self.host, self.port))
            self.listener.listen()
            self.listener.settimeout(0.2)
            self.port = self.listener.getsockname()[1]
        except BaseException:
            self.listener.close()
            raise
        self.thread = threading.Thread(target=self._accept, name="tower-accept", daemon=True)
        self.thread.start()
        self.log(f"Tower listening on {self.host}:{self.port} (mutual TLS required).")
        return self

    def _accept(self):
        while not self.stopping.is_set():
            try:
                raw, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError as error:
                if not self.stopping.is_set():
                    self.log(f"Accept failed: {error}")
                return
            if not self.slots.acquire(blocking=False):
                self.log("Connection rejected: tower connection limit reached.")
                raw.close()
                continue
            worker = threading.Thread(target=self._serve, args=(raw,), daemon=True)
            with self.lock:
                self.sockets.add(raw)
                self.workers.add(worker)
                worker.start()

    def _serve(self, raw):
        tls, session, identity = None, None, None
        try:
            # Only this worker waits for the handshake; accept and other rovers continue.
            with self.lock:
                if self.stopping.is_set():
                    return
                tls = self.context.wrap_socket(raw, server_side=True, do_handshake_on_connect=False)
                self.sockets.discard(raw)
                self.sockets.add(tls)
            tls.settimeout(HANDSHAKE_TIMEOUT)
            tls.do_handshake()
            identity = rover_identity(tls)
            session = Connection(tls, identity, lambda msg: self.handler(identity, msg),
                                 read_timeout=self.read_timeout, log=self.log)
            with self.lock:
                if self.stopping.is_set():
                    return
                if identity in self.clients:
                    raise PermissionError(f"duplicate active rover identity: {identity}")
                self.clients[identity] = session
            self.log(f"CONNECTED {identity}")
            session.run()
        except (OSError, ValueError) as error:
            self.log(f"Connection rejected/closed: {error}")
        finally:
            if session is not None:
                session.close()
            elif tls is not None:
                tls.close()
            raw.close()
            with self.lock:
                if self.clients.get(identity) is session:
                    self.clients.pop(identity, None)
                self.sockets.discard(raw)
                self.sockets.discard(tls)
                self.workers.discard(threading.current_thread())
            self.slots.release()

    def rovers(self):
        with self.lock:
            return sorted(self.clients)

    def send(self, rover_id, text, **options):
        with self.lock:
            session = self.clients.get(rover_id)
        if session is None:
            raise ValueError(f"rover is not connected: {rover_id}")
        return session.send(text, **options)

    def stop(self):
        self.stopping.set()
        if self.listener is not None:
            self.listener.close()
        # Finish accept before taking the final snapshot of workers/sockets.
        if self.thread is not None:
            self.thread.join(timeout=1)
        with self.lock:
            sessions, sockets, workers = list(self.clients.values()), list(self.sockets), list(self.workers)
        for session in sessions:
            session.close("tower shutting down")
        for connection in sockets:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        deadline = time.monotonic() + 2
        for worker in workers:
            worker.join(timeout=max(0, deadline - time.monotonic()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--certs", type=Path, default=Path(__file__).parent / "certs-multi")
    parser.add_argument("--max-clients", type=int, default=16, help="includes pending TLS handshakes")
    parser.add_argument("--read-timeout", type=float, default=READ_TIMEOUT)
    args = parser.parse_args()
    tower = None
    try:
        tower = Tower(args.certs, args.host, args.port, max_clients=args.max_clients,
                      read_timeout=args.read_timeout).start()
        print("/rovers | /send rover-1 text | /quit", flush=True)
        while True:
            try:
                line = input("tower> ")
            except EOFError:
                break
            if line == "/quit":
                break
            try:
                if line == "/rovers":
                    print("Connected rovers: " + (", ".join(tower.rovers()) or "none"), flush=True)
                elif line.startswith("/send "):
                    parts = line.split(maxsplit=2)
                    if len(parts) != 3:
                        raise ValueError("usage: /send rover-1 text")
                    receipt = tower.send(parts[1], parts[2])
                    print(f"RECEIVED by {parts[1]} id={receipt['id']} echo={receipt['text']!r}", flush=True)
                else:
                    print("Use /rovers, /send rover-1 text, or /quit.", flush=True)
            except (OSError, ValueError) as error:
                print(f"SEND/INPUT ERROR: {error}", flush=True)
    except KeyboardInterrupt:
        print("\nTower shutting down.")
    except (OSError, ValueError) as error:
        parser.exit(1, f"Tower error: {error}\n")
    finally:
        if tower is not None:
            tower.stop()


if __name__ == "__main__":
    main()
