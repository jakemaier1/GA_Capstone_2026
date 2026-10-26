"""Rover client: connect to the tower and exchange text in both directions."""

import argparse
import math
from pathlib import Path
import socket

from connection import Connection
from protocol import CONNECT_TIMEOUT, HANDSHAKE_TIMEOUT, READ_TIMEOUT, ProtocolError
from security import authorized_tower, client_context


def handle_message(message):
    """CPE INTEGRATION POINT: currently prints tower text; performs no robot action."""
    print(f"FROM tower id={message['id']} text={message['text']!r}", flush=True)


def connect(host, port, context, server_name="tower.local"):
    raw = socket.create_connection((host, port), timeout=CONNECT_TIMEOUT)
    tls = None
    try:
        raw.settimeout(HANDSHAKE_TIMEOUT)
        tls = context.wrap_socket(raw, server_hostname=server_name)
        if not authorized_tower(tls):
            raise PermissionError("server certificate lacks the authorized tower SAN identity")
        return tls
    except BaseException:
        (tls if tls is not None else raw).close()
        raise


def show_receipt(receipt):
    print(f"RECEIVED by tower id={receipt['id']} echo={receipt['text']!r}", flush=True)


def run(args):
    if not math.isfinite(args.read_timeout) or args.read_timeout <= 0:
        raise ValueError("read-timeout must be finite and positive")
    context = client_context(args.certs, args.rover_id)
    session = None
    try:
        tls = connect(args.host, args.port, context, args.server_name)
        print(f"TLS connected to tower ({tls.version()}); credential={args.rover_id}.", flush=True)
        session = Connection(tls, "tower", handle_message, read_timeout=args.read_timeout).start()
        if args.message is not None:
            show_receipt(session.send(args.message))
            return
        print("Type a message. /quit or Ctrl+D disconnects. Receipts mean message received.", flush=True)
        while not session.closed.is_set():
            try:
                text = input("rover> ")
            except EOFError:
                return
            if text == "/quit":
                return
            try:
                show_receipt(session.send(text))
            except ProtocolError as error:
                print(f"INPUT REJECTED: {error}", flush=True)
    finally:
        if session is not None:
            session.close("rover exiting")
            session.join()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--server-name", default="tower.local")
    parser.add_argument("--certs", type=Path, default=Path(__file__).parent / "certs-multi")
    parser.add_argument("--rover-id", default="rover-1", help="selects this rover's certificate/key")
    parser.add_argument("--read-timeout", type=float, default=READ_TIMEOUT)
    parser.add_argument("--message", help="send one text message and exit")
    args = parser.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        print("\nDisconnected.")
    except (OSError, ValueError) as error:
        parser.exit(1, f"Rover error: {error}\n")


if __name__ == "__main__":
    main()
