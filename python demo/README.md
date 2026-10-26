# Airport rover secure communications

A Python MVP for **one tower and multiple rovers**, with secure text messaging in
both directions. The tower is the TCP/TLS server; each rover connects as a client.
The tower can address a specific rover, and rovers can send their own messages.

**A receipt means “message received,” not “robot action completed.”**
Telemetry and commands are represented by text; robot behavior and video are not
implemented.

## Run a two-rover demo

Requires Python **3.11+** with TLS 1.3 support and the `openssl` command.
No pip packages are needed. Run all commands from the repository root.

Generate credentials once. This example provisions four rover identities so you
can demonstrate either two or four connected clients:

```sh
python3 generate_certs.py --rovers 4
```

Credentials go in `certs-multi/`, separate from the old demo's `certs/`.
Generation refuses to overwrite existing credentials. Certificates last seven
days. For replacements, use `--out certs-new --rovers 4` and pass
`--certs certs-new` when starting the tower and each rover.

**Terminal 1 — tower:**

```sh
python3 ground_station.py
```

**Terminal 2 — first rover:**

```sh
python3 rover_client.py --rover-id rover-1
```

**Terminal 3 — second rover:**

```sh
python3 rover_client.py --rover-id rover-2
```

Type text in either rover terminal. The tower displays the authenticated sender,
and the rover receives a matching receipt. At the tower prompt, try:

```text
/rovers
/send rover-1 Hello first rover
/send rover-2 Hello second rover
```

Only the selected rover receives each tower message. Both endpoints keep reading
incoming messages while waiting for keyboard input. Output may appear beside an
unfinished prompt. Type `/quit` or press Ctrl+C to exit an endpoint. Disconnecting
one rover leaves the others working. Restart a rover manually to reconnect.

To try four clients, open two more terminals with `--rover-id rover-3` and
`--rover-id rover-4`. For a single-message client:

```sh
python3 rover_client.py --rover-id rover-3 --message "Hello tower"
```

## Security and limits

Mutual TLS encrypts traffic and verifies both endpoints. The demo CA signs the
tower and rover certificates; both endpoints trust its public certificate,
`ca.crt`. Each endpoint proves possession of its private key during the handshake.
The rover verifies `tower.local` and the tower SAN URI
`urn:airport-rover:ground-station`. The tower identifies clients by their verified
SAN URI, such as `urn:airport-rover:rover:rover-1`, and rejects duplicate active
identities. `--rover-id` selects a credential; it does not override that identity.

**Never commit private keys.** Credential directories and keys are ignored by Git.
For separate computers, give the tower `ca.crt`, `tower.crt`, and `tower.key`; give
each rover `ca.crt` and only its own certificate/key. Keep `ca.key` with the issuer.
On the tower use `--host 0.0.0.0`; on rovers use `--host TOWER_IP`. Both support
`--port` and `--certs`. The default connection is `127.0.0.1:8443`, with expected
certificate name `tower.local` even when connecting by IP.

Messages retain UTF-8 JSON and the two-byte big-endian total-length header,
including the header itself. Text is limited to 512 UTF-8 bytes and frames to
4096 bytes. Receipts must match the message UUID and echoed text.

TCP connect uses a 3-second timeout; TLS handshakes, writes, and receipt waits use
5 seconds. Both endpoints default to a 60-second idle/full-frame read timeout;
change it with `--read-timeout 120`. A missing/invalid receipt closes that session
and leaves delivery unconfirmed; there are no automatic retries or heartbeats.
After an idle disconnect, press Enter or `/quit` to leave a waiting console and
restart the rover. The tower defaults to 16 connection workers, including pending
handshakes; adjust `--max-clients` as needed. Two and four clients are covered by
tests; a larger deployment capacity has not been measured.

## Files and team integration

| File                     | Purpose                                                            |
| ------------------------ | ------------------------------------------------------------------ |
| `ground_station.py`      | Tower server, connected-rover list, `/rovers`, and `/send`.        |
| `rover_client.py`        | Rover client, interactive input, and `--message` mode.             |
| `connection.py`          | Background reception, serialized TLS writes, and receipt matching. |
| `protocol.py`            | Existing message validation, framing, limits, and I/O deadlines.   |
| `security.py`            | TLS contexts and certificate identity checks.                      |
| `generate_certs.py`      | Demo CA, tower credentials, and separate rover credentials.        |
| `run_tests.py`, `tests/` | Readable test results and protocol/TLS/CLI checks.                 |

`handle_message(rover_id, message)` in `ground_station.py` receives rover text;
`handle_message(message)` in `rover_client.py` receives tower text. These are the
CPE integration points and currently only print messages. Keep handlers short
and keep receipts separate from future action results. Navigation, override
priority, physical stopping, ROS, structured telemetry, and video remain CPE
integration work. `Reference_464/` is the unchanged networking assignment reference.

## Tests

```sh
python3 run_tests.py
```

Each test prints what it checks and `ok` (pass), `FAIL`, or `ERROR`, followed by a
summary. A rejection check passes when unsafe input is rejected. Tests create
temporary credentials and connections; no running tower or rovers are needed.
They cover two/four clients, targeted delivery, simultaneous two-way messages,
receipt matching, disconnect isolation, stalled handshakes, certificate/identity
failures, framing, and timeouts. Network tests need localhost socket permission.
The standard `python3 -m unittest discover -s tests -v` command also works.
