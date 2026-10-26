"""Mutual TLS for a tower server and individually identified rover clients."""

from pathlib import Path
import re
import ssl

STATION_ID = "urn:airport-rover:ground-station"
ROVER_ID_PREFIX = "urn:airport-rover:rover:"


def check_rover_id(identity):
    if not isinstance(identity, str) or not re.fullmatch(r"rover-[1-9][0-9]*", identity):
        raise ValueError("rover identity must be rover-1, rover-2, etc.")
    return identity


def server_context(certs):
    certs = Path(certs)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_3
    context.load_cert_chain(certs / "tower.crt", certs / "tower.key")
    context.load_verify_locations(cafile=certs / "ca.crt")
    context.verify_mode = ssl.CERT_REQUIRED
    context.verify_flags |= ssl.VERIFY_X509_STRICT
    return context


def client_context(certs, rover_id):
    check_rover_id(rover_id)
    certs = Path(certs)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    context.minimum_version = ssl.TLSVersion.TLSv1_3
    context.load_verify_locations(cafile=certs / "ca.crt")
    context.load_cert_chain(certs / f"{rover_id}.crt", certs / f"{rover_id}.key")
    context.verify_flags |= ssl.VERIFY_X509_STRICT
    # PROTOCOL_TLS_CLIENT enforces the CA chain and server hostname.
    return context


def rover_identity(connection):
    identities = [value[len(ROVER_ID_PREFIX):]
                  for kind, value in connection.getpeercert().get("subjectAltName", ())
                  if kind == "URI" and value.startswith(ROVER_ID_PREFIX)]
    if len(identities) != 1:
        raise PermissionError("client certificate needs exactly one authorized rover SAN identity")
    try:
        return check_rover_id(identities[0])
    except ValueError as error:
        raise PermissionError(str(error)) from error


def authorized_tower(connection):
    return ("URI", STATION_ID) in connection.getpeercert().get("subjectAltName", ())
