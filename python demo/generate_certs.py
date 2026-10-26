"""Generate a private, short-lived demonstration CA and role-specific certificates."""

import argparse
from pathlib import Path
import re
import subprocess
import tempfile

from security import ROVER_ID_PREFIX, STATION_ID


def openssl(*args):
    try:
        subprocess.run(["openssl", *map(str, args)], check=True, capture_output=True, text=True)
    except FileNotFoundError:
        raise RuntimeError("OpenSSL command is required to generate demo certificates") from None
    except subprocess.CalledProcessError as error:
        raise RuntimeError(error.stderr.strip()) from error


def generate(out, tower_name="tower.local", rovers=2):
    out = Path(out)
    if not re.fullmatch(r"[A-Za-z0-9](?:[A-Za-z0-9.-]{0,251}[A-Za-z0-9])?", tower_name):
        raise ValueError("tower-name must be a DNS name")
    if type(rovers) is not int or rovers < 1:
        raise ValueError("rovers must be a positive integer")
    if out.exists() and any(out.iterdir()):
        raise ValueError(f"refusing to overwrite nonempty certificate directory: {out}")
    out.mkdir(parents=True, exist_ok=True, mode=0o700)
    out.chmod(0o700)
    with tempfile.TemporaryDirectory(prefix="rover-cert-config-") as temp:
        config = Path(temp) / "ca.cnf"
        config.write_text('''[req]
distinguished_name = dn
x509_extensions = ca
prompt = no
[dn]
CN = Airport Rover Demo CA
[ca]
basicConstraints = critical,CA:TRUE,pathlen:0
keyUsage = critical,keyCertSign,cRLSign
subjectKeyIdentifier = hash
authorityKeyIdentifier = keyid:always
''')
        # Precreate keys with restricted permissions before OpenSSL writes data.
        names = ["tower"] + [f"rover-{number}" for number in range(1, rovers + 1)]
        for name in ["ca", *names]:
            key = out / f"{name}.key"
            key.touch(mode=0o600)
            openssl("genpkey", "-algorithm", "RSA", "-pkeyopt", "rsa_keygen_bits:2048", "-out", key)
        openssl("req", "-new", "-x509", "-sha256", "-days", "7", "-key", out / "ca.key",
                "-config", config, "-out", out / "ca.crt")
        for serial, name in enumerate(names, start=2):
            purpose = "serverAuth" if name == "tower" else "clientAuth"
            san = (f"DNS:{tower_name},URI:{STATION_ID}" if name == "tower"
                   else f"URI:{ROVER_ID_PREFIX}{name}")
            csr = Path(temp) / f"{name}.csr"
            ext = Path(temp) / f"{name}.ext"
            ext.write_text(f'''basicConstraints=critical,CA:FALSE
keyUsage=critical,digitalSignature,keyEncipherment
extendedKeyUsage={purpose}
subjectKeyIdentifier=hash
authorityKeyIdentifier=keyid:always
subjectAltName={san}
''')
            openssl("req", "-new", "-sha256", "-key", out / f"{name}.key",
                    "-subj", f"/CN={name}", "-out", csr)
            openssl("x509", "-req", "-sha256", "-days", "7", "-in", csr,
                    "-CA", out / "ca.crt", "-CAkey", out / "ca.key", "-set_serial", serial,
                    "-extfile", ext, "-out", out / f"{name}.crt")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path(__file__).parent / "certs-multi")
    parser.add_argument("--tower-name", default="tower.local")
    parser.add_argument("--rovers", type=int, default=2, help="generate rover-1 through rover-N (default: 2)")
    args = parser.parse_args()
    try:
        generate(args.out, args.tower_name, args.rovers)
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(1, f"Certificate generation failed: {error}\n")
    print(f"Generated demo certificates in {args.out}; valid for 7 days. Keep private keys private.")


if __name__ == "__main__":
    main()
