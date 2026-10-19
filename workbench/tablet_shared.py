"""Shared local configuration, certificate and token helpers for tablet services."""
from __future__ import annotations

import datetime
import hashlib
import hmac
import ipaddress
import json
import os
import pathlib
import secrets
import socket
import ssl
import threading

ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "config/tablet_hmi.json"


def load_config(path=None):
    path = pathlib.Path(path or CONFIG_PATH)
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("平板配置顶层必须是对象")
    return data


class TabletPaths:
    def __init__(self, root=None, config=None):
        self.root = pathlib.Path(root or ROOT)
        self.config = config or load_config(self.root / "config/tablet_hmi.json")
        paths = self.config.get("paths", {})
        self.runtime = self.root / paths.get("runtime_dir", ".runtime/tablet_hmi")
        self.certs = self.root / paths.get("certificates", ".runtime/tablet_hmi/certs")
        self.state = self.runtime / "gateway-state.json"
        self.media_secret = self.runtime / "media.secret"
        self.admin_secret = self.runtime / "admin.secret"
        self.runtime.mkdir(parents=True, exist_ok=True)
        self.certs.mkdir(parents=True, exist_ok=True)

    def ensure_secret(self, path):
        path = pathlib.Path(path)
        with _secret_lock:
            try:
                value = path.read_text(encoding="ascii").strip()
                if len(value) >= 32:
                    return value
            except OSError:
                pass
            value = secrets.token_urlsafe(48)
            path.write_text(value + "\n", encoding="ascii")
            path.chmod(0o600)
            return value

    def admin_token(self):
        return self.ensure_secret(self.admin_secret)

    def media_token_secret(self):
        return self.ensure_secret(self.media_secret)


_secret_lock = threading.Lock()


def local_addresses(hostname="piper.local"):
    values = {hostname, socket.gethostname(), "localhost"}
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.settimeout(0.2)
        probe.connect(("8.8.8.8", 80))
        values.add(probe.getsockname()[0])
        probe.close()
    except OSError:
        pass
    try:
        for row in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            values.add(row[4][0])
    except OSError:
        pass
    return sorted(values)


def create_certificates(paths: TabletPaths, force=False):
    """Create a local CA and a server certificate trusted only after installing the CA."""
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID

    ca_key_path = paths.certs / "ca.key"
    ca_cert_path = paths.certs / "ca.crt"
    server_key_path = paths.certs / "server.key"
    server_cert_path = paths.certs / "server.crt"
    if not force and all(x.exists() for x in (ca_key_path, ca_cert_path, server_key_path, server_cert_path)):
        return {"created": False, "ca": str(ca_cert_path), "server": str(server_cert_path)}

    now = datetime.datetime.now(datetime.timezone.utc)
    ca_key = rsa.generate_private_key(public_exponent=65537, key_size=3072)
    ca_name = x509.Name([
        x509.NameAttribute(NameOID.COMMON_NAME, "Piper Workbench Local CA"),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, "Piper Workbench"),
    ])
    ca_cert = (
        x509.CertificateBuilder()
        .subject_name(ca_name)
        .issuer_name(ca_name)
        .public_key(ca_key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(minutes=5))
        .not_valid_after(now + datetime.timedelta(days=3650))
        .add_extension(x509.BasicConstraints(ca=True, path_length=1), critical=True)
        .add_extension(x509.KeyUsage(
            digital_signature=True, content_commitment=False, key_encipherment=False,
            data_encipherment=False, key_agreement=False, key_cert_sign=True,
            crl_sign=True, encipher_only=False, decipher_only=False,
        ), critical=True)
        .sign(ca_key, hashes.SHA256())
    )

    hostname = paths.config.get("gateway", {}).get("hostname", "piper.local")
    names = {hostname, "localhost", "127.0.0.1", "::1"}
    names.update(local_addresses(hostname))
    san = []
    for name in sorted(names):
        try:
            san.append(x509.IPAddress(ipaddress.ip_address(name)))
        except ValueError:
            san.append(x509.DNSName(name))

    server_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    server_name = x509.Name([
        x509.NameAttribute(NameOID.COMMON_NAME, hostname),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, "Piper Workbench"),
    ])
    server_cert = (
        x509.CertificateBuilder()
        .subject_name(server_name)
        .issuer_name(ca_cert.subject)
        .public_key(server_key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(minutes=5))
        .not_valid_after(now + datetime.timedelta(days=397))
        .add_extension(x509.SubjectAlternativeName(san), critical=False)
        .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
        .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
        .sign(ca_key, hashes.SHA256())
    )

    ca_key_path.write_bytes(ca_key.private_bytes(
        serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL,
        serialization.NoEncryption(),
    ))
    ca_cert_path.write_bytes(ca_cert.public_bytes(serialization.Encoding.PEM))
    server_key_path.write_bytes(server_key.private_bytes(
        serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL,
        serialization.NoEncryption(),
    ))
    server_cert_path.write_bytes(server_cert.public_bytes(serialization.Encoding.PEM))
    for path in (ca_key_path, server_key_path):
        path.chmod(0o600)
    return {"created": True, "ca": str(ca_cert_path), "server": str(server_cert_path)}


def certificate_info(paths: TabletPaths):
    from cryptography import x509

    create_certificates(paths)
    cert_path = paths.certs / "server.crt"
    ca_path = paths.certs / "ca.crt"
    cert = x509.load_pem_x509_certificate(cert_path.read_bytes())
    now = datetime.datetime.now(datetime.timezone.utc)
    not_after = cert.not_valid_after_utc
    return {
        "state": "valid" if not_after > now else "expired",
        "subject": cert.subject.rfc4514_string(),
        "not_before": cert.not_valid_before_utc.isoformat(),
        "not_after": not_after.isoformat(),
        "days_remaining": max(0, (not_after - now).days),
        "ca_path": str(ca_path),
        "server_path": str(cert_path),
    }


def server_ssl_context(paths: TabletPaths):
    create_certificates(paths)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(paths.certs / "server.crt", paths.certs / "server.key")
    return context


def media_signature(secret, device_id, camera, expires):
    message = f"{device_id}|{camera}|{int(expires)}".encode()
    return hmac.new(secret.encode(), message, hashlib.sha256).hexdigest()


def issue_media_token(paths: TabletPaths, device_id, camera, ttl=60):
    expires = int(datetime.datetime.now(datetime.timezone.utc).timestamp() + int(ttl))
    secret = paths.media_token_secret()
    return {
        "device_id": device_id,
        "camera": camera,
        "expires": expires,
        "signature": media_signature(secret, device_id, camera, expires),
    }


def verify_media_token(paths: TabletPaths, device_id, camera, expires, signature):
    try:
        expires = int(expires)
    except (TypeError, ValueError):
        return False
    if expires < int(datetime.datetime.now(datetime.timezone.utc).timestamp()):
        return False
    expected = media_signature(paths.media_token_secret(), device_id, camera, expires)
    return hmac.compare_digest(expected, str(signature))


def token_digest(value):
    return hashlib.sha256(str(value).encode()).hexdigest()
