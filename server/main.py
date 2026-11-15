"""PhoneMotion server entry point.

Serves the mobile web client over HTTP/HTTPS and receives sensor telemetry
over WebSocket on the same port so only a single command is needed.
"""

from __future__ import annotations

import argparse
import asyncio
import datetime
import ipaddress
import socket
import ssl
from http import HTTPStatus
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID
from websockets.asyncio.server import Request, ServerConnection, serve
from websockets.datastructures import Headers
from websockets.http11 import Response

try:
    from server.input_controller import InputController
    from server.ws_handler import ControllerSession
except ImportError:
    from input_controller import InputController
    from ws_handler import ControllerSession

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CLIENT_DIR = PROJECT_ROOT / "client"
CERT_DIR = Path(__file__).resolve().parent / "certs"

STATIC_ROUTES: dict[str, tuple[str, str]] = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
    "/app.js": ("app.js", "application/javascript; charset=utf-8"),
}


def get_local_ip() -> str:
    """Return the laptop's active LAN IPv4 address, falling back to 127.0.0.1."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Does not send traffic; asks the OS routing table which interface reaches the LAN/internet.
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


def _cert_matches_ip(cert_path: Path, ip_address: str) -> bool:
    """Check whether an existing certificate already includes the current LAN IP in its SANs."""
    try:
        cert = x509.load_pem_x509_certificate(cert_path.read_bytes())
        san = cert.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
        target_ip = ipaddress.ip_address(ip_address)
        return target_ip in san.get_values_for_type(x509.IPAddress)
    except Exception:
        return False


def ensure_self_signed_cert(ip_address: str, cert_dir: Path = CERT_DIR) -> tuple[Path, Path]:
    """Generate a local self-signed TLS certificate for the given LAN IP if needed."""
    cert_dir.mkdir(parents=True, exist_ok=True)
    cert_path = cert_dir / "cert.pem"
    key_path = cert_dir / "key.pem"

    if cert_path.exists() and key_path.exists() and _cert_matches_ip(cert_path, ip_address):
        return cert_path, key_path

    private_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)

    subject = issuer = x509.Name(
        [
            x509.NameAttribute(NameOID.COMMON_NAME, f"PhoneMotion ({ip_address})"),
        ]
    )

    san_entries: list[x509.GeneralName] = [
        x509.DNSName("localhost"),
        x509.IPAddress(ipaddress.ip_address("127.0.0.1")),
    ]
    parsed_ip = ipaddress.ip_address(ip_address)
    if parsed_ip != ipaddress.ip_address("127.0.0.1"):
        san_entries.append(x509.IPAddress(parsed_ip))

    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(private_key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(minutes=5))
        .not_valid_after(now + datetime.timedelta(days=365))
        .add_extension(x509.SubjectAlternativeName(san_entries), critical=False)
        .add_extension(x509.BasicConstraints(ca=True, path_length=None), critical=True)
        .add_extension(
            x509.KeyUsage(
                digital_signature=True,
                key_encipherment=True,
                key_cert_sign=True,
                content_commitment=False,
                data_encipherment=False,
                key_agreement=False,
                crl_sign=False,
                encipher_only=False,
                decipher_only=False,
            ),
            critical=True,
        )
        .add_extension(
            x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]),
            critical=False,
        )
        .sign(private_key, hashes.SHA256())
    )

    key_path.write_bytes(
        private_key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.PKCS8,
            encryption_algorithm=serialization.NoEncryption(),
        )
    )
    cert_path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    return cert_path, key_path


def _file_response(body: bytes, content_type: str) -> Response:
    """Build an HTTP 200 response for a static client asset or certificate."""
    headers = Headers(
        [
            ("Content-Type", content_type),
            ("Content-Length", str(len(body))),
            ("Cache-Control", "no-store"),
        ]
    )
    return Response(HTTPStatus.OK, "OK", headers, body)


def process_request(connection: ServerConnection, request: Request) -> Response | None:
    """Serve static client files for HTTP requests and allow WebSocket upgrades."""
    path = request.path.split("?", 1)[0]
    upgrade = request.headers.get("Upgrade", "").lower()

    if upgrade == "websocket":
        return None

    if path == "/ws":
        return connection.respond(HTTPStatus.UPGRADE_REQUIRED, "WebSocket endpoint\n")

    if path == "/cert.pem":
        cert_path = CERT_DIR / "cert.pem"
        if not cert_path.is_file():
            return connection.respond(
                HTTPStatus.NOT_FOUND,
                "Certificate not found. Start the server with --https first.\n",
            )
        return _file_response(cert_path.read_bytes(), "application/x-x509-ca-cert")

    route = STATIC_ROUTES.get(path)
    if route is None:
        return connection.respond(HTTPStatus.NOT_FOUND, "404 Not Found\n")

    filename, content_type = route
    file_path = CLIENT_DIR / filename
    if not file_path.is_file():
        return connection.respond(HTTPStatus.NOT_FOUND, "Client file not found\n")

    return _file_response(file_path.read_bytes(), content_type)


async def run_server(port: int, use_https: bool) -> None:
    """Start the combined HTTP/HTTPS + WebSocket server."""
    local_ip = get_local_ip()
    ssl_context: ssl.SSLContext | None = None
    scheme = "http"

    if use_https:
        cert_path, key_path = ensure_self_signed_cert(local_ip)
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(certfile=str(cert_path), keyfile=str(key_path))
        scheme = "https"

    input_controller = InputController()
    session = ControllerSession(input_controller=input_controller)
    session.bind_loop(asyncio.get_running_loop())
    input_controller.start_esc_listener(session.trigger_esc_stop)

    print("=" * 64)
    print("  PhoneMotion Server (Phase 2: Mouse Control)")
    print("=" * 64)
    print(f"  Laptop Local IP : {local_ip}")
    print(f"  Open on Phone   : {scheme}://{local_ip}:{port}")
    print(f"  Local Testing   : {scheme}://localhost:{port}")
    print("  Safety Switch   : Press ESC on laptop keyboard to Emergency Stop")
    if use_https:
        print(f"  TLS Certificate : {CERT_DIR / 'cert.pem'}")
    else:
        print(f"  Android Tip     : Enable chrome://flags/#unsafely-treat-insecure-origin-as-secure")
        print(f"                    for http://{local_ip}:{port} (or run with --https)")
    print("=" * 64)
    print("Waiting for phone connection... (Press Ctrl+C to stop)\n")

    try:
        async with serve(
            session.handle_client,
            host="0.0.0.0",
            port=port,
            process_request=process_request,
            ssl=ssl_context,
        ):
            await asyncio.Future()
    finally:
        input_controller.release_all()
        input_controller.stop_esc_listener()


def parse_args() -> argparse.Namespace:
    """Parse command-line options."""
    parser = argparse.ArgumentParser(description="PhoneMotion WebSocket & Client Server")
    parser.add_argument("--port", type=int, default=8000, help="Port to listen on (default: 8000)")
    parser.add_argument(
        "--https",
        action="store_true",
        help="Optional: serve over HTTPS/WSS with an auto-generated self-signed certificate",
    )
    return parser.parse_args()


def main() -> None:
    """CLI entry point."""
    args = parse_args()
    try:
        asyncio.run(run_server(port=args.port, use_https=args.https))
    except KeyboardInterrupt:
        print("\nServer stopped.")


if __name__ == "__main__":
    main()
