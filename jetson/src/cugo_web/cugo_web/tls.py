"""HTTPS 用の自己署名証明書（ブラウザの Gamepad API は https でしか使えないため）。

~/.config/cugo/web.crt / web.key が無ければ openssl で作る（10 年有効）。
"""
import os
import socket
import ssl
import subprocess
from typing import Optional

CERT_DIR = os.path.expanduser('~/.config/cugo')


def _local_ips():
    try:
        out = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [ip for ip in out.split() if ':' not in ip]


def ensure_cert(cert_dir: str = CERT_DIR):
    crt = os.path.join(cert_dir, 'web.crt')
    key = os.path.join(cert_dir, 'web.key')
    if os.path.exists(crt) and os.path.exists(key):
        return crt, key
    os.makedirs(cert_dir, exist_ok=True)
    host = socket.gethostname()
    san = ['DNS:%s' % host, 'DNS:%s.local' % host, 'DNS:localhost', 'IP:127.0.0.1']
    san += ['IP:%s' % ip for ip in _local_ips()]
    subprocess.run(
        ['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '3650',
         '-keyout', key, '-out', crt, '-subj', '/CN=%s' % host,
         '-addext', 'subjectAltName=' + ','.join(san)],
        check=True, capture_output=True, timeout=60)
    os.chmod(key, 0o600)
    return crt, key


def ssl_context(cert_dir: str = CERT_DIR) -> Optional[ssl.SSLContext]:
    crt, key = ensure_cert(cert_dir)
    ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ctx.load_cert_chain(crt, key)
    return ctx
