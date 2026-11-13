"""HTTPS 用の自己署名証明書（ブラウザの Gamepad API は https でしか使えないため）。

~/.config/cugo/web.crt / web.key が無ければ openssl で作る（10 年有効）。
既存の証明書に今のアドレス（Tailscale の 100.x や MagicDNS 名を含む）が
載っていなければ作り直す。全部載っていれば作り直さない。
"""
import json
import os
import socket
import ssl
import subprocess
import sys
from typing import List, Optional, Set

CERT_DIR = os.path.expanduser('~/.config/cugo')


def _local_ips():
    try:
        out = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [ip for ip in out.split() if ':' not in ip]


def _tailscale_dns() -> Optional[str]:
    """Tailscale の MagicDNS 名（例: cugo-jetson.tail1234.ts.net）。使えなければ None。"""
    try:
        r = subprocess.run(['tailscale', 'status', '--json'],
                           capture_output=True, text=True, timeout=3)
        if r.returncode != 0:
            return None
        name = json.loads(r.stdout).get('Self', {}).get('DNSName') or ''
    except (OSError, subprocess.SubprocessError, ValueError, AttributeError):
        return None
    name = name.rstrip('.')
    return name or None


def cert_names(crt_path: str) -> Set[str]:
    """証明書の SAN を {'IP:1.2.3.4', 'DNS:host', ...} の形で返す。読めなければ空集合。"""
    try:
        r = subprocess.run(['openssl', 'x509', '-in', crt_path, '-noout', '-text'],
                           capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return set()
    if r.returncode != 0:
        return set()
    lines = r.stdout.splitlines()
    names = set()
    for i, line in enumerate(lines):
        if 'Subject Alternative Name' in line and i + 1 < len(lines):
            # openssl は「DNS:host, IP Address:1.2.3.4」の形で 1 行に出す
            for item in lines[i + 1].split(','):
                item = item.strip()
                if item.startswith('IP Address:'):
                    names.add('IP:' + item[len('IP Address:'):])
                elif item.startswith('DNS:'):
                    names.add(item)
    return names


def wanted_names(host: str, ips: List[str], ts_dns: Optional[str]) -> Set[str]:
    names = {'DNS:%s' % host, 'DNS:%s.local' % host, 'DNS:localhost', 'IP:127.0.0.1'}
    names |= {'IP:%s' % ip for ip in ips}
    if ts_dns:
        names.add('DNS:%s' % ts_dns)
    return names


def _san_sorted(names: Set[str]) -> List[str]:
    # DNS → IP の順（見やすさのため。中身は集合と同じ）
    return sorted(names, key=lambda n: (not n.startswith('DNS:'), n))


def ensure_cert(cert_dir: str = CERT_DIR):
    crt = os.path.join(cert_dir, 'web.crt')
    key = os.path.join(cert_dir, 'web.key')
    host = socket.gethostname()
    want = wanted_names(host, _local_ips(), _tailscale_dns())
    exists = os.path.exists(crt) and os.path.exists(key)
    if exists:
        have = cert_names(crt)
        missing = want - have
        if not missing:
            return crt, key
        # 以前の名前（研究室の別の IP など）も残す。行き来するたびに作り直さないように
        want |= have
        print('cugo_web: 証明書に %s が無いので作り直します' % ', '.join(sorted(missing)),
              file=sys.stderr)
    os.makedirs(cert_dir, exist_ok=True)
    # 一時ファイルに作ってから置き換える（失敗しても古い証明書が壊れないように）
    tmp_crt, tmp_key = crt + '.tmp', key + '.tmp'
    try:
        subprocess.run(
            ['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '3650',
             '-keyout', tmp_key, '-out', tmp_crt, '-subj', '/CN=%s' % host,
             '-addext', 'subjectAltName=' + ','.join(_san_sorted(want))],
            check=True, capture_output=True, timeout=60)
        os.chmod(tmp_key, 0o600)
        os.replace(tmp_key, key)
        os.replace(tmp_crt, crt)
    except (OSError, subprocess.SubprocessError):
        for p in (tmp_crt, tmp_key):
            if os.path.exists(p):
                os.remove(p)
        if exists:
            print('cugo_web: 証明書の作り直しに失敗。古い証明書を使います', file=sys.stderr)
            return crt, key
        raise
    return crt, key


def ssl_context(cert_dir: str = CERT_DIR) -> Optional[ssl.SSLContext]:
    crt, key = ensure_cert(cert_dir)
    ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ctx.load_cert_chain(crt, key)
    return ctx
