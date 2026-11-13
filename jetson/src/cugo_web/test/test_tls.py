import os

from cugo_web import tls


def _fail_only_tailscale(run):
    """tailscale コマンドだけ「入っていない」ことにする subprocess.run の代わり。"""
    def fake(args, *a, **kw):
        if args and args[0] == 'tailscale':
            raise FileNotFoundError('tailscale')
        return run(args, *a, **kw)
    return fake


def test_cert_contains_wanted_names(tmp_path, monkeypatch):
    monkeypatch.setattr(tls, '_local_ips', lambda: ['10.0.0.5', '100.64.1.2'])
    monkeypatch.setattr(tls, '_tailscale_dns', lambda: 'cugo.tail1234.ts.net')
    crt, _ = tls.ensure_cert(str(tmp_path))
    names = tls.cert_names(crt)
    assert {'IP:100.64.1.2', 'IP:10.0.0.5', 'DNS:cugo.tail1234.ts.net'} <= names


def test_cert_regenerated_once_when_tailscale_ip_appears(tmp_path, monkeypatch):
    # Review Focus 4: Tailscale を入れた後の初回だけ作り直す
    monkeypatch.setattr(tls, '_local_ips', lambda: ['10.0.0.5'])
    monkeypatch.setattr(tls, '_tailscale_dns', lambda: None)
    crt, _ = tls.ensure_cert(str(tmp_path))
    m0 = os.path.getmtime(crt)
    monkeypatch.setattr(tls, '_local_ips', lambda: ['10.0.0.5', '100.64.1.2'])
    os.utime(crt, (m0 - 10, m0 - 10))
    tls.ensure_cert(str(tmp_path))
    m1 = os.path.getmtime(crt)
    assert m1 > m0 - 10 and 'IP:100.64.1.2' in tls.cert_names(crt)
    tls.ensure_cert(str(tmp_path))
    assert os.path.getmtime(crt) == m1          # 2 回目は作り直さない


def test_no_tailscale_still_works(tmp_path, monkeypatch):
    # Review Focus 5: Tailscale が無くても証明書は作れる
    monkeypatch.setattr(tls, '_local_ips', lambda: ['10.0.0.5'])
    monkeypatch.setattr(tls.subprocess, 'run', _fail_only_tailscale(tls.subprocess.run))
    crt, key = tls.ensure_cert(str(tmp_path))
    assert os.path.exists(crt) and tls._tailscale_dns() is None


def test_cert_names_unreadable_is_empty(tmp_path):
    p = tmp_path / 'x.crt'
    p.write_text('garbage')
    assert tls.cert_names(str(p)) == set()


def test_old_ip_kept_when_network_changes(tmp_path, monkeypatch):
    """研究室の IP が変わっても古い IP を残す → 元のネットワークに戻っても作り直さない。"""
    monkeypatch.setattr(tls, '_tailscale_dns', lambda: None)
    monkeypatch.setattr(tls, '_local_ips', lambda: ['10.0.0.5'])
    crt, _ = tls.ensure_cert(str(tmp_path))
    monkeypatch.setattr(tls, '_local_ips', lambda: ['10.0.0.9'])
    tls.ensure_cert(str(tmp_path))
    assert {'IP:10.0.0.5', 'IP:10.0.0.9'} <= tls.cert_names(crt)
    monkeypatch.setattr(tls, '_local_ips', lambda: ['10.0.0.5'])
    m = os.path.getmtime(crt)
    os.utime(crt, (m - 10, m - 10))
    tls.ensure_cert(str(tmp_path))
    assert os.path.getmtime(crt) == m - 10
