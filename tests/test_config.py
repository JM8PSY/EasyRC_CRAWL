import pytest

from cugo_rc.config import default_config, load_config


def test_default_when_none():
    c = load_config(None)
    assert (c.motor.left.pwm_pin, c.motor.left.dir_pin, c.motor.left.invert) == (12, 5, False)
    assert (c.motor.right.pwm_pin, c.motor.right.dir_pin, c.motor.right.invert) == (13, 6, True)
    assert c.motor.pwm_freq == 20000 and c.watchdog.timeout_ms == 300
    assert c.server.port == 8000


def test_repo_config_yaml_matches_defaults():
    assert load_config("config.yaml") == default_config()


def test_partial_yaml_keeps_defaults(tmp_path):
    p = tmp_path / "c.yaml"
    p.write_text("camera:\n  width: 320\nmotor:\n  right:\n    invert: false\n")
    c = load_config(str(p))
    assert c.camera.width == 320 and c.camera.height == 480
    assert c.motor.right.invert is False and c.motor.right.pwm_pin == 13


def test_unknown_key_raises(tmp_path):
    p = tmp_path / "c.yaml"
    p.write_text("motor:\n  max_sped: 0.5\n")
    with pytest.raises(ValueError):
        load_config(str(p))
