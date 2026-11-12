import math

from cugo_web import calibration


def test_wrap_angle():
    assert math.isclose(calibration.wrap_angle(math.radians(270)), math.radians(-90))
    assert math.isclose(calibration.wrap_angle(math.radians(-190)), math.radians(170))
    assert math.isclose(calibration.wrap_angle(0.5), 0.5)


def test_save_load_roundtrip(tmp_path):
    path = str(tmp_path / 'sub' / 'calibration.yaml')
    calibration.save({'lidar_yaw': -1.57079632}, path)
    assert calibration.load(path) == {'lidar_yaw': -1.5708}


def test_load_ignores_broken_or_non_numeric(tmp_path):
    p = tmp_path / 'c.yaml'
    p.write_text('lidar_yaw: abc\nother: true\nok: 2\n')
    assert calibration.load(str(p)) == {'ok': 2.0}
    p.write_text(': : :\n[')
    assert calibration.load(str(p)) == {}
    assert calibration.load(str(tmp_path / 'missing.yaml')) == {}


def test_lidar_yaw_from_urdf():
    urdf = '''<robot>
      <joint name="base_link_to_camera" type="fixed"><origin xyz="0 0 0" rpy="0 0 9"/></joint>
      <joint name="base_link_to_laser_frame" type="fixed">
        <parent link="base_link"/><child link="laser_frame"/>
        <origin xyz="0 0 0.1" rpy="0 0 -1.5708"/>
      </joint></robot>'''
    assert calibration.lidar_yaw_from_urdf(urdf) == -1.5708
    assert calibration.lidar_yaw_from_urdf('<robot/>') is None
