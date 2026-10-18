"""TCP pivot calibration: synthetic poses only; no robot/ROS initialization."""
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

PKG = Path(__file__).resolve().parents[1] / 'src/handeye_calibration_ros'
sys.path.insert(0, str(PKG))
from handeye_calibration_ros import tcp_calibration as tcp


def poses(count=15, noise=0.):
    rng = np.random.default_rng(613)
    offset = np.array([.015, -.008, .132])
    pivot = np.array([.42, -.1, .22])
    transforms = []
    for _ in range(count):
        t = np.eye(4)
        t[:3, :3] = Rotation.random(random_state=rng).as_matrix()
        t[:3, 3] = pivot - t[:3, :3] @ offset + rng.normal(0, noise, 3)
        transforms.append(t)
    return transforms, offset, pivot


def test_known_pivot_and_preserved_orientation():
    transforms, offset, pivot = poses()
    result = tcp.solve_pivot(transforms, [.2, -.1, .3])
    assert result['quality_status'] == 'passed'
    np.testing.assert_allclose(result['tcp_offset'][:3], offset, atol=1e-10)
    np.testing.assert_allclose(result['pivot_base_m'], pivot, atol=1e-10)
    assert result['tcp_offset'][3:] == [.2, -.1, .3]
    assert result['orientation_calibrated'] is False


def test_nonzero_existing_tcp_is_fully_removed():
    flange = poses()[0][0]
    current = [.03, -.02, .06, .2, -.3, .4]
    reported = flange @ tcp.offset_transform(current)
    q = Rotation.from_matrix(reported[:3, :3]).as_quat()
    restored = tcp.tcp_to_flange(reported[:3, 3], q, current)
    np.testing.assert_allclose(restored, flange, atol=1e-12)
    np.testing.assert_allclose(tcp.tcp_to_flange(reported[:3, 3], -q, current), flange, atol=1e-12)


@pytest.mark.parametrize('transforms', [
    [np.eye(4)] * 8,
    [np.block([[Rotation.from_euler('z', a).as_matrix(), np.zeros((3, 1))],
               [np.zeros((1, 3)), np.ones((1, 1))]]) for a in np.linspace(-1, 1, 8)],
])
def test_degenerate_rotations_rejected(transforms):
    result = tcp.solve_pivot(transforms)
    assert result['quality_status'] == 'failed'
    assert result['tcp_offset'] is None
    assert result['rank'] < 6
    json.dumps(result, allow_nan=False)


def test_noisy_contacts_pass_within_project_threshold():
    transforms, offset, _ = poses(noise=.0001)
    result = tcp.solve_pivot(transforms)
    assert result['quality_status'] == 'passed'
    np.testing.assert_allclose(result['tcp_offset'][:3], offset, atol=.0003)


def test_bad_contact_fails_without_silently_deleting_sample():
    transforms, _, _ = poses()
    transforms[0][:3, 3] += [.03, -.02, .04]
    result = tcp.solve_pivot(transforms)
    assert result['quality_status'] == 'failed'
    assert len(result['residuals_mm']) == 15


@pytest.mark.parametrize('bad', [float('nan'), float('inf')])
def test_nonfinite_input_is_rejected(bad):
    transforms = poses()[0]
    transforms[0][0, 3] = bad
    with pytest.raises(ValueError):
        tcp.solve_pivot(transforms)


def test_zero_quaternion_rejected():
    with pytest.raises(ValueError):
        tcp.tcp_to_flange([0, 0, 0], [0, 0, 0, 0], [0] * 6)


def test_too_few_samples_are_not_promotable():
    result = tcp.solve_pivot(poses(count=4)[0])
    assert result['quality_status'] == 'failed'
    assert result['tcp_offset'] is None


def session_payload():
    current = [.02, .01, .08, .1, -.2, .3]
    transforms, _, _ = poses()
    samples = []
    for t in transforms:
        reported = t @ tcp.offset_transform(current)
        samples.append({'position_m': reported[:3, 3].tolist(),
                        'quaternion_xyzw': Rotation.from_matrix(reported[:3, :3]).as_quat().tolist()})
    return {'kind': 'tcp_pivot_session', 'schema_version': 1, 'units': 'm',
            'quaternion_order': 'xyzw', 'current_tcp_offset': current, 'samples': samples}


def test_offline_recomputes_raw_tcp_and_keeps_rpy(tmp_path):
    source, out = tmp_path / 'frames.json', tmp_path / 'candidate.json'
    source.write_text(json.dumps(session_payload()))
    p = subprocess.run([sys.executable, tcp.__file__, '--input', str(source), '--output', str(out)],
                       capture_output=True, text=True, env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
    assert p.returncode == 0, p.stderr
    result = json.loads(out.read_text())
    assert result['quality_status'] == 'passed'
    np.testing.assert_allclose(result['tcp_offset'], [.015, -.008, .132, .1, -.2, .3], atol=1e-10)
    assert result['applied'] is False


def test_offline_rejects_wrong_units(tmp_path):
    payload = session_payload(); payload['units'] = 'mm'
    with pytest.raises(ValueError):
        tcp.solve_session(payload)


def rows():
    return [dict(position_m=[.3, 0, .2], quaternion_xyzw=[0, 0, 0, 1],
                 received_monotonic=10. + n * .02, frame_id='', sequence=n)
            for n in range(15)]


def test_stable_pose_uses_new_samples_and_reports_legacy_frame():
    result = tcp.stable_sample(rows(), now=10.3, after=9.9)
    assert result['assumed_base_frame'] is True
    assert result['window_samples'] == 15


@pytest.mark.parametrize('scenario', ['stale', 'old', 'moving', 'wrong_frame'])
def test_capture_rejects_stale_old_moving_or_wrong_frame(scenario):
    values = rows(); now, after = 10.3, 9.9
    if scenario == 'stale': now = 12.
    if scenario == 'old': after = 11.
    if scenario == 'moving': values[-1]['position_m'][0] += .01
    if scenario == 'wrong_frame': values[-1]['frame_id'] = 'world'
    with pytest.raises(ValueError):
        tcp.stable_sample(values, now=now, after=after)


def test_tcp_shared_launcher_command_is_explicit_and_read_only(capsys):
    import runtime_launcher as launcher
    command = launcher.build_command('calibrate-tcp')
    assert command[:4] == ['ros2', 'run', 'handeye_calibration_ros', 'tcp_calibration']
    assert 'result_save_path:=' in ' '.join(command)
    assert launcher.main(['calibrate-tcp', '--dry-run']) == 0
    assert 'tcp_calibration' in capsys.readouterr().out


def test_candidate_keeps_immutable_input_when_working_samples_change(tmp_path):
    payload = session_payload()
    working, target = tmp_path / 'frames.json', tmp_path / 'candidate.json'
    tcp.save_json(working, payload)
    result = tcp.save_solution(payload, working, target)
    source = Path(result['session_file'])
    before = source.read_bytes()
    payload['samples'].pop()
    tcp.save_json(working, payload)
    assert source.read_bytes() == before
    assert len(json.loads(before)['samples']) == result['sample_count'] == 15
    assert len(json.loads(working.read_text())['samples']) == 14
    assert result['working_session_file'] == str(working)


def test_launcher_tcp_offline_does_not_inject_ros_parameters():
    import runtime_launcher as launcher
    command = launcher.build_command('calibrate-tcp', ros_args=['--input', '/tmp/frames.json'])
    assert command[0] == '/usr/bin/python3'
    assert '--ros-args' not in command
    assert '--input' in command


def test_input_snapshot_preserves_nondefault_solver_gates(tmp_path):
    result = tcp.save_solution(session_payload(), tmp_path / 'frames.json',
                               tmp_path / 'candidate.json', max_rms_mm=.5, max_condition=50.)
    saved = json.loads(Path(result['session_file']).read_text())
    recomputed = tcp.solve_session(saved)
    assert recomputed['thresholds'] == result['thresholds']
    assert recomputed['quality_status'] == result['quality_status']
    np.testing.assert_allclose(recomputed['tcp_offset'], result['tcp_offset'])
