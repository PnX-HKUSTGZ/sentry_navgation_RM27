"""Sensor-only calibration and immutable whole-turn scan roles.

Command stamps locate the experiment window; measured control yaw/XYZ never
enter this module. A full turn is a gyro-observed heading crossing, not an
integral of angular-speed magnitude. No moving acceleration is normalized.
"""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


CALIBRATION_RULES = {
    'duration_sec': 2., 'minimum_samples': 40, 'max_imu_gap_sec': .035,
    'max_gyro_norm_rad_sec': .08, 'max_vector_gyro_std_rad_sec': .015,
    'max_vector_accel_std_m_sec2': .20, 'gravity_magnitude_m_sec2': 9.81,
    'gravity_norm_tolerance_m_sec2': .8,
    'window': 'earliest raw scan with >=2s actual IMU support; native window '
              '[scan-2.0-.035,scan], no stationary-window search',
}


def check_imu(times, gyro, acceleration):
    times = np.asarray(times, dtype=np.int64)
    gyro, acceleration = np.asarray(gyro, float), np.asarray(acceleration, float)
    if (len(times) < 2 or gyro.shape != (len(times), 3)
            or acceleration.shape != gyro.shape or not np.isfinite(gyro).all()
            or not np.isfinite(acceleration).all() or np.any(np.diff(times) <= 0)
            or np.max(np.diff(times)) > 35_000_000):
        raise ValueError('invalid/noncontinuous raw IMU stream')


def calibrate(times, gyro, acceleration, scan_ns):
    """Match the frozen native initial-calibration sample window exactly."""
    check_imu(times, gyro, acceleration)
    times = np.asarray(times, np.int64)
    chosen = (times <= scan_ns) & (times >= scan_ns - 2_035_000_000)
    t, w, a = times[chosen], np.asarray(gyro)[chosen], np.asarray(acceleration)[chosen]
    if not len(t):
        raise ValueError('no initial calibration support')
    bias, gravity = w.mean(0), a.mean(0)
    std_w = float(np.sqrt(np.mean(np.sum((w - bias) ** 2, axis=1))))
    std_a = float(np.sqrt(np.mean(np.sum((a - gravity) ** 2, axis=1))))
    checks = {
        'duration': int(t[-1] - t[0]) >= 2_000_000_000,
        'samples': len(t) >= 40,
        'gyro_norm': float(np.linalg.norm(w, axis=1).max()) <= .08,
        'gyro_std': std_w <= .015, 'acceleration_std': std_a <= .20,
        'gravity_norm': abs(float(np.linalg.norm(gravity)) - 9.81) <= .8,
        'scan_bracket': int(times[0]) <= scan_ns <= int(times[-1]),
    }
    # These are the only accelerometer samples used to define orientation.
    gravity_norm = float(np.linalg.norm(gravity))
    # Retain a finite failed selfcheck report even for a zero acceleration mean.
    v = gravity / gravity_norm if gravity_norm > 1e-12 else np.array([0., 0., 1.])
    cross = np.cross(v, [0., 0., 1.])
    dot = float(v[2])
    if dot < -1 + 1e-10:
        raise ValueError('antipodal gravity initialization is unsupported')
    k = np.array([[0., -cross[2], cross[1]], [cross[2], 0., -cross[0]],
                  [-cross[1], cross[0], 0.]])
    alignment = np.eye(3) + k + k @ k / (1 + dot)
    result = {'rule': CALIBRATION_RULES, 'scan_stamp_ns': int(scan_ns),
              'first_sample_ns': int(t[0]), 'last_sample_ns': int(t[-1]),
              'samples': len(t), 'gyro_bias_rad_sec': bias.tolist(),
              'mean_acceleration_m_sec2': gravity.tolist(),
              'gyro_vector_std_rad_sec': std_w, 'acceleration_vector_std_m_sec2': std_a,
              'gravity_alignment': alignment.tolist(),
              'checks': {key: bool(value) for key, value in checks.items()},
              'stationary_selfcheck_passed': bool(all(checks.values()))}
    return result


def integrate(times, gyro, calibration):
    """Actual-timestamp midpoint SO3; no LIDAR feedback or GT initialization."""
    if not calibration['stationary_selfcheck_passed']:
        raise ValueError('initial calibration did not pass')
    times = np.asarray(times, np.int64)
    first = int(calibration['scan_stamp_ns'])
    # Include the actual scan endpoint, which need not coincide with an IMU tick.
    stamps = np.unique(np.r_[first, times[times > first]])
    if first < times[0] or first > times[-1]:
        raise ValueError('calibration endpoint lacks an IMU bracket')
    relative = (times - first) * 1e-9
    query = (stamps - first) * 1e-9
    w = np.column_stack([np.interp(query, relative, np.asarray(gyro)[:, k])
                         for k in range(3)]) - calibration['gyro_bias_rad_sec']
    delta = .5 * (w[:-1] + w[1:]) * np.diff(query)[:, None]
    current = Rotation.from_matrix(calibration['gravity_alignment'])
    quaternions = [current.as_quat()]
    for increment in Rotation.from_rotvec(delta):
        current = current * increment
        quaternions.append(current.as_quat())
    return stamps, np.asarray(quaternions)


def crossing(stamps, phase, target):
    indices = np.flatnonzero((phase[:-1] < target) & (phase[1:] >= target))
    if not len(indices):
        raise ValueError('actual gyro has no complete %.1f-turn crossing' % (target / (2*np.pi)))
    i = int(indices[0])
    fraction = (target - phase[i]) / (phase[i + 1] - phase[i])
    # Boundary time is interpolated; raw scan timestamps are never changed.
    return int(stamps[i]) + round(fraction * int(stamps[i + 1] - stamps[i]))


def turn_ledger(station, scan_stamps, gyro_stamps, quaternions, base_from_sensor):
    scan_stamps = np.asarray(scan_stamps, np.int64)
    gyro_stamps = np.asarray(gyro_stamps, np.int64)
    if len(scan_stamps) < 2 or np.any(np.diff(scan_stamps) <= 0):
        raise ValueError('station scans must have unique increasing timestamps')
    required = ['capture_start_ros_ns', 'rotation_command_start_ros_ns',
                'stop_command_ros_ns', 'capture_end_ros_ns']
    events = [station[key] for key in required]
    if any(isinstance(x, bool) or not isinstance(x, int) for x in events):
        raise ValueError('station event timestamps must be integer nanoseconds')
    capture_start, start, stop, capture_end = events
    if not capture_start <= start < stop <= capture_end:
        raise ValueError('station command event order invalid')
    if start < gyro_stamps[0] or stop > gyro_stamps[-1]:
        raise ValueError('station gyro command interval lacks full support')
    # Command direction only labels sign; its magnitude/GT progress is unused.
    speed = float(station['commanded_speed_rad_sec'])
    if not np.isfinite(speed) or speed == 0:
        raise ValueError('rotation command direction unavailable')
    stamps = np.unique(np.r_[start, gyro_stamps[(gyro_stamps > start) &
                                               (gyro_stamps < stop)], stop])
    origin = int(gyro_stamps[0])
    interpolation = Slerp((gyro_stamps - origin) * 1e-9, Rotation.from_quat(quaternions))
    sensor = interpolation((stamps - origin) * 1e-9).as_matrix()
    base = sensor @ np.asarray(base_from_sensor)[:3, :3].T
    forward = base[:, :, 0]
    if np.any(np.linalg.norm(forward[:, :2], axis=1) < 1e-6):
        raise ValueError('base heading is undefined near vertical')
    heading = np.unwrap(np.arctan2(forward[:, 1], forward[:, 0]))
    phase = np.sign(speed) * (heading - heading[0])
    boundaries, failure = [], None
    for target in (2 * np.pi, 4 * np.pi):
        try:
            boundaries.append(crossing(stamps, phase, target))
        except ValueError as error:
            failure = str(error)
            break
    scans = []
    for ordinal, ns in enumerate(scan_stamps):
        ns = int(ns)
        if not capture_start <= ns < capture_end:
            raise ValueError('scan is outside its half-open capture window')
        if ns < start:
            role = 'aux_pre'
        elif len(boundaries) == 0 or ns < boundaries[0]:
            role = 'train' if len(boundaries) >= 1 else 'aux_incomplete_turn0'
        elif len(boundaries) == 1 or ns < boundaries[1]:
            role = 'heldout' if len(boundaries) == 2 else 'aux_incomplete_turn1'
        else:
            role = 'aux_post'
        scans.append({'ordinal': ordinal, 'stamp_ns': ns, 'role': role})
    role_counts = {key: sum(x['role'] == key for x in scans)
                   for key in sorted({x['role'] for x in scans})}
    complete = len(boundaries) == 2 and role_counts.get('train', 0) > 0 and role_counts.get('heldout', 0) > 0
    if complete:
        # Discrete LiDAR endpoints do not themselves need to land at 2pi.
        for lo, hi in [(start, boundaries[0]), (boundaries[0], boundaries[1])]:
            a = np.searchsorted(scan_stamps, lo)
            b = np.searchsorted(scan_stamps, hi)
            if a == 0 or b == len(scan_stamps):
                complete, failure = False, 'LiDAR does not bracket a complete gyro turn'
                break
    return {'complete': bool(complete), 'failure': failure, 'events': dict(zip(required, events)),
            'turn_boundaries_ns': [start] + boundaries,
            'actual_signed_heading_at_stop_rad': float(phase[-1]),
            'reverse_heading_travel_rad': float(-np.minimum(np.diff(phase), 0).sum()),
            'direction_from_command_only': int(np.sign(speed)), 'control_measured_yaw_or_XYZ_used': False,
            'boundary_rule': '[start,first2pi) train; [first2pi,first4pi) heldout; exact boundary goes to later role',
            'role_counts': role_counts, 'scans': scans,
            'local_frontend_dependency': 'all station raw scans; heldout is independent of crossstation fitting only'}
