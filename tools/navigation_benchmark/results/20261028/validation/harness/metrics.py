"""Conservative, dependency-free analysis of stopped navigation runs."""
from math import hypot, isfinite

DEFAULTS = dict(stop_speed=0.05, stop_min_duration=1.0, long_stop_duration=5.0,
                start_grace=2.0, goal_radius=0.35, merge_gap=0.3)


def analyze_run(run, config=None, scenario=None):
    """Analyze contiguous stopped samples in one run.

    Times are simulation seconds and timestamps must be strictly increasing.
    Grace starts at the first active sample. Missing active, goal distance, or
    motion breaks an episode. merge_gap limits the maximum sampling interval;
    short periods of movement are not merged. Safety evidence coverage is the
    fraction of observed stop duration whose preceding sample has a nonempty
    safety_reason. At least 80% is required for exclusion. Scenario regions
    are rectangles with name and xmin/xmax/ymin/ymax; optional
    ramp_pitch_threshold identifies ramps by combined roll/pitch (radians).
    """
    cfg = dict(DEFAULTS)
    cfg.update(config or {})
    if cfg['merge_gap'] <= 0 or cfg['stop_min_duration'] < 0:
        raise ValueError('merge_gap must be positive and duration nonnegative')
    samples = run.get('samples', [])
    regions = (scenario or {}).get('regions', [])
    if isinstance(regions, dict):
        regions = [dict(bounds, name=name) for name, bounds in regions.items()]
    coverage = {key: 0 for key in ('missing_time', 'missing_active', 'missing_goal_distance',
                                    'missing_speed', 'missing_region', 'missing_safety_reason',
                                    'missing_command_speed')}
    episodes, current = [], None
    previous = None
    active_start = None

    def finish():
        nonlocal current
        if current is not None and current['duration'] >= cfg['stop_min_duration']:
            e = current
            e['long_stop'] = e['duration'] >= cfg['long_stop_duration']
            e['safety_coverage'] = e.pop('_safety_duration') / e['duration'] if e['duration'] else 0.0
            e.pop('_last_safety')
            e['safety_status'] = 'excluded_safety' if e['safety_coverage'] >= 0.8 else 'unknown'
            e['unexpected_candidate'] = not (e['excluded_start'] or e['excluded_goal'] or e['safety_status'] == 'excluded_safety')
            e['regions'] = sorted(e['regions'])
            e['safety_reasons'] = sorted(e['safety_reasons'])
            episodes.append(e)
        current = None

    for s in samples:
        t = s.get('t')
        if not _number(t):
            coverage['missing_time'] += 1
            finish(); previous = None
            continue
        if previous is not None and t <= previous['t']:
            raise ValueError('sample timestamps must be strictly increasing')
        dt = t - previous['t'] if previous is not None else None
        if dt is not None and dt > cfg['merge_gap']:
            finish()
        active = s.get('active')
        if active is True and active_start is None:
            active_start = t
        goal = s.get('goal_distance')
        if not isinstance(active, bool): coverage['missing_active'] += 1
        if not _number(goal): coverage['missing_goal_distance'] += 1
        reason = s.get('safety_reason')
        reason = reason.strip() if isinstance(reason, str) else ''
        if not reason: coverage['missing_safety_reason'] += 1
        sample_regions = set()
        if s.get('region'):
            sample_regions.add(str(s['region']).lower())
        if _number(s.get('x')) and _number(s.get('y')):
            for r in regions:
                if all(_number(r.get(k)) for k in ('xmin', 'xmax', 'ymin', 'ymax')) and r['xmin'] <= s['x'] <= r['xmax'] and r['ymin'] <= s['y'] <= r['ymax'] and r.get('name'):
                    sample_regions.add(str(r['name']).lower())
        threshold = (scenario or {}).get('ramp_pitch_threshold')
        pitch, roll = s.get('pitch'), s.get('roll')
        if _number(threshold) and (_number(pitch) or _number(roll)) and hypot(pitch if _number(pitch) else 0, roll if _number(roll) else 0) >= threshold:
            sample_regions.add('ramp')
        if not sample_regions: coverage['missing_region'] += 1
        speed = None
        if _number(s.get('vx')) and _number(s.get('vy')):
            speed = hypot(s['vx'], s['vy'])
        elif previous is not None and dt <= cfg['merge_gap'] and all(_number(v) for v in (s.get('x'), s.get('y'), previous.get('x'), previous.get('y'))):
            speed = hypot(s['x'] - previous['x'], s['y'] - previous['y']) / dt
        if speed is None: coverage['missing_speed'] += 1
        cmd = hypot(s['cmd_vx'], s['cmd_vy']) if _number(s.get('cmd_vx')) and _number(s.get('cmd_vy')) else None
        if cmd is None: coverage['missing_command_speed'] += 1
        start_excluded = active_start is not None and t - active_start < cfg['start_grace']
        goal_excluded = _number(goal) and goal <= cfg['goal_radius']
        # Split when exclusion category changes so a long stop spanning grace is
        # measured independently on either side of the boundary.
        eligible = active is True and _number(goal) and speed is not None
        if eligible and speed <= cfg['stop_speed']:
            if current is not None and (current['excluded_start'] != start_excluded or current['excluded_goal'] != goal_excluded):
                finish()
            position = {k: s.get(k) if _number(s.get(k)) else None for k in ('x', 'y', 'z')}
            if current is None:
                current = dict(start_time=t, end_time=t, duration=0.0, start_position=position,
                               end_position=position, max_cmd_speed=cmd, excluded_start=start_excluded,
                               excluded_goal=goal_excluded, regions=set(), safety_reasons=set(),
                               _safety_duration=0.0, _last_safety=bool(reason))
            else:
                if current['_last_safety']:
                    current['_safety_duration'] += t - current['end_time']
                current['end_time'] = t
                current['duration'] = t - current['start_time']
                current['end_position'] = position
                if cmd is not None:
                    current['max_cmd_speed'] = max(current['max_cmd_speed'], cmd) if current['max_cmd_speed'] is not None else cmd
                current['_last_safety'] = bool(reason)
            current['regions'].update(sample_regions)
            if reason: current['safety_reasons'].add(reason)
        else:
            finish()
        previous = s
    finish()
    counted = [e for e in episodes if not e['excluded_start'] and not e['excluded_goal']]
    return dict(total_episodes=len(counted), long_stop_count=sum(e['long_stop'] for e in counted),
                ramp_count=sum(any('ramp' in r for r in e['regions']) for e in counted),
                tunnel_count=sum(any('tunnel' in r for r in e['regions']) for e in counted),
                excluded_start_count=sum(e['excluded_start'] for e in episodes),
                excluded_goal_count=sum(e['excluded_goal'] for e in episodes),
                excluded_safety_count=sum(e['safety_status'] == 'excluded_safety' for e in counted),
                unexpected_candidate_count=sum(e['unexpected_candidate'] for e in episodes),
                episodes=episodes, coverage=dict(sample_count=len(samples), **coverage))


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and isfinite(value)
