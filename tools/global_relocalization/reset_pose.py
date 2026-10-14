"""Bounded same-pose Gazebo reset with durable attempt evidence."""
import datetime
import json
import math
from pathlib import Path
import re
import subprocess
import time


class ResetPoseError(RuntimeError):
    def __init__(self, message, attempts):
        super().__init__(message)
        self.attempts = attempts


def _text(value):
    return value.decode(errors='replace') if isinstance(value, bytes) else value or ''


def reset_pose(pose, *, attempts=None, log_path=None):
    """Return legacy (command, stdout); retry only CLI/outer timeout once.

    attempts is populated before raising, including when persisting evidence fails.
    """
    attempts = attempts if attempts is not None else []
    half = pose['yaw'] / 2
    request = (f'name: "rm27_sentry", position: {{x: {pose["x"]}, y: {pose["y"]}, z: {pose["z"]}}}, '
               f'orientation: {{x: 0, y: 0, z: {math.sin(half)}, w: {math.cos(half)}}}')
    command = ['gz', 'service', '-s', '/world/default/set_pose', '--reqtype', 'gz.msgs.Pose',
               '--reptype', 'gz.msgs.Boolean', '--timeout', '10000', '--req', request]
    for number in range(1, 3):
        started = time.monotonic()
        row = {'attempt': number, 'requested_pose': dict(pose), 'command': command,
               'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
               'cli_timeout_sec': 10, 'outer_timeout_sec': 15,
               'returncode': None, 'stdout': '', 'stderr': '', 'success': False}
        try:
            result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, timeout=15)
            row.update(returncode=result.returncode, stdout=result.stdout, stderr=result.stderr)
            combined = result.stdout + '\n' + result.stderr
            if re.search(r'\bdata\s*:\s*false\b', combined, re.I):
                row['classification'] = 'boolean_false'
            elif re.search(r'timed?\s*out|timeout', combined, re.I):
                row['classification'] = 'cli_timeout'
            elif result.returncode == 0 and re.search(r'\bdata\s*:\s*true\b', result.stdout, re.I):
                row.update(classification='success', success=True)
            else:
                row['classification'] = 'service_error'
        except subprocess.TimeoutExpired as error:
            row.update(classification='outer_timeout', stdout=_text(error.stdout), stderr=_text(error.stderr))
        except Exception as error:
            row.update(classification='execution_error', stderr=str(error))
        row.update(ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                   wall_sec=time.monotonic() - started)
        attempts.append(row)
        if log_path is not None:
            try:
                Path(log_path).write_text(json.dumps(attempts, indent=2) + '\n')
            except OSError as error:
                row['log_error'] = str(error)
                detail = row['stdout'] + row['stderr']
                raise ResetPoseError(f'Gazebo pose reset {row["classification"]}: {detail}; attempt log failed: {error}', attempts) from error
        if row['success']:
            return command, row['stdout']
        if row['classification'] not in ('cli_timeout', 'outer_timeout') or number == 2:
            details = '; '.join(f'attempt {r["attempt"]} {r["classification"]}: {r["stdout"]}{r["stderr"]}' for r in attempts)
            raise ResetPoseError('Gazebo pose reset failed: ' + details, attempts)


def reset_summary(rows):
    attempts = [row.get('reset_attempts', []) for row in rows]
    attempted = [items for items in attempts if items]
    return {'planned_trial_count': len(rows), 'attempted_trial_count': len(attempted),
            'first_attempt_success_count': sum(items[0]['success'] for items in attempted),
            'first_attempt_success_rate': (sum(items[0]['success'] for items in attempted) / len(attempted)) if attempted else None,
            'recovered_on_retry_count': sum(len(items) > 1 and items[-1]['success'] for items in attempted),
            'failed_after_reset_count': sum(not items[-1]['success'] for items in attempted),
            'total_attempt_count': sum(len(items) for items in attempted)}
