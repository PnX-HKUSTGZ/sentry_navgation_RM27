"""Whole-turn cache, immutable potential domains, and every station pair."""
from pathlib import Path
from dataclasses import asdict
from collections import OrderedDict
import itertools
import time

import numpy as np
from scipy.spatial.transform import Rotation

from pipeline_common import HERE, Inputs, new_directory, read_json, write_json, rigid, sha256, record_code
from pipeline_model import Rules, invert, transform, compact_stats, freeze_domain, supported_geometry, fit
from export_station_prior import read_binary_xyz
from refine_sensor_prior import voxel, normals


def frozen_rules(n, support_mode=False):
    result = {
        'schema_version': 1, 'base_rules': asdict(Rules()), 'station_count': n,
        'expected_pairs': n * (n - 1) // 2,
        'split': 'first actual gyro full turn train; second actual gyro full turn heldout; all tails auxiliary',
        'local_frontend_dependency': 'all post-settle geometry scans used local poses; '
                                     'pre-settle prefix retained but excluded from stationary mapping/fusion; '
                                     'only crossstation fitting excludes second-turn heldout',
        'domain': 'seed once, <=.30m to fixed good-normal target TRAIN tree; not a visibility proof',
        'heldout': 'all frozen domain points, fixed perframe denominator; true>.30 cap=.30; never reselect',
        'patch_support': 'unique .08 target cells, not independent native patches',
        'numerical_model': 'reviewed v3 train/evaluate functions; no parameter sweep',
        'graph': {
            'fixed_gauge': 'first ordered visit original sensor/LIO seed',
            'required_geometry_rank': 6 * (n - 1), 'rank_relative_tolerance': 1e-8,
            'scene_m_per_rad': 1., 'no_odometry_prior_or_information_floor': True,
            'redundancy': 'every accepted edge leave-one-out connected/fullrank; alternate different-pair cycle .03m/1deg',
            'solver': 'dense exact TRF; unchanged Huber2/default x_scale/tolerances/600 evaluations',
            'post_guard': 'actual adopted transform and inverse fixed-heldout <=seed+.002 AND<=pairfit+.002; '
                          'edge .03m/1deg; node .5m/10deg; finiteSE3; genuine positive termination before cap',
        }, 'GT_or_query_read': False, 'map_export': False,
    }
    if support_mode:
        from pipeline_support import SUPPORT_RULES
        result['pose_support'] = SUPPORT_RULES
    return result


class StationCache:
    """Memory-map raw heldout arrays and retain at most two target NN trees."""
    def __init__(self, directory, ids):
        self.directory, self.ids = Path(directory), ids
        self.loaded = OrderedDict()

    def __getitem__(self, index):
        if index not in self.loaded:
            folder = self.directory / self.ids[index]
            arrays = {key: np.load(folder / (key + '.npy'), mmap_mode='r') for key in (
                'train', 'normal', 'normal_good', 'held', 'held_frame', 'held_raw_point',
                'held_ordinals', 'full')}
            self.loaded[index] = {'id': self.ids[index], 'train': arrays['train'],
                'held': arrays['held'], 'hf': arrays['held_frame'], 'rawids': arrays['held_raw_point'],
                'ordinals': arrays['held_ordinals'].tolist(), 'full': arrays['full'],
                'geom': supported_geometry(arrays['train'], arrays['normal'], arrays['normal_good'])}
            if len(self.loaded) > 2:
                self.loaded.popitem(last=False)
        self.loaded.move_to_end(index)
        return self.loaded[index]


def build_domains(seed_path, output, allow_support=False):
    start = time.monotonic()
    out = new_directory(output)
    inputs = Inputs()
    record_code(inputs)
    seed_path = inputs.artifact(seed_path)
    seed = read_json(seed_path)
    support_mode = seed.get('pose_support_mode',False)
    if support_mode and not allow_support:
        raise ValueError('supported seed requires explicit --support domain consumption')
    if (seed.get('sensor_only') is not True or seed.get('GT_post_evaluation_files_read') is not False
            or seed.get('query_results_read') is not False):
        raise ValueError('pure sensor seed required')
    plan_path = inputs.add(seed['source_input'], seed['sources_sha256'][str(Path(seed['source_input']).resolve())])
    plan = read_json(plan_path)
    if not plan.get('all_gyro_turns_complete') or not plan.get('all_settle_sensor_selfchecks_passed'):
        raise ValueError('all actual gyro full turns required')
    ids = [s['id'] for s in seed['stations']]
    if len(ids) < 2 or len(set(ids)) != len(ids) or ids != [s['id'] for s in plan['stations']]:
        raise ValueError('seed/plan ordered station inventory differs')
    rules = Rules()
    write_json(out / 'frozen_rules.json', frozen_rules(len(ids),support_mode))
    write_json(out / 'sensor_seed.json', seed)
    inputs.add(HERE / 'pipeline_model.py')
    inputs.add(HERE / 'supported_tree.py')
    inputs.add(HERE / 'pipeline_pairs.py')
    inputs.add(HERE.parent / 'refine_sensor_prior.py')
    cache_dir = out / 'stations'
    cache_dir.mkdir()
    array_hashes, inventory = {}, []
    for station, node in zip(plan['stations'], seed['stations']):
        sid = station['id']
        poses = read_json(inputs.add(node['poses_path'], seed['sources_sha256'][str(Path(node['poses_path']).resolve())]))
        if len(poses) != len(station['scans']) or not all(p.get('pose_supported') if support_mode else p['accepted'] for p in poses):
            raise ValueError('all source local frames required')
        if support_mode:
            from pipeline_support import validate_supported_rows
            validate_supported_rows(station,poses,node['support_counts'],rigid(plan['base_from_sensor']))
        train, held, held_frame, held_raw, ordinals, split = [], [], [], [], [], []
        for ordinal, (scan, pose) in enumerate(zip(station['scans'], poses)):
            path = inputs.add(scan['path'], scan['sha256'])
            if abs(round(pose['stamp'] * 1e9) - scan['stamp_ns']) > 1:
                raise ValueError('local/raw scan timestamp mismatch')
            role = scan['role']
            if role not in ('train', 'heldout', 'aux_pre', 'aux_post'):
                raise ValueError('complete immutable two-turn split required')
            split.append({'ordinal': ordinal, 'stamp_ns': scan['stamp_ns'], 'role': role,
                          'raw_pcd': str(path), 'raw_sha256': scan['sha256'],
                          'registration_accepted':bool(pose['accepted']),
                          'pose_supported':bool(pose.get('pose_supported',pose['accepted'])),
                          'pose_source':pose.get('pose_source','registration'),
                          'eligible_for_train':bool(pose['accepted'] and role=='train')})
            if role.startswith('aux_'):
                continue
            if role=='train' and not pose['accepted']:
                # The rejected raw frame remains in the split ledger but never
                # contributes to TRAIN, frontend fusion, or its normal model.
                continue
            raw = read_binary_xyz(path).astype(float)
            ranges = np.linalg.norm(raw, axis=1)
            valid = np.isfinite(raw).all(1) & (ranges >= plan['params']['min_range']) & (ranges <= plan['params']['max_range'])
            raw_ids = np.flatnonzero(valid)
            points = transform(raw[valid], rigid(pose['reference_from_sensor']))
            if role == 'train':
                train.append(points)
            else:
                held.append(points)
                held_frame.append(np.full(len(points), ordinal, np.int32))
                held_raw.append(raw_ids.astype(np.int32))
                ordinals.append(ordinal)
        if not train or not held:
            raise ValueError('both full turns must retain raw scans')
        train = voxel(np.concatenate(train), rules.match_leaf_m)
        held = np.concatenate(held)
        if len(train) == 0:
            raise ValueError('empty range-valid training cloud')
        geometry = normals(train, rules)
        full_path = inputs.add(node['cloud'], seed['sources_sha256'][str(Path(node['cloud']).resolve())])
        arrays = {'train': train, 'normal': geometry[1], 'normal_good': geometry[2],
                  'held': held, 'held_frame': np.concatenate(held_frame),
                  'held_raw_point': np.concatenate(held_raw), 'held_ordinals': np.asarray(ordinals, np.int32),
                  'full': voxel(read_binary_xyz(full_path), rules.match_leaf_m)}
        folder = cache_dir / sid
        folder.mkdir()
        for key, array in arrays.items():
            path = folder / (key + '.npy')
            np.save(path, array)
            array_hashes[str(path)] = sha256(path)
        write_json(folder / 'frame_split.json', split)
        array_hashes[str(folder / 'frame_split.json')] = sha256(folder / 'frame_split.json')
        inventory.append({'id': sid, 'source_scans': len(split),
            'source_capture_scans':station['expected_capture_raw_frames'],
            'excluded_settle_prefix_scans':len(station['excluded_settle_prefix_scans']),
            'train_frames': sum(x['role'] == 'train' for x in split), 'heldout_frames': len(ordinals),
            'train_registration_frames':sum(x['eligible_for_train']for x in split),
            'registration_accepted_frames':sum(x['registration_accepted']for x in split),
            'imu_bridge_frames':sum(x['pose_source']=='imu_bridge'for x in split),
            'pose_supported_frames':sum(x['pose_supported']for x in split),
            'fused_frames':node['support_counts']['fused_frames'] if support_mode else len(split),
            'auxiliary_frames': sum(x['role'].startswith('aux_') for x in split),
            'train_support': compact_stats(train, geometry[1][geometry[2]]),
            'heldout_points': len(held)})
        print('cache', sid, inventory[-1]['train_frames'], 'train /', len(ordinals), 'heldout', flush=True)
    write_json(out / 'station_inventory.json', inventory)
    cache = StationCache(cache_dir, ids)
    initial = np.array([rigid(s['map_from_local_initial']) for s in seed['stations']])
    domain_meta = []
    domain_dir = out / 'domains'
    domain_dir.mkdir()
    for i, j in itertools.combinations(range(len(ids)), 2):
        for src, tgt, name in [(j, i, 'forward'), (i, j, 'reverse')]:
            arrays, meta = freeze_domain(cache[src], cache[tgt], invert(initial[tgt]) @ initial[src])
            path = domain_dir / ('domain_%04d_%04d_%s.npz' % (i, j, name))
            np.savez(path, **arrays)
            meta.update(i=i, j=j, source=src, target=tgt, direction=name,
                        domain_file=str(path.relative_to(out)), domain_sha256=sha256(path))
            domain_meta.append(meta)
    write_json(out / 'frozen_domains.json', domain_meta)
    # No registration has run at this point. Raw point IDs/masks are immutable.
    write_json(out / 'domains_complete_before_any_registration.json', {
        'complete': True, 'expected_directions': len(ids) * (len(ids) - 1), 'directions': len(domain_meta),
        'array_sha256': array_hashes,
        'domain_sha256': {m['domain_file']: m['domain_sha256'] for m in domain_meta},
        'elapsed_sec': time.monotonic() - start})
    inputs.save(out, stage='domains', expected_station_count=len(ids),
                input_seed=str(seed_path), all_domains_frozen_before_registration=True)
    return True


def load_domains(directory, inputs):
    directory = Path(directory).resolve()
    summary = read_json(inputs.artifact(directory / 'domains_complete_before_any_registration.json'))
    seed = read_json(inputs.artifact(directory / 'sensor_seed.json'))
    rules = read_json(inputs.artifact(directory / 'frozen_rules.json'))
    ids = [s['id'] for s in seed['stations']]
    if (not summary['complete'] or summary['directions'] != len(ids) * (len(ids) - 1)
            or rules != frozen_rules(len(ids),seed.get('pose_support_mode',False))):
        raise ValueError('incomplete/altered frozen observation rules')
    for path, digest in summary['array_sha256'].items():
        inputs.add(path, digest)
    metadata = read_json(inputs.artifact(directory / 'frozen_domains.json'))
    if len(metadata) != summary['directions']:
        raise ValueError('domain inventory incomplete')
    domains = {}
    for m in metadata:
        path = directory / m['domain_file']
        expected = summary['domain_sha256'][m['domain_file']]
        if m['domain_sha256'] != expected:
            raise ValueError('inconsistent domain hashes')
        inputs.add(path, expected)
        domains[(m['i'], m['j'], m['direction'])] = (path, m)
    expected_keys = {(i, j, d) for i, j in itertools.combinations(range(len(ids)), 2)
                     for d in ['forward', 'reverse']}
    if set(domains) != expected_keys:
        raise ValueError('not every station pair has both frozen domains')
    return seed, StationCache(directory / 'stations', ids), domains


def domain_value(item):
    path, meta = item
    with np.load(path) as array:
        return {'ti': array['train_indices'], 'hi': array['held_indices'], 'meta': meta}


def fit_pairs(domains_dir, output):
    start = time.monotonic()
    inputs = Inputs()
    record_code(inputs)
    seed, cache, domains = load_domains(domains_dir, inputs)
    out = new_directory(output)
    inputs.add(HERE / 'pipeline_model.py')
    inputs.add(HERE / 'pipeline_pairs.py')
    inputs.add(HERE / 'supported_tree.py')
    inputs.add(HERE.parent / 'refine_sensor_prior.py')
    initial = np.array([rigid(s['map_from_local_initial']) for s in seed['stations']])
    rules, rows, edges = Rules(), [], []
    # These are the same parameterized reviewed functions; cache returns the same arrays.
    for i, j in itertools.combinations(range(len(initial)), 2):
        z = invert(initial[i]) @ initial[j]
        f, fr, fm = fit(cache, rules, j, i, domain_value(domains[(i, j, 'forward')]), z)
        b, br, bm = fit(cache, rules, i, j, domain_value(domains[(i, j, 'reverse')]), invert(z))
        cycle = b @ f
        ct = float(np.linalg.norm(cycle[:3, 3]))
        cr = float(np.rad2deg(Rotation.from_matrix(cycle[:3, :3]).magnitude()))
        accepted = fm['accepted'] and bm['accepted'] and ct <= rules.cycle_translation_m and cr <= rules.cycle_rotation_deg
        rows.append({'i': i, 'j': j, 'source_ids': [seed['stations'][i]['id'], seed['stations'][j]['id']],
                     'accepted': bool(accepted), 'forward': fm, 'reverse': bm,
                     'forward_transform': f.tolist(), 'reverse_transform': b.tolist(),
                     'cycle_translation_m': ct, 'cycle_rotation_deg': cr})
        if accepted:
            edges.append({'i': i, 'j': j, 'measurement': f.tolist(), 'root': fr.tolist()})
        # Keep explicit partial progress if a process is interrupted.
        write_json(out / 'pair_results.json', rows)
        write_json(out / 'validated_edges.json', edges)
        print('pair', i, j, 'accepted', accepted, fm['rejected_gates'], bm['rejected_gates'], flush=True)
    write_json(out / 'summary.json', {'all_pairs_attempted': len(rows) == len(initial) * (len(initial) - 1) // 2,
        'station_count': len(initial), 'pairs_attempted': len(rows), 'accepted_edges': len(edges),
        'domains_directory': str(Path(domains_dir).resolve()), 'graph_optimization_run': False,
        'global_map_exported': False, 'GT_or_query_read': False, 'elapsed_sec': time.monotonic() - start})
    inputs.save(out, stage='pairs', all_pairs_attempted=True)
    return True
