"""Arbitrary-N geometry-only graph audit and guarded dense-exact SE3 solve."""
from collections import deque, Counter
from pathlib import Path
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from pipeline_common import HERE, Inputs, new_directory, read_json, write_json, rigid, record_code
from pipeline_model import Rules, invert, evaluate
from pipeline_pairs import load_domains, domain_value
from refine_sensor_prior import relative_error


class GraphObjective:
    """Same right-rotation/additive-world-translation residual as the reviewed solver."""
    def __init__(self, initial, edges):
        self.initial = np.asarray(initial)
        self.ii = np.array([e['i'] for e in edges], dtype=int)
        self.jj = np.array([e['j'] for e in edges], dtype=int)
        z = np.array([e['measurement'] for e in edges], dtype=float).reshape(-1, 4, 4)
        self.roots = np.array([e['root'] for e in edges], dtype=float).reshape(-1, 6, 6)
        self.rz, self.tz = np.transpose(z[:, :3, :3], (0, 2, 1)), z[:, :3, 3]
        self.dimension = 6 * (len(initial) - 1)

    def unpack(self, x):
        delta = np.zeros((len(self.initial), 6))
        delta[1:] = np.asarray(x).reshape(-1, 6)
        result = self.initial.copy()
        result[:, :3, :3] = self.initial[:, :3, :3] @ Rotation.from_rotvec(delta[:, :3]).as_matrix()
        result[:, :3, 3] += delta[:, 3:]
        return result

    def __call__(self, x):
        poses = self.unpack(x)
        ri = np.transpose(poses[self.ii, :3, :3], (0, 2, 1))
        relative_r = ri @ poses[self.jj, :3, :3]
        relative_t = np.einsum('nij,nj->ni', ri, poses[self.jj, :3, 3] - poses[self.ii, :3, 3])
        rotation = Rotation.from_matrix(self.rz @ relative_r).as_rotvec()
        translation = np.einsum('nij,nj->ni', self.rz, relative_t - self.tz)
        return np.einsum('nij,nj->ni', self.roots, np.c_[rotation, translation]).ravel()


def connected_components(n, edges, exclude=None):
    adjacency = [[] for _ in range(n)]
    for k, e in enumerate(edges):
        if k != exclude:
            adjacency[e['i']].append(e['j'])
            adjacency[e['j']].append(e['i'])
    labels = [-1] * n
    count = 0
    for root in range(n):
        if labels[root] >= 0:
            continue
        pending = [root]
        labels[root] = count
        while pending:
            for other in adjacency[pending.pop()]:
                if labels[other] < 0:
                    labels[other] = count
                    pending.append(other)
        count += 1
    return count, labels, adjacency


def geometry_information(initial, edges):
    """Actual geometry roots; preserve zero directions, no priors/eigenvalue fill."""
    n, parts = len(initial), []
    dimension = 6 * (n - 1)
    for e in edges:
        root, z = np.asarray(e['root']), np.asarray(e['measurement'])
        jac = np.zeros((6, dimension))
        for node in (e['i'], e['j']):
            if node == 0:
                continue
            for axis in range(6):
                a, b = initial.copy(), initial.copy()
                if axis < 3:
                    delta = np.zeros(3)
                    delta[axis] = 1e-6
                    a[node, :3, :3] = initial[node, :3, :3] @ Rotation.from_rotvec(delta).as_matrix()
                    b[node, :3, :3] = initial[node, :3, :3] @ Rotation.from_rotvec(-delta).as_matrix()
                else:
                    a[node, axis - 3, 3] += 1e-6
                    b[node, axis - 3, 3] -= 1e-6
                jac[:, 6 * (node - 1) + axis] = root @ (
                    relative_error(a[e['i']], a[e['j']], z) -
                    relative_error(b[e['i']], b[e['j']], z)) / 2e-6
        parts.append(jac)
    jac = np.concatenate(parts) if parts else np.zeros((0, dimension))
    h = jac.T @ jac
    values, vectors = np.linalg.eigh((h + h.T) * .5)
    tolerance = max(1e-10, float(values[-1]) * 1e-8) if len(values) else 1e-10
    rank = int(np.sum(values > tolerance))
    return {'rank': rank, 'dimension': dimension, 'threshold': tolerance,
            'eigenvalues': values.tolist(),
            'condition_positive': float(values[-1] / values[values > tolerance][0]) if rank else None,
            'scene_m_per_rad': 1., 'no_prior_or_information_fill': True}, jac, parts, vectors


def alternate_path(adjacency, begin, end):
    predecessors, pending = {begin: None}, deque([begin])
    while pending:
        node = pending.popleft()
        if node == end:
            route = []
            while node is not None:
                route.append(node)
                node = predecessors[node]
            return route[::-1]
        for other in sorted(adjacency[node]):
            if other not in predecessors:
                predecessors[other] = node
                pending.append(other)
    return None


def audit_graph(initial, edges):
    n = len(initial)
    information, jac, parts, vectors = geometry_information(initial, edges)
    components, labels, _ = connected_components(n, edges)
    lookup = {(e['i'], e['j']): np.asarray(e['measurement']) for e in edges}
    h, required = jac.T @ jac, 6 * (n - 1)
    leave_one_out = []
    for k, edge in enumerate(edges):
        nc, _, adjacency = connected_components(n, edges, exclude=k)
        hh = h - parts[k].T @ parts[k]
        values = np.linalg.eigvalsh((hh + hh.T) * .5)
        rank = int(np.sum(values > information['threshold']))
        row = {'i': edge['i'], 'j': edge['j'], 'components_without_edge': nc,
               'geometry_rank_without_edge': rank, 'required_rank': required,
               'redundant_connectivity_and_rank': nc == 1 and rank == required}
        route = alternate_path(adjacency, edge['i'], edge['j'])
        if route is not None:
            composed = np.eye(4)
            for u, v in zip(route, route[1:]):
                composed = composed @ (lookup[(u, v)] if (u, v) in lookup else invert(lookup[(v, u)]))
            closure = invert(np.asarray(edge['measurement'])) @ composed
            td = float(np.linalg.norm(closure[:3, 3]))
            rd = float(np.rad2deg(Rotation.from_matrix(closure[:3, :3]).magnitude()))
            row.update(alternate_station_path=route, alternate_cycle_translation_m=td,
                       alternate_cycle_rotation_deg=rd, alternate_cycle_passes=td <= .03 and rd <= 1.)
        else:
            row['alternate_cycle_passes'] = False
        leave_one_out.append(row)
    weak = [{'eigenvalue': information['eigenvalues'][k], 'per_free_node': [
        {'node': node, 'body_rotation_xyz': vectors[6*(node-1):6*(node-1)+3, k].tolist(),
         'world_translation_xyz': vectors[6*(node-1)+3:6*node, k].tolist()}
        for node in range(1, n)]} for k in range(min(6, required))]
    checks = {'connected': components == 1, 'full_geometry_rank': information['rank'] == required,
              'all_edges_redundant': bool(edges) and all(r['redundant_connectivity_and_rank'] for r in leave_one_out),
              'all_alternate_cycles_consistent': bool(edges) and all(r['alternate_cycle_passes'] for r in leave_one_out)}
    return {'station_count': n, 'components': components, 'component_labels': labels,
            'geometry_information': information, 'checks': checks,
            'edge_leave_one_out': leave_one_out, 'weak_modes': weak,
            'support_correlation_warning': 'edges sharing station scans/surfaces are correlated, not independent probability votes',
            'all_structural_guards_passed': all(checks.values())}, jac


def dense_solve(initial, edges, rules=Rules()):
    objective = GraphObjective(initial, edges)
    zero = np.zeros(objective.dimension)
    before = objective(zero)
    result = least_squares(objective, zero, tr_solver='exact', loss='huber', f_scale=2.,
                           max_nfev=rules.graph_max_nfev, ftol=1e-6, xtol=1e-6, gtol=1e-6)
    after = objective(result.x)
    return objective.unpack(result.x), {
        'success': bool(result.success), 'status': int(result.status), 'message': str(result.message),
        'nfev': int(result.nfev), 'njev': int(result.njev), 'maximum_nfev': rules.graph_max_nfev,
        'optimality': float(result.optimality), 'cost': float(result.cost),
        'initial_normalized_rmse': float(np.sqrt(np.mean(before ** 2))),
        'final_normalized_rmse': float(np.sqrt(np.mean(after ** 2))),
        'tr_solver': 'exact', 'x_scale': 'unchanged default', 'loss': 'huber', 'f_scale': 2.,
    }


def post_heldout(post, seed, pair):
    failures = []
    if post['fixed_domain_points'] != seed['fixed_domain_points']:
        failures.append('domain_count_changed')
    if not post['every_frame_min15']:
        failures.append('insufficient_unique_patch_count')
    if post['frame_equal_coverage'] < .65:
        failures.append('domain_coverage_below_.65')
    for metric in ['frame_equal_plane_rmse_m', 'frame_equal_capped_nn_rmse_m']:
        if post[metric] > seed[metric] + .002:
            failures.append(metric + '_worse_than_seed')
        if post[metric] > pair[metric] + .002:
            failures.append(metric + '_worse_than_pair')
    return failures


def solve_graph(pair_directory, output):
    start = time.monotonic()
    inputs = Inputs()
    record_code(inputs)
    pair_directory = Path(pair_directory).resolve()
    summary = read_json(inputs.artifact(pair_directory / 'summary.json'))
    rows = read_json(inputs.artifact(pair_directory / 'pair_results.json'))
    edges = read_json(inputs.artifact(pair_directory / 'validated_edges.json'))
    if not summary['all_pairs_attempted']:
        raise ValueError('every frozen station pair must finish before graph audit')
    seed, cache, domains = load_domains(summary['domains_directory'], inputs)
    n = len(seed['stations'])
    expected = {(i, j) for i in range(n) for j in range(i+1, n)}
    if len(rows) != len(expected) or {(r['i'], r['j']) for r in rows} != expected:
        raise ValueError('pair inventory missing/duplicated')
    accepted_rows = {(r['i'], r['j']): r for r in rows if r['accepted']}
    if len(edges) != len(accepted_rows) or {(e['i'], e['j']) for e in edges} != set(accepted_rows):
        raise ValueError('validated edge inventory differs from accepted pairs')
    for e in edges:
        if not np.array_equal(e['measurement'], accepted_rows[(e['i'], e['j'])]['forward_transform']):
            raise ValueError('graph measurement changed from recorded forward fit')
    inputs.add(HERE / 'pipeline_graph.py')
    inputs.add(HERE / 'pipeline_model.py')
    inputs.add(HERE / 'pipeline_pairs.py')
    inputs.add(HERE.parent / 'refine_sensor_prior.py')
    out = new_directory(output)
    initial = np.array([rigid(s['map_from_local_initial']) for s in seed['stations']])
    audit, jac = audit_graph(initial, edges)
    write_json(out / 'geometry_audit.json', audit)
    np.save(out / 'geometry_jacobian.npy', jac)
    gate_counts = Counter(g for r in rows for d in ['forward', 'reverse'] for g in r[d]['rejected_gates'])
    if not audit['all_structural_guards_passed']:
        write_json(out / 'summary.json', {'global_graph_adopted': False, 'global_map_exported': False,
            'graph_optimization_run': False, 'rejection_reasons': [k for k,v in audit['checks'].items() if not v],
            'geometry_audit': 'geometry_audit.json', 'all_pair_rejected_gate_counts': dict(gate_counts),
            'GT_or_query_read': False, 'elapsed_sec': time.monotonic() - start})
        inputs.save(out, stage='graph', global_graph_adopted=False)
        return False
    final, solver = dense_solve(initial, edges)
    delta = np.linalg.norm(final[:, :3, 3] - initial[:, :3, 3], axis=1)
    angle = np.rad2deg(Rotation.from_matrix(np.transpose(initial[:, :3, :3], (0,2,1)) @ final[:, :3,:3]).magnitude())
    try:
        for pose in final:
            rigid(pose)
        finite = True
    except ValueError:
        finite = False
    checks = {'solver_success': solver['success'] and solver['status'] > 0,
              'solver_before_iteration_cap': solver['nfev'] < Rules().graph_max_nfev,
              'finite_valid_SE3': finite, 'first_visit_gauge_exact': bool(np.array_equal(final[0], initial[0])),
              'node_translation_updates_bounded': bool(delta.max() <= .5),
              'node_rotation_updates_bounded': bool(angle.max() <= 10.),
              'objective_not_worse': solver['final_normalized_rmse'] <= solver['initial_normalized_rmse'] + 1e-10}
    edge_consistency, held = [], []
    for e in edges:
        i,j = e['i'],e['j']
        residual = relative_error(final[i], final[j], np.asarray(e['measurement']))
        td, rd = float(np.linalg.norm(residual[3:])), float(np.rad2deg(np.linalg.norm(residual[:3])))
        edge_consistency.append({'i':i, 'j':j, 'translation_m':td, 'rotation_deg':rd, 'passes':td <= .03 and rd <= 1.})
        z = invert(final[i]) @ final[j]
        for name, src, tgt, actual in [('forward',j,i,z), ('reverse',i,j,invert(z))]:
            post = evaluate(cache, src, tgt, domain_value(domains[(i,j,name)]), actual)
            previous = accepted_rows[(i,j)][name]
            before, pair = previous['scan_heldout_before'], previous['scan_heldout_after']
            failures = post_heldout(post, before, pair)
            held.append({'i':i, 'j':j, 'direction':name, 'passes':not failures,
                         'rejected_gates':failures, 'seed':before, 'individual_pairfit':pair, 'graph':post})
    checks['all_edge_measurements_consistent'] = all(x['passes'] for x in edge_consistency)
    checks['all_fixed_scan_heldout_preserved'] = all(x['passes'] for x in held)
    post_audit, _ = audit_graph(final, edges)
    checks['final_geometry_fullrank_and_redundant'] = post_audit['all_structural_guards_passed']
    adopted = all(checks.values())
    filename = 'optimized_nodeposes.json' if adopted else 'rejected_candidate_nodeposes.json'
    write_json(out / filename, {'sensor_only':True, 'global_graph_adopted':adopted,
        'diagnostic_only':True, 'map_export_authorized':False, 'all_guards_passed':adopted,
        'first_gauge_node':0, 'expected_station_count':n, 'source_sensor_seed':str(Path(summary['domains_directory'])/'sensor_seed.json'),
        'stations':[{'node':i, 'id':s['id'], 'map_from_local':final[i].tolist(),
                     'geometrically_constrained':True} for i,s in enumerate(seed['stations'])]})
    write_json(out / 'fixed_scan_holdout_after_graph.json', held)
    write_json(out / 'post_geometry_audit.json', post_audit)
    write_json(out / 'summary.json', {'global_graph_adopted':adopted, 'global_map_exported':False,
        'graph_optimization_run':True, 'checks':checks, 'actual_solver_termination':solver,
        'node_translation_updates_m':delta.tolist(), 'node_rotation_updates_deg':angle.tolist(),
        'edge_consistency':edge_consistency, 'candidate_nodepose_file':filename,
        'all_pair_rejected_gate_counts':dict(gate_counts), 'GT_or_query_read':False,
        'elapsed_sec':time.monotonic()-start})
    inputs.save(out, stage='graph', global_graph_adopted=adopted)
    return adopted
