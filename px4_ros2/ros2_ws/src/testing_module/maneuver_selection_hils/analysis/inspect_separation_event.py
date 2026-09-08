#!/usr/bin/env python3
"""Offline, read-only decision/budget timeline around a recorded closest approach.

Times are bag-observation times, not synchronized PX4 execution times.
This tool is never imported by a control node.
"""
import argparse
import json
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, help='result/summary/<run>')
    parser.add_argument('--start', type=float)
    parser.add_argument('--end', type=float)
    parser.add_argument('--details', action='store_true', help='Include graph, proposal and published-command transitions')
    args = parser.parse_args()
    summary = json.loads((args.run / 'summary.json').read_text())
    start = args.start if args.start is not None else summary['closest_time_s'] - 5
    end = args.end if args.end is not None else summary['closest_time_s'] + 2
    epoch = summary['actual_evaluation_start_ns']
    bag = Path(summary['bag'])
    print('run', args.run.name, 'window', start, end)
    last = {}
    for db in sorted(bag.glob('*.db3')):
        with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as conn:
            topics = {i: (name, typ) for i, name, typ in conn.execute('select id,name,type from topics')}
            classes = {i: get_message(typ) for i, (name, typ) in topics.items()
                       if name.endswith(('maneuver_selection_decision', 'maneuver_budget_trace',
                                         'interaction_graph_diagnostics'))}
            print('topics', [topics[i][0] for i in classes])
            for tid, ts, data in conn.execute(
                'select topic_id,timestamp,data from messages where timestamp between ? and ? order by timestamp,id',
                (int(epoch + start * 1e9), int(epoch + end * 1e9))):
                if tid not in classes:
                    continue
                m = deserialize_message(data, classes[tid])
                t = round((ts - epoch) / 1e9, 6)
                if topics[tid][0].endswith('maneuver_selection_decision'):
                    if args.details and m.vehicle_id in (1, 2):
                        pkey = (int(m.proposal_epoch), bool(m.proposal_valid),
                                bool(m.proposal_consensus_confirmed), tuple(m.proposed_candidate_ids))
                        if last.get(('proposal', tid)) != pkey:
                            last[('proposal', tid)] = pkey
                            print('PROPOSAL', t, m.vehicle_id, pkey, 'safe_count', m.evaluated_safe_combination_count,
                                  'best_ad', round(m.maximum_evaluated_minimum_ad_m, 3),
                                  'proposed_ad', round(m.switch_proposed_minimum_ad_m, 3))
                    # Print transitions, not repeated heartbeats carrying latched flags.
                    key = (bool(m.activation_requested), bool(m.command_execution_requested),
                           int(m.ownship_candidate_id), tuple(m.selected_candidate_ids),
                           bool(m.coordination_qualified), bool(m.safe_rejoin_active))
                    if last.get(tid) == key:
                        continue
                    last[tid] = key
                    print('DEC', t, 'node', m.vehicle_id, 'active/execute', key[:2],
                          'own', key[2], 'tuple', key[3], 'qual', key[4], 'rejoin', key[5],
                          'ad', round(m.ad_m, 3), 'selectedepoch', m.local_selection_epoch,
                          'proposalepoch', m.proposal_epoch, 'acttime', m.activation_timestamp_us,
                          'cpa', m.cpa_clear, 'post', round(m.post_release_minimum_ad_m, 3))
                elif topics[tid][0].endswith('interaction_graph_diagnostics'):
                    if args.details and m.vehicle_id == 1:
                        print('GRAPH', t, m.selection_epoch, 'tuple', tuple(m.assembled_candidate_ids),
                              'components', tuple(m.component_ids), 'minimum_ad', m.global_crosscheck_minimum_ad_m,
                              'valid_check', m.global_crosscheck_pass)
                elif args.details and m.event == 4 and m.vehicle_id in (1, 2):
                    pkey = (bool(m.active), int(m.candidate_id), round(m.lateral_acceleration_mps2, 6))
                    if last.get(('publish', tid)) != pkey:
                        last[('publish', tid)] = pkey
                        print('PUBLISH', t, m.vehicle_id, 'active,candidate,alat', pkey,
                              'wall_s', m.wall_ns / 1e9, 'state', m.state_timestamp_us)
                elif (m.vehicle_id, m.peer_id) in ((1, 2), (2, 1)) and m.event == 3:
                    print('PAIR', t, m.vehicle_id, m.peer_id,
                          'eval', m.evaluation_timestamp_us, 'cand', m.candidate_id, m.peer_candidate_id,
                          'active', m.active, 'ad', round(m.ad_m, 3), 'pmr', round(m.pmr_m, 3),
                          'u95', round(m.u95_m, 3), 'horizon', round(m.pmr_horizon_s, 3))


if __name__ == '__main__':
    main()
