#!/usr/bin/env python3
"""Read-only MASD audit of recorded intents, decisions, odometry and PX4 ULogs.

The production C++ receiver reconstructs the mean and covariance. Output is
offline evidence, never a controller calibration or a safety certificate.
"""
import argparse
import ctypes
import csv
import json
import math
import subprocess
from collections import Counter, defaultdict
from functools import lru_cache
from itertools import combinations
from pathlib import Path

import numpy as np
from pyulog import ULog
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = Path(__file__).resolve().parent
ROLLS = np.array([-50, -30, -15, 0, 15, 30, 50])
CHI2 = 7.814727903251179
BINS = [(0, .5, '0–0.5 s'), (.5, 1, '0.5–1 s'), (1, 2, '1–2 s'), (2, 4.5, '2–4.5 s')]


def stats(values):
    a = np.asarray(values, float)
    a = a[np.isfinite(a)]
    return dict(n=len(a), **({k: float(v) for k, v in zip(
        ['median', 'p95', 'p99', 'max'], np.quantile(a, [.5, .95, .99, 1]))} if len(a) else {}))


def csv_write(path, rows):
    if not rows:
        return
    with path.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def json_write(path, value):
    with path.open('w') as f:
        json.dump(value, f, indent=2, ensure_ascii=False, allow_nan=False)


def native_library(out):
    root = HERE.parents[2] / 'collision_avoidance'
    library = out / 'masd_native.so'
    sources = [HERE / 'masd_native.cpp'] + [root / 'src' / x for x in (
        'estimation/trajectory_prediction/TrajectoryPredict.cpp',
        'estimation/trajectory_prediction/TrajectoryUncertainty.cpp',
        'estimation/trajectory_prediction/TrajectoryIntent.cpp',
        'estimation/reconstruction/ReconstructTrajectory.cpp')]
    subprocess.run(['g++', '-O3', '-DNDEBUG', '-std=c++17', '-shared', '-fPIC',
                    '-I' + str(root / 'include'), '-I/usr/include/eigen3',
                    *map(str, sources), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    f = lib.masd_reconstruct
    f.argtypes = [np.ctypeslib.ndpointer(np.float32, flags='C_CONTIGUOUS'),
                  ctypes.c_ulonglong, ctypes.c_int, ctypes.c_int, ctypes.c_double,
                  np.ctypeslib.ndpointer(np.float64, flags='C_CONTIGUOUS')]
    f.restype = ctypes.c_int
    return f


def roll(q):
    return np.arctan2(2*(q[:, 0]*q[:, 1]+q[:, 2]*q[:, 3]),
                      1-2*(q[:, 1]**2+q[:, 2]**2))


def load_ulog(path):
    names = ['timesync_status', 'fixed_wing_lateral_setpoint',
             'vehicle_attitude', 'vehicle_attitude_setpoint', 'vehicle_local_position',
             'vehicle_global_position_groundtruth']
    u = ULog(str(path), names)
    data = {d.name: d.data for d in u.data_list if d.multi_id == 0}
    sync = data['timesync_status']
    if not np.all(sync['source_protocol']==2):
        raise ValueError('Expected the DDS timesync instance, not MAVLink')
    # The filtered offset creates DDS timestamp LABELS. It is not ground-truth
    # wall-clock conversion: in lockstep SILS the filter can lag observed time.
    # Raw observed offsets map ULog events to agent wall time, approximately
    # (RTT symmetry / sampling limitations still apply). Never erase negative
    # latencies or interpret filter lag as communication time.
    def absolute(d):
        ts = d['timestamp'].astype(float)
        return (ts - np.interp(ts, sync['timestamp'], sync['observed_offset'])) * 1e-6
    result = {'path': str(path), 'params': {k: u.initial_parameters.get(k)
              for k in ('FW_R_TC', 'FW_R_RMAX', 'FW_R_LIM')},
              'timesync_observation_residual_us': stats(np.abs(
                  sync['observed_offset'].astype(float)-sync['estimated_offset']))}
    result['clock_labels']=np.column_stack([
        (sync['timestamp'].astype(float)-sync['estimated_offset'])*1e-6,
        (sync['estimated_offset'].astype(float)-sync['observed_offset'])*1e-6])
    result['clock_labels']=result['clock_labels'][np.argsort(result['clock_labels'][:,0])]
    result['clock_samples']=np.column_stack([absolute(sync),
        (sync['estimated_offset'].astype(float)-sync['observed_offset'])*1e-6,
        sync['round_trip_time'].astype(float)*1e-6])
    for name, prefix in [('vehicle_attitude', 'q'), ('vehicle_attitude_setpoint', 'q_d')]:
        d = data[name]
        result[name] = (absolute(d), roll(np.column_stack([d[f'{prefix}[{i}]'] for i in range(4)])))
    d = data['fixed_wing_lateral_setpoint']
    result['lateral'] = (absolute(d), d['lateral_acceleration'].astype(float))
    d = data['vehicle_global_position_groundtruth']
    # Invert this fixture's Gazebo-classic reproject(), NOT WGS84 ECEF.
    # sitl_gazebo-classic/include/common.h: earth_radius = 6353000 m.
    # launch_5vtol.sh supplies the shared PX4_HOME reference below.
    lat0,lon0=np.deg2rad([47.397742,8.545594])
    lat,lon=np.deg2rad(d['lat']),np.deg2rad(d['lon'])
    dl=lon-lon0
    c=np.arccos(np.clip(np.sin(lat0)*np.sin(lat)+np.cos(lat0)*np.cos(lat)*np.cos(dl),-1,1))
    k=np.divide(c,np.sin(c),out=np.ones_like(c),where=c>1e-12)
    north=6353000*k*(np.cos(lat0)*np.sin(lat)-np.sin(lat0)*np.cos(lat)*np.cos(dl))
    east=6353000*k*np.cos(lat)*np.sin(dl)
    result['groundtruth']=np.column_stack([absolute(d),north,east,488.0-d['alt']])
    return result


class Audit:
    def __init__(self, args):
        self.args = args
        self.out = args.output
        self.out.mkdir(parents=True, exist_ok=True)
        self.native = native_library(self.out)
        self.intents = [dict() for _ in range(5)]
        self.by_candidate = [defaultdict(list) for _ in range(5)]
        self.decisions = [[] for _ in range(5)]
        self.graphs = [[] for _ in range(5)]
        self.odometry = [[] for _ in range(5)]
        self.traces = [[] for _ in range(5)]
        self.failures = Counter()
        self.replay_errors = []
        self.clock_probes = [json.loads(p.read_text()) for p in args.clock_probe]
        if any(not p['bounds_consistent'] for p in self.clock_probes):
            raise ValueError('Inconsistent remote clock bounds')

    def clock_offset(self, v, wall_s):
        if v != 0:
            return 0.0
        if not self.clock_probes:
            return 0.0  # Explicitly marked uncorrected in latency quality.
        points = sorted((np.mean([s['local_send_ns'] for s in p['samples']])*1e-9,
                         p['midpoint_ns']*1e-9) for p in self.clock_probes)
        return float(np.interp(wall_s, [p[0] for p in points], [p[1] for p in points]))

    def load(self):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(self.args.bag), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types()
                 if x.name.startswith('/common/')}
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if topic not in types:
                continue
            v = int(topic.split('/')[2].split('_')[-1])
            m = deserialize_message(data, types[topic])
            if topic.endswith('/trajectory_intent'):
                key = (int(m.source_timestamp_us), int(m.candidate_id), int(m.candidate_input_revision))
                self.intents[v][key] = m
            elif topic.endswith('/maneuver_selection_decision'):
                self.decisions[v].append((stamp*1e-9, m))
            elif topic.endswith('/interaction_graph_diagnostics'):
                self.graphs[v].append((stamp*1e-9, m))
            elif topic.endswith('/trans_vehicle_odometry'):
                self.odometry[v].append((m.timestamp*1e-6, *m.position, *m.velocity))
            elif topic.endswith('/maneuver_budget_trace'):
                self.traces[v].append((stamp*1e-9, m))
        for v in range(5):
            for key in sorted(self.intents[v]):
                self.by_candidate[v][key[1]].append(key)
            self.odometry[v] = np.array(sorted(self.odometry[v]), float)
        self.logs = [load_ulog(p) for p in self.args.ulog]
        self.published = []
        for v in range(5):
            self.published.append(np.array(sorted(
                (m.wall_ns*1e-9-self.clock_offset(v,m.wall_ns*1e-9),
                 m.ground_speed_command_mps,m.lateral_acceleration_mps2,float(m.active))
                for _,m in self.traces[v] if m.event==4),float).reshape(-1,4))
        self.start = self.args.start_us*1e-6
        self.end = min(o[-1, 0] for o in self.odometry)
        print('loaded', [len(x) for x in self.intents], 'intents; start=', self.start, flush=True)

    @lru_cache(maxsize=24000)
    def cone(self, v, key):
        m = self.intents[v][key]
        values = np.array([*m.initial_state, *m.initial_covariance,
                           *m.candidate_input, *m.compressed_mean], np.float32)
        out = np.zeros((46, 19), np.float64)
        if not self.native(values, key[2], key[1], m.candidate_set_size, self.args.tau_phi, out):
            raise ValueError(f'invalid packet {v}:{key}')
        return out

    def key(self, v, source, cid, revision=None):
        if revision is not None and (source, cid, revision) in self.intents[v]:
            return (source, cid, revision)
        found = [x for x in self.by_candidate[v][cid] if x[0] == source]
        if len(found) != 1:
            self.failures['missing_or_ambiguous_exact_intent'] += 1
            return None
        return found[0]

    def latest_key(self, v, cid, now, revision=None):
        keys = self.by_candidate[v][cid]
        sources = np.array([k[0] for k in keys], dtype=np.int64)
        idx = np.searchsorted(sources, round(now*1e6), side='right')-1
        while idx >= 0:
            k = keys[idx]
            if now-k[0]*1e-6 > .30:
                break
            if revision is None or k[2] == revision:
                return k
            idx -= 1
        return None

    def pair(self, i, ki, j, kj, now):
        age = np.array([now-ki[0]*1e-6, now-kj[0]*1e-6])
        if np.min(age) < -1e-5 or np.max(age) > 3:
            return None
        count = int(np.floor((4.5-max(age)+1e-6)/.1))+1
        horizon = np.arange(count)*.1
        ca, cb = self.cone(i, ki), self.cone(j, kj)
        grid = np.arange(46)*.1
        a = np.column_stack([np.interp(age[0]+horizon, grid, ca[:, k]) for k in range(19)])
        b = np.column_stack([np.interp(age[1]+horizon, grid, cb[:, k]) for k in range(19)])
        rel = b[:, :3]-a[:, :3]
        ranges = np.linalg.norm(rel, axis=1)
        los = rel/np.maximum(ranges[:, None], 1e-12)
        cov = (a[:, 3:12]+b[:, 3:12]).reshape(-1, 3, 3)
        var = np.einsum('ni,nij,nj->n', los, cov, los)
        var[ranges < 1e-9] = np.trace(cov[ranges < 1e-9], axis1=1, axis2=2)
        u95 = np.sqrt(CHI2*np.maximum(0, var))
        k = np.argmin(ranges)
        return dict(horizon=horizon, age=age, ranges=ranges, u95=u95, pmr_index=k,
                    pmr=float(ranges[k]), masd=float(12.144+u95[k]),
                    ad=float(ranges[k]-12.144-u95[k]), pmr_horizon=float(horizon[k]),
                    spline_error=np.linalg.norm(a[:, :3]-a[:, 16:19], axis=1)
                      +np.linalg.norm(b[:, :3]-b[:, 16:19], axis=1))

    def truth(self, v, times):
        o = self.logs[v]['groundtruth']
        idx = np.searchsorted(o[:, 0], times, side='right')
        valid = (idx>0)&(idx<len(o))
        idx = np.clip(idx, 1, len(o)-1)
        valid &= o[idx, 0]-o[idx-1, 0] <= .15
        return np.column_stack([np.interp(times, o[:, 0], o[:, k]) for k in (1, 2, 3)]), valid

    def truth_separation(self):
        times=np.arange(round(self.start*1e6),round(self.end*1e6),10000,dtype=np.int64)*1e-6
        rows=[]
        for i,j in combinations(range(5),2):
            a,va=self.truth(i,times);b,vb=self.truth(j,times)
            r=np.linalg.norm(a-b,axis=1);good=va&vb
            rows.extend(dict(time_s=float(t-self.start),pair=f'{i}-{j}',distance_m=float(d))
                        for t,d in zip(times[good],r[good]))
        csv_write(self.out/'groundtruth_separation.csv',rows)
        low=min(rows,key=lambda r:r['distance_m'])
        unsafe=set(round(r['time_s'],2) for r in rows if r['distance_m']<10)
        return dict(minimum=low,any_pair_below_10m_duration_s=len(unsafe)*.01,
                    grid_s=.01,truth='ULog vehicle_global_position_groundtruth; Gazebo-classic reprojection inverse',
                    maximum_allowed_interpolation_gap_s=.15)

    def matching_until(self, v, key):
        """Actual PX4 lateral input must match from source, not only after commit."""
        m = self.intents[v][key]
        ts, alat = self.logs[v]['lateral']
        start = key[0]*1e-6
        idx = np.searchsorted(ts, start, side='right')-1
        if idx < 0 or abs(alat[idx]-m.candidate_input[3]) > .02:
            return start-1
        stop = np.searchsorted(ts, start+4.51, side='right')
        bad = np.flatnonzero(~np.isfinite(alat[idx:stop]) | (np.abs(alat[idx:stop]-m.candidate_input[3])>.02))
        return float(ts[idx+bad[0]]) if len(bad) else min(start+4.5, ts[-1])

    def speed_lateral_matching_until(self, v, key):
        """A stricter observed subset, not a full longitudinal-model certificate."""
        samples=self.published[v]
        start=key[0]*1e-6
        if not len(samples) or (v==0 and not self.clock_probes):return start-1
        idx=np.searchsorted(samples[:,0],start,side='right')-1
        if idx<0 or start-samples[idx,0]>.15:return start-1
        inp=self.intents[v][key].candidate_input
        stop=np.searchsorted(samples[:,0],start+4.5,side='right')
        segment=samples[idx:stop]
        good=(np.abs(segment[:,1]-inp[0])<=.02)&(np.abs(segment[:,2]-inp[3])<=.02)&(segment[:,3]>0)
        # A missed trace cannot certify an unobserved command interval.
        good[1:] &= np.diff(segment[:,0])<=.15
        bad=np.flatnonzero(~good)
        until=segment[bad[0],0] if len(bad) else min(start+4.5,segment[-1,0])
        return min(float(until),self.matching_until(v,key))

    def coverage(self):
        rows = []
        # One canonical aircraft's assembled graph tuple per epoch: do not
        # count five independent copies of the same selection as five trials.
        seen = set()
        for _, g in self.graphs[0]:
            now = g.evaluation_timestamp_us*1e-6
            if now < self.start or not g.global_crosscheck_evaluated or g.selection_epoch in seen:
                continue
            seen.add(g.selection_epoch)
            keys = [self.key(v, int(g.candidate_source_timestamps_us[v]), int(g.assembled_candidate_ids[v])) for v in range(5)]
            pair_ads=[]
            for i, j in combinations(range(5), 2):
                if keys[i] is None or keys[j] is None:
                    continue
                p = self.pair(i, keys[i], j, keys[j], now)
                if p is None:
                    continue
                pair_ads.append(p['ad'])
                target = now+p['horizon']
                ai, vi = self.truth(i, target); aj, vj = self.truth(j, target)
                actual = np.linalg.norm(ai-aj, axis=1)
                error = np.maximum(0, p['ranges']-actual)
                match_until = min(self.matching_until(i, keys[i]), self.matching_until(j, keys[j]))
                strict_until = min(self.speed_lateral_matching_until(i,keys[i]),self.speed_lateral_matching_until(j,keys[j]))
                ids = (keys[i][1], keys[j][1])
                high = any(abs(ROLLS[x]) == 50 for x in ids)
                # State-opposed roll direction is observable even when the
                # prior requested input was not logged in an old bag.
                reversal = any(float(self.intents[v][keys[v]].initial_state[6])*ROLLS[keys[v][1]] < 0
                               and abs(float(self.intents[v][keys[v]].initial_state[6])) > np.deg2rad(5)
                               for v in (i, j))
                maxbank = max(abs(ROLLS[x]) for x in ids)
                for k, h in enumerate(p['horizon']):
                    if not (vi[k] and vj[k]):
                        continue
                    hb = next(label for lo, hi, label in BINS if lo-1e-6 <= h <= hi+1e-6)
                    rows.append(dict(epoch=int(g.selection_epoch), pair=f'{i}-{j}', eval_time_s=now-self.start,
                        horizon_s=float(h), horizon_bin=hb, effective_horizon_s=float(h+max(p['age'])),
                        first_id=ids[0], second_id=ids[1], maneuver=f'max_bank_{maxbank}',
                        high_bank=high, state_opposed_roll=reversal,
                        lateral_command_matched=bool(target[k] < match_until-1e-6),
                        speed_and_lateral_matched=bool(target[k] < strict_until-1e-6),
                        predicted_range_m=float(p['ranges'][k]), actual_range_m=float(actual[k]),
                        error_positive_m=float(error[k]), u95_m=float(p['u95'][k]),
                        covered=bool(error[k] <= p['u95'][k]), excess_m=float(max(0,error[k]-p['u95'][k])),
                        at_pmr=k==p['pmr_index'], spline_pair_bound_m=float(p['spline_error'][k])))
            if len(pair_ads)==10 and g.assembled_candidate_valid_mask==31:
                self.replay_errors.append(abs(min(pair_ads)-float(g.global_crosscheck_minimum_ad_m)))
            if len(seen)%100 == 0:
                print('coverage epochs', len(seen), 'samples',len(rows), flush=True)
        csv_write(self.out/'coverage_samples.csv', rows)
        groups = []
        for scope in ('operational', 'lateral_command_matched','speed_and_lateral_matched'):
            subset = [r for r in rows if scope=='operational' or r[scope]]
            for dimension in ('all','horizon_bin','maneuver','high_bank','state_opposed_roll','at_pmr'):
                values = ['all'] if dimension=='all' else sorted(set(str(r[dimension]) for r in rows))
                for value in values:
                    rr = subset if dimension=='all' else [r for r in subset if str(r[dimension])==value]
                    if not rr:
                        groups.append(dict(scope=scope,dimension=dimension,group=value,n=0,epochs=0,coverage=None,
                            epoch_bootstrap_ci_low=None,epoch_bootstrap_ci_high=None,error_p95_m=None,u95_median_m=None,excess_max_m=None))
                        continue
                    ec=defaultdict(lambda:[0,0])
                    for r in rr:
                        ec[r['epoch']][0]+=int(r['covered']);ec[r['epoch']][1]+=1
                    ec=np.array(list(ec.values()))
                    rng=np.random.default_rng(7201)
                    samples=ec[rng.integers(0,len(ec),(1000,len(ec)))].sum(axis=1)
                    ci=np.quantile(samples[:,0]/samples[:,1],[.025,.975])
                    groups.append(dict(scope=scope,dimension=dimension,group=value,n=len(rr),epochs=len(ec),
                        coverage=float(np.mean([r['covered'] for r in rr])),epoch_bootstrap_ci_low=float(ci[0]),
                        epoch_bootstrap_ci_high=float(ci[1]),error_p95_m=float(np.quantile([r['error_positive_m'] for r in rr],.95)),
                        u95_median_m=float(np.median([r['u95_m'] for r in rr])),excess_max_m=max(r['excess_m'] for r in rr)))
        csv_write(self.out/'coverage_groups.csv',groups)
        return groups

    def ad_loss(self):
        rows=[]
        # Re-evaluate each previous cycle's exact candidate commands at the
        # next cycle using updated source states. Frozen source propagation
        # alone is reported separately by age; no stale output field assumed fresh.
        for cadence,dt in [('20Hz',.05),('4Hz',.25)]:
            previous={}
            # Integer microsecond grid avoids accumulating floating-point error
            # when np.arange starts at a Unix epoch near 1.8e9 seconds.
            times=(np.arange(round(self.start*1e6),round((self.end-.1)*1e6),round(dt*1e6),dtype=np.int64)*1e-6)
            dec_times=np.array([t for t,_ in self.decisions[0]])
            for now in times:
                ix=np.searchsorted(dec_times,now,side='right')-1
                if ix<0: continue
                m=self.decisions[0][ix][1]
                if not m.coordination_qualified or m.selected_candidate_valid_mask!=31: continue
                ids=tuple(map(int,m.selected_candidate_ids));revs=tuple(map(int,m.selected_candidate_input_revisions))
                keys=[self.latest_key(v,ids[v],now,revs[v]) for v in range(5)]
                for i,j in combinations(range(5),2):
                    if keys[i] is None or keys[j] is None: continue
                    p=self.pair(i,keys[i],j,keys[j],now)
                    if p is None: continue
                    identity=(i,j,ids[i],ids[j],revs[i],revs[j])
                    old=previous.get((i,j))
                    if old is not None:
                        t0,id0,q=old
                        if abs(now-t0-dt)<1e-4:
                            same=identity==id0
                            rows.append(dict(cadence=cadence,time_s=now-self.start,pair=f'{i}-{j}',
                                same_inputs=same,previous_ad_m=q['ad'],ad_m=p['ad'],delta_ad_m=p['ad']-q['ad'],
                                loss_m=max(0,q['ad']-p['ad']),delta_pmr_m=p['pmr']-q['pmr'],
                                delta_masd_m=p['masd']-q['masd'],previous_pmr_horizon_s=q['pmr_horizon'],
                                pmr_horizon_s=p['pmr_horizon'],crossed_zero=q['ad']>=0 and p['ad']<0))
                    previous[(i,j)]=(now,identity,p)
            print('AD-loss',cadence,'rows',len(rows),flush=True)
        csv_write(self.out/'ad_loss_samples.csv',rows)
        summaries={}
        for cadence in ('20Hz','4Hz'):
            for same in (True,False):
                rr=[r for r in rows if r['cadence']==cadence and r['same_inputs']==same]
                summaries[cadence+('_same_pair_inputs' if same else '_input_changed')]=dict(
                    loss_m=stats([r['loss_m'] for r in rr]),delta_ad_m=stats([r['delta_ad_m'] for r in rr]),
                    zero_crossing_count=sum(r['crossed_zero'] for r in rr))
        # Actual 4 Hz global best sequence, distinct from fixed-pair replay.
        for v in range(5):
            seq=[g for _,g in self.graphs[v] if g.evaluation_timestamp_us*1e-6>=self.start and g.global_crosscheck_evaluated]
            rr=[max(0,float(a.global_crosscheck_minimum_ad_m-b.global_crosscheck_minimum_ad_m))
                for a,b in zip(seq,seq[1:]) if b.selection_epoch==a.selection_epoch+1]
            summaries[f'4Hz_graph_assembled_minimum_node{v}']=dict(loss_m=stats(rr))
        return summaries

    def latency(self):
        rows=[]
        for v in range(5):
            seen={}; committed=set()
            lat_t,lat_a=self.logs[v]['lateral']
            sp_t,sp_phi=self.logs[v]['vehicle_attitude_setpoint']
            att_t,phi=self.logs[v]['vehicle_attitude']
            for t,m in self.decisions[v]:
                if t<self.start: continue
                identity=(int(m.proposal_epoch),int(m.proposal_timestamp_us),tuple(m.proposed_candidate_ids))
                if m.proposal_valid: seen.setdefault(identity,t)
                selected=(int(m.local_selection_epoch),int(m.selection_timestamp_us),tuple(m.selected_candidate_ids))
                if m.new_best_accepted and selected in seen and selected not in committed:
                    committed.add(selected)
                    rows.append(dict(vehicle=v,stage='proposal_observed_to_commit_observed',time_s=t-self.start,
                                     delay_s=t-seen[selected],quality='central_bag_arrival_proxy'))
            events=sorted((m for _,m in self.traces[v]),key=lambda m:m.steady_ns)
            proposals={};commits={};previous_publish=None;paired_commit=set()
            publish_for_receipt={}
            for m in events:
                wall=m.wall_ns*1e-9-self.clock_offset(v,m.wall_ns*1e-9)
                if wall<self.start: continue
                clock_quality=('clock_probe_interpolated_Pi_to_agent_clock' if v==0 and self.clock_probes
                               else 'UNCORRECTED_cross_host_clock' if v==0 else 'same_PC_system_clock')
                key=(int(m.epoch),int(m.candidate_id),int(m.input_revision))
                def add(stage,delay,quality='instrumented_same_host_steady_clock'):
                    rows.append(dict(vehicle=v,stage=stage,time_s=wall-self.start,
                                     delay_s=float(delay),quality=quality))
                if m.event==1:
                    proposals[key]=m
                    add('candidate_source_to_proposal_ready',wall-m.source_timestamp_us*1e-6,clock_quality)
                    if m.state_sample_timestamp_us:
                        add('latest_fusion_sample_to_proposal_ready',wall-m.state_sample_timestamp_us*1e-6,clock_quality)
                    if m.state_timestamp_us:
                        add('latest_belief_publication_to_proposal_ready',wall-m.state_timestamp_us*1e-6,clock_quality)
                    label_clock=self.logs[v]['clock_labels']
                    for stage,label in [('source_actual_to_proposal_ready',m.source_timestamp_us),
                                        ('latest_fusion_actual_to_proposal_ready',m.state_sample_timestamp_us),
                                        ('latest_belief_actual_to_proposal_ready',m.state_timestamp_us)]:
                        if label:
                            raw_time=label*1e-6+np.interp(label*1e-6,label_clock[:,0],label_clock[:,1])
                            add(stage,wall-raw_time,'estimated-label undone; observed-timesync + '+clock_quality)
                elif m.event==2:
                    commits[key]=m
                    if key in proposals:
                        add('proposal_ready_to_commit',(m.steady_ns-proposals[key].steady_ns)*1e-9)
                elif m.event==4:
                    add('ros_publish_call_duration',(m.publish_end_wall_ns-m.wall_ns)*1e-9)
                    changed=(previous_publish is None or m.active!=previous_publish.active
                        or abs(m.lateral_acceleration_mps2-previous_publish.lateral_acceleration_mps2)>.02)
                    previous_publish=m
                    if m.active and key in commits and key not in paired_commit:
                        paired_commit.add(key)
                        add('commit_to_first_selected_ros_publish',(m.steady_ns-commits[key].steady_ns)*1e-9)
                        add('commit_active_to_selected_ros_publish' if commits[key].active else
                            'commit_inactive_to_later_active_ros_publish',
                            (m.steady_ns-commits[key].steady_ns)*1e-9)
                        if key in proposals:
                            add('proposal_to_first_selected_ros_publish',(m.steady_ns-proposals[key].steady_ns)*1e-9)
                    if changed and m.active:
                        # Match a VALUE TRANSITION at PX4, not any subsequent
                        # same-valued sample. Never silently clamp a negative delay.
                        changes=np.r_[True,np.abs(np.diff(lat_a))>.02]
                        candidates=np.flatnonzero(changes & (np.abs(lat_a-m.lateral_acceleration_mps2)<.02)
                            & (lat_t>=wall-.02) & (lat_t<=wall+.3))
                        if len(candidates)==1:
                            k=candidates[0]; receipt=lat_t[k]
                            add('ros_publish_to_px4_receipt',receipt-wall,
                                'PX4_hrt_observed_timesync_mapped; '+clock_quality)
                            if receipt>=wall:publish_for_receipt[k]=(m,wall,key)
                        else:self.failures['unmatched_or_ambiguous_px4_receipt']+=1
            # ULog response analysis also works for old bags: actual command
            # transitions are selected from PX4, never from a stale decision flag.
            changes=np.flatnonzero(np.r_[False,np.abs(np.diff(lat_a))>.02])
            for k in changes:
                input_t=lat_t[k]
                if input_t<self.start or input_t>=self.end or not np.isfinite(lat_a[k]):continue
                desired=math.atan(lat_a[k]/9.80665)
                if min(abs(desired-np.deg2rad(ROLLS)))>np.deg2rad(.1):continue
                end=min(input_t+1.0,lat_t[-1],self.end)
                steps=np.arange(input_t,end,.01)
                if len(steps)<40:continue
                command=lat_a[np.clip(np.searchsorted(lat_t,steps,side='right')-1,0,len(lat_a)-1)]
                if np.max(np.abs(command-lat_a[k]))>.02:continue
                actual=np.interp(steps,att_t,phi);initial=actual[0]
                if abs(desired-initial)<math.radians(10):continue
                sp_indices=np.flatnonzero((sp_t>=input_t)&(sp_t<=input_t+.15)&(np.abs(sp_phi-desired)<math.radians(1)))
                if len(sp_indices):
                    rows.append(dict(vehicle=v,stage='px4_receipt_to_attitude_target',time_s=input_t-self.start,
                                     delay_s=float(sp_t[sp_indices[0]]-input_t),quality='ULog_same_PX4_clock'))
                # A threshold-defined response time is descriptive and includes
                # roll dynamics. It is NOT an additive delay budget.
                response=np.flatnonzero((actual-initial)*np.sign(desired-initial)>=.1*abs(desired-initial))
                if len(response):
                    response_time=float(steps[response[0]])
                    rows.append(dict(vehicle=v,stage='px4_receipt_to_10percent_roll_response',time_s=input_t-self.start,
                                     delay_s=float(steps[response[0]]-input_t),quality='includes_roll_dynamics_do_not_add_to_MASD'))
                    if k in publish_for_receipt:
                        pub,wall,key=publish_for_receipt[k]
                        rows.append(dict(vehicle=v,stage='ros_publish_to_10percent_roll_response',time_s=wall-self.start,
                            delay_s=response_time-wall,quality='linked_actual_input_transition; includes_roll_dynamics'))
                        if key in proposals:
                            prop=proposals[key]
                            prop_wall=prop.wall_ns*1e-9-self.clock_offset(v,prop.wall_ns*1e-9)
                            rows.append(dict(vehicle=v,stage='proposal_to_10percent_roll_response',time_s=prop_wall-self.start,
                                delay_s=response_time-prop_wall,quality='linked_actual_input_transition; includes_roll_dynamics'))
                scores=[]
                for delay in np.arange(0,.201,.005):
                    prediction=[initial]
                    for tt in steps[1:]:
                        x=prediction[-1]
                        rate=0 if tt-input_t<delay else np.clip((desired-x)/self.args.tau_phi,-math.radians(70),math.radians(70))
                        prediction.append(x+.01*rate)
                    scores.append(np.mean((actual-np.array(prediction))**2))
                rows.append(dict(vehicle=v,stage='roll_model_additional_delay_fit',time_s=input_t-self.start,
                                 delay_s=int(np.argmin(scores))*.005,quality='conditional_fixed_tau_fit_not_proven_transport_delay'))
        csv_write(self.out/'latency_samples.csv',rows)
        summary={s:dict(distribution_s=stats([r['delay_s'] for r in rows if r['stage']==s]),
                       negative_count=sum(r['delay_s']<0 for r in rows if r['stage']==s),
                       by_vehicle={str(v):stats([r['delay_s'] for r in rows if r['stage']==s and r['vehicle']==v]) for v in range(5)},
                       quality=sorted(set(r['quality'] for r in rows if r['stage']==s))) for s in sorted(set(r['stage'] for r in rows))}
        if 'roll_model_additional_delay_fit' in summary:
            summary['roll_model_additional_delay_fit']['upper_search_boundary_count']=sum(
                r['stage']=='roll_model_additional_delay_fit' and r['delay_s']>=.2 for r in rows)
            summary['roll_model_additional_delay_fit']['search_interval_s']=[0,.2]
        return summary

    def monitored_ad_loss(self):
        rows=[]
        for v in range(5):
            previous={}
            for _,m in sorted(self.traces[v],key=lambda x:x[1].wall_ns):
                if m.event!=3 or m.evaluation_timestamp_us*1e-6<self.start:continue
                old=previous.get(m.peer_id)
                if old is not None:
                    elapsed=(m.evaluation_timestamp_us-old.evaluation_timestamp_us)*1e-6
                    if 0 < elapsed <= .1:
                        same=(m.candidate_id,m.peer_candidate_id,m.input_revision,m.peer_input_revision,m.active)==(
                            old.candidate_id,old.peer_candidate_id,old.input_revision,old.peer_input_revision,old.active)
                        rows.append(dict(vehicle=v,peer=m.peer_id,time_s=m.evaluation_timestamp_us*1e-6-self.start,
                            dt_s=elapsed,same_inputs_and_mode=same,previous_ad_m=old.ad_m,ad_m=m.ad_m,
                            loss_m=max(0,old.ad_m-m.ad_m),delta_ad_m=m.ad_m-old.ad_m,
                            delta_pmr_m=m.pmr_m-old.pmr_m,delta_masd_m=m.masd_m-old.masd_m,
                            pmr_horizon_s=m.pmr_horizon_s,previous_pmr_horizon_s=old.pmr_horizon_s,
                            first_age_s=(m.evaluation_timestamp_us-m.source_timestamp_us)*1e-6,
                            second_age_s=(m.evaluation_timestamp_us-m.peer_source_timestamp_us)*1e-6))
                previous[m.peer_id]=m
        csv_write(self.out/'runtime_pair_ad_loss.csv',rows)
        return {str(same):dict(loss_m=stats([r['loss_m'] for r in rows if r['same_inputs_and_mode']==same]),
                              dt_s=stats([r['dt_s'] for r in rows if r['same_inputs_and_mode']==same])) for same in (True,False)}

    def plot(self, coverage, loss):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,3,figsize=(16,4.7),layout='constrained')
        matched_scope=('speed_and_lateral_matched' if any(r['scope']=='speed_and_lateral_matched' and r['n'] for r in coverage)
                       else 'lateral_command_matched')
        for scope,mark in [('operational','o'),(matched_scope,'s')]:
            rr=[r for r in coverage if r['scope']==scope and r['dimension']=='horizon_bin']
            rr=sorted(rr,key=lambda r:[x[2] for x in BINS].index(r['group']))
            valid=[r for r in rr if r['n']]
            axes[0].plot([r['group'] for r in valid],[100*r['coverage'] for r in valid],mark+'-',label=scope.replace('_',' '))
            if scope==matched_scope:
                for r in valid:
                    axes[0].annotate(f"n={r['n']}"+('\ninsufficient' if r['epochs']<5 else ''),
                        (r['group'],100*r['coverage']),xytext=(0,-24),textcoords='offset points',ha='center',fontsize=8)
        axes[0].axhline(95,color='red',ls='--');axes[0].set(ylabel='One-sided coverage [%]',title='Separation-error coverage')
        axes[0].legend(fontsize=8);axes[0].tick_params(axis='x',rotation=20)
        rr=[r for r in coverage if r['scope']==matched_scope and r['dimension']=='maneuver' and r['n']]
        axes[1].bar([r['group'].replace('max_bank_','') for r in rr],[100*r['coverage'] for r in rr])
        axes[1].axhline(95,color='red',ls='--');axes[1].set(title='Command-matched subset (one run)',xlabel='Maximum commanded |bank| [deg]',ylabel='Coverage [%]')
        for r in rr:
            axes[1].text(r['group'].replace('max_bank_',''),100*r['coverage']+1,f"n={r['n']}",ha='center',fontsize=8)
        for k,c in enumerate(('20Hz','4Hz')):
            s=loss[c+'_same_pair_inputs']['loss_m']
            if s['n']:axes[2].bar(np.arange(4)+k*.35,[s[x] for x in ('median','p95','p99','max')],width=.35,label=c)
        axes[2].set_xticks(np.arange(4)+.175,['median','p95','p99','max']);axes[2].set(ylabel='AD loss [m]',title='Same-pair / same-input offline replay')
        axes[2].legend()
        for ax in axes:ax.grid(alpha=.2)
        fig.savefig(self.out/'masd_budget_overview.png',dpi=160)
        plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ulog',type=Path,nargs=5,required=True)
    p.add_argument('--start-us',type=int,required=True,help='Common fixed-wing evaluation start, epoch us')
    p.add_argument('--tau-phi',type=float,required=True,help='Recorded run predictor tau, not current source default')
    p.add_argument('--reuse-analysis',action='store_true',help='Reuse existing coverage/AD-loss results; refresh latency only')
    p.add_argument('--clock-probe',type=Path,nargs='*',default=[],help='Read-only Pi-minus-PC probe JSON before and after run')
    args=p.parse_args()
    a=Audit(args);a.load()
    if args.reuse_analysis:
        old=json.loads((args.output/'masd_budget_summary.json').read_text())
        coverage=old['coverage'];loss=old['ad_loss']
    else:
        coverage=a.coverage();loss=a.ad_loss()
    latency=a.latency()
    report=dict(bag=str(args.bag.resolve()),tau_phi=args.tau_phi,
        groundtruth_separation=a.truth_separation(),
        budget=dict(aircraft_size_m=2.144,dsd_m=10,communication_delay_margin_m=0,
                    chi_squared_3d=CHI2,process_noise_diagonal=[.25,.25,.25,.04,.001,.04,.001]),
        coverage=coverage,ad_loss=loss,runtime_monitor_ad_loss=a.monitored_ad_loss(),latency=latency,
        ulogs=[{k:v for k,v in l.items() if k in ('path','params','timesync_observation_residual_us')} for l in a.logs],
        diagnostics=dict(a.failures),
        reconstruction_vs_recorded_global_ad_error_m=(old['reconstruction_vs_recorded_global_ad_error_m'] if args.reuse_analysis else stats(a.replay_errors)),
        clock_probes=[{k:v for k,v in p.items() if k!='samples'} for p in a.clock_probes],
        trace_event_counts=[dict(Counter(m.event for _,m in tt)) for tt in a.traces],
        trace_max_worker_drop_count=[max((m.dropped_trace_count for _,m in tt),default=0) for tt in a.traces],
        timestamp_label_error_vs_raw_timesync_s=[stats(l['clock_samples'][
            (l['clock_samples'][:,0]>=a.start)&(l['clock_samples'][:,0]<=a.end),1]) for l in a.logs],
        limitations=['Coverage is pointwise separation-error support, not simultaneous whole-path probability.',
          'Epoch bootstrap conditions on this one encounter; it is not an independent Monte Carlo confidence interval.',
          'Matched lateral commands do not certify unchanged longitudinal ground/EAS input.',
          'Speed-and-lateral subset requires actual avoidance plus matching published ground speed and PX4 lateral input; it does not certify altitude/EAS closed-loop tracking.',
          'ULog wall times use raw DDS timesync observations, not the lagged filtered timestamp label. RTT asymmetry and between-probe timing remain limitations.',
          'Old bags lack actual commit and ROS command publication timestamps; bag arrival is only a proxy.',
          '20/4 Hz fixed-pair results are common-time offline replay, not claimed to be the exact runtime cache view.',
          'Hypothetical candidates not actually executed are operational forecast errors, not Q calibration samples.'])
    json_write(args.output/'masd_budget_summary.json',report)
    a.plot(coverage,loss)
    print(json.dumps(dict(output=str(args.output),latency=latency,ad_loss=loss),indent=2),flush=True)


if __name__=='__main__':main()
