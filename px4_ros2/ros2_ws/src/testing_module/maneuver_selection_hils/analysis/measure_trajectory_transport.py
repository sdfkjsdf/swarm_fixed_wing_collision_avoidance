#!/usr/bin/env python3
"""Offline application send->callback latency. No imports from flight code.

TX is system_clock immediately before publish(message), after serialization-field
assembly; RX is system_clock at subscription callback entry. Same packet joins use
(source vehicle, trajectory reference timestamp, epoch, candidate, input revision).
Reference timestamps are identity/window fields, NEVER latency endpoints.
Clock probes run only before/after flight; do not step either system clock.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import shlex
import subprocess
import time

import numpy as np


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def probe(target, count):
    # Bounds need no assumption of symmetric Wi-Fi paths: offset=Pi-PC lies in
    # [Pi_send-PC_receive, Pi_receive-PC_send]. Midpoint is only an estimate.
    if count < 2: raise ValueError('At least two clock exchanges are required')
    remote = ('import sys,time\n'
              'for line in sys.stdin:\n'
              ' r=time.time_ns(); s=time.time_ns(); print(r,s,flush=True)\n')
    command = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
               '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=3',
               target, shlex.join(['python3', '-u', '-c', remote])]
    rows = []
    with subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                          text=True, bufsize=1) as process:
        for _ in range(count):
            mono_start = time.monotonic_ns()
            t1 = time.time_ns()
            process.stdin.write('probe\n'); process.stdin.flush()
            response = process.stdout.readline()
            t4 = time.time_ns()
            mono_end = time.monotonic_ns()
            if not response:
                raise RuntimeError('Clock probe SSH stream ended')
            t2, t3 = map(int, response.split())
            rows.append(dict(t1=t1, t2=t2, t3=t3, t4=t4,
                lower_ns=t3-t4, upper_ns=t2-t1,
                pc_wall_minus_monotonic_elapsed_ns=(t4-t1)-(mono_end-mono_start)))
        process.stdin.close()
        if process.wait(timeout=10):
            raise RuntimeError('Clock probe SSH failed')
    lower = max(r['lower_ns'] for r in rows)
    upper = min(r['upper_ns'] for r in rows)
    def status(cmd):
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        return dict(returncode=result.returncode, stdout=result.stdout, stderr=result.stderr)
    return dict(offset_definition='Pi system_clock minus PC system_clock',
        lower_ns=lower, upper_ns=upper, bounds_consistent=lower<=upper,
        midpoint_ns=(lower+upper)//2, pc_mid_time_ns=(rows[0]['t1']+rows[-1]['t4'])//2,
        pc_ntp=status(['timedatectl', 'show', '-p', 'NTPSynchronized']),
        pi_ntp=status(['ssh', '-o', 'BatchMode=yes', target,
                       'timedatectl show -p NTPSynchronized']), samples=rows)


def parse(text):
    blocks, current, seen = [], None, set()
    for line in text.splitlines():
        if not line.startswith('[stop-transport'): continue
        f = line.split(',')
        if f[0] == '[stop-transport-begin]':
            if current is not None or len(f) != 7 or f[1] != '1':
                raise ValueError('Invalid transport header')
            vehicle, peer, role, count, dropped = int(f[2]), int(f[3]), f[4], int(f[5]), int(f[6])
            key = (vehicle, peer, role)
            if key in seen or role not in ('tx', 'rx') or min(count, dropped)<0:
                raise ValueError('Duplicate/invalid transport stream')
            if (role == 'tx' and peer != -1) or (role == 'rx' and peer == vehicle):
                raise ValueError('Invalid endpoint identity')
            seen.add(key)
            current = dict(vehicle=vehicle, peer=peer, role=role, count=count, dropped=dropped, rows=[])
        elif f[0] == '[stop-transport]':
            if current is None or len(f)!=8:
                raise ValueError('Record outside stream or wrong field count')
            r = tuple(map(int, f[1:]))
            if r[4] <= 0: raise ValueError('Missing actual event timestamp')
            current['rows'].append(r)
        elif f[0] == '[stop-transport-end]':
            if (current is None or len(f)!=5 or
                (int(f[1]), int(f[2]), f[3], int(f[4])) !=
                (current['vehicle'], current['peer'], current['role'], current['count']) or
                len(current['rows']) != current['count']):
                raise ValueError('Incomplete/mismatched transport footer')
            blocks.append(current); current = None
        else:
            raise ValueError('Unknown transport record')
    if current is not None: raise ValueError('Missing transport footer')
    return blocks


def stats(values):
    if not values: return dict(count=0)
    a = np.asarray(values, dtype=float)
    return dict(count=len(values), median_ms=float(np.median(a)), p95_ms=float(np.percentile(a,95)),
        p99_ms=float(np.percentile(a,99)), max_ms=float(a.max()), min_ms=float(a.min()),
        negative_count=int(np.count_nonzero(a<0)))


def matched_delays(tx, rx, pi_vehicle, offset_ns, start_us=0, end_us=2**64):
    send = {}
    for block in tx:
        if block['dropped']: raise ValueError('TX buffer overflow: cannot certify complete sample')
        for row in block['rows']:
            key = (block['vehicle'], *row[:4])
            if key in send: raise ValueError('Ambiguous duplicate TX identity')
            send[key] = row
    result, totals = defaultdict(list), defaultdict(lambda: dict(received=0, unmatched=0, duplicated_rx=0))
    for block in rx:
        if block['dropped']: raise ValueError('RX buffer overflow: cannot certify complete sample')
        src, dst = block['peer'], block['vehicle']
        seen = set()
        for row in block['rows']:
            if not start_us <= row[0] <= end_us: continue
            totals[(src,dst)]['received'] += 1
            key = (src,*row[:4])
            if key in seen: totals[(src,dst)]['duplicated_rx'] += 1
            seen.add(key)
            if key not in send:
                totals[(src,dst)]['unmatched'] += 1
                continue
            sent = send[key]
            sign = int(src == pi_vehicle)-int(dst == pi_vehicle)
            # Pi->PC adds Pi-PC offset; PC->Pi subtracts it. Same PC needs none.
            correction = sign*offset_ns
            raw = row[4]-sent[4]
            result[(src,dst)].append(dict(source_us=row[0], epoch=row[1], candidate=row[2], input_revision=row[3],
                tx_wall_ns=sent[4], rx_wall_ns=row[4], raw_ms=raw/1e6,
                dds_source_wall_ns=row[5], dds_receive_wall_ns=row[6],
                clock_correction_ns=correction,
                corrected_ms=(raw+correction)/1e6,
                send_to_dds_publish_ms=(row[5]-sent[4])/1e6 if row[5]>0 else None,
                send_to_dds_receive_ms=(row[6]-sent[4]+correction)/1e6 if row[6]>0 else None,
                dds_to_callback_ms=(row[4]-row[6])/1e6 if row[6]>0 else None,
                dds_publish_to_callback_ms=(row[4]-row[5]+correction)/1e6 if row[5]>0 else None))
    return result, totals


def direction_summary(rows, link_reports, error_ms):
    """Observed application delivery, not radio-only delay or a future bound."""
    values=[r['corrected_ms'] for r in rows]
    distribution=stats(values)
    coverage={k:sum(r[k] for r in link_reports) for k in
        ('received','matched','sent','unmatched','duplicated_rx','sent_not_matched')}
    coverage['complete_unique_delivery']=(coverage['sent']>0 and
        coverage['received']==coverage['matched']==coverage['sent'] and
        not any(coverage[k] for k in ('unmatched','duplicated_rx','sent_not_matched')))
    result=dict(corrected_send_to_callback=distribution,
        clock_uncertainty_ms=error_ms, coverage=coverage,
        application_delivery_seconds={k[:-3]+'_s':v/1000 for k,v in distribution.items() if k.endswith('_ms')},
        corrected_lower_bound=stats([v-error_ms for v in values]),
        corrected_upper_bound=stats([v+error_ms for v in values]),
        negative_beyond_clock_envelope_count=sum(v+error_ms<0 for v in values),
        send_to_dds_receive=stats([r['send_to_dds_receive_ms'] for r in rows if r['send_to_dds_receive_ms'] is not None]),
        local_dds_to_callback=stats([r['dds_to_callback_ms'] for r in rows if r['dds_to_callback_ms'] is not None]))
    if rows:
        worst=dict(max(rows,key=lambda r:r['corrected_ms']))
        worst['estimated_seconds']=worst['corrected_ms']/1000
        worst['clock_envelope_seconds']=[(worst['corrected_ms']-error_ms)/1000,
                                         (worst['corrected_ms']+error_ms)/1000]
        # One message only: do not add independent per-segment maxima.
        worst['same_message_segments_ms']=dict(
            publish_to_dds_timestamp=worst['send_to_dds_receive_ms'],
            dds_timestamp_to_callback=worst['dds_to_callback_ms'])
        result['worst_message']=worst
    else:
        result['worst_message']=None
    return result


def attach_kernel_boundary(report, kernel):
    """Join an existing offline capture report; never create new live capture."""
    if kernel['run']!=report['run']: raise ValueError('Kernel report belongs to another run')
    direction=report['directions']['PC_to_Pi']
    worst=direction['worst_message']
    if worst is None: raise ValueError('No PC->Pi message to match')
    identities={'source':'source','source_us':'source_us','epoch':'epoch','candidate':'candidate',
        'input_revision':'revision','tx_wall_ns':'tx_ns','rx_wall_ns':'callback_ns',
        'dds_receive_wall_ns':'dds_ns'}
    matches=[r for r in kernel['worst_total'] if all(worst[k]==r[v] for k,v in identities.items())]
    if len(matches)!=1: raise ValueError('Kernel worst message identity/timestamps do not match')
    r=matches[0]
    first,last,dds,callback=(r[k] for k in ('kernel_first_ns','kernel_last_ns','dds_ns','callback_ns'))
    if not first<=last<=dds<=callback or r['copies_before_dds']<1:
        raise ValueError('Invalid same-host kernel/DDS/callback ordering')
    correction=worst['clock_correction_ns']
    segments=dict(publish_to_first_pi_kernel=(first-r['tx_ns']+correction)/1e6,
        first_pi_kernel_to_dds_timestamp=(dds-first)/1e6,
        dds_timestamp_to_callback=(callback-dds)/1e6)
    if abs(sum(segments.values())-worst['corrected_ms'])>1e-6:
        raise ValueError('Same-message segments do not add to total delivery')
    direction['kernel_boundary']=dict(
        matched=kernel['matched'],expected_rx=direction['coverage']['received'],
        complete_rx_match=kernel['matched']==direction['coverage']['received'],
        same_message_segments_ms=segments,copies_before_dds=r['copies_before_dds'],
        accepted_copy_identified=r['copies_before_dds']==1,
        last_pi_kernel_to_dds_timestamp_ms=(dds-last)/1e6,
        scope='Pi software ingress, not NIC hardware or radio arrival',
        pre_kernel_clock_uncertainty_ms=direction['clock_uncertainty_ms'],
        local_segments_need_cross_host_clock_correction=False)


def readable_report(report):
    lines=['기체 간 정보 전달 지연 — 기존 기록의 오프라인 분석',
        f"실행: {report['run']}",
        '기준: 실제 ROS publish 직전 → 상대 subscription callback 진입.',
        'DDS·운영체제·네트워크·수신 대기를 포함한 전체 전달 시간이며, 순수 Wi-Fi 지연이 아닙니다.',
        'PX4 상태시각/궤적 source age 및 송신 전 예측 계산은 이 지연에서 제외합니다.',
        '시계 보정 범위는 비행 전후 probe 사이 더 큰 시계 변동이 없다는 가정이며 통계적 신뢰구간이 아닙니다.', '']
    for name,d in report['directions'].items():
        label='Pi → PC' if name=='Pi_to_PC' else 'PC → Pi'
        s=d['application_delivery_seconds']; c=d['coverage']; w=d['worst_message']
        lines.append(f"{label}: 수신 {c['received']}개 / 송신 일치 {c['matched']}개 / 송신 {c['sent']}개")
        if w is None:
            lines.append('측정 가능한 일치 메시지 없음. 지연 0으로 해석하면 안 됩니다.');continue
        lines.append(f"중앙값 {s['median_s']:.6f}초, p99 {s['p99_s']:.6f}초, 관측 최대 {s['max_s']:.6f}초 ({s['max_s']*1000:.3f} ms)")
        lower,upper=w['clock_envelope_seconds']
        lines.append(f"최대값의 시계 보정 범위: {lower:.6f}~{upper:.6f}초 (±{d['clock_uncertainty_ms']/1000:.6f}초)")
        lines.append(f"누락/중복 없는 전체 전달: {c['complete_unique_delivery']}; 시계 범위 밖 음수: {d['negative_beyond_clock_envelope_count']}개")
        if 'kernel_boundary' in d:
            k=d['kernel_boundary']; a=k['same_message_segments_ms']
            lines.append(f"같은 최악 메시지: Pi 커널 수신 전 {a['publish_to_first_pi_kernel']:.6f} ms + "
                f"커널→DDS {a['first_pi_kernel_to_dds_timestamp']:.6f} ms + "
                f"DDS→콜백 {a['dds_timestamp_to_callback']:.6f} ms = {w['corrected_ms']:.6f} ms")
            lines.append(f"커널 기록 일치: {k['matched']}/{k['expected_rx']}; 수신 전 복사본 수: {k['copies_before_dds']}")
            if not k['accepted_copy_identified']:
                lines.append('재수신 복사본 중 실제 채택된 패킷은 불명확합니다. 첫 도착 기준 분해입니다.')
        else:
            lines.append('커널 경계 분해 없음. DDS 수신 시각을 네트워크 카드 도착 시각으로 해석하지 않습니다.')
        lines.append('')
    lines.extend(['알 수 있는 것: 정의된 송신→수신 구간의 전달 지연.',
        '이 기록만으로 분리할 수 없는 것: PC 송신 대기 / 무선·AP / Pi 드라이버 각각의 시간.',
        '원인 구간이 미확정이라는 것과 전체 전달 지연을 측정하지 못했다는 것은 다릅니다.',
        '다른 메시지의 구간별 최대값은 더하지 않습니다. 음수나 긴 지연 샘플도 임의로 제거하지 않습니다.',
        '관측 최대는 이번 실행의 값이며, 이후 모든 전달의 상한이나 실시간 마감 보장이 아닙니다.'])
    return '\n'.join(lines)+'\n'


def analyze(log_dir, before, after, summary, pi_vehicle):
    if pi_vehicle not in range(5): raise ValueError('Pi vehicle must be one of the five participants')
    for p in (before, after):
        if not p['bounds_consistent'] or p['lower_ns']>p['upper_ns']:
            raise ValueError('Inconsistent clock probe bounds')
    # Conservative endpoint envelope. Inter-flight clock stability remains an
    # explicit assumption, not a claim that two probes prove no intervening step.
    lo = min(before['lower_ns'], after['lower_ns'])
    hi = max(before['upper_ns'], after['upper_ns'])
    center = (lo+hi)//2
    start = int(summary['actual_evaluation_start_ns'])//1000
    end = start+200_000_000
    if summary['common_duration_s']<200: raise ValueError('Common window shorter than 200 s')
    if not before['pc_mid_time_ns'] <= start*1000 < end*1000 <= after['pc_mid_time_ns']:
        raise ValueError('Clock probes do not bracket the evaluation window')
    blocks=[]
    for path in sorted(log_dir.glob('guidance_*.log')): blocks.extend(parse(path.read_text()))
    tx = [b for b in blocks if b['role']=='tx']; rx = [b for b in blocks if b['role']=='rx']
    expected={(s,d) for s in range(5) for d in range(5) if s!=d}
    if (len(tx)!=5 or {b['vehicle'] for b in tx}!=set(range(5)) or
        len(rx)!=20 or {(b['peer'],b['vehicle']) for b in rx}!=expected):
        raise ValueError('Missing or duplicate five-node TX/RX stream identities')
    matched, counts = matched_delays(tx,rx,pi_vehicle,center,start,end)
    links={}
    for src,dst in sorted(expected):
        rows=matched.get((src,dst),[])
        error=(hi-lo)/2e6 if pi_vehicle in (src,dst) else 0
        corrected=[r['corrected_ms'] for r in rows]
        sent_count=sum(start<=r[0]<=end for b in tx if b['vehicle']==src for r in b['rows'])
        unique_matches=len({(r['source_us'],r['epoch'],r['candidate'],r['input_revision']) for r in rows})
        links[f'{src}->{dst}']=dict(**counts[(src,dst)], matched=len(rows), sent=sent_count,
            sent_not_matched=sent_count-unique_matches,
            raw_send_to_callback=stats([r['raw_ms'] for r in rows]),
            corrected_send_to_callback=stats(corrected),
            clock_uncertainty_ms=error,
            corrected_lower_bound=stats([x-error for x in corrected]),
            corrected_upper_bound=stats([x+error for x in corrected]),
            send_to_dds_publish=stats([r['send_to_dds_publish_ms'] for r in rows if r['send_to_dds_publish_ms'] is not None]),
            send_to_dds_receive=stats([r['send_to_dds_receive_ms'] for r in rows if r['send_to_dds_receive_ms'] is not None]),
            local_dds_to_callback=stats([r['dds_to_callback_ms'] for r in rows if r['dds_to_callback_ms'] is not None]),
            dds_publish_to_callback=stats([r['dds_publish_to_callback_ms'] for r in rows if r['dds_publish_to_callback_ms'] is not None]))
    directions={}
    for name,select in [('Pi_to_PC',lambda s,d:s==pi_vehicle),('PC_to_Pi',lambda s,d:d==pi_vehicle)]:
        rows=[dict(r,source=s,destination=d) for (s,d),rr in matched.items() if select(s,d) for r in rr]
        directions[name]=direction_summary(rows,
            [links[f'{s}->{d}'] for s,d in sorted(expected) if select(s,d)],(hi-lo)/2e6)
    return dict(run=log_dir.name, window_s=200, timestamp_window='trajectory reference timestamp', directions=directions,
        endpoints='system_clock immediately before ROS publish -> subscription callback entry',
        measurement_contract=dict(metric='application_to_application_information_delivery',
            includes=['sender DDS/OS delivery','network path','receiver DDS/executor wait'],
            excludes=['prediction calculation before publish','trajectory source age','PX4 command application'],
            dds_receive_timestamp_is_kernel_ingress=False,radio_only_delay_measured=False,
            cross_host_clock='Pi minus PC endpoint-probe envelope; conditional estimate',
            intra_flight_clock_excursions_excluded=False),
        clock=dict(pi_minus_pc_lower_ns=lo,pi_minus_pc_upper_ns=hi, midpoint_ns=center,
                   endpoint_probe_intervals_overlap=max(before['lower_ns'],after['lower_ns'])<=min(before['upper_ns'],after['upper_ns']),
                   before=before,after=after), links=links,
        limitations=['Includes DDS/OS/network/executor delivery; excludes trajectory calculation before publish.',
            'Endpoint clock bounds assume no larger unobserved clock step/drift during the flight.',
            'No negative measurements are clamped or silently discarded.',
            'Missing matches are reported; source timestamp is not used as a send timestamp.'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    commands=p.add_subparsers(dest='command',required=True)
    q=commands.add_parser('probe');q.add_argument('--target',required=True);q.add_argument('--count',type=int,default=64);q.add_argument('--output',type=Path,required=True)
    q=commands.add_parser('analyze');q.add_argument('--log-dir',type=Path,required=True);q.add_argument('--before',type=Path,required=True);q.add_argument('--after',type=Path,required=True);q.add_argument('--summary',type=Path,required=True);q.add_argument('--pi-vehicle',type=int,default=0);q.add_argument('--output',type=Path,required=True)
    q.add_argument('--kernel-report',type=Path,help='Existing matching offline Pi kernel capture report; no live capture')
    q.add_argument('--text-report',type=Path,help='Readable Korean report, with seconds and clock bounds')
    args=p.parse_args()
    if args.command=='probe': result=probe(args.target,args.count)
    else:
        result=analyze(args.log_dir,json.loads(args.before.read_text()),json.loads(args.after.read_text()),json.loads(args.summary.read_text()),args.pi_vehicle)
        if args.kernel_report: attach_kernel_boundary(result,json.loads(args.kernel_report.read_text()))
    save(args.output,result)
    if args.command=='analyze' and args.text_report:
        args.text_report.parent.mkdir(parents=True,exist_ok=True)
        args.text_report.write_text(readable_report(result))
    print(json.dumps({k:v for k,v in result.items() if k not in ('samples','clock')},indent=2))


if __name__=='__main__': main()
