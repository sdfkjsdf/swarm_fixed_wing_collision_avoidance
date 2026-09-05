#!/usr/bin/env python3
"""Read-only remote-minus-local clock bounds over an established SSH stream.

No clock is adjusted. If the remote timestamp was captured between local send
and receive, the true offset lies in [remote-receive, remote-send]. The
midpoint is an estimate, not an assumption of symmetric Wi-Fi delay.
"""
import argparse
import json
import subprocess
import time


def probe(target, count):
    remote = 'import sys,time\nfor line in sys.stdin: print(time.time_ns(),flush=True)'
    import shlex
    p = subprocess.Popen(['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                          target, 'python3 -u -c ' + shlex.quote(remote)],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    samples=[]
    try:
        for _ in range(count):
            begin=time.time_ns()
            p.stdin.write('probe\n');p.stdin.flush()
            line=p.stdout.readline()
            end=time.time_ns()
            if not line:raise RuntimeError('SSH clock probe ended early')
            remote_ns=int(line)
            samples.append(dict(local_send_ns=begin,local_receive_ns=end,remote_ns=remote_ns,
                                offset_lower_ns=remote_ns-end,offset_upper_ns=remote_ns-begin))
        lower=max(x['offset_lower_ns'] for x in samples)
        upper=min(x['offset_upper_ns'] for x in samples)
        return dict(target=target,definition='remote system clock minus local system clock',
                    lower_ns=lower,upper_ns=upper,bounds_consistent=lower<=upper,
                    midpoint_ns=(lower+upper)/2,samples=samples)
    finally:
        p.stdin.close();p.wait(timeout=5)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target')
    parser.add_argument('--count',type=int,default=50)
    parser.add_argument('--output',required=True)
    a=parser.parse_args()
    result=probe(a.target,a.count)
    with open(a.output,'w') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2))
