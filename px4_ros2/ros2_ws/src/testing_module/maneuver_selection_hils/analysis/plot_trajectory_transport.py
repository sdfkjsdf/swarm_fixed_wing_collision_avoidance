#!/usr/bin/env python3
"""Offline one-way transport plots from matched stopped TX/RX records.

Same report style as plot_activation_deadline.py; never imported by flight code.
Wi-Fi and both endpoints' delivery processing are included. No delay target is
inferred from the trajectory update rate. Preserve every measured message.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from measure_trajectory_transport import matched_delays, parse
from plot_activation_deadline import clean


def make_figure(x, y, title, clock_error_ms, upper_limit, fig=None):
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12,
        'path.simplify':False,'pdf.fonttype':42,'svg.fonttype':'none'})
    color,gray='#27688F','#666666'
    mean=float(y.mean()); peak=int(np.argmax(y)); maximum=float(y[peak])
    # A shared 40-60 ms break is permitted only if it removes NO observations.
    broken=maximum>60 and not np.any((y>40)&(y<60)) and y.min()>=0
    if fig is None:
        fig=plt.figure(figsize=(12,5.65))
    if broken:
        grid=fig.add_gridspec(2,1,height_ratios=[1.0,3.0],left=.10,right=.985,
            top=.745,bottom=.17,hspace=.13)
        upper=fig.add_subplot(grid[0]); ax=fig.add_subplot(grid[1],sharex=upper)
        axes=[upper,ax]
        upper.set_ylim(60,upper_limit); upper.set_yticks(np.arange(60,upper_limit+.01,5))
        upper.tick_params(axis='x',bottom=False,labelbottom=False)
        ax.set_ylim(0,40);ax.set_yticks([0,10,20,30,40])
    else:
        ax=fig.add_subplot(111);axes=[ax]
        fig.subplots_adjust(left=.10,right=.985,top=.745,bottom=.17)
        ax.set_ylim(min(0,float(y.min())-clock_error_ms),upper_limit)
    for panel in axes:
        clean(panel)
        panel.grid(axis='both',color='#DEDEDE',lw=.6)
        # Independent messages from four links: points, not interpolation
        # between unrelated sender/receiver streams. All points retained.
        panel.scatter(x,y,s=1.1,c=color,alpha=.42,linewidths=0,rasterized=True)
        if panel.get_ylim()[0]<=mean<=panel.get_ylim()[1]:
            panel.axhline(mean,color=gray,ls='--',lw=1.25,zorder=3)
        if panel.get_ylim()[0]<=maximum<=panel.get_ylim()[1]:
            panel.errorbar(x[peak],maximum,yerr=clock_error_ms,fmt='D',ms=5,
                color=color,ecolor=color,capsize=4,elinewidth=1,lw=0,zorder=5)
    if broken:
        upper.spines['bottom'].set_visible(False)
        for panel,height in ((upper,0),(ax,1)):
            panel.plot([0,1],[height,height],transform=panel.transAxes,
                marker=[(-1,-.5),(1,.5)],markersize=7,ls='none',color='#777777',
                markeredgewidth=.9,clip_on=False)
    ax.set_xlim(0,200);ax.set_xticks([0,40,80,120,160,200])
    ax.set_xlabel('Time [s]',labelpad=12,fontsize=14)
    fig.text(.027,.44,'Communication delay [ms]',rotation=90,ha='center',va='center',fontsize=14)
    heading=fig.text(.54,.973,title,ha='center',va='top',fontsize=16)
    handles=[Line2D([],[],color=color,marker='.',ms=5,ls='none',label='Measured delay'),
        Line2D([],[],color=gray,ls='--',lw=1.3,label='Mean'),
        Line2D([],[],color=color,marker='D',ms=5,ls='none',label='Maximum')]
    legend=fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.54,.895),
        ncol=3,frameon=False,fontsize=11,handlelength=2,columnspacing=2.6)
    text=fig.text(.54,.785,
        f'Mean {mean:.2f} ms   |   Max {maximum:.2f} ms',
        ha='center',va='bottom',fontsize=12,color='#444444')
    # Keep the same clean header layout as the earlier computation plots.
    fig.canvas.draw();renderer=fig.canvas.get_renderer()
    boxes=[a.get_window_extent(renderer) for a in (heading,legend,text)]
    if any(a.overlaps(b) for i,a in enumerate(boxes) for b in boxes[i+1:]):
        raise ValueError('Header/legend/statistics overlap')
    if any(b.x0<fig.bbox.x0 or b.x1>fig.bbox.x1 for b in boxes):raise ValueError('Header clipped')
    if any(b.overlaps(panel.get_window_extent()) for b in boxes for panel in axes):
        raise ValueError('Header overlaps data')
    if broken and np.any((y>40)&(y<60)):raise ValueError('Axis break hides samples')
    return fig,dict(count=len(y),mean_ms=mean,median_ms=float(np.median(y)),
        p95_ms=float(np.percentile(y,95)),p99_ms=float(np.percentile(y,99)),
        max_ms=maximum,max_s=maximum/1000,maximum_time_s=float(x[peak]),
        clock_uncertainty_ms=clock_error_ms,
        maximum_clock_envelope_ms=[maximum-clock_error_ms,maximum+clock_error_ms],
        broken_y_interval_ms=[40,60] if broken else None,hidden_message_count=0,
        above_50_ms=int(np.count_nonzero(y>50)))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--log-dir',type=Path,required=True)
    p.add_argument('--summary',type=Path,required=True)
    p.add_argument('--transport-report',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();summary=json.loads(a.summary.read_text())
    report=json.loads(a.transport_report.read_text())
    if report['run']!=a.log_dir.name:raise ValueError('Run mismatch')
    if report['window_s']!=200 or summary['common_duration_s']<200:raise ValueError('Window mismatch')
    start=summary['actual_evaluation_start_ns']//1000
    blocks=[b for f in sorted(a.log_dir.glob('guidance_*.log')) for b in parse(f.read_text())]
    values,_=matched_delays([b for b in blocks if b['role']=='tx'],
        [b for b in blocks if b['role']=='rx'],0,report['clock']['midpoint_ns'],start,start+200_000_000)
    specs=[('PC_to_Pi','01_pc_to_raspberry_pi_communication_delay',
            'Communication Delay: PC to Raspberry Pi 5',lambda s,d:d==0),
           ('Pi_to_PC','02_raspberry_pi_to_pc_communication_delay',
            'Communication Delay: Raspberry Pi 5 to PC',lambda s,d:s==0)]
    series={}
    for name,_,_,select in specs:
        d=report['directions'][name]
        if not d['coverage']['complete_unique_delivery']:
            raise ValueError('Incomplete/duplicate delivery: do not export an unqualified full-run plot')
        rows=[r for (s,t),rr in values.items() if select(s,t) for r in rr]
        rows.sort(key=lambda r:(r['source_us'],r['tx_wall_ns'],r['rx_wall_ns']))
        x=np.asarray([(r['source_us']-start)/1e6 for r in rows])
        y=np.asarray([r['corrected_ms'] for r in rows])
        expected=d['corrected_send_to_callback']
        if len(y)!=expected['count'] or y.max()!=expected['max_ms']:
            raise ValueError('Plot samples differ from the validated transport report')
        if not np.isfinite(y).all():raise ValueError('Nonfinite latency')
        series[name]=(x,y,d['clock_uncertainty_ms'])
    upper_limit=max(75,5*np.ceil(max(y.max()+e for x,y,e in series.values())/5))
    a.output.mkdir(parents=True,exist_ok=True)
    pdf_dir=a.output/'output/pdf';pdf_dir.mkdir(parents=True,exist_ok=True)
    metrics={}
    for name,filename,title,_ in specs:
        x,y,error=series[name]
        fig,metrics[name]=make_figure(x,y,title,error,upper_limit)
        fig.savefig(a.output/(filename+'.png'),dpi=300)
        fig.savefig(a.output/(filename+'.svg'),dpi=300)
        fig.savefig(pdf_dir/(filename+'.pdf'),dpi=300)
        plt.close(fig)
    # Render both directions from the same complete data, rather than resizing
    # screenshots. Subfigures retain readable labels and identical axis scales.
    combined=plt.figure(figsize=(12,11.3))
    panels=combined.subfigures(2,1,hspace=.025)
    for panel,(name,_,title,_) in zip(panels,specs):
        x,y,error=series[name]
        _,combined_metrics=make_figure(x,y,title,error,upper_limit,fig=panel)
        if combined_metrics!=metrics[name]:
            raise ValueError('Combined plot changes measured values')
    combined_filename='03_bidirectional_communication_delay'
    combined.savefig(a.output/(combined_filename+'.png'),dpi=300)
    combined.savefig(a.output/(combined_filename+'.svg'),dpi=300)
    combined.savefig(pdf_dir/(combined_filename+'.pdf'),dpi=300)
    plt.close(combined)
    result=dict(run=report['run'],window_s=200,series=metrics,
        measured_endpoints=report['endpoints'],
        y_axis='System-clock-corrected ROS publish -> peer callback; Wi-Fi INCLUDED',
        x_axis='Trajectory reference timestamp relative to common Formation start; not used to calculate latency',
        samples='All matched messages; four PC->Pi or four Pi->PC links pooled per direction',
        uncertainty='One shared clock-offset envelope, NOT packet jitter or a statistical confidence interval; error bar shown at maximum',
        limitations=['No radio-only or guaranteed future delivery bound.',
            '40-60 ms empty interval omitted explicitly where valid; no measured values hidden.',
            'All measured points retained; dense scatter rasterized at 300 dpi in vector exports.'],
        runtime_changes=False)
    (a.output/'plot_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
