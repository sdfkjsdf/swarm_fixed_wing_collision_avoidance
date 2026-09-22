#!/usr/bin/env python3
"""Report plots from offline deadline JSON only; no live ROS or controller imports."""
import argparse
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
import numpy as np

COLORS = ['#315C8A', '#C97E35']
RED = '#BA3535'


def clean(ax):
    ax.grid(axis='y', color='#E3E6E8', lw=.55)
    ax.set_axisbelow(True)
    for s in ['top','right']:
        ax.spines[s].set_visible(False)
    for s in ['left','bottom']:
        ax.spines[s].set_color('#7B858D')
        ax.spines[s].set_linewidth(.6)
    ax.tick_params(length=3)


def broken_axes(title, low_max=5, xlabel='공통 비행 시작 후 시간 [s]'):
    fig=plt.figure(figsize=(9.5,4.9))
    gs=fig.add_gridspec(2,1,height_ratios=[.8,4],left=.095,right=.975,
        top=.72,bottom=.16,hspace=.1)
    upper=fig.add_subplot(gs[0]); ax=fig.add_subplot(gs[1],sharex=upper)
    clean(upper); clean(ax)
    upper.spines['bottom'].set_visible(False)
    upper.set_ylim(48,55); upper.set_yticks([50])
    upper.axhline(50,color=RED,lw=1.3)
    upper.tick_params(axis='x',bottom=False,labelbottom=False)
    ax.set_ylim(0,low_max)
    ax.set_ylabel('소요 시간 [ms]'); ax.set_xlabel(xlabel)
    for target,y in [(upper,0),(ax,1)]:
        target.plot([0,1],[y,y],transform=target.transAxes,marker=[(-1,-.5),(1,.5)],
            markersize=7,linestyle='none',color='#6D757C',markeredgewidth=.8,clip_on=False)
    fig.text(.095,.94,title,fontsize=15,weight='bold',ha='left')
    fig.text(.095,.04,'세로축 생략 표시: 측정 범위를 확대하고 50 ms 기준선을 함께 표시',fontsize=9,color='#555E66')
    return fig,ax


def save(fig,out,name):
    fig.savefig(out/(name+'.png'),dpi=250)
    fig.savefig(out/(name+'.svg'))
    plt.close(fig)


def english_report(doc, out):
    """One preserved run per figure, matching the report's existing visual style."""
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12})
    metrics = {}
    for key, name, title in [
        ('trajectory_compute', '01_seven_candidate_trajectory_prediction_time',
         'Seven-Candidate Trajectory Prediction Time'),
        ('ad_monitor_compute', '02_avoidance_decision_activation_computation_time',
         'Avoidance Decision and Activation Computation Time')]:
        xy = np.asarray(doc['series'][key])
        mean, maximum = float(xy[:,1].mean()), float(xy[:,1].max())
        assert maximum < 4, 'Expand the measured y-axis before plotting larger values'
        fig = plt.figure(figsize=(12,5.3))
        grid = fig.add_gridspec(2,1,height_ratios=[.8,3.5],left=.10,right=.985,
            top=.735,bottom=.18,hspace=.14)
        upper = fig.add_subplot(grid[0])
        ax = fig.add_subplot(grid[1], sharex=upper)
        for panel in [upper, ax]:
            clean(panel)
        upper.grid(False)
        upper.spines['bottom'].set_visible(False)
        upper.tick_params(axis='x',bottom=False,labelbottom=False)
        upper.set_ylim(48,53); upper.set_yticks([50])
        upper.axhline(50,color=RED,lw=1.4)
        ax.plot(xy[:,0],xy[:,1],color='#27688F',lw=.6)
        ax.axhline(mean,color='#666666',ls='--',lw=1.2)
        peak = int(np.argmax(xy[:,1]))
        ax.plot(*xy[peak],marker='D',ms=5,color='#27688F',clip_on=False,zorder=5)
        ax.set_ylim(0,4); ax.set_yticks([0,1,2,3,4])
        ax.set_xlim(0,200); ax.set_xticks([0,30,60,90,120,150,180,200])
        ax.grid(axis='both',color='#DEDEDE',lw=.6)
        ax.set_xlabel('Time from common Formation start [s]',labelpad=12,fontsize=14)
        ax.set_ylabel('Computation time [ms]',labelpad=10,fontsize=14)
        for panel,y in [(upper,0),(ax,1)]:
            panel.plot([0,1],[y,y],transform=panel.transAxes,
                marker=[(-1,-.5),(1,.5)],markersize=7,linestyle='none',
                color='#777777',markeredgewidth=.9,clip_on=False)
        heading=fig.text(.54,.97,'Raspberry Pi 5 | '+title,ha='center',va='top',fontsize=16)
        handles=[Line2D([],[],color='#27688F',lw=1.2,label='Measured computation'),
            Line2D([],[],color='#666666',ls='--',lw=1.4,label='Mean'),
            Line2D([],[],color=RED,lw=1.4,label='Computation-time target'),
            Line2D([],[],color='#27688F',marker='D',ms=5,ls='none',label='Maximum')]
        legend=fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.54,.89),
            ncol=4,frameon=False,fontsize=11,handlelength=2,columnspacing=1.7)
        stats_text=fig.text(.54,.78,
            f'Mean {mean:.2f} ms   |   Max {maximum:.2f} ms   |   Computation target 50 ms (20 Hz)',
            ha='center',va='bottom',fontsize=12,color='#444444')
        fig.canvas.draw()
        renderer=fig.canvas.get_renderer()
        boxes=[a.get_window_extent(renderer) for a in [heading,legend,stats_text]]
        assert not any(a.overlaps(b) for i,a in enumerate(boxes) for b in boxes[i+1:])
        assert all(not legend.get_window_extent(renderer).overlaps(p.get_window_extent(renderer)) for p in [upper,ax])
        assert all(b.x0>=0 and b.x1<=fig.bbox.width for b in boxes), 'Header overflow'
        save(fig,out,name)
        metrics[key]=dict(count=len(xy),mean_ms=mean,max_ms=maximum,target_ms=50,
            title=title,above_target_count=int(np.sum(xy[:,1]>50)))
    (out/'english_report_summary.json').write_text(json.dumps(dict(
        run=doc['run'],source_commit=doc['source_commit'],window_s=doc['window_s'],
        vehicle=doc['vehicle'],metrics=metrics,
        scope='Pure stage computation; activation means software logic, not physical maneuver execution'),indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--measurements',type=Path,nargs=2,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--english-report',action='store_true',
                   help='Export two English report figures from the latest supplied run only')
    args=p.parse_args(); out=args.output; out.mkdir(parents=True,exist_ok=True)
    docs=[json.loads(path.read_text()) for path in args.measurements]
    if args.english_report:
        plt.rcParams['path.simplify'] = False
        english_report(docs[-1], out)
        return
    regular=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    bold=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc')
    font_manager.fontManager.addfont(str(regular))
    if bold.exists():
        font_manager.fontManager.addfont(str(bold))
    family=font_manager.FontProperties(fname=str(regular)).get_name()
    plt.rcParams.update({'font.family':family,'font.size':11,'axes.unicode_minus':False,
        'svg.fonttype':'none','path.simplify':False})
    aggregate=dict(source_commit='050c549',runs=[d['run'] for d in docs],
                   windows_s=[d['window_s'] for d in docs],vehicle=0)
    for key,name,title in [
        ('trajectory_compute','01_trajectory_compute','7개 후보 궤적 갱신 — 순수 연산시간'),
        ('ad_monitor_compute','02_ad_monitor_compute','AD 감시·회피 발동/종료 판단 — 순수 연산시간')]:
        fig,ax=broken_axes(title)
        handles=[]
        all_values=[]
        for i,d in enumerate(docs):
            xy=np.asarray(d['series'][key]); all_values.extend(xy[:,1])
            ax.plot(xy[:,0],xy[:,1],lw=.5,color=COLORS[i],alpha=.85)
            peak=np.argmax(xy[:,1]); ax.plot(*xy[peak],marker='D',ms=4,color=COLORS[i])
            handles.append(Line2D([],[],color=COLORS[i],lw=1.3,
                label=f'{i+1}차: 최대 {xy[:,1].max():.2f} ms'))
        handles.append(Line2D([],[],color=RED,lw=1.3,label='50 ms 연산 예산'))
        fig.legend(handles=handles,loc='upper left',bbox_to_anchor=(.085,.88),ncol=3,frameon=False,fontsize=10)
        ax.set_xlim(0,200); ax.set_xticks(np.arange(0,201,25))
        aggregate[key]=dict(count=len(all_values),max_ms=float(max(all_values)),above_50_ms=int(sum(y>50 for y in all_values)))
        save(fig,out,name)

    events=[dict(e,run=i+1) for i,d in enumerate(docs) for e in d['activation_events']]
    assert all(not d['unmatched_activations'] for d in docs)
    fig,ax=broken_axes('회피 시작 판단 → 첫 회피 ROS 명령 발행 완료',14,'Pi 회피 시작 사건 (2회 실험, 총 6회)')
    xs=np.arange(len(events)); comp=np.array([e['ad_compute_ms'] for e in events])
    total=np.array([e['ad_start_to_publish_return_ms'] for e in events])
    ax.bar(xs,comp,width=.58,color=COLORS[0],label='AD 감시·판단')
    ax.bar(xs,total-comp,bottom=comp,width=.58,color='#9CB6CD',label='결과 전달 대기·ROS 발행')
    for x,y in zip(xs,total):
        ax.text(x,y+.45,f'{y:.2f} ms',ha='center',fontsize=11)
    ax.set_xticks(xs,[f"{e['run']}차 · {i%3+1}번\n"+('자체 AD' if e['local_trigger'] else '그룹 요청') for i,e in enumerate(events)],fontsize=10)
    ax.set_xlim(-.6,len(events)-.4)
    handles,labels=ax.get_legend_handles_labels()
    handles.append(Line2D([],[],color=RED,lw=1.3)); labels.append('50 ms 기준')
    fig.legend(handles,labels,loc='upper left',bbox_to_anchor=(.085,.88),ncol=3,frameon=False,fontsize=10)
    fig.text(.975,.04,'명령의 PX4 수신·기체 반응 시간은 제외',ha='right',fontsize=9,color='#555E66')
    # Move the axis-break note away from the endpoint definition.
    fig.texts[1].set_text('세로축 생략 표시')
    aggregate['activation_events']=events
    aggregate['ad_start_to_publish_return']=dict(count=len(events),max_ms=float(total.max()),above_50_ms=int(np.sum(total>50)))
    save(fig,out,'03_ad_to_command')

    fig,ax=plt.subplots(figsize=(9.5,4.9))
    fig.subplots_adjust(left=.095,right=.975,bottom=.17,top=.71)
    clean(ax)
    handles=[]
    for i,d in enumerate(docs):
        xy=np.asarray(d['series']['pc_full_batch_arrival_gap'])
        ax.plot(xy[:,0],xy[:,1],color=COLORS[i],lw=.6,alpha=.8)
        handles.append(Line2D([],[],color=COLORS[i],lw=1.3,
            label=f'{i+1}차: 최대 {xy[:,1].max():.2f} ms'))
    ax.axhline(50,color=RED,lw=1.1)
    handles.append(Line2D([],[],color=RED,lw=1.1,label='50 ms 갱신 간격 기준'))
    fig.legend(handles=handles,loc='upper left',bbox_to_anchor=(.085,.88),ncol=3,frameon=False,fontsize=10)
    ax.set_xlim(0,200); ax.set_ylim(0,565)
    ax.set_xlabel('공통 비행 시작 후 시간 [s]'); ax.set_ylabel('연속 도착 간격 [ms]')
    fig.text(.095,.94,'Pi 궤적 7개 세트가 PC 기록기에 도착한 간격',fontsize=15,weight='bold')
    fig.text(.095,.045,'연산시간·순수 통신지연이 아님. 세트 누락과 기록기 지연을 포함할 수 있음.',fontsize=9,color='#555E66')
    save(fig,out,'04_pc_batch_arrival_gap')

    fig,ax=plt.subplots(figsize=(9.5,4.9))
    fig.subplots_adjust(left=.105,right=.975,bottom=.19,top=.71)
    clean(ax); handles=[]
    for i,d in enumerate(docs):
        xy=np.array([[r['time_s'],r['pi_callback_minus_pc_bag_ms']] for r in d['communication_samples']])
        ax.plot(xy[:,0],xy[:,1],color=COLORS[i],lw=.55,alpha=.8)
        handles.append(Line2D([],[],color=COLORS[i],lw=1.3,
            label=f'{i+1}차: 중앙값 {np.median(xy[:,1]):.2f} / 최대 {xy[:,1].max():.2f} ms'))
    ax.axhline(0,color='#666F78',lw=.8)
    ax.set_xlim(0,200); ax.set_ylim(-20,295)
    ax.set_xlabel('공통 비행 시작 후 시간 [s]'); ax.set_ylabel('Pi 콜백 시각 − PC 기록 시각 [ms]')
    fig.text(.105,.94,'동일 상태 메시지의 PC·Pi 관측 시각 차이',fontsize=15,weight='bold')
    fig.legend(handles=handles,loc='upper left',bbox_to_anchor=(.095,.88),ncol=1,frameon=False,fontsize=10)
    fig.text(.105,.065,'PC·Pi 시계 차이와 PC 기록기 지연이 미보정된 부호 있는 관측값.',fontsize=9,color='#555E66')
    fig.text(.105,.025,'음수는 전송시간이 음수라는 뜻이 아님. 이 값을 일방향 통신지연으로 사용하지 않음.',fontsize=9,color='#555E66')
    save(fig,out,'05_cross_host_arrival_proxy')
    aggregate['communication_one_way_delay_ms']=None
    aggregate['communication_delay_status']='Not identifiable: no calibrated matching publisher->subscriber timestamps'
    aggregate['within_50ms_scope']='Measured Pi AD-pass start to first matching ROS publish return (6 starts), excluding network and PX4 actuation'
    aggregate['new_runtime_instrumentation']=False
    (out/'plot_summary.json').write_text(json.dumps(aggregate,indent=2)+'\n')
    for i,path in enumerate(args.measurements):
        shutil.copy2(path,out/f'measurements_run_{i+1}.json')
    print(json.dumps({k:v for k,v in aggregate.items() if k!='activation_events'},indent=2))


if __name__=='__main__':
    main()
