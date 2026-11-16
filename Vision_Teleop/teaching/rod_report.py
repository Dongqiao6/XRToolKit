"""Whole-attempt metrics for the approved rod task; never discard failed trials."""
import numpy as np
from .rod_task import axis_angle


def statistics(values,weights=None):
    if not values:return None
    values=np.asarray(values,float)
    if weights is None:weights=np.ones(len(values))
    order=np.argsort(values);v=values[order];w=np.asarray(weights,float)[order]
    c=np.cumsum(w)/np.sum(w)
    return dict(n=len(v),p95=float(v[min(np.searchsorted(c,.95),len(v)-1)]),
                median=float(v[min(np.searchsorted(c,.5),len(v)-1)]),max=float(v[-1]))


def intervals(rows,flags):
    result=[];start=None
    for i,(r,bad) in enumerate(zip(rows,flags)):
        if bad and start is None:start=i
        if start is not None and (not bad or i==len(rows)-1):
            end=i if bad else i-1
            until=rows[i]['time_s'] if not bad else rows[end]['time_s']+.01
            result.append(dict(start_s=rows[start]['elapsed_s'],end_s=rows[end]['elapsed_s'],
                               duration_s=until-rows[start]['time_s']))
            start=None
    return result


def lag_report(rows,criteria):
    """Causal interval matching; static/revisited targets are explicitly ambiguous.

    Bounds include acquisition timestamps. The conservative upper bound is used
    for the limit check; neither bound is asserted to be an exact measured lag.
    """
    results=[];history=[];last_match=-float('inf')
    for r in rows:
        now=r['time_s'];history=[h for h in history if now-h['time_s']<=criteria['lag_window_s']]
        if r['paused'] or not r['required_fresh'] or r['raw_m'] is None or r['raw_axis'] is None:
            results.append(dict(time_s=now,status='unavailable'));continue
        history.append(r)
        pos=np.asarray([h['raw_m'] for h in history]);axes=np.asarray([h['raw_axis'] for h in history])
        moving=(np.max(np.linalg.norm(pos-pos[-1],axis=1))>2*criteria['position_m'] or
                max(axis_angle(a,axes[-1]) for a in axes)>2*criteria['axis_deg'])
        if not moving:
            results.append(dict(time_s=now,status='static_or_insufficient_excursion'));continue
        good=[]
        for h in history:
            if h['phase']!=r['phase'] or h['time_s']<last_match:continue
            if (np.linalg.norm(np.asarray(h['raw_m'])-r['actual_m'])<=criteria['position_m'] and
                    axis_angle(h['raw_axis'],r['actual_axis'])<=criteria['axis_deg']):good.append(h)
        if not good:
            results.append(dict(time_s=now,status='unmatched_within_2s_or_spatial_error'));continue
        if any(b['time_s']-a['time_s']>.15 for a,b in zip(good,good[1:])):
            results.append(dict(time_s=now,status='revisited_ambiguous'));continue
        stamps=[]
        for h in good:
            valid=[h.get('position_source_s')]
            if h['phase']=='turn':valid.append(h.get('axis_source_s'))
            stamps.append(min(v for v in valid if v is not None) if any(v is not None for v in valid) else h['time_s'])
        last_match=good[0]['time_s']
        results.append(dict(time_s=now,status='matched_interval',lower_s=max(0.,now-max(stamps)),upper_s=max(0.,now-min(stamps))))
    matched=[r for r in results if r['status']=='matched_interval']
    stats=statistics([r['upper_s'] for r in matched])
    # Keep unmatchable moving samples as unknown upper bounds in the p95 test,
    # rather than dropping them or silently requiring every sample to match.
    moving=[r for r in results if r['status'] in
            ('matched_interval','unmatched_within_2s_or_spatial_error','revisited_ambiguous')]
    bounds=sorted(r.get('upper_s',float('inf')) for r in moving)
    bound=bounds[max(0,int(np.ceil(.95*len(bounds)))-1)] if bounds else float('inf')
    return dict(upper_bound_s=stats,status_counts={s:sum(r['status']==s for r in results) for s in sorted({r['status'] for r in results})},
                evaluation_p95_upper_s=bound if np.isfinite(bound) else None,
                pass_=bool(np.isfinite(bound) and bound<=criteria['lag_p95_s']),samples=results,
                limits='Tolerance-based causal lag intervals, not exact exposure-to-display latency. Unmatched motion is not deleted; pure spatial errors cannot be diagnosed as lag alone.')


def report_task(rows,events,task):
    c=task['criteria'];start=next((e['time_s'] for e in events if e['event']=='start' and e['accepted']),None)
    if not rows:
        faults=[e for e in events if e['event']=='fault']
        return dict(status='initialization_failed' if faults else 'not_started',human_task_pass=None,
                    failure_categories=['initialization_or_assembly'] if faults else [],events=events,criteria=c)
    weights=np.diff([r['time_s'] for r in rows]+[rows[-1]['time_s']+.01])
    # Time-weighted precision; invalid observations remain in freshness/attempt denominator.
    metrics={}
    for key in ('position_error_m','axis_error_deg','tip_error_m'):
        selected=[(r[key],w) for r,w in zip(rows,weights) if r['required_fresh'] and not r['paused'] and r[key] is not None]
        metrics[key]=statistics([v for v,w in selected],[w for v,w in selected])
    exceed=[bool(r['required_fresh'] and not r['paused'] and
                 ((r['position_error_m'] is not None and r['position_error_m']>c['position_m']) or
                  (r['axis_error_deg'] is not None and r['axis_error_deg']>c['axis_deg']))) for r in rows]
    excursions=intervals(rows,exceed)
    fresh=float(sum(w for r,w in zip(rows,weights) if r['required_fresh'])/sum(weights))
    path_failures=[dict(time_s=r['time_s'],actual=r['actual_path_failures'],raw=r['raw_path_failures']) for r in rows
                   if r['actual_path_failures'] or r['raw_path_failures'] or r.get('clipped')]
    stop=next((e['time_s'] for e in events if e['event']=='stop_request' and e['accepted']),None)
    held_since=None;confirmed=None;entered=None
    for r in rows:
        ok=(stop is not None and r['time_s']>=stop and r['phase']=='turn' and not r['paused'] and r['required_fresh'] and
            r['goal_tip_error_m']<=c['goal_tip_m'] and r['goal_axis_error_deg']<=c['axis_deg'] and r['corridor'] and
            not r['actual_path_failures'])
        if ok:
            if held_since is None:held_since=r['time_s']
            if r['time_s']-held_since>=c['hold_s'] and confirmed is None:confirmed=r['time_s'];entered=held_since
        else:held_since=None
    safety_failures=[e for e in events if e['event']=='fault' or (e['event']=='run_end' and e.get('reason'))]
    errors=[r for r in rows if r['safety'].get('violation')]
    final_event=events[-1] if events else {}
    aborted=bool(any(e['event']=='fault' for e in events) or
                 (final_event.get('event')=='run_end' and final_event.get('reason') not in (None,'user_exit_or_duration')))
    safe=not errors and not any(e.get('safety',{}).get('violation') for e in events)
    precision=bool(metrics['position_error_m'] and metrics['axis_error_deg'] and
        metrics['position_error_m']['p95']<=c['position_m'] and metrics['axis_error_deg']['p95']<=c['axis_deg'] and
        max([e['duration_s'] for e in excursions],default=0)<=c['max_excursion_s'])
    lag=lag_report(rows,c)
    auxiliary=any(e['event'] in ('pause','re_zero_resume') and e['accepted'] for e in events)
    elapsed=confirmed-start if confirmed is not None and start is not None else None
    catchup=entered-stop if entered is not None and stop is not None else None
    numeric=any(r.get('input_kind')=='numerical_fixture' for r in rows)
    categories=[]
    if fresh<c['fresh_fraction']:categories.append('observation_unavailable_or_stale')
    if any(r['raw_path_failures'] or r.get('clipped') for r in rows):categories.append('mapping_or_input_task_violation')
    if any(r['actual_path_failures'] for r in rows):categories.append('actual_path_requirement_violation')
    if not safe:categories.append('execution_safety_constraint')
    if any(e.get('exception_type')=='NoSolutionFound' for e in events):categories.append('local_solver_no_solution')
    if confirmed is None:categories.append('no_completed_solution_observed_not_global_infeasibility')
    complete=bool(not aborted and safe and not path_failures and confirmed is not None and
                  precision and fresh>=c['fresh_fraction'] and lag['pass_'] and catchup<=c['catchup_s'] and elapsed<=c['total_s'])
    remaining=rows[-1]['elapsed_s']
    stop_rows=[r for r in rows if stop is not None and r['time_s']>=stop and r['raw_axis'] is not None and r['raw_m'] is not None]
    stability=None
    remaining_bound=None
    if stop_rows:
        first=stop_rows[0]
        stability=dict(max_axis_change_deg=max(axis_angle(first['raw_axis'],r['raw_axis']) for r in stop_rows),
                       max_position_change_m=max(float(np.linalg.norm(np.asarray(first['raw_m'])-r['raw_m'])) for r in stop_rows),
                       caveat='Measured target stability after explicit stop marker, not anatomical ground truth')
        if first.get('command_m') is not None and first.get('command_axis') is not None:
            distance=float(np.linalg.norm(np.asarray(first['raw_m'])-first['command_m']))
            angle=axis_angle(first['raw_axis'],first['command_axis'])
            remaining_bound=dict(command_position_gap_m=distance,command_axis_gap_deg=angle,
                                 rate_only_lower_bound_s=max(distance/.1,np.radians(angle)/.5))
    chain={}
    for name,left,right,angular in [
        ('raw_to_command_position_m','raw_m','command_m',False),
        ('raw_to_command_axis_deg','raw_axis','command_axis',True),
        ('ik_to_command_position_m','ik_m','command_m',False),
        ('ik_to_command_axis_deg','ik_axis','command_axis',True),
        ('actual_to_ik_position_m','actual_m','ik_m',False),
        ('actual_to_ik_axis_deg','actual_axis','ik_axis',True)]:
        values=[axis_angle(r[left],r[right]) if angular else float(np.linalg.norm(np.asarray(r[left])-r[right]))
                for r in rows if r.get(left) is not None and r.get(right) is not None]
        chain[name]=statistics(values)
    return dict(status='complete' if complete else 'failed_or_incomplete',human_task_pass=None if numeric else complete,
        numerical_fixture_only=numeric,failure_categories=categories,criteria=c,precision=metrics,excursions=excursions,
        freshness_fraction=fresh,path_failures=path_failures,safety_records=safety_failures,
        minimum_distance_m=min(r['safety']['minimum_distance_m'] for r in rows),
        geometry_confirmed_s=confirmed,catchup_s=catchup,task_total_s=elapsed,observed_duration_s=remaining,
        stop_target_stability=stability,stop_command_rate_bound=remaining_bound,error_chain=chain,
        lag=lag,assisted=auxiliary,continuous_completion=complete and not auxiliary,
        components=dict(precision=precision,freshness=fresh>=c['fresh_fraction'],path=not path_failures,safety=safe,
                        endpoint=confirmed is not None,lag=lag['pass_'],no_run_error=not aborted),events=events,
        limits='New axis-task criteria never override previous full-orientation free-motion failures. No task-specific camera recording has been inferred from numerical fixtures.')
