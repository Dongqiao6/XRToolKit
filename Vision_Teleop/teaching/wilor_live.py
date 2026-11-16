"""Current camera -> original dual WiLoR/RGB-D -> Franka simulation entry."""
from collections import Counter, deque
from datetime import datetime
import io
import hashlib
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from .capture import DualCapture
from .calibration_state import load_bound_calibration
from .hand_tracking import TrackingWorker, VideoHand, select_right_hand
from .live import RecordedCapture
from .position_motion import PositionMotion
from .replay import SmoothTarget, preposition, task_anchor
from .wilor_stream import WiLoRStream, observation_fresh, right_hand_points
from .direction_setup import DirectionSetup
from .live_display import draw_workspace, camera_values, apply_camera
from .teleop_video import TeleopVideo
from .wilor_timing import ControlWait, next_deadline


def position_observation(frame, now, max_age):
    if frame is None or not observation_fresh(frame.time_ms/1000, now, max_age):
        return None
    hand = frame.metadata['hand']
    result = hand.get('rgbd_position', {})
    if right_hand_points(hand) is None or not result.get('valid'):
        return None
    p = np.asarray(result['position_m'], float)
    return p if p.shape == (3,) and np.isfinite(p).all() else None


def position_reason(frame, now, max_age):
    if frame is None: return 'no_frame'
    if not observation_fresh(frame.time_ms/1000, now, max_age): return 'stale_frame'
    hand = frame.metadata['hand']
    if right_hand_points(hand) is None:
        return hand.get('selection', {}).get('reason', 'invalid_right_hand')
    if not hand.get('rgbd_position', {}).get('valid'):
        return 'depth:' + hand.get('rgbd_position', {}).get('reason', 'missing')
    return 'valid' if position_observation(frame, now, max_age) is not None else 'nonfinite_position'


def live_status(active, state, reason, mode, pose_ok):
    if not active:
        if state == '制动': return '暂停：制动中'
        if reason != 'valid': return '暂停：位置无效（' + reason + '）'
        if mode == 'follow' and not pose_ok: return '暂停：位置有效，等待姿态'
        return '暂停：位置有效，空格归零开始'
    if reason != 'valid': return state + '：位置无效（' + reason + '）'
    if mode == 'fixed': return state + '；工具固定朝下'
    return state + ('；姿态有效' if pose_ok else '；姿态缺失或过期')


def gain_reference(reference, point, origin, rotation, old_gain, new_gain):
    """Keep the current unbounded target continuous when the user changes gain."""
    return reference+(old_gain-new_gain)*(rotation@(point-origin))


def run(root, config, dry_run=None, headless=False, duration=None, auto_start=False, orientation_task='fixed',
        task=None,robot='franka_fr3',mount_degrees=0):
    import mujoco
    import mujoco.viewer
    from PIL import Image, ImageDraw, ImageFont
    from .robot import SimArm
    from .rod_task import RodTask,load_task,axis_rotation
    rod_config=load_task(root) if task=='rod' else None
    task_tracker=RodTask(rod_config) if rod_config else None
    if task not in (None,'rod'):raise ValueError('Unknown live task')
    if task and dry_run is None and config.get('calibration_review_required',False):
        raise ValueError('Camera setup changed; recalibration review required before rod live input')
    if auto_start and dry_run is None:
        raise ValueError('Auto-start is only for explicitly labelled recorded-input tests')
    if orientation_task not in ('free', 'follow', 'fixed'):
        raise ValueError('WiLoR live supports fixed/free/follow')
    clock_name = 'perf_counter' if orientation_task == 'follow' else 'monotonic'
    clock = getattr(time, clock_name)
    clock_ns = getattr(time, clock_name+'_ns')
    calibration, _, _ = load_bound_calibration(root, config)
    cams = config['cameras']
    if calibration['camera_serials'] != [cams[k+'_serial'] for k in ('left','right')] or calibration['image_size'] != [cams['width'],cams['height']]:
        raise ValueError('Camera roles or image size differ from bound calibration')
    if config.get('position_input') != 'primary_rgbd' or not cams.get('depth') or cams.get('depth_role') != 'left':
        raise ValueError('Current mainline requires measured left RGB-D; no source substitution')
    settings = yaml.safe_load((root/'config/offline_mapping.yaml').read_text())['replay']
    live_mapping=config.get('wilor_live',{})
    if orientation_task == 'follow':
        compact=np.asarray(live_mapping['follow_anchor_position_world_m'],float)
        if compact.shape!=(3,) or not np.isfinite(compact).all():
            raise ValueError('Finite approved follow anchor required')
        settings['task_anchor']['position_world_m']=compact.tolist()
    if 'camera_to_base_rotation' in live_mapping:
        a=np.asarray(live_mapping['camera_to_base_rotation'],float)
        if a.shape!=(3,3) or not np.allclose(a.T@a,np.eye(3)) or not np.isclose(np.linalg.det(a),1):
            raise ValueError('Human/base mapping must be a proper rotation')
        settings['camera_to_task_rotation']=a.tolist()
    saved_path=root/'config/wilor_live_setup.json'
    saved=None
    if dry_run is None or saved_path.exists():
        saved=json.loads(saved_path.read_text())
        a=np.asarray(saved['camera_to_base_rotation'],float)
        if (saved['calibration_sha256']!=config['calibration_sha256'] or a.shape!=(3,3)
                or not np.allclose(a.T@a,np.eye(3)) or not np.isclose(np.linalg.det(a),1)
                or not 0<saved['scale_min']<=saved['position_scale']<=saved['scale_max']):
            raise ValueError('Saved accepted mapping is invalid or belongs to another calibration')
        settings['camera_to_task_rotation']=a.tolist()
        settings['position_scale']=saved['position_scale']
    fusion = yaml.safe_load((root/'config/dual_wilor.yaml').read_text())
    # Qualified on paired real crops; fixed position baseline keeps original arithmetic.
    fusion['matmul_precision'] = 'high' if orientation_task == 'follow' else 'highest'
    fusion['crop_blur'] = 'opencv' if orientation_task == 'follow' else 'upstream'
    fusion['timestamp_clock'] = clock_name
    fusion['pair_policy'] = 'nearest_already_available' if orientation_task == 'follow' else 'past_right_at_left_time'
    presentation_path=root/'config/wilor_presentation.json'
    presentation=json.loads(presentation_path.read_text())
    workspace_min=np.asarray(presentation['workspace_min_world_m'],float)
    workspace_max=np.asarray(presentation['workspace_max_world_m'],float)
    from .live_display import workspace_edges
    workspace_edges(workspace_min,workspace_max)  # Validate before hardware.
    anchor=np.asarray(settings['task_anchor']['position_world_m'])
    if np.any(anchor<workspace_min) or np.any(anchor>workspace_max):
        raise ValueError('Initial TCP must be inside the configured workspace')
    output = root/'results/wilor_live'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    (output/'manifest.json').write_text(json.dumps(dict(mode='recorded_source_realtime_test' if dry_run else 'live_cameras',
        recording='user_R_start_stop_only', presentation=presentation, robot=robot+'_simulation', orientation_task=orientation_task,
        task=rod_config,mount_degrees=mount_degrees,
        settings=settings, fusion=fusion, saved_mapping=saved, calibration_sha256=config['calibration_sha256'],
        recorded_source=str(Path(dry_run).resolve()) if dry_run else None,
        requested_duration_s=duration,
        host_clock=dict(name=clock_name, **vars(time.get_clock_info(clock_name))),
        source_sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [root/'teaching/wilor_live.py',root/'teaching/wilor_stream.py',root/'teaching/wilor_crop.py',root/'teaching/wilor_timing.py',root/'scripts/wilor_stream_worker.py',root/'scripts/teach.py',root/'teaching/robot.py',root/'teaching/position_motion.py',root/'teaching/direction_setup.py',root/'teaching/capture.py',root/'teaching/live.py',root/'teaching/hand_tracking.py']},
        limits='No video until user presses R. Host arrival age is not exposure-to-photon latency. Original float32 WiLoR, latest-frame scheduling; position independent measured left depth.'), indent=2))
    print('RESULTS '+str(output), flush=True)
    if task_tracker:
        task_tracker.event_file=output/'task_events.jsonl'
        task_sources=[root/'config/rod_task.json',*[root/'teaching'/n for n in
                     ('rod_scene.py','rod_task.py','rod_report.py','gripper.py','position_priority.py','tool_axis.py')]]
        (output/'task_source_sha256.json').write_text(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in task_sources},indent=2))
    stream = WiLoRStream(root, calibration, fusion, output)
    source = None
    start = None
    workers = {}
    stopping = threading.Event()
    sim_lock = threading.RLock()
    control_thread = None
    control_wait = None
    control_metrics = deque(maxlen=200000)
    events = Counter()
    shared = dict(active=False, engage=False, mode=orientation_task, state='准备模型', error=None, clipped=False)
    task_keys=deque()
    raw_recorders=[]
    raw_current=None
    record_lock=threading.Lock()
    arm=None
    setup=None  # Accepted setup is loaded; no calibration UI in normal teleoperation.
    scale_min=saved['scale_min'] if saved else settings['position_scale']/3
    scale_max=saved['scale_max'] if saved else settings['position_scale']
    shared['pending_scale']=None
    native_keys=deque()
    setup_notice='每个方向单独记录起点和终点；舒适移动至少10cm'
    clip=None
    clip_renderer=None
    timer = None
    if __import__('os').name == 'nt':
        import ctypes
        timer=ctypes.WinDLL('winmm')
        timer.timeBeginPeriod(1)
    try:
        arm = SimArm(robot, mount_degrees=mount_degrees, with_gripper=True,rod_task=rod_config)
        arm.model.vis.global_.offwidth=max(arm.model.vis.global_.offwidth,1280)
        arm.model.vis.global_.offheight=max(arm.model.vis.global_.offheight,720)
        home_p, home_r = arm.pose()
        anchor_p, anchor_r = task_anchor(settings['task_anchor'], home_p, home_r)
        target = SmoothTarget(home_p, home_r, settings)
        if task_tracker:
            if not np.allclose(anchor_p,rod_config['start_m']):raise ValueError('Approved rod task start and live anchor differ')
            (output/'task_config.json').write_text(json.dumps(rod_config,indent=2))
            (output/'collision_pairs.json').write_text(json.dumps(arm.task_safety.audit,indent=2))
            mujoco.mj_saveModel(arm.model,str(output/'assembled.mjb'),None)
            arm.task_safety.check_initial()
            with (output/'preposition.jsonl').open('w') as log:
                prep = preposition(arm, target, anchor_p, anchor_r, settings['preposition'], log)
        else:
            prep = preposition(arm, target, anchor_p, anchor_r, settings['preposition'], io.StringIO())
        (output/'preposition_summary.json').write_text(json.dumps(prep,indent=2))
        if prep['status'] != 'ready':
            raise RuntimeError('Robot preposition failed')
        arm.posture.set_target_from_configuration(arm.configuration)
        arm.position_only = orientation_task == 'free'
        arm.position_priority = orientation_task == 'follow'
        arm.axis_priority = task_tracker is not None
        simulation_spec = mujoco.mjtState.mjSTATE_INTEGRATION
        simulation_state = np.empty(mujoco.mj_stateSize(arm.model,simulation_spec))
        (output/'simulation_reference.json').write_text(json.dumps(dict(
            integration_state_spec=int(simulation_spec),posture_target_q=arm.posture.target_q.tolist(),
            state_phase='after_each_logged_control_step',limits='Offline intervention restart only; not a new observation')))
        motion = PositionMotion(target.position, settings)
        start_wait = clock()
        while not stream.ready.wait(.1):
            stream.check()
            if clock()-start_wait > 120:
                raise RuntimeError('WiLoR model startup timeout')
        stream.check()
        print('WILOR_READY '+json.dumps(stream.info), flush=True)
        def completed(role, frame):
            stream.submit(role, frame)
            hand = frame.metadata['hand']
            diagnostic = dict(role=role, frame_id=frame.identity, host_time_ms=frame.time_ms,
                selection=hand.get('selection'), depth_reason=hand.get('rgbd_position', {}).get('reason'),
                depth_valid=hand.get('rgbd_position', {}).get('valid'), inference_ms=hand['inference_ms'])
            # Separate per-camera files have one writer and retain every processed frame.
            with (output/('input_'+role+'.jsonl')).open('a', encoding='utf-8') as log:
                log.write(json.dumps(diagnostic)+'\n')
        for role in ('left', 'right'):
            workers[role] = TrackingWorker(role, lambda: VideoHand(root/config['live']['hand_model']),
                completed, lambda row: None, config['rgbd_position'] if role == 'left' else None,
                selector=select_right_hand, clock_ns=clock_ns)
        for worker in workers.values():
            if not worker.ready.wait(30):
                raise RuntimeError('Detector startup timeout')
            worker.check()
        def on_frame(frame):
            role = frame.metadata['camera_role']
            if frame.metadata['camera_serial'] != cams[role+'_serial'] or list(frame.image.shape[1::-1]) != calibration['image_size']:
                raise ValueError('Unexpected camera frame identity')
            workers[role].submit(frame)
            if task_tracker:
                with record_lock:
                    if raw_current is not None:raw_current.frame(frame)
        source = (RecordedCapture(Path(dry_run), cams, lambda row: None, on_frame, duration, clock_ns=clock_ns,
                                  precise_wait=orientation_task=='follow') if dry_run else
                  DualCapture(cams, event=lambda row: None, on_frame=on_frame, clock_ns=clock_ns))
        source.start()
        start = clock()
        print('CAPTURE_READY '+('recorded source' if dry_run else 'live cameras'), flush=True)
        shared['state'] = '暂停：右手放在舒适中心，空格开始'

        control_wait = ControlWait() if orientation_task == 'follow' else None

        def control():
            p0 = r0 = robot_reference = robot_rotation = None
            deadline = time.perf_counter()
            auto_engaged = False
            try:
                while not stopping.is_set():
                    compute_start = time.perf_counter()
                    now = clock()
                    frames, pose = stream.snapshot()
                    point = position_observation(frames['left'], now, settings['stale_after_s'])
                    reason = position_reason(frames['left'], now, settings['stale_after_s'])
                    pose_ok = bool(pose and pose['valid'] and observation_fresh(pose['stamp'], now, settings['stale_after_s']))
                    with sim_lock:
                        pending=shared['pending_scale']
                        if pending is not None and task_tracker is not None:
                            shared['pending_scale']=None
                            task_tracker.event('scale_change_rejected',now,False)
                        elif pending is not None and (not shared['active'] or point is not None):
                            new_gain=float(np.clip(pending,scale_min,scale_max))
                            if shared['active']:
                                robot_reference=gain_reference(robot_reference,point,p0,
                                    np.asarray(settings['camera_to_task_rotation']),settings['position_scale'],new_gain)
                            settings['position_scale']=new_gain
                            shared['pending_scale']=None
                            with (output/'scale_changes.jsonl').open('a') as changes:
                                changes.write(json.dumps(dict(scale=new_gain,active=shared['active'],elapsed_s=now-start))+'\n')
                        if setup is not None and not setup.complete:
                            setup.add(frames['left'].time_ms/1000 if frames['left'] else now,point)
                        needs_pose=shared['mode']=='follow' and (task_tracker is None or task_tracker.phase=='turn')
                        if auto_start and not auto_engaged and point is not None and (not needs_pose or pose_ok):
                            shared['engage'] = True
                            auto_engaged = True
                        if shared['engage']:
                            shared['engage'] = False
                            ready=(setup is None or setup.complete) and point is not None and (not needs_pose or pose_ok) and np.linalg.norm(motion.velocity)<1e-5
                            if ready and task_tracker:
                                ready=task_tracker.engage(now,arm.pose()[0],point is not None,pose_ok)
                            if ready:
                                p0 = point.copy()
                                robot_reference = motion.position.copy()
                                robot_rotation = target.rotation.copy()
                                r0 = np.asarray(pose['rotation']) if pose_ok else None
                                shared['active'] = True
                                events['engagements'] += 1
                                if task_tracker:
                                    task_tracker.event('mapping_reference',now,position_m=p0.tolist(),
                                        robot_reference_m=robot_reference.tolist(),
                                        human_rotation=None if r0 is None else r0.tolist(),robot_rotation=robot_rotation.tolist())
                            else:
                                if task_tracker:
                                    task_tracker.event('engage_rejected',now,False,reason='input_or_braking_not_ready')
                                shared['state'] = '请等手部位置有效、制动停止后再按空格'
                        if task_tracker:
                            while task_keys:
                                key=task_keys.popleft()
                                if key=='turn' and task_tracker.request_turn(now,arm.pose()[0],point is not None,pose_ok):
                                    r0=np.asarray(pose['rotation']).copy()
                                    robot_rotation=target.rotation.copy()
                                    task_tracker.event('turn_rotation_reference',now,human_rotation=r0.tolist(),robot_rotation=robot_rotation.tolist())
                                elif key=='finish':task_tracker.request_finish(now)
                                elif key=='confirm' and task_tracker.confirm(now):
                                    shared['active']=False;stopping.set()
                                elif key=='correction':task_tracker.event('user_correction',now)
                        desired = None
                        raw_task=None
                        rotation = None
                        shared['clipped'] = False
                        if shared['active'] and point is not None:
                            a = np.asarray(settings['camera_to_task_rotation'])
                            raw = robot_reference + settings['position_scale']*(a@(point-p0))
                            raw_task=raw.copy()
                            desired = np.clip(raw, workspace_min, workspace_max)
                            shared['clipped'] = bool(np.any(np.abs(raw-desired)>1e-10))
                            if task_tracker and task_tracker.phase=='transport':
                                rotation=axis_rotation(target.rotation,np.asarray(rod_config['transport_axis']))
                                target.update(None,rotation,.01)
                            elif shared['mode']=='follow' and pose_ok and r0 is not None:
                                rotation = a@(np.asarray(pose['rotation'])@r0.T)@a.T@robot_rotation
                                if task_tracker:rotation=axis_rotation(target.rotation,rotation[:,2])
                                target.update(None, rotation, .01)
                            elif task_tracker and task_tracker.phase=='turn':
                                # No position continuation with stale turn orientation.
                                desired=None
                        if shared['mode']=='fixed':
                            target.rotation=anchor_r.copy()  # Tool +Z approaches world -Z.
                        commanded = motion.update(desired, .01)
                        # Braking may overshoot a clipped goal. Reject, never conceal it.
                        if np.any(commanded<workspace_min-1e-7) or np.any(commanded>workspace_max+1e-7):
                            raise RuntimeError('Position braking reached workspace boundary; simulation stopped')
                        arm.set_target(commanded, target.rotation)
                        velocity = arm.step(.01)
                        state = ('跟随位置' if desired is not None else '制动' if np.linalg.norm(motion.velocity)>1e-5 else '保持')
                        shared['state'] = live_status(shared['active'], state, reason, shared['mode'], pose_ok)
                        events[state] += 1
                        if shared['clipped']: events['clipped_control_steps'] += 1
                        actual, actual_rotation = arm.pose()
                        if dry_run is not None:
                            mujoco.mj_getState(arm.model,arm.data,simulation_state,simulation_spec)
                        observed_rotation = None
                        if shared['active'] and shared['mode']=='follow' and r0 is not None and pose and pose['valid']:
                            a = np.asarray(settings['camera_to_task_rotation'])
                            observed_rotation = a@(np.asarray(pose['rotation'])@r0.T)@a.T@robot_rotation
                        ik_site = arm.configuration.data.site(arm.site)
                        if task_tracker:
                            task_axis=(None if observed_rotation is None else observed_rotation[:,2])
                            task_row=task_tracker.observe(now,actual,actual_rotation,raw_task,task_axis,
                                point is not None,pose_ok,dict(arm.task_safety.last),
                                input_kind='recorded_camera' if dry_run else 'live_camera',
                                command_m=commanded.tolist(),command_axis=target.rotation[:,2].tolist(),
                                ik_m=ik_site.xpos.tolist(),ik_axis=ik_site.xmat.reshape(3,3)[:,2].tolist(),
                                q=arm.data.qpos.tolist(),command_velocity=np.asarray(velocity).tolist(),
                                solver=dict(arm.priority_diagnostics),clipped=shared['clipped'],
                                position_source_s=frames['left'].time_ms/1000 if frames['left'] else None,
                                axis_source_s=pose['stamp'] if pose else None)
                            if task_row:
                                with (output/'task_trace.jsonl').open('a') as task_log:task_log.write(json.dumps(task_row)+'\n')
                                if task_row['actual_path_failures'] or task_row['raw_path_failures'] or shared['clipped']:
                                    raise RuntimeError('Task route/mapping boundary violation; simulation stopped')
                            shared['state']=task_tracker.status+('；已暂停' if task_tracker.paused else '；输入过期，制动保持' if desired is None else '')
                        shared['task_error_mm']=float(np.linalg.norm(actual-desired)*1000) if desired is not None else None
                        control_metrics.append(dict(compute_ms=(time.perf_counter()-compute_start)*1000,
                            position_age_ms=(now-frames['left'].time_ms/1000)*1000 if frames['left'] else None,
                            position_valid=point is not None, orientation_fresh=pose_ok,
                            orientation_age_ms=(now-pose['stamp'])*1000 if pose else None,
                            orientation_command_error_deg=float(np.degrees(Rotation.from_matrix(actual_rotation@target.rotation.T).magnitude())),
                            orientation_raw_error_deg=float(np.degrees(Rotation.from_matrix(actual_rotation@rotation.T).magnitude())) if rotation is not None else None,
                            orientation_command_rotvec=Rotation.from_matrix(target.rotation).as_rotvec().tolist(),
                            orientation_raw_rotvec=Rotation.from_matrix(rotation).as_rotvec().tolist() if rotation is not None else None,
                            observed_goal_rotvec=Rotation.from_matrix(observed_rotation).as_rotvec().tolist() if observed_rotation is not None else None,
                            observed_goal_is_fresh=pose_ok,
                            integration_state=simulation_state.tolist() if dry_run is not None else None,
                            ik_configuration_q=arm.configuration.q.tolist() if dry_run is not None else None,
                            ik_position_m=ik_site.xpos.tolist(),
                            ik_rotation_rotvec=Rotation.from_matrix(ik_site.xmat.reshape(3,3)).as_rotvec().tolist(),
                            source_time_s=(pose['stamp']-source.start_ns/1e9) if pose and dry_run is not None else None,
                            orientation_source_stamp_s=pose['stamp'] if pose else None,
                            orientation_source_valid=bool(pose and pose['valid']),
                            orientation_source=pose['source'] if pose else 'none',
                            orientation_reason=pose['reason'] if pose else 'no_result',
                            position_reason=reason, active=shared['active'],
                            raw_position_m=desired.tolist() if desired is not None else None,
                            command_position_m=commanded.tolist(), actual_position_m=actual.tolist(),
                            joint_position_rad=arm.data.qpos[arm.q_indices].tolist(),
                            joint_velocity_command_rad_s=np.asarray(velocity).tolist(),
                            actual_rotation_rotvec=Rotation.from_matrix(actual_rotation).as_rotvec().tolist(),
                            elapsed_s=now-start,
                            command_error_m=float(np.linalg.norm(actual-commanded)),
                            joint_speed=float(np.max(np.abs(velocity))),
                            tool_down_error_deg=float(np.degrees(np.arccos(np.clip(-actual_rotation[2,2],-1,1)))),
                            violation=arm.priority_diagnostics.get('max_inequality_violation',0.)))
                    if control_wait is not None:
                        deadline, missed = next_deadline(deadline,compute_start,time.perf_counter())
                        if missed: events['control_deadline_missed'] += 1
                        control_wait.wait(deadline,stopping)
                    else:
                        deadline += .01
                        if time.perf_counter()>deadline:
                            events['control_deadline_missed'] += 1
                            deadline = time.perf_counter()
                        stopping.wait(max(0,deadline-time.perf_counter()))
            except Exception as exc:
                if task_tracker:
                    task_tracker.event('fault',clock(),False,error=str(exc),safety=dict(arm.task_safety.last),
                        exception_type=type(exc).__name__,actual_m=arm.pose()[0].tolist(),actual_axis=arm.pose()[1][:,2].tolist(),q=arm.data.qpos.tolist())
                shared['error'] = str(exc)
                stopping.set()
        control_thread = threading.Thread(target=control, name='franka-control', daemon=True)
        control_thread.start()
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(arm.model, camera)
        camera.lookat[:] = [.2,0,1.35]
        camera.distance = 2.2
        camera.azimuth = live_mapping.get('view_azimuth',130)
        camera.elevation = -22
        apply_camera(camera,presentation['camera'])
        viewer = None
        if not headless:
            with sim_lock:
                viewer=mujoco.viewer.launch_passive(arm.model,arm.data,key_callback=lambda key:native_keys.append(key),
                                                   show_left_ui=False,show_right_ui=False)
                viewer.cam.lookat[:]=camera.lookat
                viewer.cam.distance=camera.distance
                viewer.cam.azimuth=camera.azimuth
                viewer.cam.elevation=camera.elevation
        font = ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',20) if not headless else None
        window = 'WiLoR LIVE - '+robot+' simulation'
        if not headless:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window,1120,460)
            def scale_changed(value):
                with sim_lock:
                    shared['pending_scale']=scale_min+(scale_max-scale_min)*value/100
            if task_tracker is None:
                cv2.createTrackbar('Scale',window,round(100*(settings['position_scale']-scale_min)/(scale_max-scale_min)),100,scale_changed)
        last_report = 0.
        try:
            while not stopping.is_set() and (duration is None or clock()-start < duration):
                tick = clock()
                source.check()
                while source.get_pair() is not None: pass
                stream.check()
                for worker in workers.values(): worker.check()
                for recorder in raw_recorders:recorder.check()
                frames, pose = stream.snapshot()
                if tick-last_report > 5:
                    print(json.dumps(dict(elapsed=round(tick-start,1), state=shared['state'],
                        inference_count=len(stream.metrics), control_steps=len(control_metrics),
                        last_inference_ms=pose['inference_ms'] if pose else None,
                        position_age_ms=control_metrics[-1]['position_age_ms'] if control_metrics else None,
                        orientation_age_ms=(tick-pose['stamp'])*1000 if pose else None,
                        source=pose['source'] if pose else None,
                        command_error_mm=control_metrics[-1]['command_error_m']*1000 if control_metrics else None)),flush=True)
                    last_report=tick
                if headless:
                    stopping.wait(.02)
                    continue
                if not viewer.is_running(): break
                canvas = np.zeros((500,1280,3),np.uint8)
                for i, role in enumerate(('left','right')):
                    frame = frames[role]
                    # Display prediction on its own source image, never a newer image.
                    matched = pose.get(role+'_frame') if pose else None
                    prediction = pose.get('predictions',{}).get(role) if pose else None
                    display = matched if matched is not None and prediction else frame
                    if display is not None:
                        image = display.image.copy()
                        if prediction:
                            uv=np.asarray(prediction['pixels'])
                            if uv.shape==(21,2) and np.isfinite(uv).all():
                                for finger in range(5):
                                    chain=[0]+list(range(1+4*finger,5+4*finger))
                                    for j,k in zip(chain,chain[1:]):
                                        cv2.line(image,tuple(uv[j].astype(int)),tuple(uv[k].astype(int)),(255,0,255),2)
                        canvas[:360,i*640:(i+1)*640] = cv2.resize(image,(640,360))
                with sim_lock:
                    with viewer.lock():
                        viewer.user_scn.ngeom=0
                        draw_workspace(viewer.user_scn,workspace_min,workspace_max)
                    viewer.sync()
                    status = dict(shared)
                pil = Image.fromarray(cv2.cvtColor(canvas,cv2.COLOR_BGR2RGB))
                draw = ImageDraw.Draw(pil)
                age = (tick-pose['stamp'])*1000 if pose else None
                lines=[f"{status['state']}  |  工具固定朝下  |  "+('● 正在录像，R结束' if clip else '未录像，R开始'),
                    f"XYZ统一比例 {settings['position_scale']:.4f}（{scale_min:.4f}–{scale_max:.4f}）  拖动滑条或 + / - 调整",
                    '空格：开始/暂停   R：开始/结束录像   V：保存当前视角   Q：退出',
                    '已到目标活动边界' if status['clipped'] else '线框：60×80×70cm目标边界（非全部可达）；方向与比例已加载']
                if task_tracker:
                    lines=[status['state'],f"绕障—转向—对准  |  比例固定 {settings['position_scale']:.4f}  |  "+('正在录像' if clip else 'R开始录像'),
                           '空格：启停/恢复归零  T：转向  F：已停住  Enter：确认  C：标记纠偏  Q：退出',
                           '绿框：+X通过门  蓝框：转向区  青色：对准走廊；相机失效时保留失败记录']
                if status.get('task_error_mm') is not None:
                    lines[0]+=f"  |  目标偏差 {status['task_error_mm']:.0f}mm（含限速滞后）"
                if setup is not None and not setup.complete:
                    with sim_lock:
                        prompt=setup.prompt
                        lengths=setup.lengths.copy() if setup.lengths is not None else None
                        gain=setup.scale
                        extent=setup.robot_extent
                    lines=['WiLoR · 方向与缩放设置（机器人保持）',prompt,
                           setup_notice,
                           '右/前/上是你希望机器人移动的方向',
                           '每次停稳半秒，空格记录；B退回一步',
                           '无需精确回旧原点；以每次记录的起点为准',
                           'Q退出；相机标定不改变', '', '']
                    if lengths is not None:
                        lines[3]=f'右/前/上幅度：{lengths[0]*100:.1f}/{lengths[1]*100:.1f}/{lengths[2]*100:.1f} cm'
                        lines[4]=f'最大手幅度 {max(lengths)*100:.1f} cm → 机器人 {extent*100:.0f} cm'
                        lines[5]=f'统一比例 {gain:.3f}；+ / - 调整机器人幅度'
                        lines[6]='空格确认；B退回；Q退出'
                        lines[7]='确认后仍需再按空格才开始遥操'
                for i,line in enumerate(lines): draw.text((18,367+i*31),line,font=font,fill='white')
                if clip:
                    with viewer.lock():
                        view=camera_values(viewer.cam)
                    apply_camera(camera,view)
                    with sim_lock:
                        clip_renderer.update_scene(arm.data,camera=camera)
                        q=arm.data.qpos.tolist()
                    draw_workspace(clip_renderer.scene,workspace_min,workspace_max)
                    robot_image=cv2.cvtColor(clip_renderer.render(),cv2.COLOR_RGB2BGR)
                    composite=np.zeros((720,1280,3),np.uint8)
                    composite[:180,:320]=cv2.resize(canvas[:360,:640],(320,180))
                    composite[180:360,:320]=cv2.resize(canvas[:360,640:],(320,180))
                    composite[:,320:]=robot_image
                    cv2.putText(composite,f'REC {(time.perf_counter()-clip.start):.1f}s',(12,400),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,255),2)
                    cv2.putText(composite,f'Scale {settings["position_scale"]:.3f}',(12,435),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
                    clip.submit(composite,dict(qpos=q,scale=settings['position_scale'],camera=view,
                        left_arrival_s=frames['left'].time_ms/1000 if frames['left'] else None,
                        orientation_stamp_s=pose['stamp'] if pose else None,state=status['state']))
                cv2.imshow(window,cv2.cvtColor(np.asarray(pil),cv2.COLOR_RGB2BGR))
                key = cv2.waitKey(1)&255
                if native_keys: key=native_keys.popleft()
                if key in (ord('q'),ord('Q')) or cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1: break
                if key in (ord('r'),ord('R')):
                    if clip:
                        path=clip.path
                        clip.close();clip=None
                        clip_renderer.close();clip_renderer=None
                        print('RECORDING_SAVED '+str(path/'teleoperation.mp4'),flush=True)
                        if task_tracker:
                            with record_lock:
                                finished=raw_current;raw_current=None
                                if finished:finished.record_video.clear()
                            if finished:finished.close()
                            task_tracker.event('record_end',clock(),raw_session=str(finished.path) if finished else None)
                    else:
                        clip_renderer=mujoco.Renderer(arm.model,height=720,width=960)
                        clip=TeleopVideo(output,dict(mapping=saved,workspace=presentation,scale_at_start=settings['position_scale']))
                        if task_tracker:
                            from .recording import Recorder
                            with record_lock:
                                raw_current=Recorder(root/config['output_dir'],config)
                                raw_current.record_video.set();raw_recorders.append(raw_current)
                                raw_current.event(dict(type='rod_task_link',run=str(output),task_config=rod_config,
                                                       host_clock='perf_counter',original_view_recording=True))
                            task_tracker.event('record_start',clock(),raw_session=str(raw_current.path),
                                               composite=str(clip.path),note='Original RGB MJPG and measured lossless left depth; not exposure synchronization')
                        print('RECORDING_STARTED '+str(clip.path),flush=True)
                if key in (ord('v'),ord('V')):
                    with viewer.lock(): presentation['camera']=camera_values(viewer.cam)
                    presentation_path.write_text(json.dumps(presentation,indent=2))
                    print('VIEW_SAVED '+json.dumps(presentation['camera']),flush=True)
                with sim_lock:
                    if setup is not None and not setup.complete:
                        try:
                            if key==32:
                                setup.confirm(clock())
                                with (output/'direction_attempts.jsonl').open('a') as attempts:
                                    attempts.write(json.dumps(dict(accepted=True,**setup.last_diagnostic))+'\n')
                                setup_notice='已记录；请按下一步提示移动'
                                if setup.complete:
                                    result=setup.result()
                                    settings['camera_to_task_rotation']=result['camera_to_base_rotation']
                                    settings['position_scale']=result['position_scale']
                                    (output/'direction_setup.json').write_text(json.dumps(result,indent=2))
                                    with (output/'direction_setups.jsonl').open('a') as history:
                                        history.write(json.dumps(result)+'\n')
                                    shared['state']='方向与比例已确认；空格开始遥操'
                                    print('DIRECTION_SETUP '+json.dumps(result),flush=True)
                            elif key in (ord('+'),ord('=')): setup.adjust(.01)
                            elif key in (ord('-'),ord('_')): setup.adjust(-.01)
                            elif key in (ord('b'),ord('B')):
                                setup.undo()
                                setup_notice='已退回，请重新确认本步'
                        except ValueError as exc:
                            setup_notice=str(exc)
                            with (output/'direction_attempts.jsonl').open('a') as attempts:
                                attempts.write(json.dumps(dict(accepted=False,reason=str(exc),**setup.last_diagnostic))+'\n')
                    elif task_tracker and key in (ord('t'),ord('T'),ord('f'),ord('F'),13,257,ord('c'),ord('C')):
                        task_keys.append('turn' if key in (ord('t'),ord('T')) else 'finish' if key in (ord('f'),ord('F')) else
                                         'correction' if key in (ord('c'),ord('C')) else 'confirm')
                    elif key in (ord('+'),ord('='),ord('-'),ord('_')) and task_tracker is None:
                        value=cv2.getTrackbarPos('Scale',window)+(5 if key in (ord('+'),ord('=')) else -5)
                        cv2.setTrackbarPos('Scale',window,int(np.clip(value,0,100)))
                    elif key==32:
                        if shared['active']:
                            shared['active']=False
                            if task_tracker:task_tracker.pause(clock())
                            shared['state']='已暂停；制动后可重新归零开始'
                        else: shared['engage']=True
                    # Orientation task is explicit at launch; no accidental hotkey change.
                stopping.wait(max(0,.05-(clock()-tick)))
        finally:
            if viewer: viewer.close()
        if shared['error']: raise RuntimeError(shared['error'])
    except Exception as exc:
        if task_tracker and not any(e['event']=='fault' for e in task_tracker.events):
            task_tracker.event('fault',clock(),False,error=str(exc),safety=dict(arm.task_safety.last) if arm else {})
        shared['error']=str(exc)
        raise
    finally:
        stopping.set()
        if control_thread: control_thread.join(timeout=5)
        cleanup_errors=[]
        if control_thread and control_thread.is_alive():
            cleanup_errors.append('Control thread did not stop within five seconds')
        elif control_wait:
            try: control_wait.close()
            except Exception as exc: cleanup_errors.append(str(exc))
        if clip:
            try:
                clip.close()
                print('RECORDING_SAVED '+str(clip.path/'teleoperation.mp4'),flush=True)
            except Exception as exc: cleanup_errors.append(str(exc))
        if clip_renderer: clip_renderer.close()
        for resource in ([source] if source else [])+list(workers.values())+[stream]:
            try:
                resource.stop() if resource is source else resource.close()
            except Exception as exc:
                cleanup_errors.append(str(exc))
        for recorder in raw_recorders:
            try:recorder.close()
            except Exception as exc:cleanup_errors.append(str(exc))
        cv2.destroyAllWindows()
        if timer: timer.timeEndPeriod(1)
        def stats(values):
            values=[v for v in values if v is not None]
            return dict(n=len(values),median=float(np.median(values)),p95=float(np.percentile(values,95)),max=float(max(values))) if values else {}
        metrics=list(control_metrics)
        if dry_run is not None or task_tracker is not None:
            (output/'control_metrics.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in metrics))
            (output/'inference_metrics.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in stream.metrics))
        if task_tracker:
            from .rod_report import report_task
            task_tracker.event('run_end',clock(),reason=shared['error'] or 'user_exit_or_duration')
            (output/'task_events.json').write_text(json.dumps(task_tracker.events,indent=2))
            result=report_task(task_tracker.rows,task_tracker.events,rod_config)
            result['run_error']=shared['error']
            result['raw_sessions']=[str(r.path) for r in raw_recorders]
            (output/'task_report.json').write_text(json.dumps(result,indent=2))
        summary=dict(events=dict(events), error=shared['error'], cleanup_errors=cleanup_errors, control_steps=len(metrics),
            control_wait=control_wait.info if control_wait else dict(backend='legacy_event_wait'),
            elapsed_wall_s=clock()-start if start is not None else None,
            control={k:stats([r[k] for r in metrics]) for k in ('compute_ms','position_age_ms','command_error_m','joint_speed','violation','tool_down_error_deg','orientation_age_ms','orientation_command_error_deg','orientation_raw_error_deg')},
            position_valid_steps=sum(r['position_valid'] for r in metrics), orientation_fresh_steps=sum(r['orientation_fresh'] for r in metrics),
            wilor={k:stats([r[k] for r in stream.metrics]) for k in ('inference_ms','model_ms','ipc_roundtrip_ms','arrival_to_result_ms','arrival_to_submit_ms','prepare_ms')},
            gpu=getattr(stream,'info',None),
            wilor_sources=dict(Counter(r['source'] for r in stream.metrics)),
            wilor_batch2={k:stats([r[k] for r in stream.metrics if r['batch_size']==2]) for k in ('inference_ms','arrival_to_result_ms')},
            limits='Host-arrival age only, not exposure/display latency. Video only on explicit R start; clip metadata records render times and robot states.')
        (output/'summary.json').write_text(json.dumps(summary,indent=2))
        print('SUMMARY '+json.dumps(summary),flush=True)
