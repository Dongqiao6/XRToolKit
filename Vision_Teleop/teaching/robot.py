"""MuJoCo/Mink adapter: weighted IK by default, optional translation-preserving solve."""
from __future__ import annotations

from pathlib import Path
import mink
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]


class SimArm:
    def __init__(self, name="franka_fr3", mount_degrees=0, with_gripper=False, position_priority=False, position_only=False, rod_task=None, rod_collision_variant='original', gravity_compensation=False):
        if gravity_compensation and (name!='franka_fr3' or mount_degrees!=0 or with_gripper or rod_task is not None):
            raise ValueError('Gravity correction currently verified only for bare default FR3')
        if rod_collision_variant != 'original' and (name != 'ur5e' or rod_task is None or rod_collision_variant != 'wrist2_mesh_parts'):
            raise ValueError('Collision candidate is restricted to the UR5e rod task')
        self.rod_collision_variant=rod_collision_variant
        self.position_priority=position_priority
        self.position_only=position_only
        self.pose_priority=False
        self.axis_only=False  # Explicit offline task; normal live remains full fixed orientation.
        self.axis_priority=False
        self.task_safety=None
        self.priority_diagnostics={}
        settings = yaml.safe_load((ROOT / "config/robots.yaml").read_text())[name]
        if rod_task is not None:
            with_gripper=True
            # Use the same fixed hand for this task only; legacy UR5e stays bare.
            if 'gripper' not in settings:
                settings['gripper']=dict(xml='external/menagerie/franka_emika_panda/hand.xml',
                                         mount_site='attachment_site',opening_m=.05)
        self.with_gripper = with_gripper
        if with_gripper:
            from .gripper import attach_fixed_hand
            self.model = attach_fixed_hand(ROOT/settings["scene"],settings["gripper"],ROOT,rod_task,rod_collision_variant)
        elif gravity_compensation:
            from .task_control import compensated_fr3_model
            self.model = compensated_fr3_model(ROOT / settings["scene"])
        else:
            self.model = mujoco.MjModel.from_xml_path(str(ROOT / settings["scene"]))
        self.site = "tool_tcp" if with_gripper else settings["end_effector_site"]
        self.home_override = settings["gripper"].get("home_qpos") if with_gripper else None
        self.dof = settings["dof"]
        if self.model.nq != self.dof or self.model.nu != self.dof:
            raise ValueError("Unexpected model joint/actuator layout")
        base = self.model.body(settings["base_body"]).id
        original = self.model.body_quat[base].copy()
        rotate = Rotation.from_euler("y", mount_degrees, degrees=True)
        combined = (rotate * Rotation.from_quat(original[[1, 2, 3, 0]])).as_quat()
        self.model.body_quat[base] = combined[[3, 0, 1, 2]]
        # A common raised base keeps these component tests off the floor. It is
        # not a cross-mount task-space calibration or a real installation claim.
        self.model.body_pos[base, 2] = 1.0
        self.data = mujoco.MjData(self.model)
        mujoco.mj_setConst(self.model, self.data)
        self.configuration = mink.Configuration(self.model)
        self.eef = mink.FrameTask(self.site, "site", position_cost=1.0, orientation_cost=1.0, lm_damping=1e-3)
        self.posture = mink.PostureTask(self.model, cost=1e-3)
        joint_ids = self.model.actuator_trnid[:, 0]
        if len(set(joint_ids)) != self.dof:
            raise ValueError("Actuators must map one-to-one to arm joints")
        self.q_indices = self.model.jnt_qposadr[joint_ids]
        names = [self.model.joint(int(j)).name for j in joint_ids]
        geoms = [i for i in range(self.model.ngeom) if self.model.geom_contype[i] or self.model.geom_conaffinity[i]]
        self.limits = [mink.ConfigurationLimit(self.model),
                       mink.VelocityLimit(self.model, {n: 0.6 for n in names}),
                       mink.CollisionAvoidanceLimit(self.model, [(geoms, geoms)],
                                                    minimum_distance_from_collisions=0.005,
                                                    collision_detection_distance=0.03)]
        self.reset()
        if rod_task is not None:
            from .rod_scene import RodSafety
            self.task_safety=RodSafety(self,rod_task)
            self.limits[-1]=self.task_safety.limit

    def reset(self):
        self.priority_diagnostics={}
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        if self.home_override is not None:
            self.data.qpos[:] = self.home_override
            self.data.ctrl[:] = self.data.qpos[self.q_indices]
        mujoco.mj_forward(self.model, self.data)
        self.configuration.update(self.data.qpos)
        self.posture.set_target_from_configuration(self.configuration)
        self.eef.set_target(self.configuration.get_transform_frame_to_world(self.site, "site"))

    def pose(self):
        return self.data.site(self.site).xpos.copy(), self.data.site(self.site).xmat.reshape(3, 3).copy()

    def set_target(self, position, orientation):
        position, orientation = np.asarray(position), np.asarray(orientation)
        if position.shape != (3,) or orientation.shape != (3, 3) or not np.isfinite(position).all() or not np.isfinite(orientation).all():
            raise ValueError("Finite position and rotation matrix required")
        if not np.allclose(orientation.T @ orientation, np.eye(3), atol=1e-5) or np.linalg.det(orientation) < 0.999:
            raise ValueError("Orientation must be a proper rotation, not a mirror")
        self.eef.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(orientation), position))

    def step(self, dt=0.01):
        if dt <= 0 or abs(round(dt / self.model.opt.timestep) * self.model.opt.timestep - dt) > 1e-8:
            raise ValueError("Control dt must be a positive multiple of simulation timestep")
        # Keep the commanded configuration between steps, as upstream examples
        # do. Re-seeding it from actual q every step weakens position tracking
        # under gravity by continually moving the setpoint back to the robot.
        if np.max(np.abs(self.configuration.q - self.data.qpos)) > 0.2:
            raise RuntimeError("Simulation cannot track the bounded command; target updates stopped")
        if self.axis_priority:
            from .rod_task import solve_axis_first
            velocity=solve_axis_first(self,dt)
        elif self.position_only or self.axis_only:
            from .position_priority import CartesianPosition
            task=CartesianPosition(self.site,self.eef.transform_target_to_world.translation())
            tasks=[task,self.posture]
            if self.axis_only:
                from .tool_axis import ToolAxis
                tasks.append(ToolAxis(self.site,self.eef.transform_target_to_world.rotation().as_matrix()[:,2]))
            velocity=mink.solve_ik(self.configuration,tasks,dt,'daqp',limits=self.limits,safety_break=True,primal_tol=1e-9)
            violation=0.
            for limit in self.limits:
                c=limit.compute_qp_inequalities(self.configuration,dt)
                if not c.inactive:violation=max(violation,float(np.max(c.G@(velocity*dt)-c.h)))
            if violation>1e-7:raise RuntimeError('Position IK violated original safety inequalities')
            self.priority_diagnostics=dict(max_inequality_violation=violation,commanded_peak_joint_speed_rad_s=float(np.max(np.abs(velocity))))
        elif self.pose_priority:
            from .position_priority import solve_pose_first
            velocity=solve_pose_first(self,dt)
        elif self.position_priority:
            from .position_priority import solve_position_first
            velocity=solve_position_first(self,dt)
        else:
            velocity = mink.solve_ik(self.configuration, [self.eef, self.posture], dt, "daqp",
                                     limits=self.limits, safety_break=True, primal_tol=1e-9)
            violation=0.
            for limit in self.limits:
                c=limit.compute_qp_inequalities(self.configuration,dt)
                if not c.inactive:violation=max(violation,float(np.max(c.G@(velocity*dt)-c.h)))
            if violation>1e-7:raise RuntimeError('Pose IK violated original safety inequalities')
            self.priority_diagnostics=dict(max_inequality_violation=violation,commanded_peak_joint_speed_rad_s=float(np.max(np.abs(velocity))))
        previous_command=self.configuration.q.copy() if self.task_safety is not None else None
        self.configuration.integrate_inplace(velocity, dt)
        if self.task_safety is not None:
            try:
                self.task_safety.commit_checked_step(self.configuration.q,dt)
            except Exception:
                self.configuration.update(previous_command)
                raise
        else:
            self.data.ctrl[:] = self.configuration.q[self.q_indices]
            for _ in range(round(dt / self.model.opt.timestep)):
                mujoco.mj_step(self.model, self.data)
        if not np.isfinite(self.data.qpos).all():
            raise RuntimeError("Nonfinite simulation state")
        return velocity
