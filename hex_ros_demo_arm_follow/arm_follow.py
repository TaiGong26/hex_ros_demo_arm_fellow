#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-08-06
################################################################

import os
import sys
import time
import traceback
import threading
from typing import Optional, Tuple

import numpy as np

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utility import DataInterface

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntFull,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrl,
    HexDcRoboArmCtrlMode,
    HexDcRoboGripCtrl,
    HexDcRoboGripCtrlMode,
    HexDcRoboManipCtrl,
)

from TrajectoryController import Move2TargetPlanner

ARM_DOF = 6
GRIP_DOF = 1


class ArmFollow:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("arm_follow")

        ### parameters
        self.__rate_param = self.__data_interface.get_rate_param()
        self.__force_feedback_param = self.__data_interface.get_force_feedback_param()
        self.__data_interface.logi(f"work rate: {self.__rate_param['ros']} hz")
        self.__data_interface.logi(
            f"teleop rate: {self.__rate_param['teleop']} hz")

        ### dynamics (gravity used by the control builders)
        self.__gravity = np.asarray(self.__force_feedback_param["gravity"],
                                    dtype=np.float64)

        ### control presets
        self.__arm_start_pos = np.asarray(
            self.__force_feedback_param["arm_start_pos"], dtype=np.float64)
        self.__arm_end_pos = np.asarray(self.__force_feedback_param["arm_end_pos"],
                                        dtype=np.float64)
        self.__grip_stable_pos = np.asarray(
            self.__force_feedback_param["grip_stable_pos"], dtype=np.float64)
        self.__arm_stable_kp = np.asarray(self.__force_feedback_param["arm_stable_kp"],
                                          dtype=np.float64)
        self.__arm_stable_kd = np.asarray(self.__force_feedback_param["arm_stable_kd"],
                                          dtype=np.float64)
        self.__grip_stable_kp = np.asarray(self.__force_feedback_param["grip_stable_kp"],
                                           dtype=np.float64)
        self.__grip_stable_kd = np.asarray(self.__force_feedback_param["grip_stable_kd"],
                                           dtype=np.float64)
        self.__arm_slave_kp = np.asarray(
            self.__force_feedback_param["arm_slave_kp"], dtype=np.float64)
        self.__arm_slave_kd = np.asarray(
            self.__force_feedback_param["arm_slave_kd"], dtype=np.float64)
        self.__grip_slave_kp = np.asarray(
            self.__force_feedback_param["grip_slave_kp"], dtype=np.float64)
        self.__grip_slave_kd = np.asarray(
            self.__force_feedback_param["grip_slave_kd"], dtype=np.float64)
        self.__arm_slave_clip = np.asarray(
            self.__force_feedback_param["arm_slave_clip"], dtype=np.float64)
        self.__grip_slave_clip = np.asarray(
            self.__force_feedback_param["grip_slave_clip"], dtype=np.float64)

        ### threads
        self.__stop_event = threading.Event()
        self.__start_event = threading.Event()
        self.__teleop_thread = threading.Thread(target=self.__teleop_process)
        self.__teleop_dt = 1.0 / max(float(self.__rate_param["teleop"]), 1.0)

    def __is_running(self):
        return self.__data_interface.ok() and not self.__stop_event.is_set()

    ##############################################################
    # Lifecycle
    ##############################################################
    def start(self):
        self.__stop_event.clear()
        self.__start_event.clear()
        self.__teleop_thread.start()
        self.__init_process()

    def run(self):
        try:
            self.__work_process()
        except KeyboardInterrupt:
            pass
        except Exception:
            traceback.print_exc()
        finally:
            self.stop()

    def stop(self):
        self.__stop_event.set()
        if self.__teleop_thread.is_alive():
            self.__teleop_thread.join()
        self.__exit_process()
        try:
            self.__data_interface.shutdown()
        except Exception:
            pass

    ##############################################################
    # Control builders
    ##############################################################
    @staticmethod
    def __default_pose() -> HexDcBasePose:
        return HexDcBasePose(
            position=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
            orientation=HexDcBaseQuaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )

    def __build_stable_ctrl(self, jnt_pos: np.ndarray) -> HexDcRoboManipCtrl:
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.JNT,
            grav=HexDcBaseVector3(
                x=float(self.__gravity[0]),
                y=float(self.__gravity[1]),
                z=float(self.__gravity[2]),
            ),
            jnt=HexDcBaseJntFull(
                pos=jnt_pos,
                vel=np.zeros(ARM_DOF),
                eff=np.zeros(ARM_DOF),
                kp=self.__arm_stable_kp.copy(),
                kd=self.__arm_stable_kd.copy(),
                lim_vel=10.0 * np.ones(ARM_DOF, dtype=np.float64),
                lim_acc=10.0 * np.ones(ARM_DOF, dtype=np.float64),
            ),
            pose=self.__default_pose(),
        )
        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode.JNT,
            jnt=HexDcBaseJntFull(
                pos=self.__grip_stable_pos.copy(),
                vel=np.zeros(GRIP_DOF),
                eff=np.ones(GRIP_DOF),
                kp=self.__grip_stable_kp.copy(),
                kd=self.__grip_stable_kd.copy(),
                lim_vel=np.array([10.0]),
                lim_acc=np.array([10.0]),
            ),
        )
        return HexDcRoboManipCtrl(arm_ctrl=arm_ctrl, grip_ctrl=grip_ctrl)

    def __build_follow_ctrl(
            self,
            arm_jnt_pos: Optional[np.ndarray] = None,
            arm_jnt_eff: Optional[np.ndarray] = None,
            arm_jnt_vel: Optional[np.ndarray] = None,
            grip_jnt_pos: Optional[np.ndarray] = None,
            grip_jnt_vel: Optional[np.ndarray] = None,
            grip_jnt_eff: Optional[np.ndarray] = None) -> HexDcRoboManipCtrl:
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.MIT,
            grav=HexDcBaseVector3(
                x=float(self.__gravity[0]),
                y=float(self.__gravity[1]),
                z=float(self.__gravity[2]),
            ),
            jnt=HexDcBaseJntFull(
                pos=arm_jnt_pos if arm_jnt_pos is not None else self.__arm_start_pos.copy(),
                vel=arm_jnt_vel if arm_jnt_vel is not None else np.zeros(ARM_DOF),
                eff=arm_jnt_eff if arm_jnt_eff is not None else np.zeros(ARM_DOF),
                kp=self.__arm_slave_kp.copy(),
                kd=self.__arm_slave_kd.copy(),
                lim_vel=np.zeros(ARM_DOF),
                lim_acc=np.zeros(ARM_DOF),
            ),
            pose=self.__default_pose(),
        )
        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode.MIT,
            jnt=HexDcBaseJntFull(
                pos=grip_jnt_pos
                if grip_jnt_pos is not None else self.__grip_stable_pos.copy(),
                vel=grip_jnt_vel if grip_jnt_vel is not None else np.zeros(GRIP_DOF),
                eff=grip_jnt_eff if grip_jnt_eff is not None else np.zeros(GRIP_DOF),
                kp=self.__grip_slave_kp.copy(),
                kd=self.__grip_slave_kd.copy(),
                lim_vel=np.zeros(GRIP_DOF),
                lim_acc=np.zeros(GRIP_DOF),
            ),
        )
        return HexDcRoboManipCtrl(arm_ctrl=arm_ctrl, grip_ctrl=grip_ctrl)

    ##############################################################
    # Processes
    ##############################################################
    def __teleop_process(self):
        prev_q = False
        prev_s = False
        while self.__is_running():
            time.sleep(self.__teleop_dt)

            keys = self.__data_interface.get_keyboard_state(latest=True)
            if keys is None:
                continue

            curr_q = bool(keys.key_q)
            if curr_q and not prev_q:
                self.__data_interface.logi("[arm_follow]: stop and exit")
                self.__stop_event.set()
            prev_q = curr_q

            curr_s = bool(keys.key_s)
            if curr_s and not prev_s:
                self.__start_event.set()
            prev_s = curr_s

    def __move_to_stable(self, phase: str, is_start: bool = True):
        self.__data_interface.logi(
            f"[arm_follow]: move to {phase} position")

        stable_pos = self.__arm_start_pos if is_start else self.__arm_end_pos
        duration = 3.5

        # master (hello, read-only): only wait until it is connected
        master_state = self.__data_interface.get_master_manip_state(
            latest=True)
        while master_state is None and self.__data_interface.ok():
            self.__data_interface.logw(
                "waiting for master state...")
            time.sleep(0.1)
            master_state = self.__data_interface.get_master_manip_state(
                latest=True)

        # slave: wait and then move to stable position
        slave_state = self.__data_interface.get_slave_manip_state(
            latest=True)
        while slave_state is None and self.__data_interface.ok():
            self.__data_interface.logw(
                "waiting for slave state...")
            time.sleep(0.1)
            slave_state = self.__data_interface.get_slave_manip_state(
                latest=True)

        # only the slave is commanded to the stable position (master is
        # read-only, the operator holds it by hand)
        if slave_state is not None:
            slave_jnt = np.asarray(
                slave_state.manip_state.arm_state.jnt.position,
                dtype=np.float64)
            slave_planner = Move2TargetPlanner(
                slave_jnt, stable_pos, duration)
            slave_planner.start_trajectory()

            while self.__data_interface.ok():
                slave_target, slave_done = slave_planner.get_target_position()
                if slave_done:
                    break
                if slave_target is not None:
                    self.__data_interface.pub_slave_manip_ctrl(
                        self.__build_stable_ctrl(jnt_pos=slave_target))
                self.__data_interface.sleep()

    def __init_process(self):
        try:
            self.__move_to_stable("init", is_start=True)
        except Exception:
            traceback.print_exc()

    def __exit_process(self):
        try:
            self.__move_to_stable("exit", is_start=False)
        except Exception:
            traceback.print_exc()

    def __work_process(self):
        self.__data_interface.logi("press 's' to start follow control")
        self.__data_interface.logi("press 'q' to exit follow control")

        while self.__is_running() and not self.__start_event.is_set():
            self.__data_interface.sleep()

        self.__data_interface.logi("start follow control")

        self.__follow()

    def __follow(self):
        master_pos = None
        master_vel = None

        slave_pos = None
        slave_vel = None

        slave_target_pos = None

        # grip state variables
        grip_master_pos = None
        grip_master_vel = None
        grip_slave_pos = None
        grip_slave_vel = None
        grip_slave_target = None

        while self.__is_running():
            master_state = self.__data_interface.get_master_manip_state(latest=True)
            slave_state = self.__data_interface.get_slave_manip_state(latest=True)

            ## master
            if master_state is not None:
                master_pos = np.asarray(
                    master_state.manip_state.arm_state.jnt.position, dtype=np.float64)
                master_vel = np.asarray(
                    master_state.manip_state.arm_state.jnt.velocity, dtype=np.float64)

                # grip state (hello publishes an empty grip -> falls back)
                try:
                    grip_master_pos = np.asarray(
                        master_state.manip_state.grip_state.jnt.position, dtype=np.float64)
                    grip_master_vel = np.asarray(
                        master_state.manip_state.grip_state.jnt.velocity, dtype=np.float64)
                except Exception:
                    grip_master_pos = None
                    grip_master_vel = None
                    self.__data_interface.logw("master grip state not available")

            ## slave
            if slave_state is not None:
                slave_pos = np.asarray(
                    slave_state.manip_state.arm_state.jnt.position, dtype=np.float64)
                slave_vel = np.asarray(
                    slave_state.manip_state.arm_state.jnt.velocity, dtype=np.float64)

                try:
                    grip_slave_pos = np.asarray(
                        slave_state.manip_state.grip_state.jnt.position, dtype=np.float64)
                    grip_slave_vel = np.asarray(
                        slave_state.manip_state.grip_state.jnt.velocity, dtype=np.float64)
                except Exception:
                    grip_slave_pos = None
                    grip_slave_vel = None
                    self.__data_interface.logw("slave grip state not available")

            if master_pos is not None and slave_pos is not None and master_vel is not None:
                try:
                    # slave follows master: clip the per-joint position error
                    slave_target_pos, _ = self.__compute_effective_target(
                        slave_pos, master_pos, None, self.__arm_slave_clip)
                except Exception:
                    traceback.print_exc()

                ### TODO: grip follow strategy (default: slave grip stays at
                # grip_stable_pos because master hello has no grip)
                grip_slave_target = None
                if (grip_master_pos is not None and grip_slave_pos is not None
                        and grip_master_pos.shape[0] == GRIP_DOF
                        and grip_slave_pos.shape[0] == GRIP_DOF):
                    try:
                        grip_slave_target, _ = self.__compute_effective_target(
                            grip_slave_pos, grip_master_pos,
                            None, self.__grip_slave_clip)
                    except Exception:
                        traceback.print_exc()

                # master (hello) is read-only: only the slave follow ctrl
                self.__data_interface.pub_slave_manip_ctrl(
                    self.__build_follow_ctrl(
                        arm_jnt_pos=slave_target_pos,
                        arm_jnt_vel=master_vel,
                        grip_jnt_pos=grip_slave_target,
                        grip_jnt_vel=grip_master_vel,
                    ))

            self.__data_interface.sleep()

    def __compute_effective_target(
            self,
            current: np.ndarray,
            target: np.ndarray,
            deadzone: Optional[np.ndarray],
            clip_bound: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Apply deadzone compensation and saturation clipping.

        Args:
            current: Current joint positions, shape (N,)
            target: Desired joint positions, shape (N,)
            deadzone: Per-joint deadzone width, shape (N,). None = no deadzone.
            clip_bound: Per-joint saturation limit, shape (N,). None = no clipping.

        Returns:
            Effective target after deadzone + clipping, shape (N,)
        """
        e = target - current

        zero_mask = np.zeros_like(e)

        if deadzone is not None:
            e_abs = np.fabs(e)
            zero_mask = (e_abs <= deadzone)
            e[zero_mask] = 0.0
            e[~zero_mask] = e[~zero_mask] - np.sign(e[~zero_mask]) * deadzone[~zero_mask]

        if clip_bound is not None:
            e = np.clip(e, -clip_bound, clip_bound)

        return current + e, zero_mask


def main():
    arm_follow = ArmFollow()
    try:
        arm_follow.start()
        arm_follow.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
