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

# Slave grip (hand) position limits [min, max] in rad per hand type.
# Mirrors hex_driver_robot/device/hands.py `_LIMITS`
# (SdtHandGp100 / SdtHandGp80G1 / SdtHandGr100).
_GRIP_LIMITS: dict[str, tuple[float, float]] = {
    "gp100": (0.0, 1.335),
    "gp80":  (0.0, 5.65),
    "gr100": (0.0, 0.69),
}

LED_YELLOW = (1.0, 1.0, 0.0)
LED_GREEN = (0.0, 1.0, 0.0)


class ArmFollow:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("arm_follow")

        ### parameters
        self.__rate_param = self.__data_interface.get_rate_param()
        self.__follow_param = self.__data_interface.get_follow_param()
        self.__data_interface.logi(f"work rate: {self.__rate_param['ros']} hz")
        self.__data_interface.logi(
            f"teleop rate: {self.__rate_param['teleop']} hz")

        ### dynamics (gravity used by the control builders)
        self.__gravity = np.asarray(self.__follow_param["gravity"],
                                    dtype=np.float64)

        ### control presets
        self.__arm_end_pos = np.asarray(self.__follow_param["arm_end_pos"],
                                        dtype=np.float64)
        self.__grip_stable_pos = np.asarray(
            self.__follow_param["grip_stable_pos"], dtype=np.float64)
        self.__arm_stable_kp = np.asarray(self.__follow_param["arm_stable_kp"],
                                          dtype=np.float64)
        self.__arm_stable_kd = np.asarray(self.__follow_param["arm_stable_kd"],
                                          dtype=np.float64)
        self.__grip_stable_kp = np.asarray(self.__follow_param["grip_stable_kp"],
                                           dtype=np.float64)
        self.__grip_stable_kd = np.asarray(self.__follow_param["grip_stable_kd"],
                                           dtype=np.float64)
        self.__arm_slave_kp = np.asarray(
            self.__follow_param["arm_slave_kp"], dtype=np.float64)
        self.__arm_slave_kd = np.asarray(
            self.__follow_param["arm_slave_kd"], dtype=np.float64)
        self.__grip_slave_kp = np.asarray(
            self.__follow_param["grip_slave_kp"], dtype=np.float64)
        self.__grip_slave_kd = np.asarray(
            self.__follow_param["grip_slave_kd"], dtype=np.float64)

        ### follow control coefficients
        # velocity coupling coefficient (eta): master velocity term in slave torque
        self.__velocity_coupling_coeff = float(
            self.__follow_param["velocity_coupling_coeff"])
        # error proportional gain (alpha): scales error e, dynamically adjusts kp
        self.__error_proportional_gain = float(
            self.__follow_param["error_proportional_gain"])
        # kp clamp bounds: dynamic arm kp stays in [arm_kmin, arm_kmax],
        # dynamic grip kp stays in [grip_kmin, grip_kmax]
        self.__arm_kmin = float(self.__follow_param["arm_kmin"])
        self.__arm_kmax = float(self.__follow_param["arm_kmax"])
        self.__grip_kmin = float(self.__follow_param["grip_kmin"])
        self.__grip_kmax = float(self.__follow_param["grip_kmax"])
        # slave grip (hand) type -> position limits [min, max] rad
        grip_type = str(self.__follow_param["robot_grip_type"])
        self.__data_interface.logi(
            f"follow coeffs: velocity_coupling_coeff={self.__velocity_coupling_coeff}, "
            f"error_proportional_gain={self.__error_proportional_gain}, "
            f"arm_kmin={self.__arm_kmin}, arm_kmax={self.__arm_kmax}, "
            f"grip_kmin={self.__grip_kmin}, grip_kmax={self.__grip_kmax}")

        ### slave grip (hand) limits
        self.__grip_limit_min: Optional[float] = None
        self.__grip_limit_max: Optional[float] = None
        self.__grip_limit_mid: Optional[float] = None
        grip_limits = _GRIP_LIMITS.get(grip_type)
        if grip_limits is not None:
            self.__grip_limit_min, self.__grip_limit_max = grip_limits
            self.__grip_limit_mid = 0.5 * (self.__grip_limit_min + self.__grip_limit_max)
            
            self.__grip_stable_pos = np.array([self.__grip_limit_mid])
            self.__data_interface.logi(
                f"grip type {grip_type}: limits "
                f"[{self.__grip_limit_min}, {self.__grip_limit_max}] rad, "
                f"mid {self.__grip_limit_mid}")
        else:
            self.__data_interface.logw(
                f"unknown grip type '{grip_type}', grip follow disabled")

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
            Kp: Optional[np.ndarray] = None,
            arm_jnt_pos: Optional[np.ndarray] = None,
            arm_jnt_eff: Optional[np.ndarray] = None,
            arm_jnt_vel: Optional[np.ndarray] = None,
            grip_jnt_pos: Optional[np.ndarray] = None,
            grip_jnt_vel: Optional[np.ndarray] = None,
            grip_jnt_eff: Optional[np.ndarray] = None,
            grip_Kp: Optional[np.ndarray] = None) -> HexDcRoboManipCtrl:
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.MIT,
            grav=HexDcBaseVector3(
                x=float(self.__gravity[0]),
                y=float(self.__gravity[1]),
                z=float(self.__gravity[2]),
                # z=float(0.0),
            ),
            jnt=HexDcBaseJntFull(
                pos=arm_jnt_pos if arm_jnt_pos is not None else self.__arm_end_pos.copy(),
                vel=arm_jnt_vel if arm_jnt_vel is not None else np.zeros(ARM_DOF),
                eff=arm_jnt_eff if arm_jnt_eff is not None else np.zeros(ARM_DOF),
                kp= Kp if Kp is not None else self.__arm_slave_kp.copy(),
                kd=self.__arm_slave_kd.copy(),
                lim_vel=np.ones_like(arm_jnt_pos) * 0,
                lim_acc=np.ones_like(arm_jnt_pos) * 0,
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
                kp=grip_Kp if grip_Kp is not None else self.__grip_slave_kp.copy(),
                kd=self.__grip_slave_kd.copy(),
                lim_vel=np.zeros(GRIP_DOF),
                lim_acc=np.zeros(GRIP_DOF),
            ),
        )
        return HexDcRoboManipCtrl(arm_ctrl=arm_ctrl, grip_ctrl=grip_ctrl)

    def __set_master_led(self, r: float, g: float, b: float):
        """Publish an RGB color to the master (hello) LED strip."""
        self.__data_interface.pub_master_color_cmd(r, g, b)

    ##############################################################
    # Processes
    ##############################################################
    def __teleop_process(self):
        prev_q = False
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

    def __move_to_start(self):
        self.__data_interface.logi(
            "[arm_follow]: move to start position")

        # init phase: yellow LED on the master (hello) robot
        self.__set_master_led(*LED_YELLOW)

        # wait for master/slave to connected; exit the node if no data in 5s
        master_state = None
        slave_state = None
        deadline = time.monotonic() + 5.0
        while (self.__data_interface.ok()
                and time.monotonic() < deadline
                and (master_state is None or slave_state is None)):
            if master_state is None:
                master_state = self.__data_interface.get_master_manip_state(
                    latest=True)
            if slave_state is None:
                slave_state = self.__data_interface.get_slave_manip_state(
                    latest=True)

            missing = []
            if master_state is None:
                missing.append("master")
            if slave_state is None:
                missing.append("slave")
            self.__data_interface.logw(
                f"waiting for {'/'.join(missing)} manip state...")
            time.sleep(0.1)

        if master_state is None or slave_state is None:
            self.__data_interface.loge(
                "master/slave manip state not ready within 5s, "
                "exit the node")
            self.__stop_event.set()
            return

        self.__set_master_led(*LED_YELLOW)

        # init: the slave slowly comes online onto the master position.
        self.__data_interface.logw(
            "The slave arm slowly follows the master arm and comes online to the master arm's position.")

        master_pos = np.asarray(
            master_state.manip_state.arm_state.jnt.position,
            dtype=np.float64)
        slave_pos = np.asarray(
            slave_state.manip_state.arm_state.jnt.position,
            dtype=np.float64)

        # slow online: the slave commanded position starts at its current
        slave_target = slave_pos.copy()
        
        tau = 0.8    # exponential time constant of the online move [s]
        snap_tol = 0.25   # within this error [rad]: snap directly to the master
        conv_tol = 0.05  # online-to-master convergence tolerance [rad]

        prev_time = time.monotonic()
        while self.__data_interface.ok():
            now = time.monotonic()
            dt = max(now - prev_time, 0.0)
            prev_time = now

            # keep re-reading the master position while going online
            master_state = self.__data_interface.get_master_manip_state(
                latest=True)
            if master_state is not None:
                master_pos = np.asarray(
                    master_state.manip_state.arm_state.jnt.position,
                    dtype=np.float64)

            err = master_pos - slave_target
            err_norm = np.linalg.norm(err)
            
            gain = 1.0 - np.exp(-dt / tau)
            if err_norm <= snap_tol:
                gain = 1.0  
                
            self.__data_interface.logd(f"dt: {dt:.2f}, gain: {gain:.4f}")
            
            slave_target = slave_target + err * gain

            self.__data_interface.pub_slave_manip_ctrl(
                self.__build_stable_ctrl(jnt_pos=slave_target))

            # done: slave online to the master
            if err_norm <= conv_tol:
                self.__data_interface.logi(
                    "slave online to the master position")
                break
            self.__data_interface.sleep()

        self.__data_interface.logi(
            "keep the master still and press 's' to start the follow")

    def __move_to_exit(self):
        self.__data_interface.logi(
            "[arm_follow]: move to exit position")

        # exit phase: yellow LED on the master (hello) robot
        self.__set_master_led(*LED_YELLOW)

        slave_state = self.__data_interface.get_slave_manip_state(
            latest=True)
        if slave_state is None:
            return

        # command the slave back to the stable end position
        stable_pos = self.__arm_end_pos
        duration = 3.5
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
            self.__move_to_start()
        except Exception:
            traceback.print_exc()

    def __exit_process(self):
        try:
            self.__move_to_exit()
        except Exception:
            traceback.print_exc()

    def __work_process(self):
        self.__data_interface.logi("press 'q' to exit follow control")

        # while self.__is_running() and not self.__start_event.is_set():
        #     self.__data_interface.sleep()

        self.__data_interface.logi("start follow control")

        self.__follow()

    def __follow(self):
        # work phase: green LED on the master (hello) robot
        self.__set_master_led(*LED_GREEN)

        master_pos = None
        master_vel = None

        slave_pos = None
        slave_vel = None

        slave_target_pos = None

        # grip state variables (master hello has no grip motor -> slave only)
        grip_slave_pos = None
        grip_slave_vel = None
        grip_slave_target = None
        
        # This dt must be aligned with the hello driver's update rate.
        dt = 1 / 500
        
        # Joy state
        joy_state = None

        while self.__is_running():
            master_state = self.__data_interface.get_master_manip_state(latest=True)
            slave_state = self.__data_interface.get_slave_manip_state(latest=True)
            joy_state = self.__data_interface.get_joy_state(latest=True)

            ## master
            if master_state is not None:
                master_pos = np.asarray(
                    master_state.manip_state.arm_state.jnt.position, dtype=np.float64)
                master_vel = np.asarray(
                    master_state.manip_state.arm_state.jnt.velocity, dtype=np.float64)

            ## slave
            if slave_state is not None:
                slave_pos = np.asarray(
                    slave_state.manip_state.arm_state.jnt.position, dtype=np.float64)
                slave_vel = np.asarray(
                    slave_state.manip_state.arm_state.jnt.velocity, dtype=np.float64)

                try:
                    grip_slave_pos = np.asarray(
                        slave_state.manip_state.grip_state.jnt.position,
                        dtype=np.float64)
                except Exception:
                    grip_slave_pos = None
                    self.__data_interface.logw("slave grip state not available")

            if (master_pos is not None and master_vel is not None and 
                slave_pos is not None and slave_vel is not None):
                try:
                    
                    err = (master_pos - slave_pos)
                    Kp = self.__dynamic_kp(
                        e = err,  # type: ignore
                        K_min = self.__arm_kmin,
                        K_max = self.__arm_kmax,
                        alpha = self.__error_proportional_gain
                    )
                    slave_target_pos = master_pos + dt * master_vel
                
                except Exception:
                    traceback.print_exc()
                    continue

                ### slave grip: master trigger -> joint target position
                if (joy_state is not None
                        and self.__grip_limit_min is not None
                        and self.__grip_limit_max is not None
                        and self.__grip_limit_mid is not None):
                    # trigger spans [-1,1] -> normalized [0,1]:
                    # -1 -> limit min, 0 -> limit mid, +1 -> limit max.
                    # Hand-type limits all start at min=0, so the normalized
                    # input scales the [min, max] span directly.
                    trigger_norm = (joy_state.trigger + 1.0) / 2.0  # [-1,1] → [0,1]
                    grip_slave_target = np.ones_like(
                        self.__grip_stable_pos) * np.clip(
                            trigger_norm
                            * (self.__grip_limit_max - self.__grip_limit_min),
                            self.__grip_limit_min, self.__grip_limit_max)

                grip_Kp = None
                if grip_slave_target is not None and grip_slave_pos is not None:
                    # grip_slave_target / grip_slave_pos are both 1-D (1,)
                    grip_err = grip_slave_target - grip_slave_pos
                    grip_Kp = self.__dynamic_kp(
                        e=grip_err,
                        K_min=self.__grip_kmin,
                        K_max=self.__grip_kmax,
                        alpha=self.__error_proportional_gain,
                    )
                
                self.__data_interface.logd(
                    f"grip target pos: {grip_slave_target}, kp: {grip_Kp} grip pos: {grip_slave_pos}")
                
                # master (hello) is read-only: only the slave follow ctrl
                self.__data_interface.pub_slave_manip_ctrl(
                    self.__build_follow_ctrl(
                        Kp=Kp,
                        arm_jnt_pos=slave_target_pos,
                        arm_jnt_vel=self.__velocity_coupling_coeff*master_vel ,
                        grip_jnt_pos=grip_slave_target,
                        grip_jnt_vel=None,
                        grip_Kp=grip_Kp,
                    ))

            self.__data_interface.sleep()

    def __dynamic_kp(
        self, 
        e: np.ndarray, 
        K_min: float, 
        K_max: float, 
        alpha: float
    ) -> np.ndarray:        
        """
        Compute dynamic proportional gain with sigmoid saturation.

        The gain is computed using a hyperbolic tangent function:
            K_p = K_min + (K_max - K_min) * tanh(alpha * |e|)

        This provides smooth transitions between K_min (small errors) and 
        K_max (large errors), avoiding abrupt gain changes.

        Args:
            e (np.ndarray): Position error vector
            K_min (float): Minimum proportional gain (for small errors)
            K_max (float): Maximum proportional gain (for large errors)
            alpha (float): Sensitivity coefficient controlling transition sharpness

        Returns:
            np.ndarray: Dynamic proportional gain vector, same shape as e

        """
        return K_min + (K_max - K_min) * np.tanh(alpha * np.fabs(e))
    
def main():
    arm_follow = ArmFollow()
    try:
        arm_follow.start()
        arm_follow.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
