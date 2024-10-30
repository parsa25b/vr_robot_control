import argparse
from pathlib import Path
import numpy as np
from mujoco_robot_control.ik_levenberg_marquardt import LevenbegMarquardtIK
from mujoco_robot_control.ik_quadratic_programming import QuadraticProgrammingIK
from mujoco_robot_control.robot_env import RobotEnv
from vr_robot_control.hand_pose_reader import HandPoseReader
from scipy.spatial.transform import Rotation as R
import time
from ns_interface import command
from ns_interface import feedback
import ns_interface


class RobotController:
    def __init__(self, args):

        self.unlocked_joints = [
            "right_shoulder_fe",
            "right_shoulder_aa",
            "right_shoulder_ie",
            "right_elbow_fe",
            "right_elbow_ie",
            "right_wrist_fe",
            "right_wrist_aa",
            # "left_shoulder_fe",
            # "left_shoulder_aa",
            # "left_shoulder_ie",
            # "left_elbow_fe",
            # "left_elbow_ie",
            # "left_wrist_fe",
            # "left_wrist_aa",
            # "torso_aa",
            # "torso_ie",
            # "torso_fe_a",
            # "torso_fe_b",
            # "neck_ie",
            # "neck_aa",
            # "neck_fe",
        ]

        # Mujoco Robot Environment
        self.env = RobotEnv(
            model_path=Path(args.urdf_path),
            unlocked_joint_name=self.unlocked_joints,
        )

        # Quest hand pose reader
        self.hand_pose_reader = HandPoseReader()
        self.ee_reference_pose = {}

        # IK solver
        # ik = LevenbegMarquardtIK(env)
        self.ik = QuadraticProgrammingIK(self.env)
        self.weight = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])

        # End-effector we wish to control.
        self.left_frame_name = "left_hand_tracker_link"
        self.right_frame_name = "right_hand_tracker_link"

        self.env.set_timestep(0.01)

        # NS Interface
        ns_interface.init(args.config_path)
        command.start_tracking(self.unlocked_joints)

        self.is_trigger_button_pressed = False
        self.trigger_button_previous = False

    def run_simulation(self):
        """
        Runs the control loop for the robot
        """

        while True:
            # env.add_marker(
            #     env.data.body(right_frame_id).xpos,
            #     label="right_wrist",
            #     marker_size=0.05,
            # )

            self.update_tracker_data(["left_hand", "right_hand"])

            # Inverse Kinematics calculations
            qpos, qvel = self.ik.calculate(
                self.ee_reference_pose["right_hand"],
                self.right_frame_name,
                weight=self.weight,
                frame_type="body",
                only_position=False,
            )

            self.env.set_state(qpos, qvel)
            self.env.render()

            if self.is_trigger_button_pressed == True:
                # print("Publishing joint points")
                time.sleep(0.01)
                command.send_tracking_point(self.unlocked_joints, qpos)

    def update_tracker_data(self, tracker_names: list):
        """
        Updates the tracker pose in the mujoco simulation

        """

        hand_poses, button_states = self.hand_pose_reader.get_hand_poses()
        if hand_poses is None:
            print("No data received from hand pose reader")
            is_trigger_button_pressed = False

        else:
            # print("hand p/ose reader running")
            for tracker_name in tracker_names:
                mocap_id = self.env.model.body(tracker_name + "_target").mocapid[0]
                self.env.data.mocap_pos[mocap_id] = hand_poses[tracker_name]["position"]
                self.env.data.mocap_pos[mocap_id][2] += 0.5
                rotation = R.from_matrix(hand_poses[tracker_name]["rotation"])
                self.env.data.mocap_quat[mocap_id] = rotation.as_quat()[[3, 0, 1, 2]]

            trigger_button = button_states.get("X")
            if trigger_button is not None:
                if (trigger_button == True) and (self.trigger_button_previous == False):
                    self.is_trigger_button_pressed = not self.is_trigger_button_pressed

            self.trigger_button_previous = trigger_button

        for tracker_name in tracker_names:
            mocap_id = self.env.model.body(tracker_name + "_target").mocapid[0]
            self.ee_reference_pose[tracker_name] = np.concatenate(
                (
                    self.env.data.mocap_pos[mocap_id],
                    self.env.data.mocap_quat[mocap_id],
                )
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf_path",
        type=str,
        default="mujoco_robot_control/src/mujoco_robot_control/assets/phoenix/urdfs/gpr-gen8.1/scene.xml",
        help="Path to the urdf file",
    )
    parser.add_argument(
        "--trajectory_path",
        type=str,
        default="src/mujoco_robot_control/trajecotries/chicken_head.csv",
        help="Path to the trajectory file",
    )
    parser.add_argument(
        "--config_path",
        type=str,
        default="vr_robot_control/config/config_torso.yml",
        help="Path to the config file",
    )
    args = parser.parse_args()

    controller = RobotController(args)

    controller.run_simulation()