from oculus_reader.reader import OculusReader
import numpy as np


class HandPoseReader:
    def __init__(self):
        self.oculus_reader = OculusReader()

    def get_hand_poses(self):
        pose_data, button_data = self.oculus_reader.get_transformations_and_buttons()
        if len(pose_data) == 0:
            print("No data")
            return None, None
        else:
            pass
            # print(pose_data, button_data)

        # Extract and convert hand pose data
        left_hand_pose = pose_data.get("l")
        right_hand_pose = pose_data.get("r")

        left_position, left_rotation = self.extract_position_rotation(left_hand_pose)
        right_position, right_rotation = self.extract_position_rotation(right_hand_pose)

        return {
            "left_hand": {"position": left_position, "rotation": left_rotation},
            "right_hand": {"position": right_position, "rotation": right_rotation},
        }, button_data

    @staticmethod
    def extract_position_rotation(pose_matrix):
        """Extract position and rotation from a 4x4 transformation matrix."""
        if pose_matrix is None or pose_matrix.shape != (4, 4):
            print("Invalid pose matrix format.")
            return None, None

        # Extract position (last column of the matrix, excluding the last element)
        position = pose_matrix[:3, 3]
        position = np.array([position[2], position[0], position[1]]) * np.array(
            [-1, -1, 1]
        )

        # Extract rotation matrix (upper 3x3 part of the matrix)
        rotation = pose_matrix[:3, :3]
        rotation = np.array(
            [
                [-rotation[2, 0], -rotation[2, 1], -rotation[2, 2]],
                [-rotation[0, 0], -rotation[0, 1], -rotation[0, 2]],
                [rotation[1, 0], rotation[1, 1], rotation[1, 2]],
            ]
        )

        # rotate 90 degrees around y axis
        rotation = rotation @ np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])

        return position, rotation


if __name__ == "__main__":
    hand_pose_reader = HandPoseReader()

    while True:
        hand_poses, button_states = hand_pose_reader.get_hand_poses()

        if hand_poses is None:
            continue

        # # Print position and rotation of each hand
        # print("Left Hand - Position:", hand_poses["left"]["position"])
        # print("Left Hand - Rotation:\n", hand_poses["left"]["rotation"])

        # print("Right Hand - Position:", hand_poses["right"]["position"])
        # print("Right Hand - Rotation:\n", hand_poses["right"]["rotation"])

        # # Print button states if needed
        # print("Button States:", button_states)
