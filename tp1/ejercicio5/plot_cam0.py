from transforms3d import quaternions as quat
import numpy as np
import csv
from utils import get_quaternion, get_translation
from poses import transformation_matrix_from_pose, transformation_matrix_to_pose, W_T_C0
import matplotlib.pyplot as plt
import argparse

def plot_pose(ax, translation, quaternion, color):

    # Plot the position as a point
    ax.scatter(translation[0], translation[1], translation[2], color=color)

    # Plot an arrow indicating the forward direction (from the position)
    forward_direction = quat.rotate_vector(np.array([1,0,0]), quaternion)

    ax.quiver(translation[0], translation[1], translation[2], 
          forward_direction[0], forward_direction[1], forward_direction[2], 
          length=0.01, color='g')


def plot_poses(ax, num_poses, step):
    with open('mav0/state_groundtruth_estimate0/data.csv', 'r') as imu_file, \
         open('mav0/cam0/pose_data.csv', 'r') as cam_file:

        imu_reader = csv.DictReader(imu_file)
        imu_rows = list(imu_reader)

        cam_reader = csv.DictReader(cam_file)
        cam_rows = list(cam_reader)

        num_poses = min(num_poses, len(imu_rows), len(cam_rows))

        for i in range(0, num_poses, step):
            imu_t = get_translation(imu_rows[i])
            imu_q = get_quaternion(imu_rows[i])

            cam0_t_wrt_c0 = get_translation(cam_rows[i])
            cam0_q_wrt_c0 = get_quaternion(cam_rows[i])

            C0_T_Ci = transformation_matrix_from_pose(cam0_t_wrt_c0,cam0_q_wrt_c0)

            W_T_Ci = np.dot(W_T_C0,C0_T_Ci)

            cam0_t, cam0_q = transformation_matrix_to_pose(W_T_Ci)

            plot_pose(ax, imu_t, imu_q, color='b')
            plot_pose(ax, cam0_t, cam0_q, color='r')


def parse_args():
    parser = argparse.ArgumentParser(description="Plot poses of cam0 and imu.")
    parser.add_argument('--num-poses', type=int, default=10, help='Number of poses to plot')
    parser.add_argument('--step', type=int, default=1, help='Step size for selecting poses')
    return parser.parse_args()


def main():
    args = parse_args()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    plot_poses(ax, args.num_poses, args.step)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    robot_handle = ax.scatter([], [], [], color='b', label='Robot')
    camera_handle = ax.scatter([], [], [], color='r', label='Camera')

    ax.legend(handles=[robot_handle, camera_handle])

    plt.show()


if __name__ == "__main__":
    main()