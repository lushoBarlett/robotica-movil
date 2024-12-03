from transforms3d import quaternions as quat
import numpy as np
import csv
import matplotlib.pyplot as plt
import argparse

FIELD_NAMES = ['#timestamp', ' p_RS_R_x [m]', ' p_RS_R_y [m]', ' p_RS_R_z [m]', ' q_RS_w []', ' q_RS_x []', ' q_RS_y []', ' q_RS_z []']

def get_timestamp(row):
    return row[FIELD_NAMES[0]]

def get_translation(row):
    return [float(row[FIELD_NAMES[1]]), float(row[FIELD_NAMES[2]]), float(row[FIELD_NAMES[3]])]

def get_quaternion(row):
    return [float(row[FIELD_NAMES[4]]), float(row[FIELD_NAMES[5]]), float(row[FIELD_NAMES[6]]), float(row[FIELD_NAMES[7]])]

def write_pose_to_csv(writer, timestamp, t, q):
    writer.writerow({
        FIELD_NAMES[0]: timestamp,
        FIELD_NAMES[1]: t[0],
        FIELD_NAMES[2]: t[1],
        FIELD_NAMES[3]: t[2],
        FIELD_NAMES[4]: q[0],
        FIELD_NAMES[5]: q[1],
        FIELD_NAMES[6]: q[2],
        FIELD_NAMES[7]: q[3]
    })


def plot_pose(ax, translation, quaternion, color):

    # Plot the position as a point
    ax.scatter(translation[0], translation[1], translation[2], color=color)

    # Plot an arrow indicating the forward direction (from the position)
    forward_direction = quat.rotate_vector(np.array([1,0,0]), quaternion)

    ax.quiver(translation[0], translation[1], translation[2], 
          forward_direction[0], forward_direction[1], forward_direction[2], 
          length=0.01, color='g')


def plot_poses(ax, num_poses):
    with open('data/trajectory_estimation/cam.csv', 'r') as cam_file, \
         open('data/trajectory_estimation/body.csv', 'r') as body_file:

        cam_reader = csv.DictReader(cam_file)
        cam_rows = list(cam_reader)

        body_reader = csv.DictReader(body_file)
        body_rows = list(body_reader)

        num_poses = min(num_poses, len(cam_rows), len(body_rows))

        for i in range(0, num_poses):
            cam_t = get_translation(cam_rows[i])
            cam_q = get_quaternion(cam_rows[i])

            body_t = get_translation(body_rows[i])
            body_q = get_quaternion(body_rows[i])

            plot_pose(ax, body_t, body_q, color='b')
            plot_pose(ax, cam_t, cam_q, color='r')


def parse_args():
    parser = argparse.ArgumentParser(description="Plot poses of cam0 and imu.")
    parser.add_argument('--num-poses', type=int, default=10, help='Number of poses to plot')
    return parser.parse_args()


def main():
    args = parse_args()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    plot_poses(ax, args.num_poses)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    robot_handle = ax.scatter([], [], [], color='b', label='Robot')
    camera_handle = ax.scatter([], [], [], color='r', label='Camera')

    ax.legend(handles=[robot_handle, camera_handle])

    plt.show()


if __name__ == "__main__":
    main()