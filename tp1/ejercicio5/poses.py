import numpy as np
import csv
from utils import get_quaternion, get_translation
import transforms3d.affines as aff
from transforms3d import quaternions as quat

def transformation_matrix_from_pose(t,q):
    transformation_matrix = aff.compose(t, quat.quat2mat(q), np.ones(3))

    return transformation_matrix

def transformation_matrix_to_pose(matrix):
    translation, rotation_matrix, _, _ = aff.decompose44(matrix)
    quaternion = quat.mat2quat(rotation_matrix)

    translation = [round(t, 6) for t in translation]
    quaternion = [round(q, 6) for q in quaternion]
    return translation,quaternion

def cam0_wrt_imu():
    cam2imu = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
            0.0, 0.0, 0.0, 1.0]

    Bi_e_Ci = np.array(cam2imu).reshape((4, 4))

    return Bi_e_Ci

def b0_wrt_world():
    with open('mav0/state_groundtruth_estimate0/data.csv', 'r') as file:
        reader = csv.DictReader(file)

        rows = list(reader)

        row = rows[0]
        t = get_translation(row)
        q = get_quaternion(row)

        W_e_B0 = transformation_matrix_from_pose(t,q)

    return W_e_B0

B_T_C = cam0_wrt_imu()
C_T_B = np.linalg.inv(B_T_C)

W_T_B0 = b0_wrt_world()
B0_T_W = np.linalg.inv(W_T_B0)

W_T_C0 = np.dot(W_T_B0,B_T_C)
C0_T_W = np.dot(C_T_B, B0_T_W)
