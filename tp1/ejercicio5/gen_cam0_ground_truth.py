import numpy as np
import csv
from poses import transformation_matrix_from_pose, transformation_matrix_to_pose, C0_T_W,B_T_C
from utils import get_quaternion, get_translation, write_pose_to_csv, get_timestamp, FIELD_NAMES
import argparse


def algorithm(infile,outfile,ns_to_s = False):

    reader = csv.DictReader(infile)
    rows = list(reader)

    writer = csv.DictWriter(outfile, fieldnames=FIELD_NAMES)
    writer.writeheader()

    for row in rows:
        t = get_translation(row)
        q = get_quaternion(row)
        timestamp = get_timestamp(row)
        if ns_to_s:
            timestamp = float(timestamp) * 1e-9

        W_T_Bi = transformation_matrix_from_pose(t,q)

        C0_e_Ci = np.dot(np.dot(C0_T_W, W_T_Bi), B_T_C)

        cam0_t, cam0_q = transformation_matrix_to_pose(C0_e_Ci)

        write_pose_to_csv(writer,timestamp,cam0_t,cam0_q)


def parse_args():
    parser = argparse.ArgumentParser(description="Process imu data and write cam0 wrt. initial cam0 data.")
    parser.add_argument('--ns-to-s', action='store_true', help='Convert timestamps from ns to s with ns precision')
    return parser.parse_args()


def main():
    args = parse_args()

    with open('mav0/state_groundtruth_estimate0/data.csv', 'r') as infile, \
         open("mav0/cam0/pose_data.csv", 'w', newline='') as outfile:
        algorithm(infile,outfile,args.ns_to_s)


if __name__ == "__main__":
    main()
