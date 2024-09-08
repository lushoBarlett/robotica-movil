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
