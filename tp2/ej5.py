import os
import re
import matplotlib.pyplot as plt
import random
from matplotlib.gridspec import GridSpec


class DataPoint:
    def __init__(self, time, x, y, rotation, linear_velocity, angular_velocity):
        self.time = time
        self.x = x
        self.y = y
        self.rotation = rotation
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

    def __str__(self):
        return f'DataPoint(time={self.time}, x={self.x}, y={self.y}, rotation={self.rotation}, linear_velocity={self.linear_velocity}, angular_velocity={self.angular_velocity})'


def read_data_point_file(file_path):
    data_points = []
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            parts = line.split('\t')
            time = parts[0]
            sec, nanosec = re.findall(r'\d+', time)
            time = int(sec) + int(nanosec) / 1e9
            x = float(parts[1])
            y = float(parts[2])
            rotation = float(parts[3])
            linear_velocity = float(parts[4])
            angular_velocity = float(parts[5])
            data_point = DataPoint(time, x, y, rotation, linear_velocity, angular_velocity)
            data_points.append(data_point)
    return data_points


def plot_data_points(data_points, mosaic, figsize, points=False):
    def column(name):
        return [getattr(data_point, name) for data_point in data_points]

    x = column('x')
    y = column('y')

    time = column('time')
    time_start_adjusted = [t - time[0] for t in time]

    rot = column('rotation')
    linvel = column('linear_velocity')
    angvel = column('angular_velocity')

    step = len(data_points) // 6

    random_points = [step, 3 * step, 5 * step]

    if points:
        sizes = [200 if i in random_points else 1.5 for i in range(len(data_points))]
        flatsizes = [200 if i in random_points else 0 for i in range(len(data_points))]
    else:
        sizes = 1.5
        flatsizes = 0

    fig = plt.figure(layout="constrained", figsize=figsize)
    axd = fig.subplot_mosaic(mosaic)
    with plt.style.context('bmh'):
        if 'P' in axd:
            axd['P'].scatter(x, y, s=sizes, c=time_start_adjusted)
            axd['P'].set_xlabel('x')
            axd['P'].set_ylabel('y')

        if 'X' in axd:
            axd['X'].scatter(time_start_adjusted, x, s=flatsizes, c=time_start_adjusted)
            axd['X'].plot(time_start_adjusted, x, c='r')
            axd['X'].set_xlabel('Tiempo (s)')
            axd['X'].set_ylabel('x')

        if 'Y' in axd:
            axd['Y'].scatter(time_start_adjusted, y, s=flatsizes, c=time_start_adjusted)
            axd['Y'].plot(time_start_adjusted, y, c='g')
            axd['Y'].set_xlabel('Tiempo (s)')
            axd['Y'].set_ylabel('y')

        if 'R' in axd:
            axd['R'].scatter(time_start_adjusted, rot, s=flatsizes, c=time_start_adjusted)
            axd['R'].plot(time_start_adjusted, rot, c='b')
            axd['R'].set_xlabel('Tiempo (s)')
            axd['R'].set_ylabel('ϴ')

        if 'V' in axd:
            axd['V'].scatter(time_start_adjusted, linvel, s=flatsizes, c=time_start_adjusted)
            axd['V'].plot(time_start_adjusted, linvel, c='m')
            axd['V'].set_xlabel('Tiempo (s)')
            axd['V'].set_ylabel('v')

        if 'W' in axd:
            axd['W'].scatter(time_start_adjusted, angvel, s=flatsizes, c=time_start_adjusted)
            axd['W'].plot(time_start_adjusted, angvel, c='c')
            axd['W'].set_xlabel('Tiempo (s)')
            axd['W'].set_ylabel('ω')

    plt.show()
    return fig


def save_fig(fig, output_file_path):
    fig.savefig(output_file_path, format='pdf')


def std_pipeline(input_file_path, mosaic, figsize, output_file_path, points=False):
    data_points = read_data_point_file(input_file_path)
    fig = plot_data_points(data_points, mosaic, figsize, points)
    save_fig(fig, output_file_path)


if __name__ == '__main__':
    # scan ./datos folder for .txt files and output them as .pdf files
    data_folder = 'datos'
    images_folder = 'imagenes'

    if not os.path.exists(images_folder):
        os.makedirs(images_folder)

    file_name = 'log_keyboard_teleop.txt'
    mosaic = \
    """
    PPPPXX
    PPPPYY
    PPPPRR
    PPPPVV
    """
    figsize = (12, 8)
    input_file_path = os.path.join(data_folder, file_name)
    output_file_path = os.path.join(images_folder, file_name.replace('.txt', '.pdf'))
    std_pipeline(input_file_path, mosaic, figsize, output_file_path)

    file_name = 'log_circ.txt'
    mosaic = \
    """
    PPPXX
    PPPYY
    PPPRR
    """
    figsize = (10, 6)
    input_file_path = os.path.join(data_folder, file_name)
    output_file_path = os.path.join(images_folder, file_name.replace('.txt', '.pdf'))
    std_pipeline(input_file_path, mosaic, figsize, output_file_path, points=True)

    rest = ['log_lin_neg_ang_neg.txt', 'log_lin_neg_ang_pos.txt', 'log_lin_pos_ang_neg.txt', 'log_lin_pos_ang_pos.txt']
    for file_name in rest:
        mosaic = \
        """
        PPPXX
        PPPYY
        PPPRR
        """
        figsize = (10, 6)
        input_file_path = os.path.join(data_folder, file_name)
        output_file_path = os.path.join(images_folder, file_name.replace('.txt', '.pdf'))
        std_pipeline(input_file_path, mosaic, figsize, output_file_path)
