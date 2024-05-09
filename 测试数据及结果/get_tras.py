import numpy as np

def colmap2camera(input_file, output_file):  # 轨迹格式转换
    with open(input_file, 'r') as f:
        lines = f.readlines()

    lines = lines[4::2]

    with open(output_file, 'w') as f:
        for line in lines:
            columns = line.split()
            new_line = columns[9] + ' ' + ' '.join(columns[1:8]) + '\n'
            f.write(new_line)

input_file = './images.txt'
output_file = './images_c.txt'
colmap2camera(input_file, output_file)


def camera2world(input_file_path, output_file_path):  # 轨迹坐标系转换
    with open(input_file_path, 'r') as input_file:
        lines = input_file.readlines()

    for i, line in enumerate(lines):
        data_list = line.strip().split()
        qw, qx, qy, qz = map(float, data_list[1:5])
        tx, ty, tz = map(float, data_list[5:8])
        R1 = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])
        R2 = R1.T
        qw_new = 0.5 * np.sqrt(1 + R2[0, 0] + R2[1, 1] + R2[2, 2])
        qx_new = (R2[2, 1] - R2[1, 2]) / (4 * qw_new)
        qy_new = (R2[0, 2] - R2[2, 0]) / (4 * qw_new)
        qz_new = (R2[1, 0] - R2[0, 1]) / (4 * qw_new)
        data_list[1:5] = [qw_new, qx_new, qy_new, qz_new]
        data_list[5:8] = [-np.dot(R2, np.array([tx, ty, tz]))[i] for i in range(3)]
        lines[i] = ' '.join(map(str, data_list)) + '\n'

    with open(output_file_path, 'w') as output_file:
        output_file.writelines(lines)

input_file_path = './images_c.txt'
output_file_path = './images_w.txt'
camera2world(input_file_path, output_file_path)


def editpoints(input_file, output_file):  # 点云格式转换
    with open(input_file, 'r') as f:
        lines = f.readlines()

    with open(output_file, 'w') as f:
        for line in lines[3:]:
            columns = line.strip().split()[1:4]
            new_line = ' '.join(columns) + '\n'
            f.write(new_line)

# 调用函数并指定输入输出文件
input_file = './points3D.txt'
output_file = './points_w.txt'
editpoints(input_file, output_file)
