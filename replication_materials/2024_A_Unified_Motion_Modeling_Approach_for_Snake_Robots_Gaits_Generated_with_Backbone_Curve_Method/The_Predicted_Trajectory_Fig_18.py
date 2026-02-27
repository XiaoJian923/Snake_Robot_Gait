'''
复现图18中的红色曲线
'''
import math
import numpy as np
import matplotlib.pyplot as plt


def local_to_global(point_local, rotation_matrix, translation_vector):
    """
    Convert point coordinates from local coordinate system to global coordinate system

    Parameters:
    point_local: Point coordinates in local system, shape (3,) or (3, n) numpy array
    rotation_matrix: Rotation matrix from local to global system, shape (3, 3)
    translation_vector: Translation vector from local origin to global origin, shape (3,)

    Returns:
    point_global: Point coordinates in global system
    """
    # Ensure inputs are numpy arrays
    point_local = np.asarray(point_local)
    rotation_matrix = np.asarray(rotation_matrix)
    translation_vector = np.asarray(translation_vector)

    # Transformation formula: P_global = R * P_local + T
    if point_local.ndim == 1:
        # Single point
        return rotation_matrix @ point_local + translation_vector
    elif point_local.ndim == 2:
        # Multiple points, each column is a point
        return rotation_matrix @ point_local + translation_vector.reshape(-1, 1)
    else:
        raise ValueError("point_local dimension must be 1 or 2")

def plot_circle(ax, circle_points, center, str_seg, str_l_g):
    ax.plot(circle_points[0, :], circle_points[1, :], circle_points[2, :], 'b-', linewidth=2, label='Circle')
    ax.scatter(center[0], center[1], center[2], color='red', s=100, label='Center')

    ax.quiver(0, 0, 0, 0.5, 0, 0, color='r', arrow_length_ratio=0.1, label='X axis')
    ax.quiver(0, 0, 0, 0, 0.5, 0, color='g', arrow_length_ratio=0.1, label='Y axis')
    ax.quiver(0, 0, 0, 0, 0, 0.5, color='b', arrow_length_ratio=0.1, label='Z axis')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(str_seg + ' in ' + str_l_g + ' Coordinates')
    ax.legend()
    ax.set_xlim([-0.3, 1.7])
    ax.set_ylim([-0.2, 0.6])
    ax.set_zlim([-0.2, 0.5])



if __name__ == "__main__":
    # 步态参数
    r1 = 0.2
    r2 = 0.15
    beta = 2 * math.atan(r2 / r1)
    W_G = 2 * r1 * math.cos(beta / 2) + 2 * r2 * math.sin(beta / 2)
    L_G = 4 * r1

    num_points = 1000  # 等分点的数量

    # TODO: 片段11111111111111111111111111111111111111111111111111111111111
    center_local1 = np.array([0.0, -0.2, 0])  # 圆心在局部坐标系中的坐标
    theta1 = np.linspace(0, np.pi, num_points, endpoint=False)   # 生成圆周上的等分点（使用极坐标），生成角度参数

    # 在局部坐标系中生成圆上的点，圆在XY平面内，所以Z坐标为0
    circle_points_local1 = np.array([
        center_local1[0] + r1 * np.cos(theta1),  # X coordinate
        center_local1[1] + r1 * np.sin(theta1),  # Y coordinate
        np.zeros_like(theta1)  # Z coordinate
    ])

    # 定义旋转矩阵
    theta_rot1 = - beta / 2
    R1 = np.array([
        [1, 0, 0],
        [0, np.cos(theta_rot1), -np.sin(theta_rot1)],
        [0, np.sin(theta_rot1), np.cos(theta_rot1)]
    ])

    T1 = np.array([r1, W_G, 0.0])    # 定义平移向量

    circle_points_global1 = local_to_global(circle_points_local1, R1, T1)   # 将局部坐标系中的圆点转换到全局坐标系
    center_global1 = local_to_global(center_local1, R1, T1)     # 计算圆心在全局坐标系中的位置

    # TODO: 片段2222222222222222222222222222222222222222222222222222222222
    center_local2 = np.array([0, 0, 0])  # 圆心在局部坐标系中的坐标
    theta2 = np.linspace(- beta / 2 + math.pi / 2, beta / 2 + math.pi / 2, num_points, endpoint=False)  # 生成圆周上的等分点（使用极坐标），生成角度参数

    # 在局部坐标系中生成圆上的点，圆在YZ平面内，所以X坐标为0
    circle_points_local2 = np.array([
        np.zeros_like(theta2),  # X coordinate
        center_local2[1] + r2 * np.cos(theta2),  # Y coordinate
        center_local2[2] + r2 * np.sin(theta2),  # Z coordinate
    ])

    # 定义旋转矩阵
    theta_rot2 = 0
    R2 = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    T2 = np.array([2 * r1, W_G / 2, 0])  # 定义平移向量

    circle_points_global2 = local_to_global(circle_points_local2, R2, T2)  # 将局部坐标系中的圆点转换到全局坐标系
    center_global2 = local_to_global(center_local2, R2, T2)  # 计算圆心在全局坐标系中的位置

    # TODO: 片段3333333333333333333333333333333333333333333333333333333333333
    center_local3 = np.array([0.0, 0.2, 0])  # 圆心在局部坐标系中的坐标
    theta3 = np.linspace(np.pi, 2 * np.pi, num_points, endpoint=False)  # 生成圆周上的等分点（使用极坐标），生成角度参数

    # 在局部坐标系中生成圆上的点，圆在XY平面内，所以Z坐标为0
    circle_points_local3 = np.array([
        center_local3[0] + r1 * np.cos(theta3),  # X coordinate
        center_local3[1] + r1 * np.sin(theta3),  # Y coordinate
        np.zeros_like(theta3)  # Z coordinate
    ])

    # 定义旋转矩阵
    theta_rot3 = beta / 2
    R3 = np.array([
        [1, 0, 0],
        [0, np.cos(theta_rot3), -np.sin(theta_rot3)],
        [0, np.sin(theta_rot3), np.cos(theta_rot3)]
    ])

    T3 = np.array([3 * r1, 0.0, 0.0])  # 定义平移向量

    circle_points_global3 = local_to_global(circle_points_local3, R3, T3)  # 将局部坐标系中的圆点转换到全局坐标系
    center_global3 = local_to_global(center_local3, R3, T3)  # 计算圆心在全局坐标系中的位置

    # TODO: 片段444444444444444444444444444444444444444444444444444444444
    center_local4 = np.array([0, 0, 0])  # 圆心在局部坐标系中的坐标
    theta4 = np.linspace(- beta / 2 + math.pi / 2, beta / 2 + math.pi / 2, num_points,
                         endpoint=False)  # 生成圆周上的等分点（使用极坐标），生成角度参数

    # 在局部坐标系中生成圆上的点，圆在YZ平面内，所以X坐标为0
    circle_points_local4 = np.array([
        np.zeros_like(theta4),  # X coordinate
        center_local4[1] + r2 * np.cos(theta4),  # Y coordinate
        center_local4[2] + r2 * np.sin(theta4),  # Z coordinate
    ])

    # 定义旋转矩阵
    theta_rot4 = 0
    R4 = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    T4 = np.array([4 * r1, W_G / 2, 0])  # 定义平移向量

    circle_points_global4 = local_to_global(circle_points_local4, R4, T4)  # 将局部坐标系中的圆点转换到全局坐标系
    center_global4 = local_to_global(center_local4, R4, T4)  # 计算圆心在全局坐标系中的位置

    # 绘图
    fig = plt.figure(figsize=(12, 5))

    # 局部坐标系绘图
    ax1 = fig.add_subplot(241, projection='3d')
    plot_circle(ax1, circle_points_local1, center_local1, 'segment-1', 'Local')
    p1 = circle_points_global1[:, ::-1]

    ax2 = fig.add_subplot(242, projection='3d')
    plot_circle(ax2, circle_points_local2, center_local2, 'segment-2', 'Local')
    p2 = circle_points_global2[:, 1:]

    ax3 = fig.add_subplot(243, projection='3d')
    plot_circle(ax3, circle_points_local3, center_local3, 'segment-3', 'Local')
    p3 = circle_points_global3[:, 1:]

    ax4 = fig.add_subplot(244, projection='3d')
    plot_circle(ax4, circle_points_local4, center_local4, 'segment-4', 'Local')
    p4 = circle_points_global4[:, ::-1][:, 1:]

    # 全局坐标系绘图
    ax5 = fig.add_subplot(245, projection='3d')
    plot_circle(ax5, p1, center_global1, 'segment-1', 'Global')

    ax6 = fig.add_subplot(246, projection='3d')
    plot_circle(ax6, np.concatenate((p1, p2), axis=1), center_global2, 'segment-2', 'Global')

    ax7 = fig.add_subplot(247, projection='3d')
    plot_circle(ax7, np.concatenate((p1, p2, p3), axis=1), center_global3, 'segment-3', 'Global')

    # ax8 = fig.add_subplot(248, projection='3d')
    # plot_circle(ax8, np.concatenate((p1, p2, p3, p4), axis=1), center_global4, 'segment-4', 'Global')

    # TODO: 注释掉上面两行，再重复一个周期
    ax8 = fig.add_subplot(248, projection='3d')
    p5 = p1.copy()[:, 1:]
    p5[0, :] += L_G
    p6 = p2.copy()
    p6[0, :] += L_G
    p7 = p3.copy()
    p7[0, :] += L_G
    p8 = p4.copy()
    p8[0, :] += L_G
    plot_circle(ax8, np.concatenate((p1, p2, p3, p4, p5, p6, p7, p8), axis=1), center_global1, 'segment-all', 'Global')

    # plt.tight_layout()
    # plt.show()

    #TODO：下面绘制图18的红色曲线
    points_all = np.concatenate((p1, p2, p3, p4, p5, p6, p7, p8), axis=1)   # (3, n)
    vectors = points_all[:, 1:] - points_all[:, :-1]   # (3, n-1)

    # unit_x = [1, 0, 0]
    # unit_y = [0, 1, 0]

    # 相当于在x轴和y轴方向上做投影
    d_x = vectors[0, :]
    d_y = vectors[1, :]

    # 做累加和
    delta_theta1_s = (theta1[1] - theta1[0]) * r1
    delta_theta2_s = (theta2[1] - theta2[0]) * r2
    delta_s_h_array = np.concatenate([
        np.full(num_points - 1, delta_theta1_s),
        np.full(num_points - 1, delta_theta2_s),
        np.full(num_points - 1, delta_theta1_s),
        np.full(num_points - 1, delta_theta2_s),
        np.full(num_points - 1, delta_theta1_s),
        np.full(num_points - 1, delta_theta2_s),
        np.full(num_points - 1, delta_theta1_s),
        np.full(num_points - 1, delta_theta2_s)
    ])

    delta_s_h = np.cumsum(delta_s_h_array)
    d_shift_x = np.cumsum(d_x)
    d_shift_y = np.cumsum(d_y)

    d_robot_x = delta_s_h - d_shift_x
    d_robot_y = - d_shift_y

    plt.figure(figsize=(10, 6))
    plt.plot(d_robot_x, d_robot_y, 'bo-', linewidth=1, markersize=2, label='predicted')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('The predicted trajectory calculated from the model (by xjxf0923, imperfect)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.show()

