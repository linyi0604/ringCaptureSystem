# coding:utf8

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

"""


"""


def get_a_PI(x, v, maxa, kpx, kix, kpv, kiv, dest=0):
    """
    从当前和之前一段时间的 位置和速度序列 给出一个控制决策 加速度
    :param x: 以目标环建立坐标系 相对位置偏移序列
    :param v: 以目标环建立坐标系 相对速度偏移序列
    :param maxa: 决策出来的最大加速度
    :param kpx:  相对位置偏差量对决策的影响系数
    :param kix:  累计位置偏差量对决策的影响系数
    :param kpv:  相对速度偏差量对决策的影响系数
    :param kiv:  累计速度偏差两对决策的影响系数
    :param dest: 终点的位置
    :return: a   根据当前以及之前的
    """

    error_x = dest-x[0] # 当前x的偏移
    mean_x = np.mean(dest-x)    # x的累计偏移
    error_v = -v[0] # 当前速度偏移
    mean_v = np.mean(-v)   # v的累计偏移

    a = kpx * error_x + kix * mean_x + kpv * error_v + kiv * mean_v
    if np.abs(a) > maxa:
        a = np.sign(a) * maxa
    return a


N = 800    # 每条路径上控制次数
M = 5000    # 一共产生M条轨迹
t = 0.2     # 每隔0.2s一次控制  参与运算下一次速度

dest = -0.3  # 终点位置

maxa = 0.05     # 最大加速度

K = 5  # 下面四个参数的放大系数
kpx = 0.01
kix = 0.01
kpv = 0.2
kiv = 0.2

X = np.zeros([M, N])
Vx = np.zeros([M, N])
Ax = np.zeros([M, N])

Y = np.zeros([M, N])
Vy = np.zeros([M, N])
Ay = np.zeros([M, N])

Z = np.zeros([M, N])
Vz = np.zeros([M, N])
Az = np.zeros([M, N])


# 一共 M条路径， 当前为第n条路径
for n in range(0, M):
    # 初始化x -30 ~ 30
    x0 = 6 * (np.random.rand(1, 1).item() - 0.5)
    # v的方向总是朝向走廊里面
    vx0 = -0.05 * np.random.rand(1, 1).item() if x0 > 0 else 0.05 * np.random.rand(1, 1).item()

    # 初始化y -30 ~ 30
    y0 = 6 * (np.random.rand(1, 1).item() - 0.5)
    # v的方向总是朝向走廊里面
    vy0 = -0.05 * np.random.rand(1, 1).item() if y0 > 0 else 0.05 * np.random.rand(1, 1).item()

    # 初始化z  -180 ~  -200
    z0 = -20 + 2 * (np.random.rand(1, 1).item() - 0.5)
    # z0 = -20
    # z方向一定朝向目标
    vz0 = 0.5 * np.random.rand(1, 1).item()

    X[n][0] = x0
    Vx[n][0] = vx0
    Y[n][0] = y0
    Vy[n][0] = vy0
    Z[n][0] = z0
    Vz[n][0] = vz0
    print("起点位置:(%s, %s, %s)" % (x0, y0, z0))
    # 一次一次开始控制决策， i表示第i次控制决策
    for i in range(0, N):
        # 依据当前16个位置做出决策
        x = X[n][0:16]
        vx = Vx[n][0:16]
        ax = get_a_PI(x, vx, maxa, K*kpx, K*kix, K*kpv, K*kiv)
        # 当前第n条路径的Vx所有位置后移一个位置
        Vx[n][1:N] = Vx[n][0:N-1]
        Vx[n][0] = vx[0] + ax*t
        X[n][1:N] = X[n][0:N-1]
        Ax[n][1:N] = Ax[n][0:N-1]
        Ax[n][0] = ax
        # X[n][0] = x[0] + vx[0]*t + 0.5*ax*t*t + (x[0]/100)*np.random.randn(1, 1).item()
        X[n][0] = x[0] + vx[0] * t + 0.5 * ax * t * t

        # 依据当前16个位置做出决策
        y = Y[n][0:16]
        vy = Vy[n][0:16]
        ay = get_a_PI(y, vy, maxa, K * kpx, K * kix, K * kpv, K * kiv)
        # 当前第n条路径的Vy所有位置后移一个位置
        Vy[n][1:N] = Vy[n][0:N - 1]
        Vy[n][0] = vy[0] + ay * t
        Y[n][1:N] = Y[n][0:N - 1]
        Ay[n][1:N] = Ay[n][0:N - 1]
        Ay[n][0] = ay
        # Y[n][0] = y[0] + vy[0]*t + 0.5*ay*t*t + (y[0]/100)*np.random.randn(1, 1).item()
        Y[n][0] = y[0] + vy[0] * t + 0.5 * ay * t * t

        z = Z[n][0:16]
        vz = Vz[n][0:16]
        az = get_a_PI(z, vz, maxa, K * kpx, K * kix, K * kpv, K * kiv, dest)
        # 当前第n条路径的Vy所有位置后移一个位置
        Vz[n][1:N] = Vz[n][0:N - 1]
        Vz[n][0] = vz[0] + az * t
        Z[n][1:N] = Z[n][0:N - 1]
        Az[n][1:N] = Az[n][0:N - 1]
        Az[n][0] = az
        # Z[n][0] = z[0] + vz[0] * t + 0.5 * az * t * t + (z[0]/100)*np.random.randn(1, 1).item()
        Z[n][0] = z[0] + vz[0] * t + 0.5 * az * t * t
    print("终点位置: (%s, %s, %s)" % (X[n][0], Y[n][0], Z[n][0]))
    print("已完成: %s / %s" % (n+1, M))



# 将所有过程 翻转，从远到近
X = np.flip(X, 1)
Y = np.flip(Y, 1)
Z = np.flip(Z, 1)
Vx = np.flip(Vx, 1)
Vy = np.flip(Vy, 1)
Vz = np.flip(Vz, 1)
Ax = np.flip(Ax, 1)
Ay = np.flip(Ay, 1)
Az = np.flip(Az, 1)

save_path = "./data/"
np.save(save_path+"s_x.npy", X)
np.save(save_path+"s_y.npy", Y)
np.save(save_path+"s_z.npy", Z)
np.save(save_path+"v_x.npy", Vx)
np.save(save_path+"v_y.npy", Vy)
np.save(save_path+"v_z.npy", Vz)
np.save(save_path+"a_x.npy", Ax)
np.save(save_path+"a_y.npy", Ay)
np.save(save_path+"a_z.npy", Az)

np.savetxt(save_path+"s_x.csv", X)
np.savetxt(save_path+"s_y.csv", Y)
np.savetxt(save_path+"s_z.csv", Z)
np.savetxt(save_path+"v_x.csv", Vx)
np.savetxt(save_path+"v_y.csv", Vy)
np.savetxt(save_path+"v_z.csv", Vz)
np.savetxt(save_path+"a_x.csv", Ax)
np.savetxt(save_path+"a_y.csv", Ay)
np.savetxt(save_path+"a_z.csv", Az)

# b =  numpy.loadtxt("filename.txt", delimiter=',')
# b = np.load("filename.npy")



# 初始化位置
fig_init_position = plt.figure()
init_position = fig_init_position.gca(projection="3d")
init_position.scatter(X[:, 0], Y[:, 0], Z[:, 0])
init_position.set_title("init_position")
init_position.set_xlabel("x")
init_position.set_ylabel("y")
init_position.set_zlabel("z")
# 终点位置
fig_final_position = plt.figure()
final_position = fig_final_position.gca(projection="3d")
final_position.scatter(X[:, -1], Y[:, -1], Z[:, -1])
final_position.set_title("final_position")
final_position.set_xlabel("x")
final_position.set_ylabel("y")
final_position.set_zlabel("z")

# 飞行轨迹
fig_s_xyz = plt.figure()
s_xyz = fig_s_xyz.gca(projection="3d")
s_xyz.set_xlabel("s_x")
s_xyz.set_ylabel("s_y")
s_xyz.set_zlabel("s_z")
s_xyz.set_title("s_xyz")
for i in range(M):
    s_xyz.plot(X[i], Y[i], Z[i])

# 加速度
fig_a_xyz = plt.figure()
a_xyz = fig_a_xyz.gca(projection="3d")
a_xyz.set_xlabel("a_x")
a_xyz.set_ylabel("a_y")
a_xyz.set_zlabel("a_z")
a_xyz.set_title("a_xyz")
for i in range(M):
    a_xyz.plot(Ax[i], Ay[i], Az[i])
# 速度
fig_v_xyz = plt.figure()
v_xyz = fig_v_xyz.gca(projection="3d")
v_xyz.set_xlabel("v_x")
v_xyz.set_ylabel("v_y")
v_xyz.set_zlabel("v_z")
v_xyz.set_title("v_xyz")
for i in range(M):
    v_xyz.plot(Vx[i], Vy[i], Vz[i])
plt.show()
