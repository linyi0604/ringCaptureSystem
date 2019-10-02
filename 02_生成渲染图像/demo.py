import bpy
import numpy as np
import os
import shutil

# 三个方向的相对位置序列
s_x = np.load('/home/linyi/PycharmProjects/对接环目标捕获智能控制器仿真系统/01_PI控制与数据生成/data/s_x.npy')
s_y = np.load('/home/linyi/PycharmProjects/对接环目标捕获智能控制器仿真系统/01_PI控制与数据生成/data/s_y.npy')
s_z = np.load('/home/linyi/PycharmProjects/对接环目标捕获智能控制器仿真系统/01_PI控制与数据生成/data/s_z.npy')

# 三个方向加速度序列
a_x = np.load('/home/linyi/PycharmProjects/对接环目标捕获智能控制器仿真系统/01_PI控制与数据生成/data/a_x.npy')
a_y = np.load('/home/linyi/PycharmProjects/对接环目标捕获智能控制器仿真系统/01_PI控制与数据生成/data/a_y.npy')
a_z = np.load('/home/linyi/PycharmProjects/对接环目标捕获智能控制器仿真系统/01_PI控制与数据生成/data/a_z.npy')
# 三个方向速度序列
v_x = np.load('/home/linyi/PycharmProjects/对接环目标捕获智能控制器仿真系统/01_PI控制与数据生成/data/v_x.npy')
v_y = np.load('/home/linyi/PycharmProjects/对接环目标捕获智能控制器仿真系统/01_PI控制与数据生成/data/v_y.npy')
v_z = np.load('/home/linyi/PycharmProjects/对接环目标捕获智能控制器仿真系统/01_PI控制与数据生成/data/v_z.npy')

N = s_x.shape[0]  # 多少条 从远到近的接近路径
M = s_x.shape[1]  # 每条路径 控制次数 离散采样次数

r = 0.75  # 圆环半径0.75m
rcos30, rsin30 = r * np.cos(np.pi / 6), r * np.sin(np.pi / 6)
file_path = "/home/linyi/PycharmProjects/对接环目标捕获智能控制器仿真系统/02_生成渲染图像/data/"

# 镜头相关设置
bpy.context.object.data.type = 'PANO'  # 设置为全景相机
bpy.context.object.data.angle = 0.785398   # 设置镜头视野45度
bpy.context.object.data.lens_unit = 'FOV'  # 镜头单位为视野
bpy.context.object.data.clip_start = 0.1  # 起始距离
bpy.context.object.data.clip_end = 25  # 结束距离

# 设置保存为灰度图模式 图像深度8 压缩率100%  保存为png
bpy.context.scene.render.image_settings.color_mode = 'BW'
bpy.context.scene.render.image_settings.color_depth = '8'
bpy.context.scene.render.image_settings.compression = 100
bpy.data.scenes['Scene'].render.image_settings.file_format = "PNG"

# 设置保存图像的大小
bpy.context.scene.render.resolution_x = 2048
bpy.context.scene.render.resolution_y = 2048

for n in range(N):
    # 当前第n条接近路线
    # 生成路线序号的文件夹
    cur_file_path = file_path + str(n) + "/"
    if os.path.exists(cur_file_path):
        shutil.rmtree(cur_file_path)
    os.mkdir(cur_file_path)

    for m in range(M):
        # 第m次控制 离散采样
        v = (vx, vy, vz) = (v_x[n][m], v_y[n][m], v_z[n][m])
        a = (ax, ay, az) = (a_x[n][m], a_y[n][m], a_z[n][m])
        s = (sx, sy, sz) = (s_x[n][m], s_y[n][m], s_z[n][m])
        # 相机坐标
        camera0 = (camera1_x, camera1_y, camera1_z) = (sx, sy + r, sz)
        camera1 = (camera2_x, camera2_y, camera2_z) = (sx + rcos30, sy - rsin30, sz)
        camera2 = (camera3_x, camera3_y, camera3_z) = (sx - rcos30, sy - rsin30, sz)
        camera = (camera0, camera1, camera2)
        for i in range(3):
            bpy.data.objects["Camera"].location = camera[i]
            # 图像命名规范：　路径序号n_控制序号m_相机编号i.png
            file_name = "" + str(n) + "_" + str(m) + "_" + str(i)
            # 下一张图像的路径
            bpy.context.scene.render.filepath = cur_file_path + file_name

            # 将图片拍照保存到本地
            bpy.ops.render.render(write_still=True)



