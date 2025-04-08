import pyrealsense2 as rs
import numpy as np
import cv2

# 手眼标定矩阵（转换：相机坐标 -> 基坐标）
# T_cam_to_base = np.array([[0.11706393, 0.80613097, -0.58004215, 1.06607572],
#                           [0.99255536, -0.07519943, 0.09580655, -0.06666815],
#                           [0.03361379, -0.58693943, -0.80893276, 0.81717161],
#                           [0., 0., 0., 1.]])

T_cam_to_base = np.array(
[[ 0.11180418,  0.82102671, -0.55983477,  1.05438264],
 [ 0.99344147, -0.0787655,   0.08288576, -0.06608274],
 [ 0.02395576 ,-0.56543005 ,-0.82444829 , 0.83905749],
 [ 0. ,         0.,          0.,          1.        ]]
)



# -----------------------------
# 初始化 RealSense 相机
pipeline = rs.pipeline()
config = rs.config()
# 同时启用彩色图和深度图
config.enable_stream(rs.stream.color, 1280,720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# 开始流
profile = pipeline.start(config)

# 获取彩色图像内参
color_profile = profile.get_stream(rs.stream.color)
intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
print(f"Camera intrinsics: fx={intrinsics.fx}, fy={intrinsics.fy}, ppx={intrinsics.ppx}, ppy={intrinsics.ppy}")

# 构造相机内参矩阵（仅用于可选的手动计算）
CAMERA_MATRIX = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                          [0, intrinsics.fy, intrinsics.ppy],
                          [0, 0, 1]], dtype=np.float32)

# 获取深度传感器并提取深度尺度（将深度值转为米）
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print(f"Depth Scale: {depth_scale}")

# 全局变量保存当前帧，供鼠标回调使用
current_depth_frame = None
current_color_frame = None


def mouse_callback(event, x, y, flags, param):
    global current_depth_frame, current_color_frame, intrinsics, depth_scale
    if event == cv2.EVENT_LBUTTONDOWN:
        # 点击时获取该像素的深度（单位：米）
        if current_depth_frame is None:
            print("当前没有深度帧数据")
            return

        depth_value = current_depth_frame.get_distance(x, y)
        print(f"像素 ({x}, {y}) 处深度: {depth_value:.3f} m")

        # 利用 RealSense SDK 函数将像素点转换为相机坐标系下的 3D 点
        point_camera = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth_value)
        point_camera = np.array(point_camera)  # [X, Y, Z]（单位：米）
        print("相机坐标系下的 3D 点:", point_camera)

        # 转换到齐次坐标（4x1向量）
        point_camera_hom = np.append(point_camera, 1.0)
        # 使用手眼标定矩阵转换到基坐标系
        point_base_hom = T_cam_to_base @ point_camera_hom
        # 如果最后一个分量不为1，需要归一化
        point_base = point_base_hom[:3] / point_base_hom[3]
        print("基坐标系下的 3D 点:", point_base)

        # 可选：在图像上画个圆标记点击位置
        cv2.circle(current_color_frame, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow('Color Frame', current_color_frame)


# 设置显示窗口及鼠标回调
cv2.namedWindow('Color Frame', cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback('Color Frame', mouse_callback)

try:
    while True:
        # 等待一帧数据
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # 更新全局变量，便于鼠标回调获取最新帧数据
        current_depth_frame = depth_frame
        current_color_frame = np.asanyarray(color_frame.get_data())

        # 显示彩色图
        cv2.imshow('Color Frame', current_color_frame)
        key = cv2.waitKey(1)
        if key == 27:  # 按 ESC 退出
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
