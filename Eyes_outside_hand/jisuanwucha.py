import pyrealsense2 as rs
import numpy as np
import cv2
import json
import os
from pupil_apriltags import Detector
import panda_py

# Define tag size in meters
tag_size = 0.06  # 60mm

# Initialize Intel RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Get camera intrinsics dynamically
intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
fx, fy = intrinsics.fx, intrinsics.fy
cx, cy = intrinsics.ppx, intrinsics.ppy
camera_params = [fx, fy, cx, cy]

CAMERA_MATRIX = np.array([[fx, 0., cx],
                          [0., fy, cy],
                          [0., 0., 1.]], dtype=np.float32)
DIST = np.zeros((5, 1))  # 无畸变假设

print(f"Camera Intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}")

# Initialize AprilTag detector
detector = Detector(families="tag25h9")

# Create a window
cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

# Initialize variables
data = []
image_counter = 0  # Counter for image filenames
reproj_errors = []  # 保存每一帧的重投影误差

def save_to_json(filename, data):
    with open(filename, 'w') as f:
        json.dump(data, f, indent=4)

try:
    while True:
        # Wait for a coherent pair of frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        detections = detector.detect(gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)

        for detection in detections:
            # Draw border
            for idx in range(len(detection.corners)):
                pt1 = tuple(map(int, detection.corners[idx]))
                pt2 = tuple(map(int, detection.corners[(idx + 1) % 4]))
                cv2.line(color_image, pt1, pt2, (0, 255, 0), 2)

            center = tuple(map(int, detection.center))
            cv2.circle(color_image, center, 5, (0, 0, 255), -1)
            cv2.putText(color_image, f"ID: {detection.tag_id}",
                        (center[0] - 10, center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            t = detection.pose_t.flatten()
            R = detection.pose_R

            # GUI 显示
            pose_text = f"XYZ: ({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) m"
            cv2.putText(color_image, pose_text, (center[0] - 50, center[1] + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # 坐标轴可视化
            axis_length = 0.2
            axes = np.array([[axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]])
            axes_img_pts = []
            for i in range(3):
                axis_end = t + R @ axes[i]
                x, y = int(fx * axis_end[0] / axis_end[2] + cx), int(fy * axis_end[1] / axis_end[2] + cy)
                axes_img_pts.append((x, y))
            cv2.line(color_image, center, axes_img_pts[0], (0, 0, 255), 2)
            cv2.line(color_image, center, axes_img_pts[1], (0, 255, 0), 2)
            cv2.line(color_image, center, axes_img_pts[2], (255, 0, 0), 2)

            # === 构造变换矩阵 ===
            homogeneous_matrix = np.eye(4)
            homogeneous_matrix[:3, :3] = R
            homogeneous_matrix[:3, 3] = t

            # === 计算重投影误差 ===
            object_point = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)  # AprilTag 坐标系原点
            rvec, _ = cv2.Rodrigues(R)
            tvec = t.reshape(3, 1)
            projected_point, _ = cv2.projectPoints(object_point, rvec, tvec, CAMERA_MATRIX, DIST)
            projected_point = projected_point[0][0]
            reproj_error = np.linalg.norm(projected_point - np.array(center))
            reproj_errors.append(reproj_error)
            print(f"重投影误差: {reproj_error:.4f} 像素")

        # 显示图像
        cv2.imshow('RealSense', color_image)
        key = cv2.waitKey(1)

        if key == ord('r'):
            hostname = '192.168.1.100'
            panda = panda_py.Panda(hostname)
            matrix = panda.get_pose()
            print("pose_matrix: ", matrix)

            image_filename = f'{image_counter}.png'
            cv2.imwrite(image_filename, color_image)

            data.append({
                'image': image_filename,
                'matrix': matrix.tolist(),
                'homogeneous_matrix': homogeneous_matrix.tolist()
            })
            print(f"已记录图像及矩阵: {image_filename}")
            image_counter += 1

        elif key == ord('q'):
            intrinsics_data = {
                'width': intrinsics.width,
                'height': intrinsics.height,
                'ppx': intrinsics.ppx,
                'ppy': intrinsics.ppy,
                'fx': intrinsics.fx,
                'fy': intrinsics.fy
            }
            save_to_json('data.json', {'data': data, 'intrinsics': intrinsics_data})
            print("数据已保存到 data.json，退出。")

            if reproj_errors:
                mean_error = np.mean(reproj_errors)
                print(f"\n📊 平均重投影误差: {mean_error:.4f} 像素（共 {len(reproj_errors)} 帧）")
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
