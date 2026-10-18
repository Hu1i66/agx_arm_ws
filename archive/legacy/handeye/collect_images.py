import pyrealsense2 as rs
import cv2
import os

# 创建保存文件夹
os.makedirs("my_dataset", exist_ok=True)
count = 0

# 配置 RealSense 相机
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 启动相机
pipeline.start(config)

print("按 s 保存图片，按 q 退出")

while True:
    # 等待相机帧
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue

    # 转成图像
    img = np.asanyarray(color_frame.get_data())

    # 显示画面
    cv2.imshow("RealSense Camera", img)

    key = cv2.waitKey(1)
    if key == ord('s'):
        filename = f"my_dataset/img_{count:03d}.jpg"
        cv2.imwrite(filename, img)
        print(f"已保存: {filename}")
        count += 1
    elif key == ord('q'):
        break

# 关闭相机
pipeline.stop()
cv2.destroyAllWindows()