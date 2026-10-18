import cv2
from PIL import Image

try:
    # Newer OpenCV
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    marker_img = cv2.aruco.generateImageMarker(dictionary, 582, 300)
except AttributeError:
    # Older OpenCV
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    marker_img = cv2.aruco.drawMarker(dictionary, 582, 300)

# 图像大小为 300x300 像素。我们要让它在纸上打印出来正好是 30mm x 30mm。
# 30mm = 30 / 25.4 inches = 1.1811 inches.
# 分辨率 DPI = 300 pixels / 1.1811 inches ≈ 254 dpi.
pil_img = Image.fromarray(marker_img)
pil_img.save("/home/lxf/handeye/marker_582_30mm.pdf", resolution=254.0)
print("PDF generated successfully at /home/lxf/handeye/marker_582_30mm.pdf")
