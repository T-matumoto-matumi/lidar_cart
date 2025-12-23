import cv2

img = cv2.imread('square_room.pgm', cv2.IMREAD_UNCHANGED)
# 画像を2倍に拡大
img_resized = cv2.resize(img, None, fx=5.0, fy=5.0, interpolation=cv2.INTER_NEAREST)
cv2.imshow('ROS2 Map', img_resized)
cv2.waitKey(0)
cv2.destroyAllWindows()