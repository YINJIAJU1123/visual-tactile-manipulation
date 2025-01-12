import numpy as np
from PIL import Image

# 设置图片尺寸
width, height = 1280, 720
mask = np.zeros((height, width), dtype=np.uint8)  # 初始化全黑图片

#mask[:, 200:1080] = 255
mask[:, :] = 255  # 255 表示白色
mask[:, :200] = 0
mask[:, 1080:1280] = 0
#mask[:, 450:830] = 255
mask[580:720, :] = 0


# 将 NumPy 数组转换为 Pillow 图像并保存为模式为 "1" 的图片
binary_mask_image = Image.fromarray(mask, mode='L')  # 转换为灰度图像
binary_mask_image = binary_mask_image.convert('1')  # 转换为模式为 "1"
binary_mask_image.save("workspace_mask.png")  # 保存图像

# 如果需要显示这张图片，可以用以下代码
binary_mask_image.show()
