import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math

# 车辆位置和形状信息
x = 0  # x坐标
y = 0  # y坐标
yaw = math.pi / 6  # 方向角(弧度)
# yaw =0 # 方向角(弧度)
locationPoint2Head = 9.6  # 矩形中心点距离车头距离
locationPoint2Rear = 3.8  # 矩形中心点距离车尾距离

# 计算矩形的宽度和高度
width_vehi = 6.7  # 宽度
length_vehi = locationPoint2Head + locationPoint2Rear  # 长度

width_x = length_vehi  # 绘图,x轴方向定义为宽
height_y = width_vehi  # 绘图,y轴方向定义为高

# 计算矩形的左下角坐标
x_A3 = x - locationPoint2Rear * math.cos(yaw) + 0.5 * height_y * math.sin(yaw)
y_A3 = y - locationPoint2Rear * math.sin(yaw) - 0.5 * height_y * math.cos(yaw)

# 创建一个矩形对象
rectangle = patches.Rectangle((x_A3, y_A3), width=width_x, height=height_y, angle=math.degrees(yaw))

# 创建一个绘图对象
fig, ax = plt.subplots()

# 将矩形对象添加到绘图对象中
ax.add_patch(rectangle)
# 绘制散点图
ax.scatter(x, y, color="red", marker="o")

# 设置坐标轴范围
ax.set_xlim(x - 50, x + 50)
ax.set_ylim(y - 50, y + 50)
# 设置x轴和y轴的相等比例
ax.axis("equal")
# 显示图形
plt.show()
a = 1
