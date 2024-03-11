import numpy as np
import matplotlib.pyplot as plt

filename = 'map1.txt'

def plot_grid_from_txt(filename):
    # 定义字符和对应颜色的映射关系
    char_colors = {
        '*': [0.5, 0.5, 1.0],  # 蓝色
        '#': [0, 0, 0],  # 黑色
        'A': [1, 0, 0],  # 红色
        'B': [0.64, 0.16, 0.16],  # 棕色
        '.': [1, 1, 1]   # 白色
    }

    # 读取txt文件内容
    with open(filename, 'r') as f:
        content = f.readlines()

    # 创建一个空的二维数组来存储颜色编码
    colors = np.zeros((len(content), len(content[0]), 3))

    # 遍历txt文件中的每个字符，并根据字符类型分配颜色
    for i in range(len(content)):
        for j in range(len(content[i])):
            char = content[i][j]
            colors[i, j] = char_colors.get(char, [1, 1, 1])  # 获取字符对应的颜色，如果未定义则使用白色

    # 计算坐标轴范围
    x = np.arange(len(content[0]))
    y = np.arange(len(content))

    # 绘制网格图
    plt.imshow(colors, interpolation='nearest', extent=[x[0] - 0.5, x[-1] + 0.5, y[0] - 0.5, y[-1] + 0.5])
    plt.xticks([])
    plt.yticks([])

    # 添加网格线
    for i in range(len(content) + 1):
        plt.axhline(i - 0.5, color='gray', linewidth=0.5)  # 绘制水平线
    for j in range(len(content[0]) + 1):
        plt.axvline(j - 0.5, color='gray', linewidth=0.5)  # 绘制垂直线

    plt.show()


if __name__ == '__main__':
    plot_grid_from_txt(filename)
