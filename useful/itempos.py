import matplotlib.pyplot as plt

"""
    文件的格式如下
    fstream out;
    out.open("log.txt", std::ios::app);
    out << "NowFrame is " << frameID << " NewItemCount is " << NewItemCount << endl;
    out << "x is " << x << " y is " << y << " val is " << val << endl;
    out.close();

"""

def show(path):
    # 初始化列表用于存储生成物品的位置和价值信息
    x_positions = []
    y_positions = []
    values = []

    # 从文件中读取数据并提取信息
    with open(path, 'r') as file:
        for line in file:
            if "x is" in line and "y is" in line and "val is" in line:
                parts = line.split()
                x_index = parts.index("x") + 2
                y_index = parts.index("y") + 2
                val_index = parts.index("val") + 2
                x_positions.append(int(parts[x_index]))
                y_positions.append(int(parts[y_index]))
                values.append(int(parts[val_index]))

    # 创建热力图
    plt.hexbin(x_positions, y_positions, C=values, gridsize=30, cmap='inferno')

    # 添加颜色条
    plt.colorbar(label='Value')

    # 添加标题和轴标签
    plt.title('Generated Item Heatmap with Values')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')

    # 显示图形
    plt.savefig("heatmap.png")

if __name__ == '__main__':
    show('../log.txt')