# 初始化数组用于存储帧生成的新项目数量和特定价值项目的数量
new_item_count = [0] * 15000
value_count = [0] * 200

# 打开文件
def parse(path):
    with open(path, 'r') as file:
        # 逐行读取文件内容
        for line in file:
            # 如果行中包含"NowFrame"和"NewItemCount"，则提取相关信息
            if "NowFrame" in line and "NewItemCount" in line:
                # 分割行以获取关键信息
                parts = line.split()
                now_frame_index = parts.index("NowFrame") + 2
                new_item_count_index = parts.index("NewItemCount") + 2
                now_frame_value = int(parts[now_frame_index])
                new_item_count_value = int(parts[new_item_count_index])
                # 更新帧生成的新项目数量
                new_item_count[now_frame_value - 1] = new_item_count_value
            # 如果行中包含"x"、"y"和"val"，则提取相关信息
            elif "x is" in line and "y is" in line and "val is" in line:
                # 分割行以获取关键信息
                parts = line.split()
                val_index = parts.index("val") + 2
                val_value = int(parts[val_index])
                # 更新具有特定价值项目的数量
                value_count[val_value - 1] += 1

    # 打印帧生成的新项目数量
    print("帧生成的新项目数量：")
    for i in range(0, len(new_item_count), 100):
        print(new_item_count[i:i+100])

    # 打印每100帧生成物品的数量
    print("每100帧生成的物品数量：")
    for i in range(0, len(new_item_count), 100):
        total = sum(new_item_count[i:i+100])
        print("帧数范围：{}-{}，生成物品数量：{}".format(i+1, i+100, total))

    # 打印特定价值项目的数量
    print("\n特定价值项目的数量：")
    for i in range(0, len(value_count), 20):
        print(value_count[i:i+20])

    # 计算180-200价值的物品的总价值
    total_value_180_to_200 = sum((index + 180) * count for index, count in enumerate(value_count[179:200]))

    # 计算所有物品的总价值
    total_value_all = sum((index + 1) * count for index, count in enumerate(value_count))

    # 打印结果
    print("180-200价值的物品总价值：", total_value_180_to_200)
    print("所有物品的总价值：", total_value_all)

    # 计算生成物品的总数
    total_new_items = sum(new_item_count)

    # 打印生成物品的总数
    print("生成物品总数：", total_new_items)

if __name__ == "__main__":
    parse("../item.txt")