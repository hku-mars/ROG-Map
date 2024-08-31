import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV文件
data = pd.read_csv('rm_performance_log.csv')

# 获取列标签
columns = data.columns.tolist()

# 删除第一行（包含标签）
data = data.iloc[1:]

# 将数据转换为数值类型
data = data.astype(float)

# 创建一个包含列数个子图的图表
fig, axes = plt.subplots(nrows=1, ncols=len(columns), figsize=(3 * len(columns), 3))

# 绘制每一列的箱线图，并设置ylim
for i, column in enumerate(columns):
    ax = axes[i]
    ax.boxplot(data[column])
    ax.set_xlabel(column)
    if i < 4:
        ax.set_ylim([data[columns[:4]].min().min(), data[columns[:4]].max().max()])

# 设置整体标题
fig.suptitle('Box Plot of Columns')
plt.subplots_adjust(wspace=1.5)
# 调整子图之间的间距
plt.tight_layout()

# 显示图表
plt.show()
