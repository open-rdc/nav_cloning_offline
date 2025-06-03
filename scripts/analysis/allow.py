import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch
from matplotlib.path import Path
import rospkg

# ROSパッケージのパス取得
rospack = rospkg.RosPack()
pkg_dir = rospack.get_path('nav_cloning')

# CSVファイル読み込み
csv_file = pkg_dir + '/data/path/path_trajectory.csv'
df = pd.read_csv(csv_file)

x_list = df['x'].values
y_list = df['y'].values

# 20点ずつ分割
segment_size = 20
segments = []
for i in range(0, len(x_list), segment_size):
    segment = list(zip(x_list[i:i+segment_size], y_list[i:i+segment_size]))
    # 先頭と末尾を除く
    segment = segment[1:-1]
    if segment:
        segments.append(segment)

# # 連続する重複点を削除
def remove_duplicates(segment):
    filtered = []
    prev = None
    for pt in segment:
        if pt != prev:
            filtered.append(pt)
        prev = pt
    return filtered

segments = [remove_duplicates(seg) for seg in segments]

# セグメント数表示
print(f"セグメント数: {len(segments)}")

# 描画
fig, ax = plt.subplots(figsize=(10, 10))

for segment in segments:
    if len(segment) < 4:
        # 直線矢印
        start, end = segment[0], segment[-1]
        ax.annotate('',
                    xy=end, xytext=start,
                    arrowprops=dict(arrowstyle='-|>', color='blue', lw=2, alpha=0.7))
    else:
        # Bezier曲線の制御点
        start = np.array(segment[0])
        end = np.array(segment[-1])
        c1 = np.array(segment[len(segment)//3])
        c2 = np.array(segment[2*len(segment)//3])

        # Bezier曲線生成
        t = np.linspace(0, 1, 100)
        curve = (1 - t)[:, None]**3 * start + \
                3 * (1 - t)[:, None]**2 * t[:, None] * c1 + \
                3 * (1 - t)[:, None] * t[:, None]**2 * c2 + \
                t[:, None]**3 * end

        # 曲線を描画
        ax.plot(curve[:, 0], curve[:, 1], color='blue', lw=2, alpha=0.7)

        # 最後の部分に矢印
        arrow_start = curve[-2]
        arrow_end = curve[-1]
        ax.annotate('',
                    xy=arrow_end, xytext=arrow_start,
                    arrowprops=dict(arrowstyle='-|>', color='blue', lw=2, alpha=0.7))

# 軸設定など
ax.set_xlim(x_list.min() - 0.1, x_list.max() + 0.1)
ax.set_ylim(y_list.min() - 0.1, y_list.max() + 0.1)
ax.set_aspect('equal')
ax.grid(True)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('20点ずつ分割されたセグメントごとの矢印表示')

plt.show()
