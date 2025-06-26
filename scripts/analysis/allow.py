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
csv_file = pkg_dir + '/data/analysis/path_trajectory.csv'
df = pd.read_csv(csv_file, names=['x', 'y'], header=None, skip_blank_lines=False)

segments = []
current_segment = []

for _, row in df.iterrows():
    if pd.isna(row['x']) or pd.isna(row['y']):
        if current_segment:
            segments.append(current_segment)
            current_segment = []
    else:
        current_segment.append((row['x'], row['y']))

# 最後のセグメントも追加
if current_segment:
    segments.append(current_segment)

# 必要に応じて、先頭・末尾の点を除外
segments = [seg[1:-1] for seg in segments if len(seg) > 2]

# セグメント数の確認
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
        ax.plot(curve[:, 0], curve[:, 1], color='black', lw=2, alpha=0.7)

        # 最後の部分に矢印
        arrow_start = curve[-2]
        arrow_end = curve[-1]
        ax.annotate('',
                    xy=arrow_end, xytext=arrow_start,
                    arrowprops=dict(arrowstyle='-|>', color='black', lw=2, alpha=0.7))

# 軸設定など
# 全x, y座標を抽出して範囲設定に使う
all_points = [pt for segment in segments for pt in segment]
x_list = np.array([pt[0] for pt in all_points])
y_list = np.array([pt[1] for pt in all_points])

ax.set_xlim(x_list.min() - 0.1, x_list.max() + 0.1)
ax.set_ylim(y_list.min() - 0.1, y_list.max() + 0.1)
ax.set_aspect('equal')
ax.grid(True)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('Robot trajectory when off-path')

plt.savefig(pkg_dir + '/data/analysis/output.png', dpi=500)  # 画像ファイルとして保存
plt.show()