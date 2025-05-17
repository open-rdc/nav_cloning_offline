import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

path = '/home/y-takahashi/catkin_ws/src/nav_cloning/data/analysis/00_02'
files = [
    path + "/0m_5deg.csv", path + "/0m_0deg.csv", path + "/0m_-5deg.csv",
    path + "/02m_5deg.csv", path + "/02m_0deg.csv", path + "/02m_-5deg.csv",
    path + "/-02m_5deg.csv", path + "/-02m_0deg.csv", path + "/-02m_-5deg.csv"
]

# 各ファイルを読み込み、列名を設定し、角速度のデータのみを保持
dataframes_corrected = {}
for file in files:
    df = pd.read_csv(file)
    df.columns = ['Timestamp', 'Angular_Velocity']
    dataset_name = file.split("/")[-1].replace(".csv", "")
    dataframes_corrected[dataset_name] = df['Angular_Velocity']

# 統合されたデータフレームを作成
data_for_plot = pd.DataFrame()
for key, values in dataframes_corrected.items():
    temp_df = pd.DataFrame({
        'Angular_Velocity': values,
        'Condition': [key] * len(values)
    })
    data_for_plot = pd.concat([data_for_plot, temp_df], ignore_index=True)

# バイオリンプロットの作成
plt.figure(figsize=(12, 6))
sns.swarmplot(x='Condition', y='Angular_Velocity', data=data_for_plot)
plt.title('Violin Plot of Angular Velocity under Different Conditions')
plt.xlabel('Condition (Distance and Angle)')
plt.ylabel('Angular Velocity (rad/s)')
plt.xticks(rotation=45)  # ラベルを45度回転させて表示

# プロットを画像として保存
plt.savefig(path + '/angular_velocity_swarm_plot.png')
plt.show()
