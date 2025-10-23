# nav_cloning
# nav_cloning 使用ガイド

本プロジェクトでは、ROS・Gazebo環境でのロボット走行データ収集、深層学習モデルによる学習・評価を行います。

---

## 1. シミュレーションでのデータ収集

### 配置によるデータ収集（静止状態）

- **目的**：目標経路上にロボットを配置して、画像と角速度を取得  
- **保存先**：
  - `img/`：画像（命名例：`lane{lane}_{img_type}_{save_img_no}.jpg`）
  - `ang/`：角速度
- **実行コマンド**：
  ```bash
  roslaunch nav_cloning nav_cloning_sim.launch script:=set_collect.py use_waypoint_nav:=false use_cmd_vel:=false
  ```

* **備考**：
  ロボットを自動で下図のように配置し、静止状態での画像・角速度データを収集します。

  ![collect\_data\_resize](https://github.com/YukiTakahashi4690/nav_cloning/assets/72371474/d3e43a62-31b8-4a51-b581-4c9d201a0ebb)

### 自律移動によるデータ収集

* **目的**：3カメラまたは9カメラを使って走行中にデータ取得
* **実行コマンド（例：3カメラ）**：

  ```bash
  roscd nav_cloning/sh/collect
  ./collect_run_3cam_sim
  ```
  ※ 9cam版はsh名を変更して使用してください。また、任意の走行終了ポイントでプログラムを停止してください

### コントローラ走行によるデータ収集

* **目的**：コントローラを使用して走行、手動でデータを収集
* **実行コマンド（例：3カメラ）**：

  ```bash
  roscd nav_cloning/sh/collect
  ./collect_run_3cam_sim_contllor.sh
  ```
  別のターミナルで
  ```bash
  roslaunch icart_mini_driver teleop_joy.launch
  ```
  ※任意の走行終了ポイントでプログラムを停止してください
---

## 2. オフライン学習

### 学習前の準備

学習用スクリプト内で以下の変数を適切に設定してください：

* `self.pro`：使用するデータフォルダ名
* `self.data`：データ数（例：616）

### 配置データでの学習

```bash
roscd nav_cloning/sh/learning
./learning.sh
```

### 走行データ（3cam or 9cam）での学習

```bash
roscd nav_cloning/sh/learning
./learning_run_3cam.sh
```

### 3cam走行データを画像拡張し学習

  ```bash
  roscd nav_cloning/sh/learning
  ./learning_viewpoint_aug.sh
  ```
  ※9camと3camの画像拡張については安定した経路追従を確認しています

---

## 3. モデルテスト・経路追従評価

### 学習済みモデルでの経路追従テスト

```bash
roscd nav_cloning/sh/model_test
./model_test
```

### 特定のモデル出力を確認

```bash
roslaunch nav_cloning nav_cloning_sim.launch script:=model_test.py use_waypoint_nav:=true use_cmd_vel:=false model_num:=1
```

### モデルの経路復帰性能を評価
経路周辺に移動するためのpath.csvがない場合、以下で作成
```bash
roslaunch nav_cloning nav_cloning_sim.launch script:=path_collector.py
```

経路復帰性能を評価
```bash
roscd nav_cloning/sh/analysis
./path_recovery_evaluator.sh
```
※ 上記コマンドにより、経路から逸脱した際の復帰挙動も確認可能

---

## install
* 環境 ubuntu20.04, ros noetic

* ワークスペースの用意
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ../
catkin build
```
* nav_cloningの用意
```
cd ~/catkin_ws/src
wget https://raw.githubusercontent.com/KOYAMA-Yuya/nav_cloning_offline/master/nav_cloning.install
wstool init
wstool merge nav_cloning.install
wstool up
```
* 依存パッケージのインストール
```
cd ~/catkin_ws/src
rosdep init
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ../
catkin build
```
* 共通
```
pip3 install scikit-image
pip3 install tensorboard
```
* ＜CPU のみ＞
```
pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cpu
```
* ＜GPU 使用＞使用しているデバイスを確認し，セットアップします
- nvidia driver
- CUDA
- cuDNN
その後インストールしたCUDAのバージョンに対応したPytorchのバージョンを下記からダウンロードします
https://pytorch.org/get-started/locally/
## Docker
作成次第追加