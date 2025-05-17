

## 構成要素の解説

### 1. **`<arg>` タグ**
- `arg` タグは、外部から渡される引数やデフォルト値を定義するもの。
- 例:
  ```xml
  <arg name="script" default="nav_cloning_node.py"/>
  ```
  - `script` という名前の引数で、デフォルト値は `nav_cloning_node.py`。
  - 他の値を使いたい場合、`roslaunch` コマンドで `script:=別の値` を渡せる。



### 2. **シミュレーション環境の設定**
- シミュレーターとして Gazebo を利用。
- 具体的には、以下の `<include>` タグで Gazebo を起動:
  ```xml
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find nav_cloning)/world/willow_garage.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
  ```
  - `world_name` で指定されたワールドファイル（`willow_garage.world`）を読み込む。
  - `paused=false` により、シミュレーションは起動と同時に実行状態になる。
  - `use_sim_time=true` で、Gazebo のシミュレーション時間を ROS のタイムスタンプとして使用。



### 3. **ロボットの設定**
- TurtleBot3 Waffle Pi をシミュレーションに登場するロボットとして設定。
- 以下のタグでURDFモデルを読み込み、Gazebo内にロボットをスポーンさせる:
  ```xml
  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_waffle_pi.urdf.xacro" />
  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model mobile_base -x $(arg robot_x) -y $(arg robot_y) -z 0.0 -Y $(arg robot_Y) -param robot_description" />
  ```
  - `robot_description`: TurtleBot3のURDFモデルをxacroファイルから生成。
  - `spawn_model`: Gazebo内にモデルをスポーン。位置（`x, y`）や向き（`Y`）は引数から設定。



### 4. **タイマー付きノードの起動**
- `timed_roslaunch` を使用し、一定の遅延後に各ノードを起動。
- 例:
  ```xml
  <node pkg="timed_roslaunch" type="timed_roslaunch.sh"
        args="5 nav_cloning nav_cloning.launch script:=$(arg script)"
        name="timed_roslaunch" output="screen" />
  ```
  - ★起動まで5秒待機して、`nav_cloning.launch` を起動。
  - 引数 `script` の値（例: `nav_cloning_node.py`）を渡す。

#### 起動されるノードの詳細
　**`nav_cloning.launch`**
   - ナビゲーション模倣（Cloning）の処理を行うノード。
   - スクリプト名を引数として渡せるよう設定されています。



### 5. **模倣学習関連の設定**
- `rosparam` で模倣学習のモードやパラメータを設定:
  ```xml
  <rosparam param="/nav_cloning_node/num" subst_value="true">$(arg num)</rosparam>
  ```
  - パラメータ `/nav_cloning_node/num` を引数 `num` で指定（デフォルト値は1）。
  - 例えば、異なるデータセット番号を指定する用途が考えられます。



### 6. **その他の設定**
- 初期位置の設定:
  ```xml
  <arg name="initial_pose_x" default="-9.20645" />
  <arg name="initial_pose_y" default="-16.42460" />
  <arg name="initial_pose_a" default="0.0" />
  ```
  - ロボットの初期位置（`x, y, a`）を指定。

- ウェイポイントナビゲーションの有効化:
  ```xml
  <arg name="use_waypoint_nav" default="true" />
  ```

