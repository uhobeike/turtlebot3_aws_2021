# turtlebot3_aws_2021

### How to install

* `catkin build`を使用するためにインストール。

```
sudo apt install python-catkin-tools
```

* ワークスペースの構築

```
mkdir -p ~/turtlebot3_aws_ws/src
cd turtlebot3_aws_ws
catkin build
```

* ソースコードのクローン&依存パッケージのインストール

```
cd ~/turtlebot3_aws_ws/src  <--各自、変更をお願いします。
git clone --recursive https://github.com/uhobeike/turtlebot3_aws_2021.git
rosdep update
rosdep install -r -y --from-paths --ignore-src ./
```

### How to source update
* クローンしたリポジトリのアップデートを行う場合の方法
```
cd ~/turtlebot3_aws_ws/src/turtlebot3_aws_2021

cd turtlebot3
git pull
cd ..

cd turtlebot3_msgs 
git pull 
cd ..

cd turtlebot3_simulations
git pull 
```

### How to build

```
catkin build
```

* ターミナル起動時に自動で設定を読み込むように設定(適宜、書き換えお願いします)
```
echo "source ~/turtlebot3_aws_ws/devel/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### How to update
リポジトリの更新の仕方。

例
```
ubuntu@ubuntu:~/turtlebot3_aws_2021_ws/src/turtlebot3_aws_2021$ git submodule update
```

### How to mapping
```
roslaunch turtlebot3_gazebo turtlebot3_aws_2021_mapping.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
rosrun map_server map_saver -f ~/map
```

### How to navigation
* waypointのデータは、turtlebot3_navigation/csv下にある。

* turtlebot3_navigation.launchの中を見ると使っているものが分かる。
```
roslaunch turtlebot3_gazebo turtlebot3_aws_2021.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
開始するためには、ターミナルでgoと打つ
終了する時は、ctl+cとqを打つ
```

### How to waypoint set 
* 正直、使い方が分かっていないと使いづらいが、慣れれば問題ない。
```
roslaunch turtlebot3_navigation waypoint_set.launch 
次にsを打ってスタートさせる。
rvizにある、2D Pose Estimateを使ってwaypointをおいていく。
置きミスのときは、jを打って削除できる。
終了時は、fを押して、csvを出力する。
```
