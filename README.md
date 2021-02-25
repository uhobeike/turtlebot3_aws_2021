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
cd ~/turtlebot3_aws_ws/src
git clone --recursive https://github.com/uhobeike/turtlebot3_aws_2021.git
rosdep update
rosdep install -r -y --from-paths --ignore-src ./
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

### How to use

```
roslaunch turtlebot3_gazebo turtlebot3_aws_2021.launch 
```