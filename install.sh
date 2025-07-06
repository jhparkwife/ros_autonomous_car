mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/your_repo/lane_follower_pkg.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash