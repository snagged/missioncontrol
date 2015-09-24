source /opt/ros/indigo/setup.bash
cp src/gazebo_worlds/models/. ~/.gazebo/models/ -R
cp src/gazebo_worlds/world/buoyancy.conf ~/.ros/ -R
catkin_make
source $PWD/devel/setup.bash
catkin_make
catkin_make
catkin_make
APPEND="source /opt/ros/indigo/setup.bash 
source $PWD/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PWD"
echo "$APPEND" >> ~/.bashrc
bash
