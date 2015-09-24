rosservice call gazebo/unpause_physics
#
nohup python moving_script.py --filename=movement_script_bluefin &
nohup python moving_script.py --filename=movement_script_bluefin_2 &
nohup python moving_script.py --filename=movement_script_bluefin_3
#
rosservice call gazebo/pause_physics
echo finished!
