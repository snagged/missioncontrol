rosservice call gazebo/unpause_physics
#
nohup python moving_script.py --filename=movement_script_bluefin &
nohup python moving_script.py --filename=movement_script_hydrocamel &
nohup python moving_script.py --filename=movement_script_test 
#
rosservice call gazebo/pause_physics
echo finished!