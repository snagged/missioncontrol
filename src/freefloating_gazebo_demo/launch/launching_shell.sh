rosservice call gazebo/unpause_physics

rosrun gazebo_ros spawn_model -file /home/lar1/.gazebo/models/kayak_ros/KAYAK.sdf -sdf -model KAYAK -y 0.2 -x -0.3 -z 0.0

rostopic pub -1 /gazebo/buoyancy_plugin_params freefloating_gazebo/fluid_objects_params "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
model_name: 'KAYAK'
link_name: 'kayak_ros'
compensation: 1.1
limit: 0.1
x_damping: 5
y_damping: 5
z_damping: 5"

rosservice call gazebo/pause_physics

rosservice call /gazebo/set_model_state '{model_state: { model_name: KAYAK, pose: { position: { x: 0, y: 0 ,z: 0 }, orientation: {x: 3.14, y: 3.14, z: 3.14, w: 3.14 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0 } } , reference_frame: world } }'
