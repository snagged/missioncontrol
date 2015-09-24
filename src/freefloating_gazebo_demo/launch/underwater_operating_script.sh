rosservice call /gazebo/apply_body_wrench "body_name: 'bluefin::bluefin_body'
reference_frame: ''
reference_point: {x: 0.0, y: 0.0, z: 0.0}
wrench:
 force: {x: 0.0, y: -150.0, z: 0.0}
 torque: {x: 0.0, y: 0.0, z: 0.0}
start_time: {secs: 0, nsecs: 0}
duration: {secs: 2, nsecs: 0}"

rosservice call /gazebo/apply_body_wrench "body_name: 'bluefin::bluefin_body'
reference_frame: ''
reference_point: {x: 0.0, y: 0.0, z: 0.0}
wrench:
 force: {x: 0.0, y: 0.0, z: 0.0}
 torque: {x: 0.0, y: -150.0, z: -150.0}
start_time: {secs: 2, nsecs: 0}
duration: {secs: 1, nsecs: 0}"

rosservice call /gazebo/apply_body_wrench "body_name: 'bluefin::bluefin_body'
reference_frame: ''
reference_point: {x: 0.0, y: 0.0, z: 0.0}
wrench:
 force: {x: -150.0, y: 0.0, z: 0.0}
 torque: {x: 0.0, y: 0.0, z: 0.0}
start_time: {secs: 3, nsecs: 0}
duration: {secs: 1, nsecs: 0}"

# rosservice call /gazebo/set_model_state "model_state:
#   model_name: 'bluefin'
#   pose:
#     position:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#       w: 0.0
#   twist:
#     linear:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#     angular:
#       x: 0.0
#       y: 0.0
#       z: 10.0
#   reference_frame: 'bluefin::bluefin_body'"
#   
#  sleep 0.2
#  
#  rosservice call /gazebo/set_model_state "model_state:
#   model_name: 'bluefin'
#   pose:
#     position:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#       w: 0.0
#   twist:
#     linear:
#       x: 0.0
#       y: 10.0
#       z: 0.0
#     angular:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#   reference_frame: 'bluefin::bluefin_body'"
  
 #sleep 0.3
 
#  rosservice call /gazebo/set_model_state "model_state:
#  model_name: 'bluefin'
#   pose:
#     position:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#       w: 0.0
#   twist:
#     linear:
#       x: 0.0
#       y: 1.0
#       z: 0.0
#     angular:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#   reference_frame: 'bluefin::bluefin_body'"