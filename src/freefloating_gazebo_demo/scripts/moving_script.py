# python script that moves vehicles according to current orientation.

#testing variables. should be set False when not testing.
debug_mode = False
dry_run = False
auto_sleep = True


import sys, os
from time import sleep
import subprocess

command_format = """rostopic pub --once /gazebo/relative_force_engine/%s relative_force_engine/object_params "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
force: {x: %f, y: %f, z: %f}
torque: {x: %f, y: %f, z: %f}
force_duration: %f
torque_duration: %f
command_types: %d"
"""

zero_vector_3 = (0.0, 0.0, 0.0)


  
def start_script(params):
  if (params[1].find("--filename=") == 0):
    print(params[1][len("--filename="):])
    move_object_from_file(params[1][len("--filename="):])
  else:
    move_object_from_parameters(params[1:])

    
    
    
# every line in the file should be in format: "model_name a b c d e f g h"  (a,b,c)-force, (d,e,f)-torque, g-force duration, h-torque duration - write (0,0,0) if no force/torque is needed.
# if need to wait, write "sleep x" when x is seconds to sleep
def move_object_from_file(filename):
  filename.replace('"','').replace("'","")
  f = open(filename, 'r')
  for line in f.readlines():
    listOfParams = line.split(" ")
    
    listOfParams = cleanComments(listOfParams)
    
    if len(listOfParams) == 0: continue
    else: move_object_from_parameters(listOfParams)
      

      
      
# the command should be in format: "model_name a b c d e f g h"  (a,b,c)-force, (d,e,f)-torque, g-force duration, h-torque duration - write (0,0,0) if no force/torque is needed.
# if need to wait, write "sleep x" when x is seconds to sleep      
def move_object_from_parameters(listOfParams):
  if listOfParams[0] == "sleep":
      if debug_mode:
	print ("sleeping for %f seconds" % float(listOfParams[1]) )
      if not dry_run:
	sleep(float(listOfParams[1]))
  else:
    modelName = listOfParams[0]
    force = (float(listOfParams[1]), float(listOfParams[2]), float(listOfParams[3]))
    torque = (float(listOfParams[4]), float(listOfParams[5]), float(listOfParams[6]))
    forceDuration = float(listOfParams[7])
    torqueDuration = float(listOfParams[8])
    command_types = 0

    if (force != zero_vector_3 and forceDuration != 0.0):
      command_types += 1
    if (torque != zero_vector_3 and torqueDuration != 0.0):
      command_types += 2
    
    formatted_command = command_format % (modelName, force[0], force[1], force[2],
					    torque[0], torque[1], torque[2],
					    forceDuration, torqueDuration, command_types)
    if not dry_run:
      subprocess.Popen(formatted_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE)
    if debug_mode:
      print (formatted_command)
    if (auto_sleep):
      sleep(max(forceDuration, torqueDuration))
      
      

  
# Auxiliary function, cleans all values that are with comments.  
def cleanComments(myList):
  for i in range(len(myList)):
    if myList[i].find("#") == -1: 
      continue
    elif myList[i].find("#") == 0:
      myList = myList[0:i]
      break
    else:
      myList = myList[0:i+1]
      myList[i] = myList[i][0:myList[i].find("#")]
      break
  return myList

  
  

if __name__ == "__main__":
  start_script(sys.argv)