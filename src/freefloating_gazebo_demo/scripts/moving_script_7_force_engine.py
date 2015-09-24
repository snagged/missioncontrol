# python script that moves vehicles according to current orientation.

#testing variables. should be set False when not testing.
debug_mode = False
dry_run = False


import sys, os
from time import sleep
import subprocess

command_format = """rostopic pub -1 /gazebo/seven_engines_force/%s seven_engines_force/engine_forces "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
engine_forces: [%f, %f, %f,%f, %f, %f, %f]"
"""



zero_vector_3 = (0.0, 0.0, 0.0)


  
def start_script(params):
  if (params[1].find("--filename=") == 0):
    print(params[1][len("--filename="):])
    move_object_from_file(params[1][len("--filename="):])
  else:
    move_object_from_parameters(params[1:])

    
    
    
# every line in the file should be in format: "model_name e1 e2 e3 e4 e5 e6 e7"  the seven engine forces.
# if need to wait, write "sleep x" when x is seconds to sleep
def move_object_from_file(filename):
  filename.replace('"','').replace("'","")
  f = open(filename, 'r')
  for line in f.readlines():
    listOfParams = line.split(" ")
    
    listOfParams = cleanComments(listOfParams)
    
    if len(listOfParams) == 0: continue
    else: move_object_from_parameters(listOfParams)
      

      
      
# the command should be in format: "model_name e1 e2 e3 e4 e5 e6 e7"  the seven engine forces.
# if need to wait, write "sleep x" when x is seconds to sleep      
def move_object_from_parameters(listOfParams):
  if listOfParams[0] == "sleep":
      if debug_mode:
	print ("sleeping for %f seconds" % float(listOfParams[1]) )
      if not dry_run:
	sleep(float(listOfParams[1]))
  else:
    modelName = listOfParams[0]
    engine_forces = [float(listOfParams[i]) for i in range(1,8)]
    
    
    formatted_command = command_format % (modelName, engine_forces[0], engine_forces[1], engine_forces[2],
					    engine_forces[3], engine_forces[4], engine_forces[5], engine_forces[6])
    if not dry_run:
      subprocess.Popen(formatted_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE)
    if debug_mode:
      print (formatted_command)
      
      

  
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