#!/usr/bin/env python
"""
from:
data[procedure]["pick_up_coordinates"]=
function:
get_coordinates_from_opt1_opt2_opt3()
"""
	#Option1: There are no fix coordinates, robot must use Object Detection to get the absolute coordinates of the objects
	#Option2: Get fix coordinates from database
	#Option3: Get rough coordintes from database to do OpenCV detection, Lets use this for demonstrator
	#Lets add the specific custom pick skill for the procedural step
"""
#testing purpose from "target"
input="NPN25984_C__CASING_HPC_FRONT_BOTTOM"

#Firstly, look if we already have an entry in the "abs_coordinate_db"-database
#How to read the "database"-file
"""

def get_coordinates_from_opt1_opt2_opt3(input,db_path='media/SRP_Database/abs_coordinates_db.json'):
	import json
	with open(db_path) as json_file:
		abs_coordinate_db = json.load(json_file)

	#Find the entry
	#print(abs_coordinate_db)
	#print(json.dumps(abs_coordinate_db, indent = 4))
	no_entry_need_to_find_object=False
	need_to_verify=False
	robot_abs_coordinates=[]
	robot_rel_coordinates=[]
	calibration_skill=[]
	robot_combined_coordinates=[]
	if input in abs_coordinate_db:
		print("Yes there's an entry")
		if   abs_coordinate_db[input]["rough_coordinates_only"]==True:
			robot_abs_coordinates=abs_coordinate_db[input]["abs_coordinates"]
			need_to_verify=True
		elif abs_coordinate_db[input]["rough_coordinates_only"]==False:
			robot_abs_coordinates=abs_coordinate_db[input]["abs_coordinates"]
			need_to_verify=False
		else:
			#print('Wrong Value for rough_coordinates_only entry in dictionary.')
			raise ValueError('Wrong Value for rough_coordinates_only entry in dictionary.')
		if "rel_coordinates" in abs_coordinate_db[input]:
			robot_rel_coordinates=abs_coordinate_db[input]["rel_coordinates"]
			#now calculate the object coordinates abs + rel for each pick-up object (bushings in tray)
			for obj in robot_rel_coordinates:
				robot_combined_coordinates.append(list(map(sum,zip(robot_abs_coordinates,obj))))
		if "calibration_skill" in abs_coordinate_db[input]:
			calibration_skill=abs_coordinate_db[input]["calibration_skill"]
	else:
		print("no entry")
		no_entry_need_to_find_object=True

	if no_entry_need_to_find_object==False and need_to_verify==True:
		print("there were absolute coordinates for %s found, but only rough ones."%input)
		option=1
	elif no_entry_need_to_find_object==False and need_to_verify==False:
		print("there were absolute coordinates for %s found, and they are exact (fixture)."%input)
		option=2
	elif no_entry_need_to_find_object==True and need_to_verify==False:
		print("there were no coordinates found, the robot needs to search for the root object.")
		option=3
	return option, robot_abs_coordinates, robot_rel_coordinates,robot_combined_coordinates,calibration_skill

#Opt1: False,True,[xyz] #rough coordinates
#Opt2: False,False,[xyz] #exact coordinates
#Opt3: True, False, [] #nothing found
"""
#Testing
option, robot_abs_coordinates = get_coordinates_from_opt1_opt2_opt3(input)
print(get_coordinates_from_opt1_opt2_opt3(input))

"""
