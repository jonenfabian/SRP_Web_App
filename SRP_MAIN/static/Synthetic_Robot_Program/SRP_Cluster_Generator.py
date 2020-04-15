#!/usr/bin/env python3
#SYNTHETIC ROBO PROGRAM GENERATOR
#Input for testing purposes
import sys
sys.path.append('..')
import json
#import Models (Database) to save data to the django Database
from SRP_APP.models import SrpClusterDb
#import SRP_Subfunctions.func_00_extract_operation_names_from_dfi_local as not_used
import static.Synthetic_Robot_Program.SRP_Subfunctions.func_astar_pathplanning as astar
import static.Synthetic_Robot_Program.SRP_Subfunctions.func_01_get_coordinates_from_opt1_opt2_opt3 as get_abs_coords
import static.Synthetic_Robot_Program.SRP_Subfunctions.func_02_use_relation_and_look_up_in_pick_skill_database as get_skill
import static.Synthetic_Robot_Program.SRP_Subfunctions.func_03_get_relative_highway_safe_points as get_safe_grid
import static.Synthetic_Robot_Program.SRP_Subfunctions.func_04_read_BOM_and_extract_relative_coordinates as get_rel_coords
import static.Synthetic_Robot_Program.SRP_Subfunctions.func_05_create_waypoints_to_reach_target_coordinates as get_safe_path
import static.Synthetic_Robot_Program.SRP_Subfunctions.func_06_use_relation_and_look_up_in_assembly_skill_database as get_ass_skill
import pickle
#print(len(read_BOM.func_read_BOM_and_extract_relative_coordinates(keyword="FW51530",BOM_file_path="media/bom_files/DAP009373_U_HPC_FRONT_CASE_AND_VANE_ASSY.txt")))
#From trash_extract_operation_names_from_dfi_local.py
def srp_cluster_generator(data): #data = Fitting Instruction
	return_data_dict={}
	for procedure in data:
		#try:

		print("Hi I'm in the function!")
		data[procedure]["absolute_coordinates_root_relation"]=get_abs_coords.get_coordinates_from_opt1_opt2_opt3(input=data[procedure]["relation"],db_path='media/SRP_Database/abs_coordinates_db.json')
		#Option1: There are no fix coordinates, robot must use Object Detection to get the absolute coordinates of the objects
		#Option2: Get fix coordinates from database
		#Option3: Get rough coordintes from database to do OpenCV detection, Lets use this for demonstrator
		#Function returns (1, [0.5, 0, 0, 0, 0, 0], [list with all relative coordinates (Bushings etc.)], calibration_skill.script) with Option-ID and absolute coordinates in robot base-Frame (if any)
		#TODO Function that controlls the robot to seek for objects depending on option-ID
		data[procedure]["absolute_coordinates_root_target"]=get_abs_coords.get_coordinates_from_opt1_opt2_opt3(input=data[procedure]["target"],db_path='media/SRP_Database/abs_coordinates_db.json')
		#Option1: There are no fix coordinates, robot must use Object Detection to get the absolute coordinates of the objects
		#Option2: Get fix coordinates from database
		#Option3: Get rough coordintes from database to do OpenCV detection, Lets use this for demonstrator
		#Function returns (1, [0.5, 0, 0, 0, 0, 0],calibration_skill.script) with Option-ID and absolute coordinates in robot base-Frame (if any)
		#TODO Function that controlls the robot to seek for objects depending on option-ID

		data[procedure]["custom_pick_skill"]=get_skill.use_relation_and_look_up_in_pick_skill_database(input=data[procedure]["relation"][0:data[procedure]["relation"].find(";")],db_path='media/SRP_Database/pick_skill_db.json') #gets a script-code with relative coordinates
		#Will be a relative start and end position to the absolute pose of the object. Which is created in Unity by Immanuel.Horn@ferchau.de
		#Startposition gripper/tool-action has to be programmed within the robot!
		#  Endposition gripper/tool-action has to be programmed within the robot!
		#Function returns ([0, 0, 0.05, 0, 0, 0], [0, 0, 0.0, 0, 0, 0])-> Start and End Pose realtive to origin of the object

		data[procedure]["safety_highway_points"]=get_safe_grid.get_relative_highway_safe_points_from_where_robot_can_move_to_target_coordinates(
					BOM_file_path_input="media/bom_files/DAP009373_U_HPC_FRONT_CASE_AND_VANE_ASSY.txt",
					BOM_file_relation_object="FW515", #"", #data[procedure]["relation"][0:data[procedure]["relation"].find("_")],
					#BOM_file_relation_object="",
					root_object_number="",
					cell_width=0.1,
					area_size=[-1.3,1.3],
					thickness_of_safety_zone=1,
					abs_coordinates=data[procedure]["absolute_coordinates_root_target"][1])
		#In 3D workspace we have obstacles (pointclouds from BOM) where we must maneuver around.
		#For each object in the robot workspace we extract subpart positions to find restricted area. with A* Algorithm we find routes which avoid these obstacles later
		#Function returns a grid (maze)
		#[ [[11],[12]]
		#  [[21],[22]] ]
		#like
		#[ [[rowID, colID, [x1[meters], y1[meters],x2[meters], y2[meters]], value=1/0 ],[rowID, colID, [x1[meters], y1[meters],x2[meters], y2[meters]], value=1/0 ]],
		#  [[rowID, colID, [x1[meters], y1[meters],x2[meters], y2[meters]], value=1/0 ],[rowID, colID, [x1[meters], y1[meters],x2[meters], y2[meters]], value=1/0 ]] ]

		#Lets extract the relative coordinates for the "relation"-object from the BOM
		data[procedure]["target_coordinates_abs"],data[procedure]["target_coordinates_rel"]=get_rel_coords.func_read_BOM_and_extract_relative_coordinates( #returns a list with robot friendly coordinates [[X,Y,Z,RX,RY,RZ],[X,Y,Z,RX,RY,RZ], ...]
			#keyword=data[procedure]["relation"][0:data[procedure]["relation"].find("_")], #use the FW/NPN/... ID 7-9 digits to find entries in the BOM-file
			keyword=data[procedure]["relation"][0:data[procedure]["relation"].find("_")],#"FW515", #"",
			BOM_file_path="media/bom_files/DAP009373_U_HPC_FRONT_CASE_AND_VANE_ASSY.txt", #BOM-file for part
			abs_coordinates=data[procedure]["absolute_coordinates_root_target"][1]) #the absolute coordinates of the target object (e.g. Front Case)
		#Function returns a list with robot friendly coordinates of the relative positions of "relation" objects (Objects to assemble) [[X,Y,Z,RX,RY,RZ],[X,Y,Z,RX,RY,RZ], ...]
		#Lets generate safe paths to each target coordinate (assembly object)
		save_waypoints_list=[]
		save_raw_path_for_visualization=[]
		end_poses_ur=[]
		counter=0 #this counter is used to also extract the pick-up positions --> data[procedure]["absolute_coordinates_root_relation"][3][counter]
		successfull_paths=0
		for target_coordinate in data[procedure]["target_coordinates_abs"]:
			temp_save_waypoints,temp_save_raw_path_for_visualization,success_flag,safe_end_pose=get_safe_path.depending_on_safe_highway_points_create_waypoints_to_reach_target_coordinates(data[procedure]["safety_highway_points"], #the grid, from func_03_get_relative_highway_safe_points.py
																			target_coordinate,# the target robot pose for assembly [x,y,z,rx,ry,rz] from func_04_read_BOM_and_extract_relative_coordinates.py
																			#start_pos=[-.50,-.50,.50]
																			start_pos=data[procedure]["absolute_coordinates_root_relation"][3][counter]
																			)
			#print("path: ",temp_save_waypoints)
			if success_flag==0:
				print("Object-ID %d: for %s there was no path suitable "%(counter,str(target_coordinate)))
				temp_save_waypoints=[]
				temp_save_raw_path_for_visualization=[]
			save_waypoints_list.append(temp_save_waypoints) #startpos in robot base coordinate system TODO: info on input (Tray) are not yet in "data"
			save_raw_path_for_visualization.append(temp_save_raw_path_for_visualization)
			end_poses_ur.append(safe_end_pose)
			print(counter," of ",len(data[procedure]["target_coordinates_abs"]), " but list lenght of pick up poses is ",len(data[procedure]["absolute_coordinates_root_relation"][3]))
			counter+=1
			successfull_paths+=success_flag

		data[procedure]["safe_move_to_target"]=save_waypoints_list
		#print("paths: ",save_waypoints_list)
		print("Objects processed: ",counter)
		print("successfull paths generated: ",successfull_paths)
		return_data_dict[data[procedure]["relation"]]={
			"Objects processed":counter,
			"successfull paths generated":successfull_paths,
		}
		#With a startpose and an endpose we can obtain a safe path through the environment
		#for starting lets use the A* (Astar) pathfinding algorithm in 2D (https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2)
		#This function returns a path consisting of robotic waypoints in base frame for each relation
		#sys.exit()
		#Lets show the results
		#get_safe_path.show_results_path(posetrans_poses=end_poses_ur,paths=save_raw_path_for_visualization,objects=data[procedure]["target_coordinates_abs"],the_grid=data[procedure]["safety_highway_points"],area_size=[-1.3,1.3],width=800,height=800)

		#Lets show the results in 3D
		get_safe_path.show_results_path3D(posetrans_poses=end_poses_ur,paths=data[procedure]["safe_move_to_target"],objects=data[procedure]["target_coordinates_abs"],save_name=data[procedure]["relation"])#,the_grid=data[procedure]["safety_highway_points"],area_size=[-1.3,1.3],width=800,height=800)



		data[procedure]["operation_skill"]=get_ass_skill.use_relation_and_look_up_in_assembly_skill_database(input=data[procedure]["relation"][0:data[procedure]["relation"].find(";")],db_path='media/SRP_Database/operation_skill_db.json')
		#the default is spiral search
		#The function returns Universal Robots Script Code of a skill, which beforehand has to be programmed by an expert
		#sys.exit()
		#except:
		#	print("Error! Procedural Step "+data[procedure]["original"]+" was not correctly processed")
			#raise GeneratorError("Ups, something went wrong!")
	#print(json.dumps(data, indent = 4))
		db_data=SrpClusterDb(
			original=data[procedure]["original"],
			operation=data[procedure]["operation"],
			target=data[procedure]["target"],
			relation= data[procedure]["relation"],
			absolute_coordinates_root_relation=data[procedure]["absolute_coordinates_root_relation"],
		    absolute_coordinates_root_target=data[procedure]["absolute_coordinates_root_target"],
		    custom_pick_skill=data[procedure]["custom_pick_skill"],
		    safety_highway_points=data[procedure]["safety_highway_points"],
		    target_coordinates_abs=data[procedure]["target_coordinates_abs"],
		    safe_move_to_target=data[procedure]["safe_move_to_target"],
		    operation_skill=data[procedure]["operation_skill"],
			)
		db_data.save()

	with open('SRP_Stored_Data.pkl', 'wb') as f:  # Python 3: open(..., 'wb')
		pickle.dump(data, f)
	f.close()

	# OPTIONAL: Getting back the objects:
	#with open('SRP_Stored_Data.pkl') as f:  # Python 3: open(..., 'rb')
	#    data = pickle.load(f)

	return return_data_dict,data
def main():
	data={
		"procedural_step_0": {
			"original": "Assemble all BUSH_BEARING onto HPC_FRONT_BOTTOM.",
			"operation": "Assemble",
			"target": "NPN25984_C__CASING_HPC_FRONT_BOTTOM.asm;0;2:",
			"relation": "FW51528_D__BUSH_BEARING;0;92:"
		},
		"procedural_step_1": {
			"original": "Assemble all BUSH_BEARING onto HPC_FRONT_BOTTOM.",
			"operation": "Assemble",
			"target": "NPN25984_C__CASING_HPC_FRONT_BOTTOM.asm;0;2:",
			"relation": "FW51530_D__BUSH_BEARING;0;92:"
		}
	}
	srp_cluster_generator(data)
if __name__ == "__main__":
	main()
