import pickle
from operator import add
import math
"""
#lets chain the data together to form a robot program
1. data[procedure]["absolute_coordinates_root_relation"]
2. data[procedure]["custom_pick_skill"]
3. data[procedure]["safe_move_to_target"]
4. data[procedure]["target_coordinates_abs"] #needs to be adapted with pose trans
5. data[procedure]["operation_skill"]
"""
def convert_cluster_2_script():
	# reading the data cluster from Pickle:
	with open('../../SRP_Stored_Data.pkl',"rb") as f:  # Python 3: open(..., 'rb')
		data = pickle.load(f)

	#for each procedure create a .Script-File

	for procedure in data:
		waypoint_list="##WAYPOINTS FOR %s\n"%data[procedure]["original"]
		#Lets check the options on how exact the coordinates of the target object are
		#Opt1: #rough coordinates only, require verification/calibration
		# if data[procedure]["absolute_coordinates_root_target"][0]==1:
		# 	#lets calibrate the target object
		# 	#waypoint_list+="#the rough coordinate of the target object\n"
		# 	waypoint_list+="Target_root_object_start_pose=p"+str(data[procedure]["absolute_coordinates_root_target"][1])+"\n" #[0.5, 0, 0, 0, 0, 0]
		# 	#waypoint_list+=data[procedure]["absolute_coordinates_root_target"][4] #e.g. skill of how to calibrate the front case
		# 	#output of the skill must be "Preal" (global variable) = the confirmed absolute coordinates [X,Y,Z,RX,RY,RZ]
		# elif data[procedure]["absolute_coordinates_root_target"][0]==2:
		# 	#Do nothing because the trustworthy cooordinates are already given
		# 	None
		# elif data[procedure]["absolute_coordinates_root_target"][0]==3:
		# 	#Oh Lord we have nothing and must find th eobject ourself!
		# 	#The RTDE_get_object_coordinates.py Script with Deep Learning object localization is not yet working
		# 	raise ObjectCoordinatesError("No Object found in Database, self detector not yet implemented")

		#Lets check the options on how exact the coordinates of the target object are
		#waypoint_list+=data[procedure]["absolute_coordinates_root_relation"][4] #e.g. skill of how to calibrate the bushing tray


		for i0 in range(len(data[procedure]["target_coordinates_abs"])): #number of objects to be assembled
			#waypoint_list+="##Handling %s Number %d\n"%(data[procedure]["relation"],i0+1)
			#each relation object has a pick up position
			abs_target_pose_object=data[procedure]["absolute_coordinates_root_relation"][1] #The position of the pick up object (e.g. the tray)
			rel_target_pose_child=data[procedure]["absolute_coordinates_root_relation"][2][i0] #The position of a child object (e.g. the bushing)
			#abs_target_pose=list(map(add, abs_target_pose_object, rel_target_pose_child))
			abs_target_pose=data[procedure]["absolute_coordinates_root_relation"][3][i0]
			rel_pick_skill_poses=data[procedure]["custom_pick_skill"] #relative to absolute coordinate
			true_pick_poses=[list(map(add, abs_target_pose, rel_pick_skill_poses[0])),list(map(add, abs_target_pose, rel_pick_skill_poses[1]))] #Vector addition
			#Linear Pick Pose
			#waypoint_list+="#Linear Pick Pose\n"
			waypoint_list+="movej(p[%f,%f,%f,%f,%f,%f])\n"%(true_pick_poses[0][0],true_pick_poses[0][1],true_pick_poses[0][2],true_pick_poses[0][3],true_pick_poses[0][4],true_pick_poses[0][5])
			#linear Move
			#waypoint_list+="#linear Move\n"
			waypoint_list+="movel(p[%f,%f,%f,%f,%f,%f])\n"%(true_pick_poses[1][0],true_pick_poses[1][1],true_pick_poses[1][2],true_pick_poses[1][3],true_pick_poses[1][4],true_pick_poses[1][5])
			#Gripper Action
			#waypoint_list+="GRIPPER_ACTION.script\n"
			#back to Linear Pick Pose
			#waypoint_list+="#back to Linear Pick Pose\n"
			waypoint_list+="movej(p[%f,%f,%f,%f,%f,%f])\n"%(true_pick_poses[0][0],true_pick_poses[0][1],true_pick_poses[0][2],true_pick_poses[0][3],true_pick_poses[0][4],true_pick_poses[0][5])
			#Safe move to pre-target position
			#waypoint_list+="#Safe move to pre-target position\n"
			for pose in data[procedure]["safe_move_to_target"][i0]: #the path of the actual object is stored here
				waypoint_list+="movej(p[%f,%f,%f,%f,%f,%f])\n"%(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5])
			#moving to a linear position before target position
			#waypoint_list+="#moving to a linear position before target position\n"
			if data[procedure]["absolute_coordinates_root_target"][0]==1: #Opt1: #rough coordinates only, calibration has been done
				#target_pose="pose_add(Preal,p"+str(data[procedure]["target_coordinates_rel"][i0][0:6])+")" #Add the relative Coordinates to absolute coordinates from calibration (Preal) #TODO Mabe its pose_trans?!
				#since we were not able to calibrate in oofline mode we use the rough position of the front case and add the relatove position of a bushing
				#degrees have to be converted to rad:
				data[procedure]["absolute_coordinates_root_target"][1][3:6]=list(map(math.radians,data[procedure]["absolute_coordinates_root_target"][1][3:6]))
				#now calculate the positions
				target_pose="p"+str(list(map(add,data[procedure]["absolute_coordinates_root_target"][1],data[procedure]["target_coordinates_rel"][i0][0:6])))
			elif data[procedure]["absolute_coordinates_root_target"][0]==2: #Opt2: Fix/trustworthy coordinates from database did not require calibration
				target_pose="p"+str(data[procedure]["target_coordinates_abs"][i0][0:6])

			waypoint_list+="movej("+target_pose+")\n" #TODO how far should be the distance?
			#waypoint_list+="#Perform the operation Skill\n"
			#waypoint_list+=data[procedure]["operation_skill"]+"\n"
			#move back linear
			#waypoint_list+="movel(pose_trans(get_actual_tcp_pose(),p[0,0,-0.03,0,0,0])) #back off linear after insertion\n" #Safe move back to pick-up position
			#waypoint_list+="#Safe move back to pick-up position\n"
			reverse_safe_way_list=data[procedure]["safe_move_to_target"][i0]
			reverse_safe_way_list.reverse()
			for pose in reverse_safe_way_list: #the path of the actual object is stored here
				waypoint_list+="movej(p[%f,%f,%f,%f,%f,%f])\n"%(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5])

			#waypoint_list+="#\n#\n#\n"
	script_file = open("./waypoints.script", 'w')
	script_file.write(waypoint_list)
	script_file.close
	return waypoint_list
def main():
	convert_cluster_2_script()
if __name__ == "__main__":
	main()
