
#from:
#data[procedure]["custom_pick_skill"]=
#function:
#use_relation_and_look_up_in_pick_skill_database()
	#gets a script-code with relative coordinates
	#Will be a relative start and end position to the absolute pose of the object. Which is created in Unity by Immanuel.Horn@ferchau.de
	#Startposition gripper/tool-action has to be programmed within the robot!
	#  Endposition gripper/tool-action has to be programmed within the robot!

"""
import json
#Firstly, look if we already have an entry in the "pick_skill_db"-database
#How to read the "database"-file
"""
def use_relation_and_look_up_in_pick_skill_database(input,db_path='media/SRP_Database/pick_skill_db.json'):
	import json
	with open(db_path) as json_file:
		pick_skill_db = json.load(json_file)

	#Find the entry
	print(json.dumps(pick_skill_db, indent = 4))
	if input in pick_skill_db:
		print("Yes there is an entry for the pick-skill of %s"%input)
		start_pose_rel_to_object_origin=pick_skill_db[input]["start_pose_rel_to_object_origin"]
		end_pose_rel_to_object_origin  =pick_skill_db[input]["end_pose_rel_to_object_origin"]
	else:
		raise ValueError('There is no start/end pose for the pick-skill in the database for %s'%input)
	return start_pose_rel_to_object_origin,end_pose_rel_to_object_origin
"""
#testing the function
#testing purpose from "relation"
input="FW51528_D__BUSH_BEARING"
print(use_relation_and_look_up_in_pick_skill_database(input,db_path='media/SRP_Database/pick_skill_db.json'))
"""
