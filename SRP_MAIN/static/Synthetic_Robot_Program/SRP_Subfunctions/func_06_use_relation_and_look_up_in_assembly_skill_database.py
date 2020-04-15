#
#from:
#data[procedure]["operation_skill"]=
#function:
#use_relation_and_look_up_in_assembly_skill_database()
	#the default is spiral search
#"""
"""
import json
#Firstly, look if we already have an entry in the "operation_skill_db"-database
#How to read the "database"-file
"""
def use_relation_and_look_up_in_assembly_skill_database(input,db_path='media/SRP_Database/operation_skill_db.json'):
	import json
	with open(db_path) as json_file:
		operation_skill_db = json.load(json_file)

	#Find the entry
	#print(json.dumps(operation_skill_db, indent = 4))
	if input in operation_skill_db:
		print("Yes there is an entry for the operation-skill of %s"%input)
		ur_script_code=operation_skill_db[input]["operation_skill"]
	else:
		print('There is no operations skill in the database for %s. Using Default (Spiral Search)'%input)
		ur_script_code=operation_skill_db["DEFAULT"]["operation_skill"]
	return ur_script_code
"""
#testing the function
#testing purpose from "relation"
input="FW51528_D__BUSH_BEARING"
print(use_relation_and_look_up_in_operation_skill_database(input,db_path='media/SRP_Database/operation_skill_db.json'))
"""
