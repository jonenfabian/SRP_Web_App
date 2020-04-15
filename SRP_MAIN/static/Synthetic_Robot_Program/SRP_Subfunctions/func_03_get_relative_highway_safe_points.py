#Function get_relative_highway_safe_points_from_where_robot_can_move_to_target_coordinates()
#We want to create save highwaypoints within our workspace
#Lets create a grid with cells looking like this 
#the_grid=[layer,layer,layer,...] #for the diffenret heights
#layer=
#[
#  [[rowID, colID,layerID, [x1[meters], y1[meters],x2[meters], y2[meters]], value=1/0, [z1[meters],z2[meters]]],[rowID, colID, [x1[meters], y1[meters],x2[meters], y2[meters]], value=1/0,...],...],
#  [[rowID, colID,layerID, [x1[meters], y1[meters],x2[meters], y2[meters]], value=1/0, [z1[meters],z2[meters]]],[rowID, colID, [x1[meters], y1[meters],x2[meters], y2[meters]], value=1/0,...],...],
#  ...
#]

import sys
import static.Synthetic_Robot_Program.SRP_Subfunctions.func_04_read_BOM_and_extract_relative_coordinates as get_rel_coords
import numpy as np
import cv2
def look_up_database():
	#has to be defined!
	raise FunctionError("Sorry this function has not been created")
	return
def get_relative_highway_safe_points_from_where_robot_can_move_to_target_coordinates(
	BOM_file_path_input="media/bom_files/DAP009373_U_HPC_FRONT_CASE_AND_VANE_ASSY.txt",
	BOM_file_relation_object="", #if a specific part is needed, "" means all objects
	root_object_number="",
	cell_width=0.1,
	area_size=[-1.3,1.3],
	thickness_of_safety_zone=2,
	abs_coordinates=[0,0,0,0,0,0]):
	print("BOM_file_path_input,keyword=BOM_file_relation_object: ",BOM_file_relation_object)
	import numpy as np	
	#Lets get the restricted area by using the dimension of root objects
	if BOM_file_path_input!="":
		#root_object_child_objects_list=get_dimensions_of_root_object(BOM_file_path_input,keyword=BOM_file_relation_object)
		root_object_child_objects_list,trash=get_rel_coords.func_read_BOM_and_extract_relative_coordinates(BOM_file_path=BOM_file_path_input,keyword=BOM_file_relation_object,abs_coordinates=abs_coordinates)
	elif root_object_number!="":
		root_object_child_objects_list=look_up_database()
	else:
		raise PathError("No root object found for function input, database is not up to date or BOM is missing")
	
	area_size[1]=area_size[1]+cell_width
	area_size_steps=np.arange(area_size[0],area_size[1],cell_width) #meters x meters (Robot workspace)
	the_grid=[]
	#print(int((abs(area_size[0])+abs(area_size[1]))/cell_width))
	#layers
	for layer in range(0,int((abs(area_size[0])+abs(area_size[1]))/cell_width)-1):
		the_grid.append([])
		#add row to layer (x)
		for row in range(0,int((abs(area_size[0])+abs(area_size[1]))/cell_width)-1):
			#add column to row (y)
			#print(row)
			the_grid[layer].append([])
			for col in range(0,int((abs(area_size[0])+abs(area_size[1]))/cell_width)-1):
				#print(col)
				
				for waypoint in root_object_child_objects_list:
					#print(waypoint)
					if area_size_steps[layer]<=0<area_size_steps[layer+1]:
						value=2 #restricted -> the table!
						break
					elif area_size_steps[row]<=waypoint[0]<area_size_steps[row+1] and area_size_steps[col]<=waypoint[1]<area_size_steps[col+1] and area_size_steps[layer]<=waypoint[2]<area_size_steps[layer+1]:
						value=1 #restricted
						#print("found: "+str(area_size_steps[row])+"<"+str(waypoint[1])+"<"+str(area_size_steps[row+1]) +" or "+ str(area_size_steps[col])+"<"+str(waypoint[0])+"<"+str(area_size_steps[col+1]))
						break
					else:
						value=0 #free
				the_grid[layer][row].append([row,col,layer,[area_size_steps[row],area_size_steps[col],area_size_steps[row+1],area_size_steps[col+1]],value,[area_size_steps[layer],area_size_steps[layer+1]]])
	#print(np.array(the_grid))
	#sys.exit()

	#Now lets add a safety radius to each cell with value==1 in a horizontal direction
	for i0 in range(len(the_grid)):
		for i1 in range(len(the_grid[i0])):
			for i2 in range(len(the_grid[i0][i1])):
				if the_grid[i0][i1][i2][4]==1: #found a forbidden zone
					#lets create a safety cube
					for i0_ in range(-thickness_of_safety_zone,thickness_of_safety_zone+1):
						for i1_ in range(-thickness_of_safety_zone,thickness_of_safety_zone+1):
							for i2_ in range(-thickness_of_safety_zone,thickness_of_safety_zone+1):
								try:
									if the_grid[i0+i0_][i1+i1_][i2+i2_][4]!=1:
										the_grid[i0+i0_][i1+i1_][i2+i2_][4]=2
								except:
									None
								

	return the_grid










	



"""


the_grid = get_relative_highway_safe_points_from_where_robot_can_move_to_target_coordinates(
	BOM_file_path_input="media/bom_files/DAP009373_U_HPC_FRONT_CASE_AND_VANE_ASSY.txt",
	BOM_file_relation_object="",
	root_object_number="",
	cell_width=cell_width,
	area_size=area_size,
	thickness_of_safety_zone=2)
the_grid_np=np.array(the_grid)
print(the_grid_np)
print(len(the_grid[:]))
import cv2

#Lets use the A*-pathplanner
#For Testing purposes
start_pos=[-1.0,-1.0] #meters
end_pos=[1.0,1.0] #meters
 
path,start_pos_cell,end_pos_cell = astar.astar(the_grid,start_pos,end_pos)
print(path)

#Lets test the results

for path_cell in path:
	path_cell_found=0
	for row in the_grid:
		if path_cell_found==1:
			break
		for col in row:
			p1=(int(col[2][0]*factor_px_m+abs_x),int(col[2][1]*factor_px_m+abs_x))
			p2=(int(col[2][2]*factor_px_m+abs_y),int(col[2][3]*factor_px_m+abs_y))
			if col[0]==path_cell[0] and col[1]==path_cell[1]:
				cv2.rectangle(blank_image,p1,p2,(255, 0, 0), -1)
				path_cell_found=0
				cv2.imshow("image",blank_image)
				cv2.waitKey(1)
				time.sleep(0.025)
				break
#cv2.imshow("image",blank_image)
cv2.imwrite("./Results/path_result.png",blank_image)
cv2.waitKey()
cv2.destroyAllWindows()		
"""