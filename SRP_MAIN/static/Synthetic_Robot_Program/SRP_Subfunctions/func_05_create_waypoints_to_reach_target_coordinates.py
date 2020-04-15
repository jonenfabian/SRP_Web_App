#from:
#data[procedure]["safe_move_to_target"]=
#function:
#depending_on_safe_highway_points_create_waypoints_to_reach_target_coordinates()
#We have a grid with restricted areas and save areas from "03_func_get_relative_highway_safe_points_from_where_robot_can_move_to_target_coordinates.py"
#Now we get a "relation"-object (an object that needs to be assembled)
#We want to move the robot through the safe zone to the edge of the restricted area which is closest to the "relation" object
#The destination point (realtion coordinate) is within a restricted area. We need to identify the cell and find the nearest neighbor who either
#is the nearest safe cell
#or  is the nearest safe cell  and is in line with the z-axis (?)

##Function Start
import static.Synthetic_Robot_Program.SRP_Subfunctions.func_astar_pathplanning as astar_sub
#import SRP_Subfunctions.func_03_get_relative_highway_safe_points as get_safe_grid
import static.Synthetic_Robot_Program.SRP_Subfunctions.func_04_read_BOM_and_extract_relative_coordinates as get_rel_coords
import numpy as np
import time
#from astar import AStar
import sys
import math
import cv2
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import os
import re

def depending_on_safe_highway_points_create_waypoints_to_reach_target_coordinates(  the_grid,			#the grid, from func_03_get_relative_highway_safe_points.py
																					target_coordinates,# the target robot pose for assembly [x,y,z,rx,ry,rz] from func_04_read_BOM_and_extract_relative_coordinates.py
																					start_pos=[-1.0,-1.0,-1.0]): #startpos in robot base coordinae system
	target_cell_ID=0
	for layer in the_grid:
		for row in layer:
			for col in row:
				#if our target_coordinates are within the interval of the cell, note cell-ID
				if col[3][0]<target_coordinates[0]<col[3][2] and col[3][1]<target_coordinates[1]<col[3][3] and col[5][0]<target_coordinates[2]<col[5][1]: #object zone
					target_cell_ID=[col[0],col[1],col[2]]
	if target_cell_ID==0:
		raise ValueError('There is no cell in the grid that inherits the target coordinate for ...')
	#Now we have the restricted cell containing the target pose
	#lets find the next safe neighbour in -z direction
	safe_neighbour_target_cell_ID=0
	test_safe_neighbour_target_cell_ID=0
	#while we have no safe_neighbour_target_cell_ID move further down the -z axis until we find one
	increment=-0.015 #meters
	traveled_distance=0
	start_pose_for_assembly=target_coordinates
	while safe_neighbour_target_cell_ID==0:
		start_pose_for_assembly=get_rel_coords.posetrans(
			Startpose=start_pose_for_assembly[0:6], #[X,Y,Z] and rotation vector
			Translation=[0,0,increment],
			Rotation=[0,0,0] #3cm in -Z
			)
		#print("start_pose_for_assembly: ", start_pose_for_assembly)
		for layer in the_grid:
			for row in layer:
				for col in row:
					#print("accessing cell",col[3], "%f and %f not here"%(start_pose_for_assembly[0],start_pose_for_assembly[1]) )
					#if our target_coordinates are within the interval of the cell and it is a safe zone, use cell-ID
					if col[3][0]<start_pose_for_assembly[0]<col[3][2] and col[3][1]<start_pose_for_assembly[1]<col[3][3] and col[5][0]<start_pose_for_assembly[2]<col[5][1] and col[4]==0 and [col[0],col[1],col[2]]!=target_cell_ID: #col[4]==0 <-Safe Zone
						safe_neighbour_target_cell_ID=[col[0],col[1],col[2]] #the while loop ends here
						#print("found the cell!")
						#print("accessing cell",col[3], "%f and %f fit here"%(start_pose_for_assembly[0],start_pose_for_assembly[1]) )
						break
				if safe_neighbour_target_cell_ID!=target_cell_ID and safe_neighbour_target_cell_ID!=0:
					break
		traveled_distance+=increment

		if traveled_distance<-2:
			print("seems like there is no safe_neighbour_target_cell, we already traveled %f meters"%traveled_distance)
			raise ValueError("seems like there is no safe_neighbour_target_cell, we already traveled %f meters"%traveled_distance)
	print("traveled_distance: ",traveled_distance, " from ",target_cell_ID," to ",safe_neighbour_target_cell_ID, " area code ",col[4])
	if target_cell_ID[0]<safe_neighbour_target_cell_ID[0]:
		print("traveled to the right")
	else:
		print("traveled to the left")
	#Now lets use the A*-Pathplanner to get a safe path to the safe_neighbour_target_cell_ID

	end_pos=[start_pose_for_assembly[0],start_pose_for_assembly[1],start_pose_for_assembly[2]] #meters
	start_pos_xyz=[start_pos[0],start_pos[1],start_pos[2]]

	##V1
	path,start_pos_cell,end_pos_cell, success_flag = astar_sub.astar(the_grid,start_pos_xyz,end_pos,time.time())

	#lets convert the cell coordinates from the path into real coordinates
	#the path var with cell ID's looks like this: [(2, 2, 2), (3, 3, 3), (4, 4, 4), ...]
	#from these cell-ID's we will extract the middle point and use it as our real robot waypoint
	#Also transition the RXRYRZ-Values from pick-up pose to targetpose linear
	save_waypoint_path=[]
	number_of_waypoints=len(path)
	start_rotation=start_pos[3:6]
	target_rotation=target_coordinates[3:6]
	#linear equation y=m*x+n
	m_rx=(target_rotation[0]-start_rotation[0])/number_of_waypoints
	m_ry=(target_rotation[1]-start_rotation[1])/number_of_waypoints
	m_rz=(target_rotation[2]-start_rotation[2])/number_of_waypoints
	n_rx=start_rotation[0]
	n_ry=start_rotation[1]
	n_rz=start_rotation[2]
	counter=0 #used to do the linear angle transition
	for cell in path:
		x1y1x2y2=the_grid[cell[2]][cell[0]][cell[1]][3]
		x_middle=x1y1x2y2[0]+(x1y1x2y2[2]-x1y1x2y2[0])/2
		y_middle=x1y1x2y2[1]+(x1y1x2y2[3]-x1y1x2y2[1])/2
		z1z2=the_grid[cell[2]][cell[0]][cell[1]][5]
		z_middle=z1z2[0]+(z1z2[1]-z1z2[0])/2
		save_waypoint_path.append([x_middle,y_middle,z_middle, 		# X Y Z
			m_rx*counter+n_rx,m_ry*counter+n_ry,m_rz*counter+n_rz]) #RXRYRZ
			#target_coordinates[3],target_coordinates[4],target_coordinates[5]])# TODO only xyz are processed, whats with the rest?  Maybe we transition the angles linear from start to end pose? we hae the endpose RXRyRZ, where is startpose?
		counter+=1


	#print(path)
	return save_waypoint_path, path, success_flag,end_pos#,start_pos




def show_results_path(posetrans_poses,paths,objects,the_grid,area_size=[-1.3,1.3],width=800,height=800):
	the_grid_np=np.array(the_grid)
	abs_x=width/2
	abs_y=height/2
	#px_step=int(width/len(the_grid)) #factor px/m
	factor_px_m=width/(abs(area_size[0])+abs(area_size[1])) #px/m
	#lets create a blank_image
	blank_image = np.zeros((width,height,3), np.uint8)
	blank_image[:,0:width] = (255,255,255) #white background
	#lets create a pixe grid
	for row in the_grid:
		for col in row:
			p1=(int(col[3][0]*factor_px_m+abs_x),int(col[3][1]*factor_px_m+abs_y))
			p2=(int(col[3][2]*factor_px_m+abs_x),int(col[3][3]*factor_px_m+abs_y))
			if col[4]==1: #object zone
				cv2.rectangle(blank_image,p1,p2,(0, 0, 255), -1)
			elif col[4]==2: #safety zone
				cv2.rectangle(blank_image,p1,p2,(0, 255, 0), -1)
			else:
				cv2.rectangle(blank_image,p1,p2,(173, 216, 230), 1)	#light blue
	#Lets test the results
	#print(paths)
	cv2.imwrite("C:/Users/Administrator/b-tu.de/Carsten Wedemeyer - COCKPIT 4.0/08_Demonstratoren/D3_MRK/Synthetic Robot Program/Results/the_grid.png",blank_image)
	blank_image = np.zeros((width,height,3), np.uint8)
	counter=0 #with this counter we iterate paralel through the objects
	for path in paths:
		#print(path)
		blank_image = np.zeros((width,height,3), np.uint8)
		blank_image[:,0:width] = (255,255,255) #white background
		#blank_image[:,0:width//2] = (255,0,0)
		#lets create a pixe grid
		for row in the_grid:
			for col in row:
				p1=(int(col[3][0]*factor_px_m+abs_x),int(col[3][1]*factor_px_m+abs_y))
				p2=(int(col[3][2]*factor_px_m+abs_x),int(col[3][3]*factor_px_m+abs_y))
				if col[4]==1: #object zone
					cv2.rectangle(blank_image,p1,p2,(0, 0, 255), -1)
				elif col[4]==2: #safety zone
					cv2.rectangle(blank_image,p1,p2,(0, 255, 0), -1)
				else:
					cv2.rectangle(blank_image,p1,p2,(173, 216, 230), 1)	#light blue
		#cv2.destroyAllWindows()
		not_blank_image=blank_image
		#cv2.imshow("image",not_blank_image)
		#lets add a dot where the objetc is
		object=objects[counter]

		dot_px_x=int(object[0]*factor_px_m+abs_x)
		dot_px_y=int(object[1]*factor_px_m+abs_y)
		if not path:
			cv2.circle(blank_image, (dot_px_x,dot_px_y), 3, (255,255,0), -1)
		else:
			cv2.circle(blank_image, (dot_px_x,dot_px_y), 3, (255,0,0), -1)
		#lets add a dot where the safe start pose is  (posetrans)
		posetrans_pose=posetrans_poses[counter]
		dot_px_x=int(posetrans_pose[0]*factor_px_m+abs_x)
		dot_px_y=int(posetrans_pose[1]*factor_px_m+abs_y)
		if not path:
			cv2.circle(blank_image, (dot_px_x,dot_px_y), 3, (255,255,0), -1)
		else:
			cv2.circle(blank_image, (dot_px_x,dot_px_y), 3, (255,0,0), -1)




		cv2.imshow("image",blank_image)
		cv2.waitKey(1)
		time.sleep(0.1)
		for path_cell in path:
			path_cell_found=0
			for row in the_grid:
				if path_cell_found==1:
					break
				for col in row:
					p1=(int(col[3][0]*factor_px_m+abs_x),int(col[3][1]*factor_px_m+abs_y))
					p2=(int(col[3][2]*factor_px_m+abs_x),int(col[3][3]*factor_px_m+abs_y))
					if col[0]==path_cell[0] and col[1]==path_cell[1]:
						cv2.rectangle(blank_image,p1,p2,(255, 0, 0), -1)
						path_cell_found=0
						cv2.imshow("image",blank_image)
						cv2.waitKey(1)
						time.sleep(0.015)
						break
		#cv2.imshow("image",not_blank_image)
		cv2.imwrite("C:/Users/Administrator/b-tu.de/Carsten Wedemeyer - COCKPIT 4.0/08_Demonstratoren/D3_MRK/Synthetic Robot Program/Results/path_result%04d.png"%counter,blank_image)
		counter+=1
	cv2.waitKey(1)
	time.sleep(0.1)
	cv2.destroyAllWindows()
	return #save_waypoint_path#,end_pos,start_pos

def show_results_path3D(posetrans_poses,paths,objects,save_name):
	from SRP_APP.models import SrpClusterDb
	import random
	import cv2
	#import numpy as np
	import glob
	counter=0
	img_array = [] #for the creation of the video
	save_name=re.sub("\W","",save_name)
	#Create Folder
	root_path=os.getcwd()
	# define the name of the directory to be created
	path=root_path+"/media/frames/%s"%save_name #3d_path_%s_%04d.png"%(save_name,counter)
	try:
		os.makedirs(path)
	except OSError:
		print ("Creation of the directory %s failed" % path)
	else:
		print ("Successfully created the directory %s" % path)

	if not path: #if there was no path found in the path planner, return here
		return
	#posetrans_poses=end_poses_ur, 					# - add the posetrans_pose
	#paths=data[procedure]["safe_move_to_target"],  # - add the path points
	#objects=data[procedure]["target_coordinates"]  # - add the target points
	counter=0
	# find teh min max to get axis limits
	max_x_temp=[]
	max_y_temp=[]
	max_z_temp=[]
	min_x_temp=[]
	min_y_temp=[]
	min_z_temp=[]
	for i0 in range(len(paths)):
		if not paths[i0]:
			None
		else:
			a=np.array(paths[i0])
			#print(a)
			#print(a[:,0])
			max_x_temp.append(max(a[:,0]))
			max_y_temp.append(max(a[:,1]))
			max_z_temp.append(max(a[:,2]))
			min_x_temp.append(min(a[:,0]))
			min_y_temp.append(min(a[:,1]))
			min_z_temp.append(min(a[:,2]))
	max_x=max(max_x_temp)
	max_y=max(max_y_temp)
	max_z=max(max_z_temp)
	min_x=min(min_x_temp)
	min_y=min(min_y_temp)
	min_z=min(min_z_temp)
	#for i0 in range(len(paths)): #we want as many images as we have paths
	#create a random list to get 9 images for displaying in the app.
	index_list=list(range(len(paths)))
	random.shuffle(index_list)
	index_list=index_list[0:9]
	for i0 in index_list: #we want only 9 images
		if not paths[i0]:
			None
		else:
			fig = plt.figure()
			ax = fig.add_subplot(111, projection='3d')
			#posetrans pose
			xs1pose=posetrans_poses[i0][0]
			ys1pose=posetrans_poses[i0][1]
			zs1pose=posetrans_poses[i0][2]
			ax.scatter(xs1pose, ys1pose, zs1pose, c='b', marker='o')
			xs1path=[]
			ys1path=[]
			zs1path=[]
			for i1 in range(len(paths[i0])): #points in paths
				xs1path.append(paths[i0][i1][0])
				ys1path.append(paths[i0][i1][1])
				zs1path.append(paths[i0][i1][2])
			#ax.scatter(xs1path, ys1path, zs1path, c='g', marker='o')
			ax.plot(xs1path, ys1path, zs1path, c='g')
			xs1target=[]
			ys1target=[]
			zs1target=[]
			for coordinate in objects:
				xs1target.append(coordinate[0])
				ys1target.append(coordinate[1])
				zs1target.append(coordinate[2])
			ax.scatter(xs1target, ys1target, zs1target, c='r', marker='o')
			ax.set_xlabel('X')
			ax.set_ylabel('Y')
			ax.set_zlabel('Z')
			ax.set_xlim([min_x,max_x])
			ax.set_ylim([min_y,max_y])
			ax.set_zlim([min_z,max_z])
			#plt.show()
			#plt.savefig(path+"/3d_path_%s_%04d.png"%(save_name,counter),
			plt.savefig(path+"/temporary_image.png",
			dpi=300,
			facecolor='w',
			edgecolor='w',
			orientation='portrait',
			papertype=None,
			format=None,
			transparent=True,
			bbox_inches=None,
			pad_inches=0.1,
			frameon=None,
			metadata=None
			)
			plt.close()
			counter+=1
			print("saved figure ",counter)
			#append temporary image to image_array
			img = cv2.imread(path+"/temporary_image.png")
			h, w, layers = img.shape
			size = (w,h)
			img_array.append(img)
	#Create a Video out of frames


	# print("path: ",path)
	# time.sleep(1)
	# files=glob.glob(path+"/*.png")
	# for i1 in range(len(files)):
	# 	img = cv2.imread(files[i1])
	# 	h, w, layers = img.shape
	# 	#if h<w:
	# 	#	rotated_image = np.rot90(img)
	# 	#	img=rotated_image
	# 	#h, w, layers = img.shape
	# 	size = (w,h)
	# 	img_array.append(img)
	# 	counter+=1
	# print('part done, read images: '+str(counter))
	# out of image array write a video
	print("creating video out of %d Frames"%counter)
	out = cv2.VideoWriter(path+'/00_3d_Ani_%s.mp4'%save_name,cv2.VideoWriter_fourcc(*'MP4V'), 5, size)
	for i in range(len(img_array)):
		out.write(img_array[i])
	out.release()
	print("Done.")
	#Upload to DB doesnt work and is not recommended, store to media file and save the link would be good, but dont know how that works.
	# db_video=SrpClusterDb(pathplanning_video=path+'/00_3d_Ani_%s.mp4'%save_name)
	# #db_video=SrpClusterDb(pathplanning_video=out)
	# db_video.save()
		#pathplanning_video=out

	"""
	##3D scatterplot
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(xs1, ys1, zs1, c='r', marker='o')
	ax.set_xlabel('X Label')
	ax.set_ylabel('Y Label')
	ax.set_zlabel('Z Label')
	plt.show()
	"""
