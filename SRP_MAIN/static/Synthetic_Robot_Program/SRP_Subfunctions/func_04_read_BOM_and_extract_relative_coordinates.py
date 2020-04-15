##read_BOM and create UR-SCRIPT file with Waypoints 
#Whats the input [FW5128]
#Whats the output List with Coordinates like [[X,Y,Z,RX,RY,RZ],[X,Y,Z,RX,RY,RZ],[X,Y,Z,RX,RY,RZ],[X,Y,Z,RX,RY,RZ], ...]
#The Original model is in a horizontal position, to make it stand in a vertical position, a rotation y(or x?) is neccessary
#The Z-Direction of Objects is in case of the objects in the opposite direction. me will flip the z axis with "posetrans"

import csv
import sys
import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
#def func_read_BOM_and_extract_relative_coordinates(keyword="FW51530",BOM_file_path="media/bom_files/DAP009373_U_HPC_FRONT_CASE_AND_VANE_ASSY.txt"):
def euler2mat(rotation=[0,0,0],translation=[0,0,0],point=[0,0,0]): #Zum Verschieben des Tools in gewuenschte Richtung (Point)
	#Muss noch getestet werden
	#Point nicht implementiert
	xC, xS = math.cos(rotation[0]), math.sin(rotation[0])
	yC, yS = math.cos(rotation[1]), math.sin(rotation[1])
	zC, zS = math.cos(rotation[2]), math.sin(rotation[2])
	dX = translation[0]
	dY = translation[1]
	dZ = translation[2]
	Translate_matrix = np.array([[1, 0, 0, dX],[0, 1, 0, dY],[0, 0, 1, dZ],[0, 0, 0, 1]])
	Rotate_X_matrix = np.array([[1, 0, 0, 0],[0, xC, -xS, 0],[0, xS, xC, 0],[0, 0, 0, 1]])
	Rotate_Y_matrix = np.array([[yC, 0, yS, 0],[0, 1, 0, 0],[-yS, 0, yC, 0],[0, 0, 0, 1]])
	Rotate_Z_matrix = np.array([[zC, -zS, 0, 0],[zS, zC, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
	matrix =np.dot(Rotate_Z_matrix,np.dot(Rotate_Y_matrix,np.dot(Rotate_X_matrix,Translate_matrix)))
	new_point=np.dot(matrix,[point[0],point[1],point[2],1])
	if point[0]+point[1]+point[2]!=0:
		return new_point
	else:
		return matrix
def rxryrz2mat(rotationVector):
  theta = 0.0
  u=np.array([0.0,0.0,0.0])
  b_rotationVector=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
  c_rotationVector=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
  b_rotationMatrix=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
  rotationMatrix=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
  rotationMatrixV2=[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]
  I=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
  beta=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
  scale = 3.3121686421112381E-170
  for k in range (0,3):
    absxk = abs(rotationVector[k])
    if absxk > scale:
      t = scale / float(absxk)
      theta = 1.0 + theta * t * t
      scale = absxk
    else:
      t = absxk / float(scale)
      theta += t * t
  theta = scale * math.sqrt(theta)
  if theta > 1.0E-6:
    for i0 in range (0,3):
      u[i0] = float(rotationVector[i0])/float(theta)
    scale = math.cos(theta)
    absxk = math.sin(theta)
    for i0 in range(0,9):
      I[i0] = 0
    beta[0] = absxk * 0.0
    beta[3] = absxk * -u[2]
    beta[6] = absxk * u[1]
    beta[1] = absxk * u[2]
    beta[4] = absxk * 0.0
    beta[7] = absxk * -u[0]
    beta[2] = absxk * -u[1]
    beta[5] = absxk * u[0]
    beta[8] = absxk * 0.0
    for k in range(0,3):
      I[k + 3 * k] = 1
      b_rotationVector[k] = rotationVector[k] / theta
      for i0 in range(0,3):
        c_rotationVector[k + 3 * i0] = b_rotationVector[k] * u[i0]
    for i0 in range(0,3):
      for k in range(0,3):
        b_rotationMatrix[k + 3 * i0] = I[k + 3 * i0] * scale + beta[k + 3 * i0] + (1.0 - scale) * c_rotationVector[k + 3 * i0]
  for i0 in range(0,3): #(i0 = 0; i0 < 3; i0++) {
    for k in range(0,3): # (k = 0; k < 3; k++) {
      rotationMatrix[k + 3 * i0] = b_rotationMatrix[i0 + 3 * k]
  rotationMatrixV2[0][0]=rotationMatrix[0]
  rotationMatrixV2[0][1]=rotationMatrix[1]
  rotationMatrixV2[0][2]=rotationMatrix[2]
  rotationMatrixV2[1][0]=rotationMatrix[3]
  rotationMatrixV2[1][1]=rotationMatrix[4]
  rotationMatrixV2[1][2]=rotationMatrix[5]
  rotationMatrixV2[2][0]=rotationMatrix[6]
  rotationMatrixV2[2][1]=rotationMatrix[7]
  rotationMatrixV2[2][2]=rotationMatrix[8]
  return rotationMatrixV2
def euler2rxryrz(Euler): #Rotationswinkel von ZYX in Rotationsvektoren transformieren,
			#welche vom Roboter interpretiert werden koennen
			# entspricht der Funktion von Pose_Trans
	##print'Convert Euler to Rotation VEctor'
	roll,pitch,yaw= Euler[0],Euler[1],Euler[2]
	yawMatrix = np.matrix([
	[math.cos(yaw), -math.sin(yaw), 0],
	[math.sin(yaw), math.cos(yaw), 0],
	[0, 0, 1]
	])
	pitchMatrix = np.matrix([
	[math.cos(pitch), 0, math.sin(pitch)],
	[0, 1, 0],
	[-math.sin(pitch), 0, math.cos(pitch)]
	])
	rollMatrix = np.matrix([
	[1, 0, 0],
	[0, math.cos(roll), -math.sin(roll)],
	[0, math.sin(roll), math.cos(roll)]
	])
	R = yawMatrix * pitchMatrix * rollMatrix
	theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
	multi = 1 / (2 * math.sin(theta))
	rx = multi * (R[2, 1] - R[1, 2]) * theta
	ry = multi * (R[0, 2] - R[2, 0]) * theta
	rz = multi * (R[1, 0] - R[0, 1]) * theta
	##print rx, ry, rz
	##print 'Goal is to read Rotation Vector, convert to Rotation matrix and rotate?'
	return [rx,ry,rz]
def mat2euler(matrix): #Zum ermitteln von Euler Angles aus Rotationsmatrix
	sy = math.sqrt(matrix[0][0] * matrix[0][0] +	matrix[1][0] * matrix[1][0])
	Rx = math.atan2(matrix[2][1] , matrix[2][2])
	Ry = math.atan2(-matrix[2][0], sy)
	Rz = math.atan2(matrix[1][0], matrix[0][0])
	#rad2deg=180/math.pi
	return [Rx, Ry, Rz]
def posetrans(Startpose,Translation=[0,0,0],Rotation=[0,0,0]):
#Erwatet Startpose mit Radians (rxryrz-Rotationsvektor) und Rotation mit Degrees
#Translation in XYZ-Richtung (Tool-Koordinatensystem)
#Rotation um XYZ-Achse (Tool) xxx Achszuordnung muss geklaert werden!
	matrix=rxryrz2mat(Startpose[3:6]) #Rotationsmatrix aus Rotationsvektor erstellen
	euler=mat2euler(matrix) #Rotationsmatrix in Eulerwinkel wandeln
	if np.linalg.norm(Rotation)==0 and np.linalg.norm(Translation)!=0:# Nur Translation
		temp=euler2mat(euler,[0,0,0],Translation) #zu addierende Koordindaten fuer Translation ermitteln
		new_xyz=list(map(sum, zip(temp,Startpose[0:3]))) #Vektoren addieren
		return new_xyz+Startpose[3:6]
	elif np.linalg.norm(Rotation)!=0 and np.linalg.norm(Translation)==0: #Nur Rotation
		Rotation=[math.radians(Rotation[0]),math.radians(-Rotation[2]),math.radians(Rotation[1])] #Angepasst an Tool-KS vom Roboter
		new_euler=list(map(sum, zip(euler,Rotation)))
		new_rxryrz=euler2rxryrz(new_euler)
		return Startpose[0:3]+new_rxryrz
	elif np.linalg.norm(Rotation)!=0 and np.linalg.norm(Translation)!=0: #Erst Rotation dann Translation
		Rotation=[math.radians(Rotation[0]),math.radians(-Rotation[2]),math.radians(Rotation[1])]
		new_euler=list(map(sum, zip(euler,Rotation)))
		new_rxryrz=euler2rxryrz(new_euler) #Ausgabe fur Roboter
		temp=euler2mat(new_euler,[0,0,0],Translation) #zu addierende Koordindaten fuer Translation ermitteln
		new_xyz=list(map(sum, zip(temp,Startpose[0:3]))) #Vektoren addieren
		return new_xyz+new_rxryrz
	elif np.linalg.norm(Rotation)==0 and np.linalg.norm(Translation)==0: #keine Transformation
		return Startpose	
def rotate_frame(rotation=[[0,0,0],[0,0,0],[0,0,0]],trans=[0,0,0],axis="y",rot_angle=0):
	#This function uses a given rotation and tranlation matrix and rotates it around axis for rot_angle [degrees]
	#create a 4x4 frame which we can rotate
	rotation[0].append(trans[0]) 
	rotation[1].append(trans[1]) 
	rotation[2].append(trans[2]) 
	rotation.append([0,0,0,1])
	frame=np.array(rotation)
	#print("#############function debugging################")
	#print(frame)
	#now we have a 4x4 matrix
	#lets find the rotation matrix
	theta = np.radians(rot_angle)
	c, s = np.cos(theta), np.sin(theta)
	if axis=="z" or axis=="Z":
		R = np.array(((c,-s, 0, 0), (s, c, 0, 0), (0, 0, 1, 0), (0,0,0,1)))
	elif axis=="y" or axis=="Y":
		R = np.array(((c,0, s, 0), (0, 1, 0, 0), (-s, 0, c, 0), (0,0,0,1)))	
	elif axis=="x" or axis=="X":
		R = np.array(((1, 0, 0, 0), (0, c, -s, 0), (0, s, c, 0), (0,0,0,1)))
	else:
		print("wrong axis name")
	# Multiply R with Frame
	rotated_frame=np.matmul(R,frame)
	#print(rotated_frame)
	new_rotation=[rotated_frame[0][0:3],rotated_frame[1][0:3],rotated_frame[2][0:3]]
	new_trans=[rotated_frame[0][3],rotated_frame[1][3],rotated_frame[2][3]]
	return new_rotation,new_trans
	##END OF FUNCTIONS########
def func_read_BOM_and_extract_relative_coordinates(keyword="FW51530",BOM_file_path="media/bom_files/DAP009373_U_HPC_FRONT_CASE_AND_VANE_ASSY.txt",abs_coordinates=[0.5, 0, 0, 0, 0, 0]):
	#For displaying the results
	xs1=[]
	ys1=[]
	zs1=[]
	rs1=[]
	object_all_data_mat=[]
	with open(BOM_file_path, 'r') as f:
		csv_reader = csv.reader(f, delimiter='~')
		line_count = 0
		for row in csv_reader:
			if row[1].find(keyword)!=-1:
			#if row[1]==" "+keyword:
				object_all_data_mat.append((row[8].split()))
			elif keyword=="":
				object_all_data_mat.append((row[8].split()))
			

	height_list=[]
	radius_list=[]
	translation=[[],[],[]]
	extracted_waypoints=[]
	extracted_waypoints_rel=[]
	#bring data in waypoint form
	#print(len(object_all_data_mat))
	for object in object_all_data_mat:
		#Lets fit it into the rotation matrix frame
		rotationMatrixV2=[[[],[],[]],[[],[],[]],[[],[],[]]]
		rotationMatrixV2[0][0]=float(object[0]) #1st row
		rotationMatrixV2[0][1]=float(object[4]) #1st row
		rotationMatrixV2[0][2]=float(object[8]) #1st row
		rotationMatrixV2[1][0]=float(object[1]) #2nd row
		rotationMatrixV2[1][1]=float(object[5]) #2nd row
		rotationMatrixV2[1][2]=float(object[9]) #2nd row
		rotationMatrixV2[2][0]=float(object[2]) #3rd row
		rotationMatrixV2[2][1]=float(object[6]) #3rd row
		rotationMatrixV2[2][2]=float(object[10])#3rd row

		#Extrakt Translation
		translation=[[],[],[]]
		translation[0]=float(object[12])
		translation[1]=float(object[13])
		translation[2]=float(object[14])
		#print(rotationMatrixV2,translation)
		#Now rotate the whole model around the desired axis (in 3d model the engine is horizontally, but in assembly most parts are vertical)
		new_rotationMatrix,new_translation = rotate_frame(rotationMatrixV2,translation,axis="y",rot_angle=-90)
		"""
		print(new_rotationMatrix,new_translation)
		new_rotationMatrix[0].tolist()
		new_rotationMatrix[1].tolist()
		new_rotationMatrix[2].tolist()
		#new_translation.tolist()
		print(new_rotationMatrix,new_translation)
		#Now rotate the whole model for the absolute angle
		new_rotationMatrix,new_translation = rotate_frame(new_rotationMatrix,new_translation,axis="z",rot_angle=-90)
		"""
		euler_temp=mat2euler(new_rotationMatrix)
		# Flip Z-Axis
		Flip=[0,180,0] #Degrees
		Flip_Rotation=[math.radians(Flip[0]),math.radians(-Flip[2]),math.radians(Flip[1])] #Angepasst an Tool-KS vom Roboter
		new_euler=tuple(map(sum, zip(euler_temp,Flip_Rotation)))
		
		#relative coodinates where base is (0/0) is required if the absolute position of Target Object (Front Case) is only rough and will be calibrated
		new_translation_rel=[0,0,0]
		new_translation_rel[0]=new_translation[0] #UR Conform
		new_translation_rel[1]=new_translation[1] #UR Conform
		new_translation_rel[2]=new_translation[2] #UR Conform
		#transform relative coordinates to absolute coordinates
		new_translation[0]=new_translation[0]+abs_coordinates[0] #UR Conform
		new_translation[1]=new_translation[1]+abs_coordinates[1] #UR Conform
		new_translation[2]=new_translation[2]+abs_coordinates[2] #UR Conform
		
		#Now rotate the whole model for the absolute angle
		#new_rotationMatrix,new_translation = rotate_frame(rotationMatrixV2,new_translation,axis="z",rot_angle=-90)
		#Convert Euler to Rotation Vector
		try:
			RXRYRZ_temp=euler2rxryrz(new_euler)
		except:
			print("Div/0 Error: ",new_euler)
			print("using [0,0,0] for rxryrz")
			RXRYRZ_temp=new_euler	
		#For Demonstration Purpose: Half Cases can not be separated, so we need to separate our self TODO How can this be fixed??
		"""
		extracted_waypoints.append([
			new_translation[0],
			new_translation[1],
			new_translation[2],
			RXRYRZ_temp[0],
			RXRYRZ_temp[1],
			RXRYRZ_temp[2]
			])
		"""
		if new_translation_rel[1]>=0: #Check Y value to get Half Case only
			xs1.append(new_translation[0])
			ys1.append(new_translation[1])
			zs1.append(new_translation[2])
			extracted_waypoints_rel.append([
				new_translation_rel[0], #UR Conform
				new_translation_rel[1], #UR Conform
				new_translation_rel[2], #UR Conform
				RXRYRZ_temp[0], #UR Conform
				RXRYRZ_temp[1], #UR Conform
				RXRYRZ_temp[2], #UR Conform
				new_euler[0], #for further processing it is good to keep th eeuler angles
				new_euler[1], #for further processing it is good to keep th eeuler angles
				new_euler[2]  #for further processing it is good to keep th eeuler angles
				])
			extracted_waypoints.append([
				new_translation[0], #UR Conform
				new_translation[1], #UR Conform
				new_translation[2], #UR Conform
				RXRYRZ_temp[0], #UR Conform
				RXRYRZ_temp[1], #UR Conform
				RXRYRZ_temp[2], #UR Conform
				new_euler[0], #for further processing it is good to keep th eeuler angles
				new_euler[1], #for further processing it is good to keep th eeuler angles
				new_euler[2]  #for further processing it is good to keep th eeuler angles
				])
			#rs1.append(math.sqrt(new_translation[1]*new_translation[1]+new_translation[0]*new_translation[0]))	
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
	#print(len(extracted_waypoints))
	return extracted_waypoints,extracted_waypoints_rel
#Main
#print(len(func_read_BOM_and_extract_relative_coordinates("FW51530"))+len(func_read_BOM_and_extract_relative_coordinates("FW51528")))
