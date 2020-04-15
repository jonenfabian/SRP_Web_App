import numpy as np
from numpy import linalg

import sys
import cmath
import math
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi

import socket

global mat
mat=np.matrix


# ****** Coefficients ******


global d1, a2, a3, a7, d4, d5, d6
d1 =  0.1273
a2 = -0.612
a3 = -0.5723
a7 = 0.075
d4 =  0.163941
d5 =  0.1157
d6 =  0.0922

global d, a, alph

#d = mat([0.089159, 0, 0, 0.10915, 0.09465, 0.0823]) ur5
d = mat([0.1273, 0, 0, 0.163941, 0.1157, 0.0922])#ur10 mm
# a =mat([0 ,-0.425 ,-0.39225 ,0 ,0 ,0]) ur5
a =mat([0 ,-0.612 ,-0.5723 ,0 ,0 ,0])#ur10 mm
#alph = mat([math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0 ])  #ur5
alph = mat([pi/2, 0, 0, pi/2, -pi/2, 0 ]) # ur10

def euler2mat(rotation=[0,0,0],translation=[0,0,0],point=[0,0,0]): #Zum Verschieben des Tools in gewuenschte Richtung (Point)
	#Muss noch getestet werden
	#Point nicht implementiert
    #EINSATZ NUR FÜR 3X3 ROTATIONSMATRIX TRANSLATION AUS UNITY EINBINDEN
	xC, xS = cos(rotation[0]), sin(rotation[0])
	yC, yS = cos(rotation[1]), sin(rotation[1])
	zC, zS = cos(rotation[2]), sin(rotation[2])
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

# ************************************************** FORWARD KINEMATICS

def AH( n,th,c  ):

  T_a = mat(np.identity(4), copy=False)
  T_a[0,3] = a[0,n-1]
  T_d = mat(np.identity(4), copy=False)
  T_d[2,3] = d[0,n-1]

  Rzt = mat([[cos(th[n-1,c]), -sin(th[n-1,c]), 0 ,0],
	         [sin(th[n-1,c]),  cos(th[n-1,c]), 0, 0],
	         [0,               0,              1, 0],
	         [0,               0,              0, 1]],copy=False)
      

  Rxa = mat([[1, 0,                 0,                  0],
			 [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
			 [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
			 [0, 0,                 0,                  1]],copy=False)

  A_i = T_d * Rzt * T_a * Rxa


  return A_i

def HTrans(th,c ):  
  A_1=AH( 1,th,c  )
  A_2=AH( 2,th,c  )
  A_3=AH( 3,th,c  )
  A_4=AH( 4,th,c  )
  A_5=AH( 5,th,c  )
  A_6=AH( 6,th,c  )
      
  T_06=A_1*A_2*A_3*A_4*A_5*A_6

  return T_06

# ************************************************** INVERSE KINEMATICS 

def invKine(desired_pos):# T60
  th = mat(np.zeros((6, 8)))
  P_05 = (desired_pos * mat([0,0, -d6, 1]).T-mat([0,0,0,1 ]).T)
  
  # **** theta1 ****
  
  psi = atan2(P_05[2-1,0], P_05[1-1,0])
  phi = acos(d4 /sqrt(P_05[2-1,0]*P_05[2-1,0] + P_05[1-1,0]*P_05[1-1,0]))
  #The two solutions for theta1 correspond to the shoulder
  #being either left or right
  th[0, 0:4] = pi/2 + psi + phi
  th[0, 4:8] = pi/2 + psi - phi
  th = th.real
  
  # **** theta5 ****
  
  cl = [0, 4]# wrist up or down
  for i in range(0,len(cl)):
	      c = cl[i]
	      T_10 = linalg.inv(AH(1,th,c))
	      T_16 = T_10 * desired_pos
	      th[4, c:c+2] = + acos((T_16[2,3]-d4)/d6);
	      th[4, c+2:c+4] = - acos((T_16[2,3]-d4)/d6);

  th = th.real
  
  # **** theta6 ****
  # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
	      c = cl[i]
	      T_10 = linalg.inv(AH(1,th,c))
	      T_16 = linalg.inv( T_10 * desired_pos )
	      th[5, c:c+2] = atan2((-T_16[1,2]/sin(th[4, c])),(T_16[0,2]/sin(th[4, c])))
		  
  th = th.real

  # **** theta3 ****
  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
	      c = cl[i]
	      T_10 = linalg.inv(AH(1,th,c))
	      T_65 = AH( 6,th,c)
	      T_54 = AH( 5,th,c)
	      T_14 = ( T_10 * desired_pos) * linalg.inv(T_54 * T_65)
	      P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
	      t3 = cmath.acos((linalg.norm(P_13)**2 - a2**2 - a3**2 )/(2 * a2 * a3)) # norm ?
	      th[2, c] = t3.real
	      th[2, c+1] = -t3.real

  # **** theta2 and theta 4 ****

  cl = [0, 1, 2, 3, 4, 5, 6, 7]
  for i in range(0,len(cl)):
	      c = cl[i]
	      T_10 = linalg.inv(AH( 1,th,c ))
	      T_65 = linalg.inv(AH( 6,th,c))
	      T_54 = linalg.inv(AH( 5,th,c))
	      T_14 = (T_10 * desired_pos) * T_65 * T_54
	      P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
	      
	      # theta 2
	      th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a3* sin(th[2,c])/linalg.norm(P_13))
	      # theta 4
	      T_32 = linalg.inv(AH( 3,th,c))
	      T_21 = linalg.inv(AH( 2,th,c))
	      T_34 = T_32 * T_21 * T_14
	      th[3, c] = atan2(T_34[1,0], T_34[0,0])
  th = th.real

  return th

def startIK(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, sh_side, wrst_side):
    rot_matrix=euler2mat(rotation=[math.radians(rot_x), math.radians(rot_y), math.radians(rot_z)])
    rot_matrix[0][3]= pos_x
    rot_matrix[1][3]= pos_y
    rot_matrix[2][3]= pos_z
    print(rot_matrix)
    desired_pos = rot_matrix

    invkin_m = invKine(desired_pos)
    solution = ""
    np.str(solution)
    count = 0
    if sh_side == "lft":
        if wrst_side == "up":
            for i in invkin_m[:,2]:
                count = count + 1
                solution = solution + "%i-Joint:%f;" %( count, math.degrees(i))
        elif wrst_side == "down":
            for i in invkin_m[:,0]:
                count = count + 1
                solution = solution + "%i-Joint:%f;" %( count, math.degrees(i))
        else:
            print("ERROR IN WRISTSIDE VARIABLE")
    elif sh_side == "rght":
        if wrst_side == "up":
            for i in invkin_m[:,5]:
                count = count + 1
                solution = solution + "%i-Joint:%f;" %( count, math.degrees(i))
        elif wrst_side == "down":
            for i in invkin_m[:,7]:
                count = count + 1
                solution = solution + "%i-Joint:%f;" %( count, math.degrees(i))
        else:
            print("ERROR OM WRISTSIDE VARIABLE")
    else:
        print("ERROR IN SHOULDERSIDE VARIABLE")
    return solution

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('127.0.0.1', 65432))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print('Connected by', addr)
            while True:
                data = conn.recv(1024)
                text = data.decode('utf-8')
                pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, sh_side, wr_side = text.split(";", 7)
                text = startIK( float(pos_x.replace(',','.')), 
                                float(pos_y.replace(',','.')),
                                float(pos_z.replace(',','.')), 
                                float(rot_x.replace(',','.')), 
                                float(rot_y.replace(',','.')), 
                                float(rot_z.replace(',','.')), 
                                sh_side,
                                wr_side)
                data = text.encode('utf-8')
                conn.sendall(data)

main()