#from astar import AStar
#https://gist.github.com/ryancollingwood/32446307e976a11a1185a5394d6657bc
import sys
import math
import time
from warnings import warn
class Node():
	"""A node class for A* Pathfinding"""

	def __init__(self, parent=None, position=None):
		self.parent = parent
		self.position = position

		self.g = 0
		self.h = 0
		self.f = 0

	def __eq__(self, other):
		return self.position == other.position
def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path

def astar(maze, start_pos, end_pos,start_time,allow_diagonal_movement = True):
	"""Returns a list of tuples as a path from the given start to the given end in the given maze"""
	#Get the cells for the pick_up position and operation_position
	start_pos_cell="None"
	end_pos_cell="None"
	for layer in maze:
		if start_pos_cell!="None" and end_pos_cell!="None":
			break
		for row in layer:
			for col in row:
				#print("col: ",col)
				p1=[col[3][0],col[3][1],col[5][0]]
				#print(p1)
				p2=[col[3][2],col[3][3],col[5][1]]
				if p1[0]<=start_pos[0]<=p2[0] and p1[1]<=start_pos[1]<=p2[1] and p1[2]<=start_pos[2]<=p2[2]:
					start_pos_cell=(col[0],col[1],col[2])
				elif p1[0]<=end_pos[0]<=p2[0] and p1[1]<=end_pos[1]<=p2[1] and p1[2]<=end_pos[2]<=p2[2]:
					end_pos_cell=(col[0],col[1],col[2])
			if start_pos_cell!="None" and end_pos_cell!="None":
				break
	if start_pos_cell=="None" or end_pos_cell=="None":
		print("could not find start or end pose in grid for pathplanning")
	#print(start_pos_cell)
	#print(end_pos_cell)
	# Create start and end node
	start_node = Node(None, start_pos_cell)
	start_node.g = start_node.h = start_node.f = 0
	end_node = Node(None, end_pos_cell)
	end_node.g = end_node.h = end_node.f = 0

	# Initialize both open and closed list
	open_list = []
	closed_list = []

	# Add the start node
	open_list.append(start_node)
	
	# Adding a stop condition
	outer_iterations = 0
	max_iterations = (len(maze) // 2) ** 3

	# what squares do we search
	adjacent_squares = ((0, -1, 0), (0, 1, 0), (-1, 0, 0), (1, 0, 0),(0, 0, -1),(0, 0, 1),) # 6 neighbours
	if allow_diagonal_movement:
		"""
		#create cube
		thickness_of_safety_zone=2
		adjacent_squares=[]
		for i0_ in range(-thickness_of_safety_zone,thickness_of_safety_zone+1):
			for i1_ in range(-thickness_of_safety_zone,thickness_of_safety_zone+1):
				for i2_ in range(-thickness_of_safety_zone,thickness_of_safety_zone+1):
					if i0_==0 and i1_==0 and i2_==0:
						None
					else:
						adjacent_squares.append([i0_,i1_,i2_])
		"""
		adjacent_squares = ((0, -1, 0), (0, 1, 0), (-1, 0, 0), (1, 0, 0), (-1, -1, 0), (-1, 1, 0), (1, -1, 0), (1, 1, 0),
							(0, -1, 1), (0, 1, 1), (-1, 0, 1), (1, 0, 1), (-1, -1, 1), (-1, 1, 1), (1, -1, 1), (1, 1, 1),(0, 0, 1),
							(0, -1, -1), (0, 1, -1), (-1, 0, -1), (1, 0, -1), (-1, -1, -1), (-1, 1, -1), (1, -1, -1), (1, 1, -1),(0, 0, -1))
	# Loop until you find the end
	while len(open_list) > 0:
		outer_iterations += 1
		
		# Get the current node
		current_node = open_list[0]
		current_index = 0
		for index, item in enumerate(open_list):
			if item.f < current_node.f:
				current_node = item
				current_index = index
				
		if outer_iterations > max_iterations:
			# if we hit this point return the path such as it is
			# it will not contain the destination
			warn("giving up on pathfinding too many iterations")
			success_flag=0
			return [],start_pos_cell,end_pos_cell,success_flag

		# Pop current off open list, add to closed list
		open_list.pop(current_index)
		closed_list.append(current_node)

		# Found the goal
		if current_node == end_node:
			success_flag=1
			return return_path(current_node),start_pos_cell,end_pos_cell,success_flag

		# Generate children
		children = []
		
		for new_position in adjacent_squares:  # Adjacent squares

			# Get node position
			node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1], current_node.position[2] + new_position[2])

			# Make sure within range
			within_range_criteria = [
				node_position[2] > (len(maze) - 1),
				node_position[2] < 0,
				node_position[0] > (len(maze[len(maze) - 1]) - 1),
				node_position[0] < 0,
				node_position[1] > (len(maze[len(maze[len(maze) - 1]) - 1]) - 1),
				node_position[1] < 0,
			]
			
			if any(within_range_criteria):
				continue

			# Make sure walkable terrain
			if maze[node_position[2]][node_position[0]][node_position[1]][4] != 0:
				continue

			# Create new node
			new_node = Node(current_node, node_position)

			# Append
			children.append(new_node)

		# Loop through children
		for child in children:
			
			# Child is on the closed list
			if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
				continue

			# Create the f, g, and h values
			child.g = current_node.g + 1
			#child.h = ((child.position[0] - end_node.position[0]) ** 2) + \
			#		  ((child.position[1] - end_node.position[1]) ** 2) + \
			#		  ((child.position[2] - end_node.position[2]) ** 2)
			child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2) + ((child.position[2] - end_node.position[2]) ** 2)
					  
					  
			child.f = child.g + child.h

			# Child is already in the open list
			if len([open_node for open_node in open_list if child == open_node and child.g > open_node.g]) > 0:
				continue

			# Add the child to the open list
			open_list.append(child)
			
