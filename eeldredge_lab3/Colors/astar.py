#!/usr/bin/env python
import rospy, numpy, math
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point, Twist, Pose
from Queue import PriorityQueue
from math import sqrt

# Global Vars
global resolution

class A_Star:
	pub_path = rospy.Publisher('/mapData/Path', GridCells, queue_size = 10)

	resolution = 0.3

	def __init__(self, node_list, cost, x, y):
		self.data = data
		self.node = node_list
		self.path = []
		self.cost = cost
		self.x = x
		self.y = y
		
		self.gscore = gscore
		self.fscore = fscore
		
		self.origin = True
		self.output_openSet = True



	def find_path(self, begin, end):
		frontier = set() #set for all the nodes to try traversing
	    traversed = set() #list of nodes we have tried 
	    path = [] #eventually will be the list of nodes in the actual path
	    waypoints = []
	    curr_node = begin
	    curr_cost = 0
	    frontier.add(curr_node) #put the first node in frontier list
		
		while frontier:
			curr_node = min(frontier,key=lambda f:total_cost(curr_node,end))
			if curr_node == end:
				path = make_path(curr_node, path, waypoints)
				publish_grid_cells(path, pub_path)
				break #not sure if we need this
			children = get_children(curr_node)


	def make_path(self, waypoints):
		self.path.append(self.node)
		publish_grid_cells(self.path, pub_path)
		publish_grid_cells(waypoints, pub_ways)
		
		rospy.sleep(0.1)

		while curr_node.parent is not None:
			if(curr_node.parent.parent.x and curr_node.parent.parent.y):
				waypoints.append(curr_node.parent)
			make_path(curr_node.parent, path, waypoints)

		return path, waypoints

class iterate_nodes(gaaa):

	def __init__(self, iteration):
		helpme = A_Star.

	def arrived_goal(self, current, end):
		return current == true

	def iterate_nodes

	def astar(self, begin, end):
		if self.arrivedAtGoal(begin, end):
			return [begin]

		searchNodes = A_Star.iterate_nodes()
		beginNode = searchNodes[begin] = A_Star.iterate_nodes(begin, gcost = 0, fscore = self.cost_estimate(begin, end))

		openSet = [] #borrowed idea, what we were missing

		push_heap(openSet, beginNode) #push the current node to the heap

		while openSet: #from that idea above^
			current =  pop_heap(openSet)
			if self.at_goal(current.data):
				return self.give_back_path(current, backwardsPlot)

			current.output_openSet = True
			current.closed = True

			for neighbor in [searchNodes[n] for n in self.neighbors(current.data)]: #borred from same idea^
				if neighbor.closed:
					continue
				temp_gscore = current.gscore + self.point_difference(current.data, neighbor.data)

				if temp_gscore >= neighbor.gscore:
					continue

				neighbor.origin = current
				neighbor.gscore = temp_gscore
				neighbor.fscore = temp_gscore + self.cost_estimate(neighbor.data, end)
				if neighbor.output_openSet:
					neighbor.output_openSet = False
					push_heap(openSet, neighbor)
		return None


	def publish_grid_cells(data, send):
		cells = GridCells()
		cells.header.frame_id = 'map'
		cells.cell_width = resolution
		cells.cell_height = resolution
		for node in data
			cord = dist(curr_node, end)
			cells.cells.append(cord)
		pub.publish(cells
)

	def dist(self, current, end):
			return math.sqrt((current.x-end.x)**2 + (current.y-end.y)**2)

	def get_children(self, current):
		curr_x = current.x
		curr_y = current.y
		kids = []
		#this needs to be finished 
		#check nodes in surrounding area of current node
		#if they aren't the currend node then add to list of children
		for x range(-1,1):
			for y in range(-1,1):
				if(not(x == 0 and y == 0)):
					kids.append(self.nodes [current.x + x] [current.y + y])
		return kids


		