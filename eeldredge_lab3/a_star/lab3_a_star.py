import math
class Node: 
    def __init__(self,x,y)
        self.x = x
        self.y = y
        self.gCosts = 0
    # def calcGCosts(self)
        # self.gCost = ???


def distManhattan(a, b): #helper to calculate manhattan distance
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def distEuclidean(a, b): #helper to calculate euclidean distance
    (x1, y1) = a
    (x2, y2) = b
    d = math.sqrt(math.pow((x1-x2),2)+math.pow((y1-y2),2))
    return d

def a_star_search(graph, first, goalNode):
    frontier = PriorityQueue() #queue for all the nodes to try traversing
    traversed = {} #list of nodes we have tried 
    actual_path = {} #eventually will be the list of nodes in the actual path
    frontier.put(first, 0) #put the first node in frontier list
    came_from = {} #nodes since last backtrace 
    recent_nodes = {} #nodes added in most recent 
    cost_so_far = {} 
    came_from[first] = None
    cost_so_far[first] = 0
    doneFlag = False

    while (not frontier.empty()) and doneFlag is False:
        traversed.put(current)
        actual_path.put(current) #add current to actual path. it will be taken out later if it doesn't work
        current = frontier.get() #new node is lowest node in the priority list
        
        if current == goalNode: 
            doneFlag = True
            break #leave while loop if we have arrived at the goalNode
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next) #add cost of current node to total cost

            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + distManhattan(goalNode, next) #heuristic is the manhattan distance from the 
                # next node to the goal node added to the current calculated cost 
                frontier.put(next, priority) #
                came_from[next] = current
    
    return actual_path, cost_so_far
