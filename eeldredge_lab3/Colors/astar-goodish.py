class GridCell(x, y, gcost, hcost, isWall):
    
    def __init__(self, x, y, gcost, hcost, isWall = false):
        self.x = x
        self.y = y
        self.gcost = gcost
        self.hcost = hcost

class aStar():

    def generateMap(gridList):
        #occupancygrid_sub = rospy.Subscriber('map', OccupancyGrid, mapCallBack, queue_size=10)

        theMap = list()
        for y in range (0, height):
            rows = list()
            for x in range (0, width):
                newCell = GridCell(x, y, 0, 0)
                if gridList[i] == 100:
                    newCell.isWall = true
                rows.append(newCell)
                i = i + 1
            theMap.append(rows)
        return theMap

    def manhattan(point,point2):
        return abs(point.point[0] - point2.point[0]) + abs(point.point[1]-point2.point[0])


    def aStar(start, goal, grid):
        #The open and closed sets
        ourMap = generateMap(grid)
        open_set = set()
        closed_set = set()
        #Current point is the starting point
        current = start
        #Add the starting point to the open set
        open_set.add(current)
        #While the open set is not empty
        while open_set:
            #Find the item in the open set with the lowest G + H score
            current = min(open_set, key=lambda o:o.gcost + o.hcost)
            #If it is the item we want, retrace the path and return it
            if current == goal:
                path = []
                while current.parent:
                    path.append(current)
                    current = current.parent
                path.append(current)
                return path[::-1]
            #Remove the item from the open set
            open_set.remove(current)
            #Add it to the closed set
            closed_set.add(current)
            #Loop through the node's children/siblings
            for node in neighbors(current,ourMap):
                if node is None:
                    continue
                #If it is already in the closed set, skip it
                if node in closed_set:
                    continue
                #Otherwise if it is already in the open set
                if node in open_set:
                    #Check if we beat the G score 
                    new_g = current.gcost + current.move_cost(node)
                    if node.gcost > new_g:
                        #If so, update the node to have a new parent
                        node.gcost = new_g
                        node.parent = current
                else:
                    #If it isn't in the open set, calculate the G and H score for the node
                    node.gcost = current.gcost + current.move_cost(node)
                    node.hcost = manhattan(node, goal)
                    #Set the parent to our current item
                    node.parent = current
                    #Add it to the set
                    openset.add(node)
        #Throw an exception if there is no path
        raise ValueError('No Path Found')

    def neighbors(current, grid):
        x = current.x
        y = current.y

        myNeighbors = list()
        up = grid[y-1][x]
        down = grid[y+1][x]
        right = grid[y][x+1]
        left = grid[y][x-1]

        
        myNeighbors.append(up)
        myNeighbors.append(down)
        myNeighbors.append(right)
        myNeighbors.append(left)

        return myNeighbors

