frontier_pub = rospy.publisher('/frontier', GridCells, queue_size=10)

name=GridCells()
name.header.frame_id = 'map'
name.cells.cell_width = 1
name.cells.cell_height = 1
name.cells.append (Point(1,1,0))
// name.cells.append (Point(1,1,0))

frontier_pub.publish(name)