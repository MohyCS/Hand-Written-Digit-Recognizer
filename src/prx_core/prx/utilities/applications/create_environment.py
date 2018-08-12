import rospy
import sys
import os

obs = sys.argv[1]
obj = sys.argv[2]

# print "Getting the parameter: ", "/utilities/obstacles/block_"+obs+"/geometries"
rosmap = rospy.get_param("/utilities/obstacles/block_"+obs+"/geometries")
if('visualization_geometry' in rosmap[0].keys()):
	rosmap[0]['visualization_geometry']['filename'] = obj
	# print "Setting the parameter: ", obj
	rospy.set_param("/utilities/obstacles/block_"+obs+"/geometries", rosmap)


