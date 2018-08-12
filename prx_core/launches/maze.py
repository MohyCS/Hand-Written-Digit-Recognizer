import sys
import random
import numpy as np


paras= [int(x) for x in sys.argv[1:]]

if len(paras)<3:
	print "Proper usage of this script requires the following parameters: python maze.py ROWS COLUMNS DENSITY\nROWS, COLUMNS are the number of rows and columns in the maze\nSPARSITY is a number between 0 and 100 that is the percentage of blocked cells."
	sys.exit()

n_rows= paras[0]
n_cols= paras[1]
prob=paras[2]/(float(100))
start_pos= [0, 0]


maze= [[np.random.choice([0,1],p=[prob,1-prob]) for x in range(n_cols)] for y in range(n_rows)]
maze[start_pos[0]][start_pos[1]]=1
maze[start_pos[0]][start_pos[1]+1]=0
maze=np.array(maze)
print(maze)

fmt= " ".join(["%.0f"])
header= '\n'.join([str(n_rows), str(n_cols)])

np.savetxt("maze", maze, fmt=fmt, header=header, comments='')
