import numpy as np
from math import *
import pylab as plt


floorMap= [[7,6,6,6,5,4,4],
           [7,0,5,0,0,0,3],
           [7,6,5,4,3,2,2],
           [7,6,0,4,3,0,1],
           [7,6,5,4,0,2,2]]

	

robotX=0
robotY=0

goalX=6
goalY=1


currentlocation=0

wall=0
#uncalculated!=0

floorMap[robotX][robotY]=7


def changeSurroundings(X, Y):
    #left
    if(floorMap[X-1][Y]!=0):
        floorMap[X-1][Y]=currentlocation  
    #right 
    if(floorMap[X+1][Y]!=0):
        floorMap[X+1][Y]=currentlocation  
    #up
    if(floorMap[X][Y-1]!=0):
        floorMap[X][Y-1]=currentlocation  
    #down
    if(floorMap[X][Y+1]!=0):
        floorMap[X][Y+1]=currentlocation    

def printMap():
    i=0
    while(i<len(floorMap)):
        print floorMap[i]
        print ""
        i+=1
    print ""
    print ""



while(floorMap[goalX][goalY]!=1):
    
    x=0
    while(x<len(floorMap[0])):
	y=0
        while(y<len(floorMap)):
            if(floorMap[x][y] > 0 and floorMap[x][y] < 8):
                currentlocation=floorMap[x][y]+1
                changeSurroundings(x,y)
                printMap()

            y+=1
        x+=1





