# -*- coding: utf-8 -*-
"""
Created on Mon Jan 23 18:21:23 2017

@author: monica
"""

import cv2
import numpy as np
import random
import math

class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent
        
    def __str__(self):
        return str(self.point)
        
        
#img = cv2.imread('pic4.png',0)
#img = cv2.imread('pic3.jpg',0)

#img = cv2.imread('shape.png')
#img1 = cv2.imread('shape.png',0)

#img = cv2.imread('pic1.jpg')
#img1 = cv2.imread('pic1.jpg',0)

img = cv2.imread('1.png')
img1 = cv2.imread('1.png',0)
img_final = cv2.imread('1.png')

#img = cv2.imread('pic4.png')
#img1 = cv2.imread('pic4.png',0)
#img_final = cv2.imread('pic4.png')

ret,world = cv2.threshold(img1,127,255,cv2.THRESH_BINARY)
height, width = img1.shape
print img1.shape

#world = img
#cv2.imshow('window',world)
        
#cv2.waitKey(20)

X_min = 0
Y_min = 0
X_max = width - 10
Y_max = height - 10

start = (10,10)
#start = (width/2,height/2)
end = (260,260)
stepSize = 15
goalFreq = 1000
color = ( 0, 0, 255 )

NUM_NODES = 5000


def distance(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def nearestNeighbour(tree,node):
    nearestNode = tree[0]    
    for n in tree:
        if distance(node.point,n.point) < distance(node.point,nearestNode.point):
            nearestNode = n
    return nearestNode

def isOccupied(node):
    """if freqCount < goalFreq:
    Takes in a node and returns if node is occupied by obstacle
    @param node <-- tuple
    @return bool
    """
    #print '\t World Value: ', world[node[0],node[1]]
    
    if world[node[1],node[0]] < 127:
        return True
    else: # world[node[0],node[1]] >= 127:
        return False
        
def isOutofBound(node):
    """
    Takes in a node and returns if node is out of image
    @param node <-- tuple
    @return bool
    """
    if node[0] > X_max or node[0] < X_min or node[1] > Y_max or node[1] < Y_min:
        #print "Node out of bound \n"        
        return True
    else:
        return False
        
    
def selectRandom():
    while(True):
        #print 'Inside select randome while*************'
        x = random.randint(X_min,X_max)
        y = random.randint(Y_min,Y_max)
        
        #print '\t', 'Selected %d, %d with %d'%(x, y, world[x, y])
        #print 'Truth Value: ', isOccupied((x,y)), '\n'
        
        if not isOccupied((x,y)):
            random_node = Node((x,y), None)
            return random_node    
 
def move_from_to_old(node1,node2):
    if distance(node1.point,node2.point) < stepSize:
        return node2
    else:
        
        angle = math.atan2(node2.point[1]-node1.point[1],node2.point[0]-node1.point[0])
        x = node1.point[0] + stepSize * math.cos(angle)
        y = node1.point[1] + stepSize * math.sin(angle)
        
        return Node((int(x),int(y)),None)


def move_from_to(node1,node2):
    d = distance(node1.point, node2.point)
    
    if d < stepSize:
        return node2
    else:
        diff = [node2.point[0] - node1.point[0], node2.point[1] - node1.point[1]]
        x = node1.point[0] + stepSize * float(diff[0]) / d 
        y = node1.point[1] + stepSize * float(diff[1]) / d
        return Node((int(x),int(y)),None)
        
def extend(tree, x):
    
    x_near = nearestNeighbour(tree,x)
    
    x_new = move_from_to(x_near,x)
    #print x_new
    
    if isOutofBound(x_new.point) == False:
        if (isOccupied(x_new.point) == False):
            
            tree.append(x_new)
            x_new.parent = x_near
        else:
            
            x_new = None
        
        return (x_new, tree)
    
    else:
        return (None, tree)
    
def build_RRT(x_init,x_goal):    
    global world
    
    cv2.circle(img, x_init.point, 3, color=1, thickness=0, lineType=8, shift=0)
    cv2.circle(img, x_goal.point, 3, color=1, thickness=0, lineType=8, shift=0)
    
    tree = list()
    tree.append(x_init)
    count = 0
    freqCount = 0
    status = 'advance'
    while(count <= NUM_NODES):
        while(status != 'goal_reached'):
            #print '--------', count
            if freqCount == goalFreq:
                x_rand = x_goal
                freqCount = 0
            else:
                #print '\t', 'Select Random Node...'
                x_rand = selectRandom()
            #cv2.circle(img, x_rand.point, 1, color=1, thickness=0, lineType=8, shift=0)
            x_new, tree = extend(tree, x_rand)
            
            if not x_new == None:
                #print 'Before CV Show',  x_new, x_new.parent
                cv2.circle(img, x_new.point, 1, color=1, thickness=0, lineType=8, shift=0)
                cv2.line(img, x_new.parent.point, x_new.point, 1, 1)
                
            else:
                pass
                #print "New node was occupied attempt failed \n"
                
            count += 1
            freqCount +=1 
            
            if not x_new == None:
                if distance(x_new.point,x_goal.point) <= 10:
                    status = 'goal_reached'
                    x_goal.parent = x_new
                    p = x_goal
                    
                    while (p.point != x_init.point):
                        cv2.circle(img_final, p.point, 1, color=1, thickness=0, lineType=8, shift=0)
                        cv2.line(img_final, p.parent.point, p.point, 1, 1)
                        p = p.parent
                    
                        cv2.imshow('window_f',img_final)
                        cv2.waitKey(20)
                        if cv2.waitKey(20)==27:
                            break
                    break
            else:
                pass
                
            cv2.imshow('window',img)
            
            cv2.waitKey(20)
            if cv2.waitKey(20)==27:
                break

        
            
        cv2.destroyAllWindows()        

if __name__ == '__main__':
    x_init = Node(start,None)
    x_end = Node(end,None)
    
    build_RRT(x_init,x_end)
    
	

