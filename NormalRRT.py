#imports
import simpleguitk as simple
from PIL import Image as IM
import math
import random
import matplotlib.path as mplPath
import numpy as np

#global parameters:
IMAGE_CENTER = [45, 45]
IMAGE_SIZE = [90, 90]
IMAGE_RADIUS = 35


curioImage = IM.open('double.png').convert("RGBA")
ROBOT_IMAGE = simple.load_image(curioImage)

WIDTH = 400#800

scaleVal = 500/10

HEIGHT = 400#10 * scaleVal
ANGLES = [s for s in range(-90,91)]
X_min = 0
X_max = WIDTH
Y_min = 0
Y_max = HEIGHT
delta_tt = 0.1

wall_list = list()

end = (70,70,0)
start = (270,170,0)
#start = (200,350,-180)
NUM_NODES = 10000
goalFreq = 100

tree = list()
final_points = list()
counter = 0
index = 0


count = 0
freqCount = 0
status = 'advance'
g_radius = 35

def scale(humanVal):
    """
    Returns the scaled value to pixel.

    @humanVal: list, or 2-tuple: [x, y] in meter 
    Return: 2-tuple: [pix_x, pix_y] in pixels

    [Assumes: scale factor is previously generated]
    """
    if not scaleVal == None:
        if type(humanVal) is list:
            return (humanVal[0] * scaleVal, humanVal[1] * scaleVal)
        else:
            return humanVal * scaleVal
    else:
        return None

stepSize = scale(0.1)
      
class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent
        self.reachableSet = None
        self.child = None
        
    def __str__(self):
        return str(self.point)
        
    def reachabilitySet(self,r_s,r_b):
        self.reachableSet = (r_s,r_b)
     
x_init = Node(start,None)
x_goal = Node(end,None)
X = x_init.point[0]
Y = x_init.point[1]
theta = math.radians(x_init.point[2])
#theta = math.atan2((x_goal.point[1] - x_init.point[1]),(x_goal.point[0] - x_init.point[0]))

class Obstacle(object):
    def __init__(self,vertices):
        dv = 4
        self.vertices = vertices
        
        self.expand = list()
        
        self.expand.append([vertices[0][0]-IMAGE_SIZE[0]/dv,vertices[0][1]-IMAGE_SIZE[1]/dv])
        self.expand.append([vertices[1][0]+IMAGE_SIZE[0]/dv,vertices[1][1]-IMAGE_SIZE[1]/dv])
        self.expand.append([vertices[2][0]+IMAGE_SIZE[0]/dv,vertices[2][1]+IMAGE_SIZE[1]/dv])
        self.expand.append([vertices[3][0]-IMAGE_SIZE[0]/dv,vertices[3][1]+IMAGE_SIZE[1]/dv])
        #print self.expand
    def collision(self,robot):
        
        o_path = mplPath.Path(np.array(self.expand))
        
        return o_path.contains_point(robot)

o1 = Obstacle([[0,0],[10,0],[10,HEIGHT],[0,HEIGHT]])
wall_list.append(o1)
o2 = Obstacle([[WIDTH-10,0],[WIDTH,0],[WIDTH,HEIGHT],[WIDTH-10,HEIGHT]])
wall_list.append(o2)
o3 = Obstacle([[0,0],[WIDTH,0],[WIDTH,10],[0,10]])
wall_list.append(o3)
o4 = Obstacle([[0,HEIGHT-10],[WIDTH,HEIGHT-10],[WIDTH,HEIGHT],[0,HEIGHT]])
wall_list.append(o4)
o5 = Obstacle([[150,150],[200,150],[200,200],[150,200]])
wall_list.append(o5)

def isOccupied(r):
    p = (r[0],r[1])
    for w in wall_list:
        if w.collision(p):
            #print "bingo its working"
            return True
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
        x = random.randint(X_min,X_max)
        y = random.randint(Y_min,Y_max)
        ang = random.uniform(-180,181)
        
        if not isOccupied((x,y)):
            random_node = Node((x,y,ang), None)
            return random_node    
            
def distance(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (abs(p1[2])-abs(p2[2]))**2)

def p_distance(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )
    
def nearestNeighbour(tree,node):
    nearestNode = tree[0]    
    for n in tree:
        if distance(node.point,n.point) < distance(node.point,nearestNode.point):
            nearestNode = n
            
    d = distance(node.point,nearestNode.point)
    return (nearestNode,d)
    
def nearestReachable(nearNode,nodeRand):
    if not (nearNode.point == nodeRand.point):
        cx_s = nearNode.point[0] + nearNode.reachableSet[0] * ((nodeRand.point[0] - nearNode.point[0]) / distance(nearNode.point,nodeRand.point))
        cy_s = nearNode.point[1] + nearNode.reachableSet[0] * ((nodeRand.point[1] - nearNode.point[1]) / distance(nearNode.point,nodeRand.point))
        
        cx_b = nearNode.point[0] + nearNode.reachableSet[1] * ((nodeRand.point[0] - nearNode.point[0]) / distance(nearNode.point,nodeRand.point))
        cy_b = nearNode.point[1] + nearNode.reachableSet[1] * ((nodeRand.point[1] - nearNode.point[1]) / distance(nearNode.point,nodeRand.point))
        
        ang = math.degrees(math.atan2(nodeRand.point[1]-nearNode.point[1],nodeRand.point[0]-nearNode.point[0]))
        
        ds = distance((cx_s,cy_s,ang),nodeRand.point)
        db = distance((cx_b,cy_b,ang),nodeRand.point)
        
        if ds <= db:
            return (Node((cx_s,cy_s,ang),None),ds)
        else:
            return (Node((cx_b,cy_b,ang),None),db)
    else:
        return (None,None)
        
def in_range(endAngles,testAngle):
    if (testAngle <= endAngles[0] and testAngle >= endAngles[1]):
        #print "Angle was in range \n"
        return True
    else:
        print "Cannot move to that angle \n"
        print endAngles, testAngle
        return False
        
def nearestState(tree,node):
    nearestNode,dist_node = nearestNeighbour(tree,node)
    
    nearReachable,dist_reachable = nearestReachable(nearestNode,node)
    
    if nearestNode.point[2] >= 0:
        maxAngle = nearestNode.point[2] + 90
        minAngle = nearestNode.point[2] - 90
        
        if maxAngle > 180:
            maxAngle = - (360 - maxAngle)
        if minAngle < -180:
            minAngle = 360 - abs(minAngle)
            
    elif nearestNode.point[2] < 0:
        maxAngle = nearestNode.point[2] + 90
        minAngle = nearestNode.point[2] - 90
        
        if maxAngle > 180:
            minAngle = - (360 - maxAngle)
        if minAngle < -180:
            maxAngle = 360 - abs(minAngle)
    
        
    return (nearestNode,nearestReachable)
#    if dist_reachable == None:
#        return (None,None)
#    elif dist_reachable <= dist_node and in_range((maxAngle,minAngle),nearReachable.point[2]):
#        return (nearestNode,nearestReachable)
#    else:
#        return (None,None)
          
def get_reachabilityBounds(t,robot):
    L = IMAGE_SIZE[0]
    w_max = 1.0
    radius_s = None
    radius_b = None
    
    #print type(t)    
    if robot.lower() == 'dd':
        lower_bound = (2.0*(w_max**3)*(t**3))/(3.0*L)
        upper_bound = (5.0*(w_max**3)*(t**3))/(6.0*L)
        
        radius_s = math.sqrt(lower_bound/math.pi)
        radius_b = math.sqrt(upper_bound/math.pi)
        
    elif robot.lower() == 'di':
        pass
    else:
        print "Robot not registered \n"
        
    return (radius_s,radius_b)

def insertNode(node,tree,delta_t):
    rs,rb = get_reachabilityBounds(delta_t,'dd')
    node.reachabilitySet(rs,rb)
    
    tree.append(node)
    
    return tree

def move_from_to(node1,node2):
    d = distance(node1.point, node2.point)
    
    if d < stepSize:
        return node2
    else:
        diff = [node2.point[0] - node1.point[0], node2.point[1] - node1.point[1]]
        x = node1.point[0] + stepSize * float(diff[0]) / d 
        y = node1.point[1] + stepSize * float(diff[1]) / d
        
        ang = math.degrees(math.atan2(node2.point[1]-node1.point[1],node2.point[0]-node1.point[0]))
    
        return Node((int(x),int(y),ang),None)
        
def newState(x_near,u):
    
    x_new = move_from_to(x_near,u)
    
   
    if isOutofBound(x_new.point) == False:
        if (isOccupied(x_new.point) == False):
            x_new.parent = x_near
            x_near.child = x_new
        else:
            x_new = None
    else:
        x_new = None
    return x_new
                
                
def drawHandler(canvas):
    canvas.draw_image(ROBOT_IMAGE , IMAGE_CENTER \
                , IMAGE_SIZE, [X, Y]\
                , IMAGE_SIZE, theta)
                
    for w in wall_list:
             canvas.draw_polygon(w.vertices, 1, 'Red','Red')
             canvas.draw_polygon(w.expand, 1, 'Blue')
    
    canvas.draw_circle((x_init.point[0],x_init.point[1]), 1, 1, 'Blue')    
    canvas.draw_circle((x_goal.point[0],x_goal.point[1]), 1, 1, 'Green')
    canvas.draw_circle((x_goal.point[0],x_goal.point[1]), g_radius, 1, 'Green')
    
    if status == 'goal_reached':
        for p in final_points:
            canvas.draw_line((p.parent.point[0],p.parent.point[1]), (p.point[0],p.point[1]), 2, 'Green')
    
    #print len(tree)
    if not status == 'goal_reached':
        for n in tree:
            if n == x_init:
                continue
            canvas.draw_line((n.parent.point[0],n.parent.point[1]), (n.point[0],n.point[1]), 1, 'blue')
        

def timerHandler():
    global theta, X, Y,counter,index,tree, final_points, count
    global freqCount,status, IMAGE_CENTER
    
    if count < NUM_NODES and status != 'goal_reached':
        
        if freqCount == goalFreq:
            x_rand = x_goal
            freqCount = 0
        else:
            x_rand = selectRandom()
    
        xNear,xrNear = nearestState(tree,x_rand)
            
        while (xNear == None):
            #print count
            #print "Differential constraint violeted \n"
            x_rand = selectRandom()
            xNear,xrNear = nearestState(tree,x_rand)
    
        u = x_rand
        x_new = newState(xNear,u)
        
        if not x_new == None:
            #print 'Inserting new node \n'
            tree = insertNode(x_new,tree,delta_tt)
            
        else:
            pass
            
        count += 1
        #print count
        freqCount +=1
        
        
        if not x_new == None:
            if p_distance(x_new.point,x_goal.point) <= g_radius or count == NUM_NODES:
                print "Goal Reached going to break"
                status = 'goal_reached'
                x_goal.parent = x_new
                x_new.child = x_goal
                p = x_goal
                
                while (p.point != x_init.point):
                    #print "Struck \t ",p.point
                    final_points.append(p)
                    p = p.parent
                
                print "Out of while"
                #break
        else:
            pass
    
    elif count == NUM_NODES or status == 'goal_reached':
        IMAGE_CENTER = [135,45]
        turningFreq = 10    
        #velo = 0.1
        
        next_x = final_points[len(final_points)-1-index].point[0]
        next_y = final_points[len(final_points)-1-index].point[1]
        next_a = final_points[len(final_points)-1-index].point[2]
        
        
        if not (X,Y) == x_goal.point:
            X = next_x
            Y = next_y
            theta = math.radians(next_a)
        else:
            X = x_goal.point[0]
            Y = x_goal.point[1]
        
        
        counter += 1
        
        if counter == turningFreq:
            #theta = math.atan2(next_y-old_y,next_x-old_x)
            print "something"
            index += 1
            counter = 0





# Create frame for rendering
myFrame =  simple.create_frame('Curio Simulator', WIDTH, HEIGHT)
# Create timer for updating the positions of robot(s)
tmrUpdate = simple.create_timer(10, timerHandler)
# Attach handler for graphics rendering
myFrame.set_draw_handler(drawHandler)
# Start the timer and rendering

#RG_RRT()
tree = insertNode(x_init,tree,delta_tt)
print "Out of the RG-RRT"
tmrUpdate.start()
myFrame.start()        
