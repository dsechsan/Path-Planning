# priority queue for OPEN list
from pqdict import pqdict
import math
import numpy as np


class AStarNode(object):
  def __init__(self, pqkey, coord, hval):
    self.pqkey = pqkey
    self.coord = coord
    self.g = math.inf
    self.h = hval
    self.parent_node = None
    self.parent_action = None
    self.closed = False
  def __lt__(self, other):
    return self.g < other.g     

class Environment(object):
    '''
    Stores the environment data i.e, boundary and blocks
    '''
    def __init__(self,boundary,blocks,goal):
        self.boundary = boundary
        self.blocks = blocks
        self.goal = goal 
        
    def getHeuristic(self,curr_coord):
        return np.linalg.norm(curr_coord - self.goal)

def isAllowed(boundary,blocks,curr,next):
    '''
    Checks if a given line segment is intersecting with any of the blocks
    Input: Environment, start point, end point
    '''
    if( next[0] <= boundary[0,0] or next[0] >= boundary[0,3] or \
        next[1] <= boundary[0,1] or next[1] >= boundary[0,4] or \
        next[2] <= boundary[0,2] or next[2] >= boundary[0,5] ):
          return False
      
    valid = True
    for k in range(blocks.shape[0]):
        block = blocks[k]
        isColliding = checkCollision(block,curr,next)
        if isColliding: 
            valid = False
    if valid:
        return True
    else:
        return False
  
    
def checkCollision(block,start,end):
    '''
    Checks if a line segment is colliding with a given block
    '''
    P1,P2 = start,end
    AABB_Min = block[0:3]
    AABB_Max = block[3:6]
    
    if max(P1[0], P2[0]) < AABB_Min[0] or min(P1[0], P2[0]) > AABB_Max[0]:
        return False
    if max(P1[1], P2[1]) < AABB_Min[1] or min(P1[1], P2[1]) > AABB_Max[1]:
        return False
    if max(P1[2], P2[2]) < AABB_Min[2] or min(P1[2], P2[2]) > AABB_Max[2]:
        return False

    # Compute direction vector of the line segment
    dir_x = P2[0] - P1[0]
    dir_y = P2[1] - P1[1]
    dir_z = P2[2] - P1[2]

    # Compute parameter values for intersection with AABB boundaries
    tx1 = (AABB_Min[0] - P1[0]) / dir_x
    tx2 = (AABB_Max[0] - P1[0]) / dir_x
    ty1 = (AABB_Min[1] - P1[1]) / dir_y
    ty2 = (AABB_Max[1] - P1[1]) / dir_y
    tz1 = (AABB_Min[2] - P1[2]) / dir_z
    tz2 = (AABB_Max[2] - P1[2]) / dir_z

    # Find largest entry for entering AABB (tEnter) and smallest entry for exiting (tExit)
    tEnter = max(min(tx1, tx2), min(ty1, ty2), min(tz1, tz2))
    tExit = min(max(tx1, tx2), max(ty1, ty2), max(tz1, tz2))

    # Check for no collision
    if tEnter > tExit or tExit < 0:
        return False

    # Check for collision
    if 0 <= tEnter <= 1 or 0 <= tExit <= 1:
        return True

    # Check for containment
    if tEnter > 1 and tExit > 1:
        return True

    return False

class AStar(object):
  @staticmethod
  def plan(start_coord, environment, res = 0.5, epsilon = 1):
    # Initialize the graph and open list
    Graph = {}
    OPEN = pqdict()
    path = []
    
    # initialization of the graph
    start_node = AStarNode(tuple(start_coord), start_coord, environment.getHeuristic(start_coord))
    start_node.g = 0
    OPEN.additem(start_node.pqkey,start_node.g + epsilon * start_node.h)
    Graph[start_node.pqkey] = start_node
    
    # TODO: Implement A* here
    blocks = environment.blocks
    boundary = environment.boundary
    numofdirs = 26
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
    dR = np.delete(dR,13,axis=1)
    dR = dR*res
    
    while OPEN: # Ideally it should be while goal is in closed:
        current_key = OPEN.popitem()[0]
        current_node = Graph[current_key]
        current_node.closed = True
        if(np.linalg.norm(current_node.coord - environment.goal)< res): # Actual exit condition
            while current_node:
                path.append(current_node.coord)
                current_node = current_node.parent_node
            path.reverse()
            return np.array(path)
        
        # Create a node in every directions if it doesn't already exist
        for i in range(numofdirs):
            child_coord = (current_node.coord + dR[:,i]).astype(np.float16)
            if(not isAllowed(boundary,blocks,current_node.coord,child_coord)):
                continue
            if(tuple(child_coord) in Graph.keys()):
                child_node = Graph[tuple(child_coord)]
            else:
                child_node = AStarNode(tuple(child_coord),child_coord,environment.getHeuristic(child_coord))
                Graph[child_node.pqkey] = child_node
            
            if(child_node.closed is False):
                if(child_node.g > current_node.g + np.linalg.norm(dR[:,i])):
                    child_node.g = current_node.g + np.linalg.norm(dR[:,i])
                    child_node.parent_node = current_node
                    if(child_node.pqkey in OPEN):
                        OPEN.updateitem(child_node.pqkey, child_node.g + epsilon*child_node.h)
                    else:
                        OPEN.additem(child_node.pqkey,child_node.g + epsilon*child_node.h)
                
        

        
        
            

