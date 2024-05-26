# priority queue for OPEN list
from pqdict import pqdict
import math
from distance3d.distance import line_segment_to_box
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


class AStar(object):
  # __slots__ = ['boundary', 'blocks']
  @staticmethod
  def plan(start_coord, environment, epsilon = 1):
    # Initialize the graph and open list
    Graph = {}
    OPEN = pqdict()
    CLOSED = set()
    open_coords = set()
    path = []
    
    # current node
    start_node = AStarNode(tuple(start_coord), start_coord, environment.getHeuristic(start_coord))
    start_node.g = 0
    OPEN.additem(start_node.pqkey,start_node.g + epsilon * start_node.h)
    Graph[start_node.pqkey] = start_node
    # open_coords.add(tuple(start_coord))
    
    # TODO: Implement A* here
    blocks = environment.blocks
    boundary = environment.boundary
    
    res=0.3
    numofdirs = 26
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
    dR = np.delete(dR,13,axis=1)
    dR = dR*res
    # dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0
    # res = np.max(dR)
    
    while OPEN:
      # print(len(OPEN),len(Graph))
      current_key = OPEN.popitem()[0]
      current_node = Graph[current_key]
      print(current_node.g)
      CLOSED.add(current_key) 
      print(np.linalg.norm(current_node.coord-environment.goal))
      if np.linalg.norm(current_node.coord-environment.goal)<0.1:
            while current_node:
                path.append(current_node.coord)
                current_node = current_node.parent_node
            path.reverse()
            return np.array(path)
      
      for k in range(numofdirs):
        next = current_node.coord + dR[:,k]
        isColliding = CheckCollision([current_node.coord,next], blocks)
        # isColliding = 0
        if( next[0] < boundary[0,0] or next[0] > boundary[0,3] or \
              next[1] < boundary[0,1] or next[1] > boundary[0,4] or \
              next[2] < boundary[0,2] or next[2] > boundary[0,5]  or \
              isColliding ):
            continue
       
        next_tuple = tuple(next)
        # if next_tuple in CLOSED or next_tuple in open_coords:
        #   continue
        
        if(next_tuple not in Graph.keys()):
          next_node = AStarNode(next_tuple,next,environment.getHeuristic(next))
          Graph[next_tuple] = next_node
        else:
          bh=4
          
        next_node = Graph[next_tuple]
        if(next_node.g > current_node.g + np.linalg.norm(dR[:,k])):
          next_node.g = current_node.g + np.linalg.norm(dR[:,k]) 
          next_node.parent_node = current_node
          next_node.parent_action = k
            
          if next_tuple in OPEN:
            OPEN.updateitem(next_node.pqkey,next_node.g + epsilon*next_node.h)
          else:
            OPEN.additem(next_node.pqkey, next_node.g + epsilon*next_node.h)
           
    
            
# def CheckCollision(seg,blocks):
#   seg_start = seg[0]
#   seg_end = seg[1]
#   for k in range(blocks.shape[0]):
#     block_min = blocks[k,0:3]
#     block_max = blocks[k,3:6]
#     block_ctr = (block_min + block_max)/2
#     block_pose = np.eye(4) # Axis aligned
#     block_pose[:3,3] = block_ctr 
#     block_size = np.array(block_max-block_min,dtype=np.float64)
    
#     dist, _, _ = line_segment_to_box(seg_start, seg_end, block_pose, block_size)
#     if(dist == 0):
#       return True
#   return False

def CheckCollision(seg,blocks):
  P1 = seg[0]
  P2 = seg[1]
  for k in range(blocks.shape[0]): 
    AABB_Min = blocks[k, 0:3]
    AABB_Max = blocks[k, 3:6]
    
    # Preliminary check
    if max(P1[0], P2[0]) < AABB_Min[0] or min(P1[0], P2[0]) > AABB_Max[0]:
        continue
    if max(P1[1], P2[1]) < AABB_Min[1] or min(P1[1], P2[1]) > AABB_Max[1]:
        continue
    if max(P1[2], P2[2]) < AABB_Min[2] or min(P1[2], P2[2]) > AABB_Max[2]:
        continue
    
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
      continue
    
    # Check for collision
    if 0 <= tEnter <= 1 or 0 <= tExit <= 1:
      return True
    
    # Check for containment
    if tEnter > 1 and tExit > 1:
      return True
    
  return False


class Environment(object):
  def __init__(self,boundary,blocks,goal):
    self.boundary = boundary
    self.blocks = blocks
    self.goal = goal 
    
  def getHeuristic(self,curr_coord):
    return np.linalg.norm(curr_coord - self.goal)

##################################################################################################
##################################################################################################



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
  def __init__(self,boundary,blocks,goal):
    self.boundary = boundary
    self.blocks = blocks
    self.goal = goal 
    
  def getHeuristic(self,curr_coord):
    return np.linalg.norm(curr_coord - self.goal)

def isAllowed(boundary,blocks,curr,next):
     
    if( next[0] < boundary[0,0] or next[0] > boundary[0,3] or \
        next[1] < boundary[0,1] or next[1] > boundary[0,4] or \
        next[2] < boundary[0,2] or next[2] > boundary[0,5] ):
          return False
      
    valid = True
    for k in range(blocks.shape[0]):
        block = blocks[k]
        isColliding = checkCollision(block,curr,next)
        
        if( next[0] >= block[0] and next[0] <= block[3] and\
            next[1] >= block[1] and next[1] <= block[4] and\
            next[2] >= block[2] and next[2] <= block[5] or isColliding):
            valid = False

    if valid:
        return True
    else:
        return False
  
    
def checkCollision(block,start,end):
    P1,P2 = start,end
    AABB_Max = block[0:3]
    AABB_Min = block[3:6]
    
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

def discretize_grid(bottom_left, top_right, resolution):
    dimensions = ((top_right - bottom_left) / resolution + 1).astype(int)
    grid = np.empty(dimensions, dtype=object)
    for x in range(dimensions[0]):
        for y in range(dimensions[1]):
            for z in range(dimensions[2]):
                coordinates = bottom_left + np.array([x, y, z]) * resolution
                grid[x, y, z] = coordinates
    return grid

class AStar(object):
  @staticmethod
  def plan(start_coord, environment, epsilon = 1):
    # Initialize the graph and open list
    Graph = {}
    OPEN = pqdict()
    path = []
    
    # current node
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
    res = 0.5
    dR = dR*res
    
    # #discretization
    # print(boundary)
    # bottom_left = boundary[0,0:3]
    # top_right = boundary[0,3:6]
    # print(top_right - bottom_left)
    # num_intervals = ((top_right - bottom_left)/ res).astype(int)
    # grid = discretize_grid(bottom_left,top_right,res)
    # goal_index =((environment.goal - bottom_left)/res).astype
    
    while OPEN:
        current_key = OPEN.popitem()[0]
        current_node = Graph[current_key]
        current_node.closed = True
        print(np.linalg.norm(current_node.coord - environment.goal),current_key)
        if(np.linalg.norm(current_node.coord - environment.goal)< res):
            while current_node:
                path.append(current_node.coord)
                current_node = current_node.parent_node
            path.reverse()
            return np.array(path)
        
        for i in range(numofdirs):
            child_coord = current_node.coord + dR[:,i]
            # print(isAllowed(boundary,blocks,current_node.coord,child_coord))
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
                
        

        
        
            

