# Basic searching algorithms

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, g, h , cost, parent):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = g         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = cost      # total cost (depend on the algorithm)
        self.parent = parent    # previous node

# Used to initialize nodes. Common for both A* and dijkstra.
def initNodes(grid, algo, goal):
  nodeList = []
  if algo == "dijkstra":
    for i in range(len(grid)):
      cols = []
      for j in range(len(grid[0])):
        cols.append(Node(i, j, grid[i][j], None, 0 , float('inf'), None))
      nodeList.append(cols)
  if algo == "A*":
    for i in range(len(grid)):
      cols = []
      for j in range(len(grid[0])):
        heuristic = abs(i - goal[0]) + abs(j - goal[1])
        cols.append(Node(i, j, grid[i][j], None, heuristic , float('inf'), None))
      nodeList.append(cols)
  return nodeList  

# Returns possible directions for a node to explore. Common for both A* and dijkstra.   
def getDirections():
  right = [0,1]
  down = [1,0]
  left = [0,-1]
  up = [-1,0]
  neighDirect = [right,down,left,up]
  return neighDirect  

# Returns feasible neighbours for a node. Common for both A* and dijkstra.
def initNeighbours(currNode,gridRows, gridCols):
  neighbours = []
  neighDirect =  getDirections()
  for i in range(4):
    potNeigh = list(map(sum, zip(currNode,neighDirect[i]))) 
    res = potNeigh[0] >= 0 and potNeigh[0] < gridRows and potNeigh[1] >= 0 and potNeigh[1] < gridCols
    if res == True:
      neighbours.append(potNeigh)
  return neighbours

# Returns Path from start to goal. Common for both A* and dijkstra.
def returnPath(start,goal,nodeList):
  path = []
  path.append(goal)
  while path[0] != start:
    currNode = path[0]
    path.insert(0, nodeList[currNode[0]][currNode[1]].parent)
  return path

# Searches the graph to find the goal. At each iteration the node with minimum cost is selected to explore. Terminates when the minimum node selected is the goal node. Common for both A* and dijkstra. In dijsktra the heuristic is 0 but in A* it is the manhattan distance to the goal node.
def searchGoal(start, goal, nodeList, gridRows, gridCols):
  steps = 0
  found = False
  nodeList[start[0]][start[1]].g = 0
  openQ = {}
  openQ[start[0], start[1]] = 0
  closedList = []
  while(len(openQ)):
    if nodeList[start[0]][start[1]].is_obs == 1 or nodeList[goal[0]][goal[1]].is_obs == 1 :
      found = False
      break
    currNode = min(openQ, key = openQ.get)
    del openQ[currNode[0], currNode[1]]
    closedList.append([currNode[0], currNode[1]])
    steps = steps + 1
    if currNode[0] == goal[0] and currNode[1] == goal[1]:
      found = True
      break
    neighbours = initNeighbours(currNode, gridRows, gridCols)
    for i in range(len(neighbours)):
      currNeigh = neighbours[i]
      if nodeList[currNeigh[0]][currNeigh[1]].is_obs == 0 and currNeigh not in closedList:
        currCost = nodeList[currNode[0]][currNode[1]].g + 1 + nodeList[currNeigh[0]][currNeigh[1]].h 
        if currCost < nodeList[currNeigh[0]][currNeigh[1]].cost:
          nodeList[currNeigh[0]][currNeigh[1]].cost = currCost
          nodeList[currNeigh[0]][currNeigh[1]].parent = [currNode[0], currNode[1]]
          nodeList[currNeigh[0]][currNeigh[1]].g = nodeList[currNode[0]][currNode[1]].g + 1
          openQ[currNeigh[0], currNeigh[1]] = currCost
  return found, steps  

# Main function which executes dijkstra.
def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    
    nodeList = initNodes(grid, "dijkstra", goal)
    found, steps = searchGoal(start, goal, nodeList, len(grid), len(grid[0]))
   
    if found:
        path = returnPath(start,goal,nodeList) 
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps

# Main function which executes A*.
def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    
    nodeList = initNodes(grid, "A*", goal)
    found, steps = searchGoal(start, goal, nodeList, len(grid), len(grid[0]))
    
    if found:
        path = returnPath(start,goal,nodeList) 
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps



# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
