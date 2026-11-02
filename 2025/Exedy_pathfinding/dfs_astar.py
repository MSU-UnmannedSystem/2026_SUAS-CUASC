import time
import math
import heapq
# Graph has x axis pointing down and y axis is pointing right


del_time = 0.4
m, n = 15,20 # M: x-axis, N: y-axis
sq_cost = 0.009 #Battery drain per square
battery = 1 #Total battery percentage
first_cord = 0 #Holds the first coordinate
last_cord = 0 #Holds the value of the last coordinate visited

LIMIT = 0.13 #Saftly limit of battery
max_drain =  (sq_cost * (m+n)) # drain to travel to farthest distance

#Creates field of size m x n with False as the value to represent unvisited
field = [[False for _ in range(n)] for _ in range(m)]

'''
Test Field 1: top row and right column have been visited already
'''
temp_field = [[False for _ in range(n)] for _ in range(m)]
temp_set = set()
for i in range(n):
    temp_field[0][i] = True
    temp_set.add((0,i))
for i in range(m):
    temp_field[i][n-1] = True 
    temp_set.add(((i,n-1)))

'''
Test Field 2: More visited nodes to see how DFS behaves
'''
temp2_field = [[False for _ in range(n)] for _ in range(m)]
temp2_set = set()
for i in range(n):
    temp2_field[0][i] = True
    temp2_set.add((0,i))
for i in range(m):
    temp2_field[i][n-1] = True 
    temp2_set.add(((i,n-1)))

for i in range(1,n):
    temp2_field[1][i] = True
    temp2_set.add((1,i))
for i in range(1,m-1):
    temp2_field[i][m-2] = True
    temp2_set.add((i,m-2))


'''
Creates dictionary that holds the nodes and connections
Keys: Nodes
Values: Nodes that are connected to each key node
Parameters: The dictionary you want to fill
'''
def create_dic():
    dic = {}
    for x in range(m):
        for y in range(n):
            if(x == 0):
                if(y == 0):
                    dic[(x,y)] = [(x,y+1), (x+1,y)]
                elif(y == n-1):
                    dic[(x,y)] = [(x+1,y), (x,y-1)]
                else:
                    dic[(x,y)] = [(x,y+1), (x+1,y), (x,y-1)]
            elif(x != m-1):
                if(y == 0):
                    dic[(x,y)] = [(x,y+1), (x+1,y), (x-1,y)]
                elif(y == n-1):
                    dic[(x,y)] = [(x+1,y), (x,y-1), (x-1,y)]
                else:
                    dic[(x,y)] = [(x,y+1), (x+1,y), (x,y-1), (x-1,y)]
            else:
                if(y == 0):
                    dic[(x,y)] = [(x,y+1), (x-1,y)]
                elif(y == n-1):
                    dic[(x,y)] = [(x,y-1), (x-1,y)]
                else:
                    dic[(x,y)] = [(x,y+1), (x,y-1), (x-1,y)]
    return dic


'''
Prints field for visualization
Parameters: field you want to visualize
'''
def print_field(field):
    print()
    for i,x in enumerate(field):
        for j,y in enumerate(x):
            if y:
                print('[X]',end=' ')
            else:
                print('[O]',end=' ')
        print()
    print()
    time.sleep(del_time) #Delay to see field change


#Parameters: Graph to be used, starting node, set of visited nodes, field to print
def dfs(g, node, visited, field):
    global battery, last_cord
    #If the current node has been visited just return the current set
    if node in visited:
        return visited
    
    #If the battery is below safty limit, end DFS by returning False
    if battery <= max_drain:
        return visited
    
    battery -= sq_cost #Drains battery per node visited
    #print('Current neighbor nodes: {}'.format(g[node]))
    show_bat()
    last_cord = node
    visited.add(node) #Adds node to set
    field[node[0]][node[1]] = True #Sets the node in the printed field to True so it can be visialized as visited
    print_field(field) #Print the current field after visiting the node

    #Recursivly calls the function to search all connected nodes 
    for neighbor in g[node]:
        if neighbor not in visited:
            result = dfs(g, neighbor, visited, field)
            
            # If the limit is reached in a recursive call, return immediately
            if battery <= max_drain:
                return result

    return visited

def show_bat():
    global battery
    print('Battery: {}'.format(battery))

def heuristic(a, b):
    """Calculate Manhattan distance as the heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    """
    Perform A* search on a grid.
    
    Parameters:
    - grid: 2D list representing the grid (True = obstacle, False = free)
    - start: Tuple (x, y) for the starting position
    - goal: Tuple (x, y) for the goal position

    Returns:
    - A list of nodes representing the path from start to goal.
    """
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))  # Priority queue with (priority, node)
    came_from = {}  # Tracks the optimal path
    g_score = {start: 0}  # Cost from start to each node
    f_score = {start: heuristic(start, goal)}  # Estimated total cost

    while open_set:
        _, current = heapq.heappop(open_set)

        # If goal is reached, reconstruct the path
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Reverse the path to go from start to goal

        x, y = current
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:  # Neighbors: right, down, left, up
            neighbor = (x + dx, y + dy)

            # Check if neighbor is within bounds and not an obstacle
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and not grid[neighbor[0]][neighbor[1]]:
                tentative_g_score = g_score[current] + 1

                # If this path is better, record it
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                    # Add neighbor to the open set
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    # If we exhaust the search and don't find the goal, return an empty list
    return []


def main():
    global battery

    graph = create_dic()

    found = False
    for i,x in enumerate(field):
        for j,y in enumerate(x):
            if((y == False) and (found == False)):
                first_cord = (i,j)
                found = True

    '''print(LIMIT)
    time.sleep(5)'''

    while found:

        g = update_graph(graph, field)
        
        dfs(g, first_cord, set(), field)

        field[first_cord[0]][first_cord[1]] = False
        field[last_cord[0]][last_cord[1]] = False

        path = astar(field, last_cord, first_cord)
        for tup in path:
            field[tup[0]][tup[1]] = True
            battery -= sq_cost
            show_bat()
            print_field(field)
        print(path)

        print('Charging...')
        time.sleep(1)
        battery = 1

        found = False
        for i,x in enumerate(field):
            for j,y in enumerate(x):
                if((y == False) and (found == False)):
                    first_cord = (i,j)
                    found = True
        battery -= sq_cost*(first_cord[0] + first_cord[1])
    
    print('Field have been fully covered!')

    
def update_graph(graph, field):
    """
    Update the graph by removing nodes marked as visited in the field and reconnecting their neighbors.
    Neighbors in the updated graph are listed in the order: right, down, left, and up.
    """
    updated_graph = {}  # New graph to store the updated connections

    for node, neighbors in graph.items():
        x, y = node
        if field[x][y]:  # Node has been visited
            continue

        # Filter neighbors to exclude visited nodes and maintain the right, down, left, up order
        new_neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:  # right, down, left, up
            neighbor = (x + dx, y + dy)
            if neighbor in neighbors and not field[neighbor[0]][neighbor[1]]:
                new_neighbors.append(neighbor)

        # Connect current node to unvisited neighbors of visited nodes
        for neighbor in neighbors:
            if field[neighbor[0]][neighbor[1]]:  # If a neighbor is visited
                for n_neighbor in graph[neighbor]:
                    if not field[n_neighbor[0]][n_neighbor[1]] and n_neighbor != node:
                        if n_neighbor not in new_neighbors:
                            new_neighbors.append(n_neighbor)

        # Store the updated neighbors in the graph
        updated_graph[node] = new_neighbors

    return updated_graph





if __name__=="__main__":
    main()