import numpy as np
import time
import heapq
import math

def fileInput(): #all the necessary input will be here
  edges = []        #array for the edges from the parsed list
  with open("edges.txt",'r') as filename:
    for line in filename:
        if not line.startswith("#"):
          edges.append(line.split(','))

  with open("sd.txt",'r') as filename:
    for line in filename:
      if not line.startswith("#"):
        if line.startswith('S'):
          strt = line.split(',')
          s = int(strt[1])        #source
        elif line.startswith('D'):
          finish = line.split(',')
          d = int(finish[1])      #destination

  max_edge, m = 0, len(edges)

  for i in range(0, m):
    max_edge = int(edges[1][1])
    if (max_edge<=int(edges[i][1])):
        max_edge = int(edges[i][1])+1 #essentially finds the number of edges in total

  grid = []
  for i in range(max_edge):
    grid.append(0)
  with open("vert.txt",'r') as filename:
    for line in filename.readlines():
      if not line.startswith("#"):
        vert,sID = line.split(",")
        vert,sID = int(vert),int(sID)
        grid[vert] = [sID/10,sID%10]
  
  return edges, max_edge, m, s, d, grid

edges, max_edge, m, s, d, grid = fileInput()

def dijkstraSP (): #function to calculate the djikstra algorithm to find shortest path

    infi = 1000000000 #random big number
     
    def pathway_dijkstra(p): #this function will be used to output the list of the vertices that create a pathway of the shortest path
      if (p == -1):
        return
      pathway_dijkstra(origin[p])
      print(p, end=" ")

    used = np.zeros(max_edge)
    grph = np.full((max_edge,max_edge), infi)

    for i in range(0,m):
      a = edges[i]
      x, y, distance = int(a[0]), int(a[1]), int(a[2].replace('\n', ''))
      grph[x][y] = grph[y][x] = distance

    distway = np.full(max_edge, infi)
    distway[s]=0
    origin = np.full(max_edge, -1)

    for i in range(0, max_edge-1):
      minimum, w = infi, -1

      for j in range (0, max_edge):
        if (used[j] == 0 and distway[j] < minimum):
          minimum = distway[j]
          w = j
      if (w==-1):
        break
      for j in range(0, max_edge):
        if (used[j]==0 and grph[w][j]!=infi):
          if (distway[i] + grph[i][j] < distway[j]):
            distway[j] = distway[i]+grph[i][j]
            origin[j] = i
      used[w] = 1

    if (distway[d]==infi):
      print("Path does not exist :(")
    else:
        print("Shortest path distance is:", distway[d])
        print("The pathway itself is:", end=" ")
        pathway_dijkstra(d)


def a_star(): #function to calculate the a* algorithm to find shortest path

  def Manhattan():
    ds=[] #array of distances
    for i in range(max_edge):
      x = (abs(grid[i][0] - grid[d][0]))
      y = (abs(grid[i][1] - grid[d][1]))
      distance = (x+y)*max_edge
      ds.append(distance)
    return ds
 
  heuristic = Manhattan()
  
  def create_grid(): #Function to store nodes and heuristics
    e_grid=[]
    for i in range(max_edge):
      e_grid.append([])
    for line in edges:
      vertexFROM,vertexTO,dest = int(line[0]),int(line[1]),int(line[2])
      e_grid[vertexFROM].append([dest+heuristic[vertexTO],vertexTO,dest])
      e_grid[vertexTO].append([dest+heuristic[vertexFROM],vertexFROM,dest])
    return e_grid

  created_grid = create_grid()

  def pathway_a_star(path,parent,D,S): #printhing the path
    path.append(parent[D])
    if parent[D] != 0:
      pathway_a_star(path,parent,parent[S],0)
      #descending for loop.
    for i in range(len(path) -1, -1,-1):
      print(path[i],end=" ")

  cost = s
  parent = []
  status = []
  visited = []
  path = [d]
  for i in range(max_edge):
    visited.append(0)
    parent.append(0)
  heapq.heappush(status, (cost, [s,cost,s]))
  while len(status) != 0:
    live_score,[live_node,live_cost,p] = heapq.heappop(status)
    if visited[live_node] == 0:
      visited[live_node] = 1
      parent[p] = p
      if live_node == d:
        print("Shortest path distance is:", int(live_score))
        print("The pathway itself is:", end=" ")
        pathway_a_star(path,parent,d,s)
      else:
        for x in created_grid[live_node]:
          heapq.heappush(status, (x[0] + live_cost, [x[1],x[2]+live_cost,live_node]))


if __name__ == '__main__':
    print("First Dijkstra Algorithm will be run...")
    start = time.time()
    dijkstraSP()
    end = time.time()
    dijkstra_time = end-start
    print("\nTime spent for Dijkstra is", "{:.3f}".format(dijkstra_time), "seconds")
    print("__________________________________________________________________________")
    print("Now A* Algorithm will be run...")
    start = time.time()
    a_star()
    end = time.time()
    astar_time = end-start
    print("\nTime spent for A* is", "{:.3f}".format(astar_time), "seconds")
    print("__________________________________________________________________________")
    print("A* was", "{:.2f}".format(dijkstra_time/astar_time), "times faster than Dijkstra!")