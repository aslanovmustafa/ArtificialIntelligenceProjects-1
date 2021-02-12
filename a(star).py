import numpy as np
import time

def dijkstraSP ():

    infi = 1000000000
    edges = []
    

    def pathway(p):
      if (p == -1):
        return
      pathway(origin[p])
      print(p, end=" ")
  
    with open("edges.txt",'r') as filename:
         for line in filename:
             if not line.startswith("#"):
               edges.append(line.split(','))

    with open("sd.txt",'r') as filename:
         for line in filename:
             if not line.startswith("#"):
               if line.startswith('S'):
                 strt = line.split(',')
                 s = int(strt[1])
               elif line.startswith('D'):
                 finish = line.split(',')
                 d = int(finish[1])

    max_edge, m = 0, len(edges)

    for i in range(0, m):
        max_edge = int(edges[1][1])
        if (max_edge<=int(edges[i][1])):
            max_edge = int(edges[i][1])+1

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
        pathway(d)
if __name__ == '__main__':
    start = time.time()
    dijkstraSP()
    end = time.time()
    print("\nTime spent:", "{:.3f}".format(end-start), "seconds")