#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt 

class Graph:
  def __init__(self, directed = False):
    self.graph = {}
    self.edges = []
    self.directed = directed
    self.path = []

  def add_vertex(self, vertex):
    if vertex not in self.graph:
      self.graph[vertex] = []

  def add_edge(self, vertex1, vertex2):
    if vertex1 not in self.graph:
      self.add_vertex(vertex1)
    if vertex2 not in self.graph:
      self.add_vertex(vertex2)
    self.graph[vertex1].append(vertex2)
    if not self.directed:
      self.graph[vertex2].append(vertex1)
    self.edges.append((vertex1, vertex2))

  def remove_edge(self, vertex1, vertex2):
    if vertex1 in self.graph:
      try:
        self.graph[vertex1].remove(vertex2)
      except ValueError:
        print("Vertex1 does not exist in graph")

    if vertex2 in self.graph and not self.directed:
      try:
        self.graph[vertex2].remove(vertex1)
      except ValueError:
        print("Vertex2 does not exist in graph")

  def remove_vertex(self, vertex):
    if vertex in self.graph:
      del self.graph[vertex]
      for vertices in self.graph.values():
        if vertex in vertices:
          vertices.remove(vertex)
      self.edges = [(v1, v2) for v1, v2 in self.edges if v1 != vertex and v2 != vertex]


  def get_adj_vertices(self,vertex):
    if vertex in self.graph:
      return self.graph[vertex]
    else:
      return []
    
  def __str__(self):
    return str(self.graph)

class RRT(Graph):
  def __init__(self, q_init, q_final, delta_q, x_bounds, y_bounds, iter=10000):
    super().__init__(directed=False)
    self.q_init = q_init
    self.q_final = q_final
    self.delta_q = delta_q
    self.iter = iter
    self.x_bounds = x_bounds
    self.y_bounds = y_bounds
    self.build_RRT(self.graph, q_init, q_final, delta_q, x_bounds, y_bounds)

  def build_RRT(self, G, q_init, q_final, delta_q, x_bounds, y_bounds):
    self.add_vertex(tuple(q_init))
    for _ in range(self.iter):
      q_rand = self.gen_random_config(x_bounds, y_bounds)
      q_near = self.nearest_vertex(q_rand)
      q_new = self.steer(q_near, q_rand)
      self.add_vertex(q_new)
      self.add_edge(q_near, q_new)
      if self.get_distance(q_new, tuple(q_final)) < delta_q:
        self.add_vertex(q_final)
        self.add_edge(q_near, q_final)
        print("Path found!")
        self.getpath()
        return True
    print("No path available.")
    return False

  def gen_random_config(self, x_bounds, y_bounds):
    rand_x = np.random.uniform(x_bounds[0], x_bounds[1])
    rand_y = np.random.uniform(y_bounds[0], y_bounds[1])
    return (rand_x, rand_y)
  
  def nearest_vertex(self, q_rand):
    dist = {}
    for vertex in self.graph:
      dist[vertex] = self.get_distance(vertex, q_rand)
      #dist[vertex] = self.get_distance(vertex, self.q_final)
    return min(dist, key=dist.get)

  def get_distance(self, vertex1, vertex2):
    distance = np.sqrt( (vertex2[0]-vertex1[0])**2 + (vertex2[1]-vertex1[1])**2 )
    return distance
  
  def steer(self, q_near, q_rand):
    #q_rand = self.q_final
    direction = np.array(q_rand) - np.array(q_near)
    length = np.linalg.norm(direction)
    direction /= length*1.0
    step = min(self.delta_q, length)
    q_new = tuple(np.array(q_near) + direction * step)
    return q_new
  
  def getpath(self):
    pass # To Do 
  
  def plot(self):
    plt.figure()
    for v1, v2 in self.edges:
      plt.plot([v1[0], v2[0]], [v1[1], v2[1]], 'g-')
    plt.plot(self.q_init[0], self.q_init[1], 'ro', label="Start")
    plt.plot(self.q_final[0], self.q_final[1], 'go', label="Goal")
    plt.xlim(self.x_bounds)
    plt.ylim(self.y_bounds)
    plt.legend()
    plt.show()

def main(args=None):
  q_init = tuple([10.0, 10.0])
  q_final = tuple([85.0, 75.0])
  x_bounds = [0.0, 100.0]
  y_bounds = [0.0, 100.0]
  delta_q = 1

  rrt = RRT(q_init, q_final, delta_q, x_bounds, y_bounds)
  rrt.plot()
    
if __name__ == "__main__":
  main()