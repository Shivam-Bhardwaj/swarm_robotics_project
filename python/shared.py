import numpy as np


class Shared:

    def __init__(self):
        self.a = 8
        self.n = 6
        self.Kf = 100
        self.Kt = 5
        self.time = 0

    def update_time(self, dt):
        self.time += dt

    def update_vel(self, vel):
        self.vel = vel

    def getLaplacian(self, E, n_vertex):
        L = np.zeros([n_vertex, n_vertex])  # our Laplacian matrix
        Delta = np.zeros([n_vertex, n_vertex])  # this is the degree matrix
        A = np.zeros([n_vertex, n_vertex])  # this is the adjacency matrix
        for e in E:  # for each edge in E
            # add degrees
            Delta[e[1], e[1]] += 1
            # add the input in the adjacency matrix
            A[e[1], e[0]] = 1
            # symmetric connection as we have undirected graphs
            Delta[e[0], e[0]] += 1
            A[e[0], e[1]] = 1
        L = Delta - A
        return L

    # get incidence matrix for directed graph E (list of edges)
    def getIncidenceMatrix(self, E, n_vertex):
        n_e = len(E)
        D = np.zeros([n_vertex, n_e])
        for e in range(n_e):
            # add the directed connection
            D[E[e][0], e] = -1
            D[E[e][1], e] = 1
        return D
