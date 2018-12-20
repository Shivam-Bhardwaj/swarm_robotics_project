import numpy as np


class Shared:

    def __init__(self):
        self.a = 8
        self.n = 6
        self.Kf = 100
        self.Kt = 5
        self.time = 0
        # self.E = np.array([[0, 1], [2, 3], [4, 5], [0, 2], [2, 4], [4, 1], [5, 3], [3, 1], [0, 5]])
        self.E = np.array([[0, 1], [2, 3], [4, 5], [0, 2], [2, 4], [4, 1]])

        self.p = np.array([[0.5, -1], [2.5, 1], [1.5, -1], [2.5, 0], [2.5, -1], [2.5, 2]])
        # print(self.p[0][2])
        # print(self.p[0, 1])
        # self.z_des = np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]])
        # self.z_des = np.array([[0.5, -0.5], [0.5, 0.5], [1.5, -0.5], [1.5, 0.5], [2.5, -0.5], [2.5, 0.5]])
        # print(self.z_des.dtype)
        self.vel_des = np.array([[0., 0.], [0., 0.], [0., 0.], [0., 0.], [0., 0.], [0., 0.]])
        self.L = self.getLaplacian(self.E, self.n)
        self.D = self.getIncidenceMatrix(self.E, self.n)
        self.Df = 2 * np.sqrt(self.Kf)  # critical damping for formation control
        self.Dt = 2 * np.sqrt(self.Kt)  # critical damping for target control
        self.p_current = np.array([[0.5, -0.5], [0.5, 0.5], [1.5, -0.5], [1.5, 0.5], [2.5, -0.5], [2.5, 0.5]])
        self.p_prev = self.p_current
        self.z_des = np.transpose(self.D) @ self.p

        self.position = np.array([[0.5, -0.5], [0.5, 0.5], [1.5, -0.5], [1.5, 0.5], [2.5, -0.5], [2.5, 0.5]])
        self.vel = np.array([[0.01, 0.01], [0.01, 0.01], [0.01, 0.01], [0.01, 0.01], [0.01, 0.01], [0.01, 0.01]])

        self.v_current = np.array([[0.1, 0.1], [0.1, 0.1], [0.1, 0.1], [0.1, 0.1], [0.1, 0.1], [0.1, 0.1]])
        self.v_prev = self.v_current
        self.Fx = np.zeros(6)
        self.Fy = np.zeros(6)

    def update_time(self,dt):
        self.time += dt

    def update_a(self):
        self.a += 1

    def update_p_current(self, p_current, x, y):
        self.p_current[x, y] = p_current

    def update_p_prev(self, p_prev, x, y):
        self.p_prev[x, y] = p_prev

    def update_p_prev_w(self, p_curr):
        self.p_prev = p_curr

    def update_v_current(self, v_current):
        self.v_current[:, 0] = v_current

    def update_v_prev_x(self, v_prev):
        self.v_prev[:, 0] = v_prev

    def update_v_prev_y(self, v_prev):
        self.v_prev[:, 1] = v_prev

    def update_pos(self, pos):
        # print(pos)
        self.position = pos

    def update_vel(self, vel):
        self.vel = vel

    def update_Fx(self):
        self.Fx = (self.Kf * (self.D.dot(self.z_des[:, 0]) - self.L.dot(self.position[:, 0])) + self.Df * (
                self.D.dot(self.vel_des[:, 0]) - self.L.dot(self.vel[:, 0])))
        # print(self.Fx)

    def update_Fy(self):
        # print(self.position)
        self.Fy = (self.Kf * (self.D.dot(self.z_des[:, 1]) - self.L.dot(self.position[:, 1])) + self.Df * (
                self.D.dot(self.vel_des[:, 1]) - self.L.dot(self.vel[:, 1])))

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
