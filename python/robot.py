import pybullet as p

import numpy as np


class Robot():
    """ 
    The class is the interface to a single robot
    """

    def __init__(self, init_pos, robot_id, dt):
        self.id = robot_id
        self.dt = dt
        self.pybullet_id = p.loadSDF("../models/robot.sdf")[0]
        self.joint_ids = list(range(p.getNumJoints(self.pybullet_id)))
        self.initial_position = init_pos
        self.reset()

        # No friction between body and surface.
        p.changeDynamics(self.pybullet_id, -1, lateralFriction=5., rollingFriction=0.)

        # Friction between joint links and surface.
        for i in range(p.getNumJoints(self.pybullet_id)):
            p.changeDynamics(self.pybullet_id, i, lateralFriction=5., rollingFriction=0.)

        self.messages_received = []
        self.messages_to_send = []
        self.neighbors = []
        self.state = np.zeros((6, 3))
        self.p = [[0.5, -1], [2.5, 1], [1.5, -1], [2.5, 0], [2.5, -1], [2.5, 2]]

        # self.p2 = [[2.5, -0.5], [2.5, 2.5], [2.5, -1.5], [2.5, 1.5], [2.5, 0.5], [2.5, 3.5]]

    def update_p(self, p):
        self.p = p

    def reset(self):
        """
        Moves the robot back to its initial position
        """
        p.resetBasePositionAndOrientation(self.pybullet_id, self.initial_position, (0., 0., 0., 1.))

    def set_wheel_velocity(self, vel):
        """ 
        Sets the wheel velocity,expects an array containing two numbers (left and right wheel vel) 
        """
        assert len(vel) == 2, "Expect velocity to be array of size two"
        p.setJointMotorControlArray(self.pybullet_id, self.joint_ids, p.VELOCITY_CONTROL,
                                    targetVelocities=vel)

    def get_pos_and_orientation(self):
        """
        Returns the position and orientation (as Yaw angle) of the robot.
        """
        pos, rot = p.getBasePositionAndOrientation(self.pybullet_id)
        euler = p.getEulerFromQuaternion(rot)
        return np.array(pos), euler[2]

    def get_messages(self):
        """
        returns a list of received messages, each element of the list is a tuple (a,b)
        where a= id of the sending robot and b= message (can be any object, list, etc chosen by user)
        Note that the message will only be received if the robot is a neighbor (i.e. is close enough)
        """
        return self.messages_received

    def send_message(self, robot_id, message):
        """
        sends a message to robot with id number robot_id, the message can be any object, list, etc
        """
        self.messages_to_send.append([robot_id, message])

    def get_neighbors(self):
        """
        returns a list of neighbors (i.e. robots within 2m distance) to which messages can be sent
        """
        return self.neighbors

    def compute_controller(self, var):
        """ 
        function that will be called each control cycle which implements the control law
        TO BE MODIFIED
        
        we expect this function to read sensors (built-in functions from the class)
        and at the end to call set_wheel_velocity to set the appropriate velocity of the robots
        """

        # here we implement an example for a consensus algorithm
        neig = self.get_neighbors()
        messages = self.get_messages()
        pos, rot = self.get_pos_and_orientation()

        # send message of positions to all neighbors indicating our position
        for n in neig:
            self.send_message(n, pos)
        # check if we received the position of our neighbors and compute desired change in position
        # as a function of the neighbors (message is composed of [neighbors id, position])
        # print(var.time)

        dx = 0
        dy = 0

        if (var.time > 10) & (var.time < 20):
            self.update_p([[2.5, -0.5], [2.5, 2.5], [2.5, -1.5], [2.5, 1.5], [2.5, 0.5], [2.5, 3.5]])

        elif (var.time > 20) & (var.time < 29):
            self.update_p([[2.5, 3.5], [2.5, 6.5], [2.5, 2.5], [2.5, 5.5], [2.5, 4.5], [2.5, 7.5]])
            # [[0.5, -1], [2.5, 1], [1.5, -1], [2.5, 0], [2.5, -1], [2.5, 2]]

        elif (var.time > 29) & (var.time < 36):
            self.update_p([[0.5, 5.5], [1.5, 5.5], [2.5, 5.5], [3.5, 5.5], [4.5, 5.5], [5.5, 5.5]])

        elif (var.time > 36) & (var.time < 46):
            self.update_p([[2.5, 3.5], [2.5, 6.5], [2.5, 2.5], [2.5, 5.5], [2.5, 4.5], [2.5, 7.5]])

        elif (var.time > 46) & (var.time < 56):
            self.update_p([[0.5, 5.5], [1.5, 5.5], [2.5, 5.5], [3.5, 5.5], [4.5, 5.5], [5.5, 5.5]])

        elif (var.time > 56) & (var.time < 66):
            self.update_p([[-3.5, 5.5], [-2.5, 5.5], [-1.5, 5.5], [-0.5, 5.5], [0.5, 5.5], [1.5, 5.5]])

        elif (var.time > 66) & (var.time < 77):
            self.update_p([[-7.5, 5.5], [-6.5, 5.5], [-5.5, 5.5], [-4.5, 5.5], [-3.5, 5.5], [-2.5, 5.5]])

        elif (var.time > 77) & (var.time < 87):
            self.update_p([[-5, 4.5], [-5.5, 5], [-5, 6.5], [-6, 5.5], [-5, 5.5], [-5.5, 6]])

        elif (var.time > 87) & (var.time < 97):
            self.update_p([[-3.5, 9.5], [-4.5, 10], [-3.5, 11.5], [-5.5, 10.5], [-3.5, 10.5], [-4.5, 11]])

        print(self.p)
        if messages:
            for m in messages:
                dx += 5 * (m[1][0] - pos[0] - self.p[m[0]][0] + self.p[self.id][0])
                dy += 5 * (m[1][1] - pos[1] - self.p[m[0]][1] + self.p[self.id][1])

            x_comp = np.minimum((self.p[self.id][0] - pos[0]), 1)
            y_comp = np.minimum((self.p[self.id][1] - pos[1]), 1)

            dx += 10 * x_comp
            dy += 10 * y_comp

            vel_norm = np.linalg.norm([dx, dy])
            if vel_norm < 0.01:
                vel_norm = 0.01
            des_theta = np.arctan2(dy / vel_norm, dx / vel_norm)
            right_wheel = np.sin(des_theta - rot) * vel_norm + np.cos(des_theta - rot) * vel_norm
            left_wheel = -np.sin(des_theta - rot) * vel_norm + np.cos(des_theta - rot) * vel_norm
            self.set_wheel_velocity([left_wheel, right_wheel])
