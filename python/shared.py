# Class for shared variables

class Shared:

    def __init__(self):
        self.kf = 10
        self.time = 0

    def update_time(self, dt):
        self.time += dt

    def update_kf(self, kf):
        self.kf = kf
