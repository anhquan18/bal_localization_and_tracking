import geometry
import math

class SoccerPlayer(object):
    def __init__(self):
        self.ball_dist = 0.0
        self.ball_pos = (0.0, 0.0, 0.0)
        self.state = 'n'

    def reset_velocity(self):
        self.shoot = 0
        self.ang_vel = 0.0
        self.x_vel = 0.0
        self.y_vel = 0.0

    def return_data(self):
        data =  str(self.shoot) + ','
        data += str(self.ang_vel) + ',' 
        data += str(self.x_vel) + ','
        data += str(self.y_vel) + ','
        data += str(self.state)
        return data

    def handle_data(self, data):
        data = data.split(',')
        if data[0] == '': return
        
        if len(data) == 4:
            self.pos = (float(data[0]), float(data[1]), math.radians(int(data[2])))
            self.ball_dist = float(data[3])
