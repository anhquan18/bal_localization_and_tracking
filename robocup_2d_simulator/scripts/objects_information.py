import rospy
import pygame
import random
import math
import geometry
import copy
import time
import os
from geometry_msgs.msg import PoseArray
from pygame.math import Vector2
from gui import GUI, FIELD_ORIGIN, FIELD_OFFSET, FIELD_SIZE

CENTER_X, CENTER_Y = FIELD_ORIGIN
GOALPOST = [(4480,-1300,0), (4480, 1300, 0)]
RED_START_POS_LIST = [(Vector2(CENTER_X - (x*100), CENTER_Y + (y*300)), y*90) for x in [2, 3] for y in [-1, 1]]
BLUE_START_POS_LIST = [(Vector2(CENTER_X + (x*100), CENTER_Y + (y*300)), y*90) for x in [2, 3] for y in [-1, 1]] 
OUTSIDE_GUI = (10000, 10000)
IMG_PATH = os.path.join(os.path.dirname(__file__), "Img/BALL.png")
RED_PATH = os.path.join(os.path.dirname(__file__), "Img/red_player.png")
BLUE_PATH = os.path.join(os.path.dirname(__file__), "Img/blue_player.png")


class Ball(object):
    def __init__(self):
        self.delta = Vector2(0,-1)
        self.pos = Vector2(CENTER_X, CENTER_Y)
        self.past_pos = Vector2(self.pos[0], self.pos[1])
        self.mem_pos_list = [self.past_pos, self.past_pos]
        self.angle = 0
        self.vel = 0

        self.diameter = 15
        img = pygame.image.load(IMG_PATH).convert()
        self.img = pygame.transform.scale(img, (self.diameter,self.diameter))
        self.img.set_colorkey(img.get_at((0,0)),pygame.RLEACCEL)

        self.rect = self.img.get_rect()
        self.rect.center = self.pos

        self.particle_poses = []
        self.past_particle_avg_poses = [self.past_pos, self.past_pos]
        self.particles_sub = rospy.Subscriber("ball_particles", PoseArray, self.particles_callback)

        # game flag
        self.last_touch = None
        self.last_scored = None

    def particles_callback(self, msg):
        self.particle_poses = msg.poses
        mean_x = sum([p.position.x for p in self.particle_poses])/len(self.particle_poses)
        mean_y = sum([p.position.y for p in self.particle_poses])/len(self.particle_poses)
        self.check_and_memorize_particle_avg_pos(Vector2(FIELD_ORIGIN[0]+mean_x, FIELD_ORIGIN[1]+mean_y))

    def check_and_memorize_pos(self, pos):
        if pos != self.mem_pos_list[-1]:
            self.mem_pos_list.append(pos)
        if len(self.mem_pos_list)>50: del self.mem_pos_list[0]

    def check_and_memorize_particle_avg_pos(self, pos):
        if pos != self.past_particle_avg_poses[-1]:
            self.past_particle_avg_poses.append(pos)
        if len(self.past_particle_avg_poses)>500: del self.past_particle_avg_poses[0]

    def set_pos(self, pos):
        self.pos = Vector2(pos[0], pos[1])
        self.rect.center = self.pos

    def get_pos(self, color):
        x = (self.pos[0] - CENTER_X) * (4500.0 / 450.0)
        y = (self.pos[1] - CENTER_Y) * (3000.0 / 300.0)
        th = 0
        if color == 'blue':
            return (-x,-y,th)
        else:
            return (x,y,th)


class Player(object):
    def __init__(self, sock, color=None, ID=None):
        self.delta = Vector2(1,0)
        self.pos  = Vector2(OUTSIDE_GUI) # initial pos can be anywhere
        self.angle = 0.0
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.angle_vel = 0.0
        self.init_angle_vel = 0.0
        self.init_pos = Vector2(1,0)
        self.diameter = 30
        filename = BLUE_PATH if color == 'blue' else RED_PATH
        img = pygame.image.load(filename).convert()
        self.img_org = pygame.transform.scale(img, (self.diameter,self.diameter))
        self.img_org.set_colorkey(img.get_at((0,0)), pygame.RLEACCEL)
        self.img = self.img_org.copy()

        self.rect = self.img.get_rect()
        self.rect.center = self.pos

        self.sock = sock
        self.ID = ID
        self.color = color
        self.old_color = color
        
        self.shoot = False
        self.offset = FIELD_ORIGIN
        self.state = 'n'

    def update(self, ball):
        if self.state == 'a':
            self.approach_ball(ball.get_pos(self.color))

    def change_color(self, color):
        filename = BLUE_PATH if color == 'blue' else RED_PATH 
        img = pygame.image.load(filename).convert()
        self.img_org = pygame.transform.scale(img, (self.diameter,self.diameter))
        self.img_org.set_colorkey(img.get_at((0,0)), pygame.RLEACCEL)
        rect = self.img_org.get_rect()
        self.img = pygame.transform.rotate(self.img_org.copy(), -self.angle)
        self.rect = self.img.get_rect(center=rect.center)
        self.pos, self.angle_vel = BLUE_START_POS_LIST[self.ID] if color == 'blue' else RED_START_POS_LIST[self.ID]
        self.rect.center = self.pos
        self.init_pos = Vector2(self.pos)
        self.init_angle_vel = copy.deepcopy(self.angle_vel)

    def set_pos(self, pos):
        self.pos = Vector2(pos[0], pos[1])
        self.rect.center = self.pos

    def get_pos(self):
        x = (self.pos[0] - CENTER_X) * (4500.0 / 450.0)
        y = (self.pos[1] - CENTER_Y) * (3000.0 / 300.0)
        th = math.radians(-self.angle)
        if self.color == 'blue':
            return (-x,-y,-th)
        else:
            th = -(math.pi+th) if th<0 else (math.pi-th)
            return (x,y,th)

    def get_ball_dis(self, ball_pos):
        self_x, self_y, self_t = self.get_pos()
        b_x, b_y, b_t = ball_pos
        return math.sqrt((self_x - b_x)**2 + (self_y - b_y)**2)

    def rotate_to_ball(self, rotate_angle):
        if 15 < abs(rotate_angle):
            self.angle_vel = 4.0*(rotate_angle/abs(rotate_angle))
        elif 0.5 <= abs(rotate_angle) <= 15.0:
            self.angle_vel = (rotate_angle/abs(rotate_angle))
        elif 0.01 <= abs(rotate_angle) < 0.5:
            self.angle_vel = 0.01*(rotate_angle/abs(rotate_angle))

    def turn_around_ball_to_target(self):
        goal_post_1_lc = geometry.coord_trans_global_to_local(self.get_pos(), GOALPOST[0]) 
        goal_post_2_lc = geometry.coord_trans_global_to_local(self.get_pos(), GOALPOST[1]) 

        min_angle = self.angle + geometry.convert_to_y_x_axis(geometry.direction_deg(goal_post_1_lc))
        max_angle = self.angle + geometry.convert_to_y_x_axis(geometry.direction_deg(goal_post_2_lc))

        if self.angle < min_angle:
            self.x_vel = 3.0
            self.y_vel = -3.0
            self.angle_vel = -5.0
        elif self.angle > max_angle:
            self.x_vel = -3.0
            self.y_vel = 3.0
            self.angle_vel = 5.0

    def ball_and_target_pos_on_straight_line(self):
        goal_post_1_lc = geometry.coord_trans_global_to_local(self.get_pos(), GOALPOST[0]) 
        goal_post_2_lc = geometry.coord_trans_global_to_local(self.get_pos(), GOALPOST[1]) 

        min_angle = self.angle + geometry.convert_to_y_x_axis(geometry.direction_deg(goal_post_1_lc))
        max_angle = self.angle + geometry.convert_to_y_x_axis(geometry.direction_deg(goal_post_2_lc))

        if min_angle < self.angle < max_angle:
            return True
        else:
            return False

    def approach_ball(self, ball_pos):
        ball_pos_lc = geometry.coord_trans_global_to_local(self.get_pos(), ball_pos)
        rotate_direction = geometry.convert_to_y_x_axis(geometry.direction_deg(ball_pos_lc))
        #print ("ball_angle:", rotate_direction)
        if abs(rotate_direction) < 0.01:
            ball_dis = self.get_ball_dis(ball_pos)
            #print ('ball_dist:', ball_dis)
            if ball_dis >= 400:
                self.x_vel = -2.5
            else:
                if self.ball_and_target_pos_on_straight_line():
                    if 150 < ball_dis < 180:
                            self.shoot = True
                    elif ball_dis <= 150:
                        self.x_vel = 5.0
                    else:
                        self.x_vel = -2.0
                else:
                    self.turn_around_ball_to_target()
        else:
            self.rotate_to_ball(geometry.normalize_deg(rotate_direction))

    def dribble(self):
        pass

    def approach(self):
        pass

    def set_selfinfo(self, info):
        self.info = info

class GameController(object):
    def __init__(self):
        self.state =  'initial'
        self.secondary_state = None

    def set_state(self, type_state, state):
        if type_state == '1st':
            self.state = state
        elif type_state == '2nd':
            self.secondary_state = state

    def get_state(self):
        return self.state

    def get_secondary_state(self):
        return self.secondary_state


class DataBase(object):
    def __init__(self):
        self.game_state = GameController()
        self.ball = Ball()
        self.player_list = []
        self.robots_random_id = []
        self.team = {'red': [], 'blue': []}

    def update_new_player(self, player):
        self.team[player.color].append(player)

    # For future strategy
    '''
    #self.player_nums = player_nums
    #self.teammates = [Player(num, BLUE) for num in range(player_nums)]
    #self.enemy = [Player(num, RED) for num in range(player_nums)]

    def get_teammate_pos(self):
        return [self.teammates[ID].pos for ID in range(self.player_nums)]
    
    def get_common_info(self):
        return [self.teammates[ID].info for ID in range(self.player_nums)]

    def send_common_info(self, ID, info):
        self.__teammates[ID].set_selfinfo(info)

    def send_kick_angle(self, angle):
        self.__ball.set_kick_angle(angle)
        # call kick physical engine function
    '''
data_base = DataBase()
