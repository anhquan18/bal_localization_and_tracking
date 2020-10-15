import rospy
from geometry_msgs.msg import Pose, PoseArray
import pygame
from pygame.math import Vector2
import math
import geometry
from gui import FIELD_ORIGIN, FIELD_OFFSET, FIELD_SIZE

CENTER = FIELD_ORIGIN
FIELD_SIZE = (FIELD_OFFSET[0], FIELD_SIZE[0]+FIELD_OFFSET[0], FIELD_OFFSET[1], FIELD_SIZE[1]+FIELD_OFFSET[1])
GOALPOST_Y = (220, 480)

def detect_collision(obj1, obj2):
    obj1_radius = obj1.diameter / 2.0
    x1, y1 = obj1.rect.centerx, obj1.rect.centery

    obj2_radius = obj2.diameter / 2.0
    x2, y2 = obj2.rect.centerx, obj2.rect.centery

    max_dist = obj1_radius + obj2_radius - 5.0
    dist = math.hypot(x1-x2, y1- y2)

    return max_dist > dist

def coord_trans_global_to_local(pos3d, pos2d):
    x1, y1, deg1 = pos3d
    x2, y2 = pos2d
    local_pos = Vector2(x2-x1, y2-y1)
    local_pos.rotate_ip(-deg1)
    
    return local_pos

class Engine(object):
    def __init__(self, ball, data_base):
        self.shoot = False

        self.ball = ball
        self.data_base = data_base
        self.player_list = self.data_base.player_list
        self.pub = rospy.Publisher("robot_ball_information", PoseArray, queue_size=5)

    def update(self):
        #self.event()
        self.update_ball_pos()
        self.update_player_pos()
        self.check_ball_collision()
        self.check_ball_shoot()
        self.check_player_collision()

    def publish_information(self):
        information = PoseArray()

        for player in self.data_base.team['blue']:
            player_pose = Pose()
            ball_pose_gl = Pose()
            ball_pose_lc = Pose()
            ball_velo = Pose()

            player_pose.position.x, player_pose.position.y, player_pose.position.z = player.get_pos()
            ball_pose_gl.position.x, ball_pose_gl.position.y, _ = self.ball.get_pos(player.color)
            ball_velo.position.x = (self.ball.pos[0] - self.ball.past_pos[0])*10.0 
            ball_velo.position.y = (self.ball.pos[1] - self.ball.past_pos[1])*10.0

            ball_lc = geometry.coord_trans_global_to_local(player.get_pos(), self.ball.get_pos(player.color))
            ball_pose_lc.position.x, ball_pose_lc.position.y, _ = ball_lc

            information.poses = [player_pose, ball_pose_gl, ball_pose_lc, ball_velo]
            if ball_velo.position.x !=0 or ball_velo.position.y !=0:
                #print("engine:", ball_velo.position.x, ball_velo.position.y)
                pass
            self.pub.publish(information)

    # DEBUG ONLY
    def event(self):
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.player_list[0].shoot = True

        pressed_key = pygame.key.get_pressed()
        self.player_list[0].angle_vel = 0
        self.player_list[0].x_vel = 0
        self.player_list[0].y_vel = 0
        if pressed_key[pygame.K_a]:
            self.player_list[0].angle_vel = -4.0
        if pressed_key[pygame.K_s]:
            self.player_list[0].angle_vel = 4.0
        if pressed_key[pygame.K_UP]:
            self.player_list[0].x_vel = -2.0
        if pressed_key[pygame.K_DOWN]:
            self.player_list[0].x_vel = 2.0
        if pressed_key[pygame.K_LEFT]:
            self.player_list[0].y_vel = -2.0
        if pressed_key[pygame.K_RIGHT]:
            self.player_list[0].y_vel = 2.0

    def update_ball_pos(self):
        self.ball.vel -= 1
        self.ball.past_pos = Vector2(self.ball.pos[0], self.ball.pos[1])
        self.ball.check_and_memorize_pos(self.ball.past_pos)

        if math.fabs(self.ball.vel) <= 1:
            self.ball.vel = 0

        delta = self.ball.delta.rotate(self.ball.angle)
        self.ball.pos += delta * self.ball.vel
        self.publish_information()


        if not FIELD_SIZE[0] < self.ball.pos[0] < FIELD_SIZE[1]:
            if self.ball.pos[0] > FIELD_SIZE[1]:
                x = CENTER[0] if self.ball.last_touch == 'red' else FIELD_SIZE[1] - 100
            elif self.ball.pos[0] < FIELD_SIZE[0]:
                x = CENTER[0] if self.ball.last_touch == 'blue' else FIELD_SIZE[0] + 100
            if self.ball.pos[1] < GOALPOST_Y[0]:
                y = FIELD_SIZE[2] + 50
            elif self.ball.pos[1] > GOALPOST_Y[1]:
                y = FIELD_SIZE[3] - 50
            else:
                x, y = CENTER
                self.ball.last_scored = 'blue' if self.ball.pos[0] < x else 'red'
            self.ball.vel = 0
            self.ball.set_pos((x,y))

        if not FIELD_SIZE[2] < self.ball.pos[1] < FIELD_SIZE[3]:
            x = self.ball.pos[0] + 100 if self.ball.last_touch == 'blue' else self.ball.pos[0] - 100
            y = FIELD_SIZE[3] - 50 if self.ball.pos[1] > FIELD_SIZE[3] else FIELD_SIZE[2] + 50
            if x < FIELD_SIZE[0] + 100: x = FIELD_SIZE[0] + 100
            elif x > FIELD_SIZE[1] - 100 : x = FIELD_SIZE[1] - 100
            self.ball.vel = 0
            self.ball.set_pos((x,y))

        self.ball.rect.center = self.ball.pos

    def update_player_pos(self):
        for player in self.player_list:
            if not FIELD_SIZE[0] <= player.pos[0] <= FIELD_SIZE[1] or not FIELD_SIZE[2] <= player.pos[1] <= FIELD_SIZE[3]:
                player.pos = Vector2(player.init_pos)
                player.angle_vel = player.init_angle_vel
                player.angle = 0.0
                player.delta = Vector2(1,0)

            if player.angle_vel != 0:
                player.delta.rotate_ip(player.angle_vel)
                player.angle += player.angle_vel
                player.angle = geometry.normalize_deg(player.angle)
                player.img = pygame.transform.rotate(player.img_org, -player.angle)
                player.rect = player.img.get_rect(center=player.rect.center)
                player.angle_vel = 0.0

            player.pos += player.delta * player.x_vel
            player.pos += Vector2(player.delta[1], -player.delta[0]) * player.y_vel

            player.rect.center = player.pos
            

    def check_ball_collision(self):
        ball = self.ball
        for player in self.player_list:
            if detect_collision(ball, player):
                dx = ball.rect.centerx - player.rect.centerx
                dy = ball.rect.centery - player.rect.centery
                tan = math.atan2(dy, dx)
                radians = tan + math.pi/2
                ball.vel = 3
                ball.angle = math.degrees(radians)
                ball.last_touch = player.color

    def check_player_collision(self):
        ball = self.ball
        for player1 in self.player_list:
            for player2 in self.player_list:
                if player1 == player2:
                    continue
                if detect_collision(player1, player2):
                    dx = player1.rect.centerx - player2.rect.centerx
                    dy = player1.rect.centery - player2.rect.centery
                    tan = math.atan2(dy, dx)
                    radians = tan + math.pi/2
                    delta = Vector2(0, -1).rotate(math.degrees(radians))
                    player1.pos += delta * 1.5 

    def check_ball_shoot(self):
        ball = self.ball
        for player in self.player_list:
            if player.shoot:
                pose = [player.pos[0], player.pos[1], player.angle - 180]
                ball_local_pos = coord_trans_global_to_local(pose, ball.pos)
                local_dir = math.degrees(math.atan2(ball_local_pos[1], ball_local_pos[0]))
                if ball_local_pos.length() < 35 and math.fabs(local_dir) < 25:
                    dx = ball.rect.centerx - player.rect.centerx
                    dy = ball.rect.centery - player.rect.centery
                    tan = math.atan2(dy, dx)
                    radians = tan + math.pi/2
                    ball.angle = math.degrees(radians)
                    ball.vel = 16
                    ball.last_touch = player.color
            player.shoot = False
