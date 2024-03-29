#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool
import math
import time
import numpy as np
import copy
import random
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal, expon, norm


def rotation2d(pos, dq):
    x, y, q = pos

    x2 = math.cos(dq) * x - math.sin(dq) * y
    y2 = math.sin(dq) * x + math.cos(dq) * y
    q2 = q + dq

    return (x2, y2, q2)


def g_normal_random_equation(loc, scale, val):
    return (1/(scale*math.sqrt(2*3.14159))) * (2.71828**(-((val-loc)**2)/(2*scale**2)))


def normalize_rad(ang):                                                         
    while ang > math.pi:
        ang = ang - 2*math.pi
    while ang < -math.pi:
        ang = ang + 2*math.pi
    return ang


def coord_trans_global_to_local(global_pos_A, global_pos_B):
    g_xa, g_ya, g_qa = global_pos_A                                             
    g_xb, g_yb, g_qb = global_pos_B
    global_pos_AB = (g_xb-g_xa, g_yb-g_ya, g_qb-g_qa)
    l_xb, l_yb, tmp_qb = rotation2d(global_pos_AB, -g_qa)

    return (l_xb, l_yb, normalize_rad(global_pos_AB[2]))


def mili_to_meter(val):
    return val/1000.0


class Particle(object):
    def __init__(self, initial_pose, weight, _id):
        self.pose = initial_pose
        self.prev_pose = None
        self._id = _id
        self.weight = weight

    def motion_update(self, vel_x, vel_y, noise=[0.0,0.0,0.0,0.0], p_id=0, grass_friction_noise_x=0.9, grass_friction_noise_y=0.9, time_interval=0.1):
        self.prev_pose = self.pose

        vel_x = (vel_x + noise[0]*1000*math.sqrt(abs(vel_x) * time_interval) + noise[1]*1000*math.sqrt(abs(vel_y) * time_interval)) * grass_friction_noise_x
        vel_y = (vel_y + noise[2]*1000*math.sqrt(abs(vel_y) * time_interval) +noise[3]*1000*math.sqrt(abs(vel_y) * time_interval)) * grass_friction_noise_y

        x = self.pose[0] + vel_x * time_interval
        y = self.pose[1] + vel_y * time_interval
        theta = math.atan2(vel_y, vel_x)

        self.pose = x, y, theta

    def observation_update(self, observation, ball_gl, robot_pose, noise):
        particle_relative_pos = coord_trans_global_to_local(robot_pose, self.pose)
        #self.weight *= multivariate_normal( mean=observation, cov=np.diag(noise) ).pdf((particle_relative_pos[0], particle_relative_pos[1]))
        #self.weight *= multivariate_normal( mean=self.pose[:2], cov=np.diag(noise) ).pdf((particle_relative_pos[0], particle_relative_pos[1]))
        self.weight *= (g_normal_random_equation(observation[0],noise[0],particle_relative_pos[0]) * g_normal_random_equation(observation[1],noise[1],particle_relative_pos[1]))
        self.weight *= (g_normal_random_equation(ball_gl[0],noise[0],self.pose[0]) * g_normal_random_equation(ball_gl[1],noise[1],self.pose[1]))


class MotionModel(object):
    def __init__(self, initial_pose, motion_noise={"xx":0.12, "xy":0.11, "yx":0.1, "yy":0.09}, 
                 stuck_time = 1e-10, escape_time = 1e10):
        self.motion_noise_rate_pdf = multivariate_normal(cov = np.diag( [motion_noise["xx"]**2, motion_noise["xy"]**2, 
                                                                        motion_noise["yx"]**2, motion_noise["yy"]**2] )) 
        self.stuck_pdf = expon(scale = 1/stuck_time)
        self.escape_pdf = expon(scale = 1/escape_time)

        self.initial_pose = initial_pose
        self.grass_friction_noise = 0.90
        self.prev_vel_x = 1e-100
        self.prev_vel_y = 1e-100

        self.is_stuck = False
        self.time_until_stuck = self.stuck_pdf.rvs()
        self.time_until_escape = self.escape_pdf.rvs()

    def update(self, vel_x, vel_y, use_past_data, time_interval): 
        if use_past_data:
            #print ('used past')
            vel_x = self.prev_vel_x
            vel_y = self.prev_vel_y

        if abs(vel_x) > 1 or abs(vel_y) > 1:
            #vel_x , vel_y = self.stuck(vel_x, vel_y, time_interval)
            self.prev_vel_x = vel_x * self.grass_friction_noise
            self.prev_vel_y = vel_y * self.grass_friction_noise
        else:
            vel_x = vel_y = 1e-100
        return vel_x, vel_y

    def stuck(self, vel_x, vel_y, time_interval):
        if self.is_stuck:
            self.time_until_escape -= time_interval
            if self.time_until_escape <= 0.0:
                self.time_until_escape += self.escape_pdf.rvs()
                self.is_stuck = False
        else:                                                                   
            self.time_until_stuck -= time_interval
            if self.time_until_stuck <= 0.0:
                self.time_until_stuck += self.stuck_pdf.rvs()
                self.is_stuck = True
        return vel_x*(not self.is_stuck), vel_y*(not self.is_stuck)


class ObservationModel(object):
    def __init__(self, static_observation_noise=(0.05, 0.05), dynamic_observation_noise=(0.01, 0.01), 
                 environment_noise=(0.01, 0.01), observation_bias=(1.2, 0.2)): 
        self.observation_noise_rate_pdf = multivariate_normal(cov = np.diag( [static_observation_noise[0], static_observation_noise[1], 
                                                                              dynamic_observation_noise[0], dynamic_observation_noise[1]] ))
        self.environment_noise = environment_noise
        self.fast_velo_observation_bias_rate_pdf = norm(loc = observation_bias[0], scale = observation_bias[1])
        #self.slow_velo_observation_bias_rate_pdf = norm(loc = 1.8, scale = 0.6)

    def update(self, ball_lc):
        noise = self.observation_noise_rate_pdf.rvs()
        noise_x = noise[0] + noise[2]
        noise_y = noise[1] + noise[3]
        
        ball_lc = ball_lc[0] + noise_x*1000, ball_lc[1] + noise_y*1000

        return ball_lc


class BallParticleFilter(object):
    def __init__(self, initial_pose, num=550): 
        self.particles = [Particle(initial_pose, 1.0/num, i) for i in range(num)] # test
        self.num = num
        self.motion_model = MotionModel(initial_pose)
        self.observation_model = ObservationModel()
        self.memo_sub = rospy.Subscriber("ball_memories", PoseArray, self.callbackk_ball_memories)
        #self.sub = rospy.Subscriber("robot_ball_information", PoseArray, self.callbackk_ball_information)
        self.pub = rospy.Publisher("ball_particles", PoseArray, queue_size=4)
        self.status_pub = rospy.Publisher("compution_status", Bool, queue_size=1)
        #self.time_interval = self.calculate_motion_update_time()
        self.time_interval = 0.1
        self.data = []
        self._data = []

    #------------------------ Particle Filter ------------------------
    #----- State transition: b(x) = p( x=xt | x0, u1->t, z1->t ) -----
    def update(self):
        start = time.time()

        if self._data:
            d = self._data.pop(0)
            r_pos = (d[0].position.x, d[0].position.y, d[0].position.z)
            b_gl = (d[1].position.x, d[1].position.y, 0.0)
            b_lc = (d[2].position.x, d[2].position.y, 0.0)
            vel_x, vel_y = d[3].position.x, d[3].position.y
            #print(r_pos, b_gl, b_lc, vel_x, vel_y)
            #print( b_gl, b_lc)
            #print("tracking:", vel_x, vel_y)

            particle_avg_pos = self.get_ball_avg_pos()
            #print(particle_avg_pos)
            #print("ball abs error:", abs(abs(particle_avg_pos[0]) - abs(b_gl[0])), abs(abs(particle_avg_pos[1]) - abs(b_gl[1])))
        else:
            r_pos = b_lc = b_gl = vel_x = vel_y = 0

        status = Bool()
        status.data = False
        self.status_pub.publish(status)

        if b_lc and b_lc[0]<3000 and -50<=math.degrees(math.atan2(b_lc[1], b_lc[0]))<=50:
            m=time.time()
            self.motion_update(0, vel_x, vel_y, self.time_interval)
            #print("motion:", time.time() -m)
            o=time.time()
            self.observation_update(r_pos, b_lc, b_gl)
            #print("observ:", time.time() -o)
            r=time.time()
            #self.resampling()
            #print("resampl:", time.time() -r)
        else: # Ball out of sight
            self.motion_update(1)

        self.publish_particles()

        status = Bool()
        status.data = True
        self.status_pub.publish(status)
        #print("calculation time:", time.time() - start)

    def publish_particles(self):
        particle_poses = PoseArray()
        for p in self.particles:
            pos = Pose()
            pos.position.x = p.pose[0]/10
            pos.position.y = p.pose[1]/10
            particle_poses.poses.append(pos)
        self.pub.publish(particle_poses)

    def callbackk_ball_memories(self, msgs):
        if len(msgs.poses) == 4:
            self._data.append(msgs.poses)

    def callbackk_ball_information(self, msgs):
        if len(msgs.poses) == 4:
            self.data.append(msgs.poses)

    def calculate_motion_update_time(self):
        start = time.time()
        self.motion_update(0, 0.1, 0.1)
        return time.time() -start

    def motion_update(self, use_past_data=0, vel_x=0.0, vel_y=0.0, time_interval=0.1):
        vel_x, vel_y = self.motion_model.update(vel_x, vel_y, use_past_data, time_interval)
        #if abs(vel_x) > 8.0 or abs(vel_y) > 8.0:
        #    motion_bias = self.observation_model.fast_velo_observation_bias_rate_pdf.rvs()
        #else:
        ##    motion_bias = self.observation_model.slow_velo_observation_bias_rate_pdf.rvs()
        #motion_bias = self.observation_model.fast_velo_observation_bias_rate_pdf.rvs()
        motion_bias = random.uniform(1.3, 1.5)
        #print (vel_x, vel_y)

        for index, p in enumerate(self.particles):
            #p.motion_update(vel_x*motion_bias, vel_y*(motion_bias-0.7), 
            #                self.motion_model.motion_noise_rate_pdf.rvs(), index, 
            #                self.motion_model.grass_friction_noise, self.motion_model.grass_friction_noise,# b^(xt) = p( xt | xt-1, ut  )
            #                time_interval) 
            p.motion_update(vel_x*motion_bias, vel_y*motion_bias, 
                            [random.uniform(-0.10, 0.10), random.uniform(-0.11, 0.11),random.uniform(-0.11, 0.11),random.uniform(-0.11, 0.11)], index, 
                            self.motion_model.grass_friction_noise, self.motion_model.grass_friction_noise,# b^(xt) = p( xt | xt-1, ut  )
                            time_interval) 

    def observation_update(self, robot_pose, ball_lc, ball_gl):
        ball_lc = self.observation_model.update(ball_lc)
        ball_lc = mili_to_meter(ball_lc[0]), mili_to_meter(ball_lc[1])
        robot_pose = mili_to_meter(robot_pose[0]), mili_to_meter(robot_pose[1]), robot_pose[2]

        for index, p in enumerate(self.particles):
            p.observation_update(ball_lc, ball_gl, robot_pose, 
                                  [(self.observation_model.environment_noise[0])**2, 
                                   (self.observation_model.environment_noise[1])**2])
            #print(index, p.pose[:2], " Weight:", p.weight)

    def get_ball_avg_pos(self):
        sum_pose = (0.0, 0.0)
        for p in self.particles:
            sum_pose = sum_pose[0] + p.pose[0], sum_pose[1] + p.pose[1]
        return (sum_pose[0]/self.num), (sum_pose[1]/self.num), 0.0 # estimate ball average pose from all particles 

    def resampling(self): #systemmatic sampling
        weight_list = np.cumsum([p.weight for p in self.particles]) # create weight list
        if weight_list[-1] < 1e-100: weight_list = [e + 1e-100 for e in weight_list] #make sure weight is not 0 
        
        step = weight_list[-1]/len(self.particles)
        #r = np.random.uniform(0.0, step)
        r = random.uniform(0.0, step)
        cur_pos = 0
        ps = []

        while(len(ps) < len(self.particles)):
            #print(r)
            if r < weight_list[cur_pos]:
                ps.append(self.particles[cur_pos])
                r += step
            else:
                cur_pos += 1
        self.particles = [copy.deepcopy(p) for p in ps]
        self.particles = sorted(self.particles, key= lambda p: p.weight)
        for p in self.particles: p.weight = 1.0/len(self.particles)


if __name__ == "__main__":
    rospy.init_node("ball_traking_particle_filter")
    ball_tracking = BallParticleFilter([0.0, 0.0, 0.0])

    while not rospy.is_shutdown():
        ball_tracking.update()
    
