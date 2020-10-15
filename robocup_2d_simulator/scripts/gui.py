import pygame
import math
import os
from pygame.locals import Rect, RLEACCEL,QUIT,MOUSEBUTTONDOWN
import ui_actions as ua
import sys
from mouse_action import MOUSE_ACTION
from geometry import convert_y_x_to_x_y_axis

# GUI PARAMETER
VERTICAL  = 700
HORIZONTAL= 1250
SCR_SIZE = (HORIZONTAL,VERTICAL)
FIELD_OFFSET = (20,50)
FIELD_SIZE   = (900,600)
FIELD_ORIGIN = (470,350) 

TIME_OFFSET = (20,5)
TIME_SIZE   = (200,30)

POINT_OFFSET = (220,5)
POINT_SIZE   = (100,30)

GC_OFFSET = (330,5)
GC_SIZE   = (50,30)
GC_ACTION_SIZE = (100,30)

STATE_OFFSET = (940,50)
STATE_SIZE   = (300,600)
STATE_EACH_SIZE = (STATE_SIZE[0]/2,STATE_SIZE[1]/4)

UI_OFFSET = (20,667)
UI_SIZE   = (225,30)

frame_rate = 30    #FPS
start_time = 600   #seconds

red_point  = 3
blue_point = 4

font_clearly = True
font_name = None
gui_flag = 0 
font_size = 25

start_flag = False
reset_flag = False

# RGB COLORS
black = (0,0,0)
white = (255,255,255)
green = (76,153,5)
gray  = (126,126,126)
dark_gray = (50, 50, 50)
red   = (255,65,65)
blue  = (65,65,255)
orange = (255,153,51)
red_goal  = (238,36,83)
blue_goal = (111,28,220)


class GUI(object):
    def __init__(self):
        pygame.init()
        print("pygame intialized")
        self.font = pygame.font.Font(font_name,font_size)
        self.screen = pygame.display.set_mode(SCR_SIZE)
        self.clock = pygame.time.Clock()
        self.frame_count = 0                 #TODO to run 
        self.red_point  = 0
        self.blue_point = 0
        self.load_flag = False
        self.mouse = MOUSE_ACTION()
        self.actions = ua.UI_ACTIONS()

    def update(self):
        self.drawing_field()
        self.drawing_frame()
        self.draw_txt()
        self.show_point(self.red_point,self.blue_point)
        self.timer_control(self.frame_count)
        self.event_process()

    def draw_ball_particles(self, poses):
        for p in poses:
            self.draw_circle(((255, 150, 51, 200)),(int(FIELD_ORIGIN[0]+p.position.x), int(FIELD_ORIGIN[1]+p.position.y)),7,0)

    def draw_ball_particles_past_trajectories(self, points):
        self.draw_lines(red,False,points,8)

    def draw_ball_past_trajectories(self, points):
        self.draw_lines(black,False,points,8)

    def display_player(self, obj):
        if obj.old_color != obj.color:
            obj.old_color = obj.color
            obj.change_color(obj.color)
        if obj.color == "blue":
            self.draw_arc(gray, (obj.pos[0]-FIELD_SIZE[0]/4, obj.pos[1]-FIELD_SIZE[0]/4), (FIELD_SIZE[0]/2,FIELD_SIZE[0]/2), math.radians(convert_y_x_to_x_y_axis(obj.angle)-50), math.radians(convert_y_x_to_x_y_axis(obj.angle)+50), FIELD_SIZE[0]/4)
        self.screen.blit(obj.img, obj.rect)

    def display_ball(self, obj):
        if obj.last_scored == 'blue': self.blue_point += 1
        if obj.last_scored == 'red': self.red_point += 1
        obj.last_scored = None
        self.screen.blit(obj.img, obj.rect)

    def display(self, obj):
        self.screen.blit(obj.img, obj.rect)

    def draw_lines(self,color,closed,points,width):
        pygame.draw.lines(self.screen,color,closed,points, width)

    def draw_arc(self,color,offset,size,start_angle,stop_angle,width):
        pygame.draw.arc(self.screen,color,Rect((offset, size)),start_angle,stop_angle, width)

    def draw_rect(self,color,offset,size,thickness):
        pygame.draw.rect(self.screen,color,Rect((offset,size)),thickness)

    def draw_circle(self,color,offset,radious,thickness):
        pygame.draw.circle(self.screen,color,offset,radious,thickness)

    def write_txt(self,color,offset,txt_str):
        self.screen.blit(self.font.render(txt_str,font_clearly,color),offset) 

    def drawing_frame(self):
        self.draw_rect((white),(TIME_OFFSET),(TIME_SIZE),0)      #time
        self.draw_rect((black),(TIME_OFFSET),(TIME_SIZE),3)

        self.draw_rect((white),(POINT_OFFSET),(POINT_SIZE),0)     #point
        self.draw_rect((black),(POINT_OFFSET),(POINT_SIZE),3)

        if  gui_flag == 0:
            self.draw_rect((white),(GC_OFFSET),(GC_SIZE),0)  #state
            self.draw_rect((black),(GC_OFFSET),(GC_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_SIZE[0],GC_OFFSET[1]),(GC_SIZE),0)
            self.draw_rect((black),(GC_OFFSET[0]+GC_SIZE[0],GC_OFFSET[1]),(GC_SIZE),3)

        elif gui_flag == 1:
            self.draw_rect((gray),(GC_OFFSET),(GC_SIZE),0)   #GC_button ,init,set,ready,play,finish
            self.draw_rect((black),(GC_OFFSET),(GC_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_SIZE[0],GC_OFFSET[1]),(GC_SIZE),0)
            self.draw_rect((black),(GC_OFFSET[0]+GC_SIZE[0],GC_OFFSET[1]),(GC_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*3+10,GC_OFFSET[1]),(GC_ACTION_SIZE),0)     
            self.draw_rect((black),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*3+10,GC_OFFSET[1]),(GC_ACTION_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*4+10,GC_OFFSET[1]),(GC_ACTION_SIZE),0)
            self.draw_rect((black),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*4+10,GC_OFFSET[1]),(GC_ACTION_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*5+10,GC_OFFSET[1]),(GC_ACTION_SIZE),0)
            self.draw_rect((black),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*5+10,GC_OFFSET[1]),(GC_ACTION_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*6+10,GC_OFFSET[1]),(GC_ACTION_SIZE),0)
            self.draw_rect((black),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*6+10,GC_OFFSET[1]),(GC_ACTION_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*7+10,GC_OFFSET[1]),(GC_ACTION_SIZE),0)
            self.draw_rect((black),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*7+10,GC_OFFSET[1]),(GC_ACTION_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*8+10,GC_OFFSET[1]),(GC_ACTION_SIZE),0)
            self.draw_rect((black),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*8+10,GC_OFFSET[1]),(GC_ACTION_SIZE),3)
        
        elif gui_flag == 2:
            self.draw_rect((white),(GC_OFFSET),(GC_SIZE),0)  #GC_button ,freekick,penaltykick
            self.draw_Rect((black),(GC_OFFSET),(GC_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_SIZE[0],GC_OFFSET[1]),(GC_SIZE),0)
            self.draw_rect((black),(GC_OFFSET[0]+GC_SIZE[0],GC_OFFSET[1]),(GC_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*3+10,GC_OFFSET[1]),(GC_ACTION_SIZE),0)     
            self.draw_rect((black),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*3+10,GC_OFFSET[1]),(GC_ACTION_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*5+10,GC_OFFSET[1]),(GC_ACTION_SIZE),0)
            self.draw_rect((black),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*5+10,GC_OFFSET[1]),(GC_ACTION_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*6+10,GC_OFFSET[1]),(GC_ACTION_SIZE),0)
            self.draw_rect((black),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*6+10,GC_OFFSET[1]),(GC_ACTION_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*7+10,GC_OFFSET[1]),(GC_ACTION_SIZE),0)
            self.draw_rect((black),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*7+10,GC_OFFSET[1]),(GC_ACTION_SIZE),3)
            self.draw_rect((white),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*8+10,GC_OFFSET[1]),(GC_ACTION_SIZE),0)
            self.draw_rect((black),(GC_OFFSET[0]+GC_ACTION_SIZE[0]*8+10,GC_OFFSET[1]),(GC_ACTION_SIZE),3)
        

        self.draw_rect((black),(STATE_OFFSET),(STATE_SIZE),6)

        self.draw_rect((black),(UI_OFFSET[0]+UI_SIZE[0]*0,UI_OFFSET[1]),(UI_SIZE),6)
        self.draw_rect((white if start_flag==False else gray),(UI_OFFSET[0]+225*0,UI_OFFSET[1]),(UI_SIZE),0)
        self.draw_rect((black),(UI_OFFSET[0]+UI_SIZE[0]*1,UI_OFFSET[1]),(UI_SIZE),6)
        self.draw_rect((white if start_flag==True else gray),(UI_OFFSET[0]+225*1,UI_OFFSET[1]),(UI_SIZE),0)
        self.draw_rect((black),(UI_OFFSET[0]+UI_SIZE[0]*2,UI_OFFSET[1]),(UI_SIZE),6)
        self.draw_rect((white),(UI_OFFSET[0]+UI_SIZE[0]*2,UI_OFFSET[1]),(UI_SIZE),0)
        self.draw_rect((black),(UI_OFFSET[0]+UI_SIZE[0]*3,UI_OFFSET[1]),(UI_SIZE),6)
        self.draw_rect((white),(UI_OFFSET[0]+UI_SIZE[0]*3,UI_OFFSET[1]),(UI_SIZE),0)
        self.draw_rect((black),(UI_OFFSET[0]+UI_SIZE[0]*4,UI_OFFSET[1]),(UI_SIZE),6)
        self.draw_rect((white),(UI_OFFSET[0]+UI_SIZE[0]*4,UI_OFFSET[1]),(UI_SIZE),0)
        for i in range (4):
            for k in range(2):
                    if k : self.draw_rect((red),(STATE_OFFSET[0]+STATE_EACH_SIZE[0]*k,STATE_OFFSET[1]+STATE_EACH_SIZE[1]*i),(STATE_EACH_SIZE),0)
                    else: self.draw_rect((blue),(STATE_OFFSET[0]+STATE_EACH_SIZE[0]*k,STATE_OFFSET[1]+STATE_EACH_SIZE[1]*i),(STATE_EACH_SIZE),0)
                    self.draw_rect((black),(STATE_OFFSET[0]+STATE_EACH_SIZE[0]*k,STATE_OFFSET[1]+STATE_EACH_SIZE[1]*i),(STATE_EACH_SIZE),2)

    def draw_txt(self):
        self.write_txt((black),(UI_OFFSET[0]+80,UI_OFFSET[1]+10),"start")
        self.write_txt((black),(UI_OFFSET[0]+305,UI_OFFSET[1]+10),"stop")
        self.write_txt((black),(UI_OFFSET[0]+530,UI_OFFSET[1]+10),"reset")
        self.write_txt((black),(UI_OFFSET[0]+755,UI_OFFSET[1]+10),"save")
        self.write_txt((black),(UI_OFFSET[0]+980,UI_OFFSET[1]+10),"ball")


        self.write_txt((black),(STATE_OFFSET[0]+155,STATE_OFFSET[1]),"red1")
        self.write_txt((black),(STATE_OFFSET[0]+155,STATE_OFFSET[1]+150),"red2")
        self.write_txt((black),(STATE_OFFSET[0]+155,STATE_OFFSET[1]+300),"red3")
        self.write_txt((black),(STATE_OFFSET[0]+155,STATE_OFFSET[1]+450),"red4")
        self.write_txt((black),(STATE_OFFSET[0]+5,STATE_OFFSET[1]),"blue1")
        self.write_txt((black),(STATE_OFFSET[0]+5,STATE_OFFSET[1]+150),"blue2")
        self.write_txt((black),(STATE_OFFSET[0]+5,STATE_OFFSET[1]+300),"blue3")
        self.write_txt((black),(STATE_OFFSET[0]+5,STATE_OFFSET[1]+450),"blue4")

        if gui_flag==0:
            self.write_txt((black),(GC_OFFSET[0]+10,GC_OFFSET[1]+5),"1st")
            self.write_txt((black),(GC_OFFSET[0]+55,GC_OFFSET[1]+5),"2nd")

        elif gui_flag==1:
            self.write_txt((black),(GC_OFFSET[0]+10,GC_OFFSET[1]+5),"1st")
            self.write_txt((black),(GC_OFFSET[0]+55,GC_OFFSET[1]+5),"2nd")
            self.write_txt((black),(GC_OFFSET[0]+335,GC_OFFSET[1]+5),"back")
            self.write_txt((black),(GC_OFFSET[0]+435,GC_OFFSET[1]+5),"initial")
            self.write_txt((black),(GC_OFFSET[0]+535,GC_OFFSET[1]+5),"ready")
            self.write_txt((black),(GC_OFFSET[0]+635,GC_OFFSET[1]+5),"set")
            self.write_txt((black),(GC_OFFSET[0]+735,GC_OFFSET[1]+5),"play")
            self.write_txt((black),(GC_OFFSET[0]+835,GC_OFFSET[1]+5),"finish")

        elif gui_flag==2:
            self.write_txt((black),(GC_OFFSET[0]+10,GC_OFFSET[1]+5),"1st")
            self.write_txt((black),(GC_OFFSET[0]+55,GC_OFFSET[1]+5),"2nd")
            self.write_txt((black),(GC_OFFSET[0]+335,GC_OFFSET[1]+5),"back")
            self.write_txt((red_goal),(GC_OFFSET[0]+535,GC_OFFSET[1]+5),"f-kick")
            self.write_txt((red_goal),(GC_OFFSET[0]+635,GC_OFFSET[1]+5),"p-kick")
            self.write_txt((blue_goal),(GC_OFFSET[0]+735,GC_OFFSET[1]+5),"f-kick")
            self.write_txt((blue_goal),(GC_OFFSET[0]+835,GC_OFFSET[1]+5),"p-kick")

    def drawing_field(self):
        self.screen.fill((gray))
        self.draw_rect((green),(FIELD_OFFSET),(FIELD_SIZE),0)
        self.draw_rect((white),(FIELD_OFFSET[0],FIELD_OFFSET[1]+50),(100,500),6)
        self.draw_rect((white),(FIELD_OFFSET[0]+800,FIELD_OFFSET[1]+50),(100,500),6)
        
        self.draw_circle((white),(FIELD_ORIGIN),75,8)
        self.draw_circle((white),(FIELD_ORIGIN),7,0)
        self.draw_circle((white),(FIELD_OFFSET[0]+690,FIELD_OFFSET[1]+300),7,0)
        self.draw_circle((white),(FIELD_OFFSET[0]+210,FIELD_OFFSET[1]+300),7,0)
        
        self.draw_rect((white),(FIELD_OFFSET),(FIELD_SIZE[0]/2,FIELD_SIZE[1]),6)
        self.draw_rect((white),(FIELD_OFFSET),(FIELD_SIZE),6)
        self.draw_rect((green),(FIELD_OFFSET[0]-8,FIELD_OFFSET[1]-8),(FIELD_SIZE[0]+16,FIELD_SIZE[1]+16),10)
        
        self.draw_circle((red_goal),(20,220),9,0)
        self.draw_circle((red_goal),(20,480),9,0)
        self.draw_circle((blue_goal),(920,220),9,0)
        self.draw_circle((blue_goal),(920,480),9,0)

    def timer_control(self,frame_count):
        global start_flag
        global reset_flag
        self.total_seconds = start_time - (frame_count // frame_rate)
        self.minutes = self.total_seconds // 60
        self.seconds = self.total_seconds % 60
        self.output_strig = "{0:02}:{1:02}".format(self.minutes,self.seconds)
        self.timer_txt = self.font.render(self.output_strig,font_clearly,(blue))
        self.screen.blit(self.timer_txt,(100,10))
        if self.total_seconds > 0 and start_flag == True:
            self.frame_count+=1
        elif self.total_seconds == 0 or reset_flag == True:
            self.frame_count=0
            start_flag=False
            reset_flag=False

    def event_process(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

            #TODO to solve KEYDOWN`s problem
            # if event.type == KEYDOWN:
            #     if event.key == K_ESCAPE:
            #         pygame.quit()
            #         sys.exit()

            if event.type == MOUSEBUTTONDOWN and event.button == 1:
                mouse_position = self.mouse.mouse_get_position()
                if mouse_position:
                    self.mouse.check_position(self.actions.start_time,(UI_OFFSET),(UI_SIZE),mouse_position)
                    self.mouse.check_position(self.actions.stop_time, (UI_OFFSET[0]+UI_SIZE[0]*1,UI_OFFSET[1]),(UI_SIZE),mouse_position)
                    self.mouse.check_position(self.actions.reset_game,(UI_OFFSET[0]+UI_SIZE[0]*2,UI_OFFSET[1]),(UI_SIZE),mouse_position)
                    self.mouse.check_position(self.actions.save_game, (UI_OFFSET[0]+UI_SIZE[0]*3,UI_OFFSET[1]),(UI_SIZE),mouse_position)
                    self.mouse.check_position(self.actions.move_ball, (UI_OFFSET[0]+UI_SIZE[0]*4,UI_OFFSET[1]),(UI_SIZE),mouse_position)
                    
                    if gui_flag==0:
                        self.mouse.check_position(self.actions.state_change1, (GC_OFFSET),(GC_SIZE),mouse_position)
                        self.mouse.check_position(self.actions.state_change2, (GC_OFFSET[0]+50,GC_OFFSET[1]),(GC_SIZE),mouse_position)

                    if gui_flag==1:
                        self.mouse.check_position(self.actions.state_change2, (GC_OFFSET[0]+50,GC_OFFSET[1]),(GC_SIZE),mouse_position)
                        self.mouse.check_position(self.actions.back,      (GC_OFFSET[0]+GC_ACTION_SIZE[0]*3+10,GC_OFFSET[1]),(GC_ACTION_SIZE),mouse_position)
                        self.mouse.check_position(self.actions.GC_init,   (GC_OFFSET[0]+GC_ACTION_SIZE[0]*4+10,GC_OFFSET[1]),(GC_ACTION_SIZE),mouse_position)
                        self.mouse.check_position(self.actions.GC_ready,  (GC_OFFSET[0]+GC_ACTION_SIZE[0]*5+10,GC_OFFSET[1]),(GC_ACTION_SIZE),mouse_position)
                        self.mouse.check_position(self.actions.GC_set,    (GC_OFFSET[0]+GC_ACTION_SIZE[0]*6+10,GC_OFFSET[1]),(GC_ACTION_SIZE),mouse_position)
                        self.mouse.check_position(self.actions.GC_play,   (GC_OFFSET[0]+GC_ACTION_SIZE[0]*7+10,GC_OFFSET[1]),(GC_ACTION_SIZE),mouse_position)
                        self.mouse.check_position(self.actions.GC_finish, (GC_OFFSET[0]+GC_ACTION_SIZE[0]*8+10,GC_OFFSET[1]),(GC_ACTION_SIZE),mouse_position)
                    
                    if gui_flag==2:
                        self.mouse.check_position(self.actions.state_change1, (GC_OFFSET),(GC_SIZE),mouse_position)
                        self.mouse.check_position(self.actions.back,              (GC_OFFSET[0]+GC_ACTION_SIZE[0]*3+10,GC_OFFSET[1]),(GC_ACTION_SIZE),mouse_position)
                        self.mouse.check_position(self.actions.red_free_kick,     (GC_OFFSET[0]+GC_ACTION_SIZE[0]*5+10,GC_OFFSET[1]),(GC_ACTION_SIZE),mouse_position)
                        self.mouse.check_position(self.actions.red_penalty_kick,  (GC_OFFSET[0]+GC_ACTION_SIZE[0]*6+10,GC_OFFSET[1]),(GC_ACTION_SIZE),mouse_position)
                        self.mouse.check_position(self.actions.blue_free_kick,    (GC_OFFSET[0]+GC_ACTION_SIZE[0]*7+10,GC_OFFSET[1]),(GC_ACTION_SIZE),mouse_position)
                        self.mouse.check_position(self.actions.blue_penalty_kick, (GC_OFFSET[0]+GC_ACTION_SIZE[0]*8+10,GC_OFFSET[1]),(GC_ACTION_SIZE),mouse_position)

    def show_point(self,red_point,blue_point):
        self.write_txt((black),(POINT_OFFSET[0]+15,POINT_OFFSET[1]+5),str(red_point))
        self.write_txt((black),(POINT_OFFSET[0]+45,POINT_OFFSET[1]+5),"-")
        self.write_txt((black),(POINT_OFFSET[0]+55,POINT_OFFSET[1]+5),str(blue_point))


if __name__ == '__main__':
    GUI()
