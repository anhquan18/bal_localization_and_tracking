import pygame
import rospy
import sys
import gui
import mouse_action as ma


class UI_ACTIONS(object):
    def __init__(self):
        import objects_information

        self.data_base = objects_information.data_base
        self.mouse=ma.MOUSE_ACTION()

    def dummy_func(self):
        pass

    def start_time(self):
        if not gui.start_flag:
            gui.start_flag = True
            print ("start")

    def stop_time(self):
        if gui.start_flag:
            gui.start_flag = False
            print ("stop")

    def reset_game(self):
        gui.reset_flag = True
        gui.start_flag = False
        print ("reset")

    def save_game(self):
        print ("save")

    def GC_init(self):
        print ("init")
        self.data_base.game_state.set_state("1st","initial")

    def GC_ready(self):
        print ("ready")
        self.data_base.game_state.set_state("1st","ready")

    def GC_set(self):
        print ("set")
        self.data_base.game_state.set_state("1st","set")

    def GC_play(self):
        print ("play")
        self.data_base.game_state.set_state("1st","playing")

    def GC_finish(self):
        print ("finish")
        self.data_base.game_state.set_state("1st","finishied")

    def move_ball(self):
        print ("move_ball")
        self.start_flag=False
        while not self.start_flag:
            for event in pygame.event.get():
                if event.type == pygame.locals.MOUSEBUTTONDOWN and event.button == 1:
                    mouse_position= self.mouse.mouse_get_position()
                    if mouse_position and self.mouse.check_position(self.dummy_func,gui.FIELD_OFFSET,gui.FIELD_SIZE,mouse_position):
                        self.data_base.ball.set_pos(mouse_position)
                    else: 
                        self.data_base.ball.set_pos(gui.FIELD_ORIGIN)
                    print ("moved")
                    self.start_flag=True

    def state_change1(self):
        if not gui.gui_flag==1:
            gui.gui_flag=1
            print ("change gui state1")

    def state_change2(self):
        if not gui.gui_flag==2:
            gui.gui_flag=2
            print ("change gui state2")

    def back(self):
        if not gui.gui_flag == 0:
            gui.gui_flag=0
            print ("back")
        
    def red_free_kick(self):
        print ("red free-kick")
        self.data_base.game_state.set_state("2nd","red freekick")

    def blue_free_kick(self):
        print ("blue free-kick")
        self.data_base.game_state.set_state("2nd","blue freekick")

    def red_penalty_kick(self):
        print ("red penalty-kick")
        self.data_base.game_state.set_state("2nd","red penalty")

    def blue_penalty_kick(self):
        print ("blue penalty-kick")
        self.data_base.game_state.set_state("2nd","blue penalty")

