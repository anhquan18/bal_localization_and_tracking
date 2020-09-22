import pygame  
import sys

class MOUSE_ACTION(object):

    def mouse_get_position(self):
        mouse_pressed = pygame.mouse.get_pressed()
        if mouse_pressed[0]:  
            self.mouse_x, self.mouse_y = pygame.mouse.get_pos()
            return(self.mouse_x,self.mouse_y)

    def check_position(self,function,offset,size,mouse_position):
            if offset[0] < mouse_position[0] < offset[0]+size[0] \
                and offset[1] < mouse_position[1] < offset[1]+size[1]:
                function()
                return True

