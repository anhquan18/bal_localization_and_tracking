#!/usr/bin/env python
import rospy
import pygame
from gui import frame_rate, GUI
from engine import Engine
from server import Server

def run():
    pygame.init()
    pygame.mixer.quit()
    clock = pygame.time.Clock()
    
    # initialize GUI
    gui = GUI()
    
    # initialize data_base and ball after created gui
    import objects_information

    ball = objects_information.data_base.ball

    # initialize engine with defined ball property
    engine = Engine(ball, objects_information.data_base)

    # initialize server
    server = Server(objects_information.data_base, objects_information.Player)

    try: 
        while not rospy.is_shutdown():
            # FPS
            clock.tick(frame_rate)

            # update engine
            engine.update()
           
            # update server
            server.update()

            # update gui
            gui.update()
            for player in engine.player_list:
                player.update(ball)
                gui.display_player(player)

            gui.draw_ball_particles(ball.particle_poses)
            gui.draw_ball_particles_past_trajectories(ball.past_particle_avg_poses)
            gui.draw_ball_past_trajectories(ball.mem_pos_list)
            gui.display_ball(ball)
            # display update
            pygame.display.update()
    finally:
        server.server_socket.close()

if __name__ == '__main__':
    rospy.init_node("strategy_simulator")
    run()
