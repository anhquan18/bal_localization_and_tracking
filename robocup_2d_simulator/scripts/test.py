#!/usr/bin/env python
import rospy
import socket, select, string, sys, time, random
import pygame
from gui import font_name, font_size, font_clearly, black, red, blue, font_size, font_name, font_clearly
from test_rcl import SoccerPlayer

frame_rate = 20

if __name__ == "__main__":
    rospy.init_node("robot_client")

    host = rospy.get_param("~host", "0.0.0.0")
    port = rospy.get_param("~port", 1234)
    color = rospy.get_param("~color", "blue")

    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2)

    # wait for server
    rospy.sleep(1.3)
    
    # connect to remote host
    try :
        s.connect((host, port))
    except :
        print ('Unable to connect')
        sys.exit()
    
    print ('Connected to remote host. Start sending messages')
    pygame.init()
    pygame.mixer.quit()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((420,320))
    font = pygame.font.Font(font_name, 30)
    random_id = random.random()
    request_access = str(color) + ',' + str(random_id)
    player = SoccerPlayer()

    try:
        while True:
            s.send(request_access)
            permission = s.recv(16)
            if permission[0:7] == 'granted': # remove unnecessary mixed data caused when transfering
                break
            time.sleep(0.1)
        while not rospy.is_shutdown():
            clock.tick(frame_rate)
            pygame.display.update()
            
            player.reset_velocity()

            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                            player.shoot = 1
            p = pygame.key.get_pressed()            
            if p[pygame.K_LEFT]: player.ang_vel = -6.0
            if p[pygame.K_RIGHT]: player.ang_vel = 6.0
            if p[pygame.K_UP]: player.x_vel = -4.0
            if p[pygame.K_DOWN]: player.x_vel = 4.0
            if p[pygame.K_a]: player.y_vel = -4.0
            if p[pygame.K_s]: player.y_vel = 4.0
            if p[pygame.K_o]: player.state = 'a'
            if p[pygame.K_n]: player.state = 'n'
            
            data = player.return_data()
            s.send(data)

            try:
                data = s.recv(16)
            except socket.timeout:
                print ("socket timeout")
                data = ''
            
            player.handle_data(data)
            
            
            if color == 'blue':
                screen.fill(blue)
            else:
                screen.fill(red)
            if player.state == 'n':
                screen.blit(font.render('MODE: Manual', font_clearly,black), (50,80))
                screen.blit(font.render('Press o for Automatic MODE', font_clearly,black), (50,130))
            else:
                screen.blit(font.render('MODE: Automatic', font_clearly,black), (50,80))
                screen.blit(font.render('Press n for Manual MODE', font_clearly,black), (50,130))
    except Exception as e:
        print "Error:", e
    finally:
        s.send("KILL")
        s.close()
