import socket
import select
import traceback
import math

MAX_PLAYER = 8

class Server(object):
    def __init__(self, data_base, create_player):
        self.connection_list = []
        self.recv_buffer = 4096
        self.port = 1234

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(("0.0.0.0", self.port))
        self.server_socket.listen(10)

        self.connection_list.append(self.server_socket)
        self.data_base = data_base
        self.create_player = create_player

        print("starting server on port: ", self.port)

    def update(self):
        read_sockets, write_sockets, error_sockets = select.select(self.connection_list, [], [])

        for sock in read_sockets:
            if sock == self.server_socket and len(self.data_base.player_list) < MAX_PLAYER:
                # new player added
                sockfd, addr = self.server_socket.accept()
                self.connection_list.append(sockfd)
                self.data_base.player_list.append(self.create_player(sockfd))
                print("new player")

            else:
                # handle player data
                try:
                    # get player data
                    data = sock.recv(self.recv_buffer) 
                    # bytes to string
                    data = data.decode("utf-8")
                    if data:
                        if "KILL" in data:
                            print("player exit")
                            sock.close()
                            self.connection_list.remove(sock)
                            for player in self.data_base.player_list:
                                if player.sock == sock:
                                    self.data_base.player_list.remove(player)
                                    self.data_base.team[player.color].remove(player)
                        else:
                            for player in self.data_base.player_list:
                                if player.sock == sock:
                                    self.recv_data(player, data)
                                    self.send_data(player, sock)
                except ValueError as ex:
                    print (ex)
                    sock.close()
                    self.connection_list.remove(sock)
                    continue
                except:
                    print("player exit")
                    print (traceback.format_exc())
                    sock.close()
                    self.connection_list.remove(sock)
                    continue


    def check_player_permission(self, player, color, rand_id):
        if len(self.data_base.team[color]) >= (MAX_PLAYER/2): 
            self.data_base.player_list.remove(player)
            raise ValueError("Usage: Can't have more " + color +" player")
        elif rand_id in self.data_base.robots_random_id: # debug duplicate player
            player.sock.send(bytes('granted', "utf-8"))
            return
        else:
            ID_list = range(4)
            player.color = color
            for robot in self.data_base.team[color]:
                if robot.ID in ID_list: ID_list.remove(robot.ID)
            player.ID = ID_list[0]
            self.data_base.update_new_player(player)
            self.data_base.robots_random_id.append(rand_id)
            player.sock.send(bytes('granted', "utf-8"))

    def recv_data(self, player, data):
        data = data.split(",")
        if len(data) == 5:
            if data[0][0]=='b' or data[0][0]=='r': return # remove unnecessary mixed data caused when transfering
            player.shoot = bool(int(data[0]))
            player.angle_vel = float(data[1])
            player.x_vel = float(data[2])
            player.y_vel = float(data[3])
            player.state = data[4]
        elif len(data) == 2:
            self.check_player_permission(player, str(data[0]), str(data[1]))
        else:
            player.shoot = False
            player.angle_vel = 0.0
            player.x_vel = 0.0
            player.y_vel = 0.0
            player.state = 'n'

    def send_data(self, player, sock):
        ball_pos = self.data_base.ball.get_pos(player.color)
        ball_dist = player.get_ball_dis(ball_pos)

        data = str(ball_dist)
        
        sock.send(bytes(data, "utf-8"))

    def broadcast_data(self, sock, data):
        for socket in self.connection_list:
            if socket != self.server_socket and socket != sock:
                try:
                    socket.send(data)
                except:
                    socket.close()
                    self.connection_list.remove(socket)

if __name__ == '__main__':
    server = Server()
    server.server_socket.close()
