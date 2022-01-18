import sys

from astar import *
from croblink import *
from math import *
import statistics
import xml.etree.ElementTree as ET

CELLROWS = 7
CELLCOLS = 14


class MyRob(CRobLinkAngs):
    initial_pos = ()  # posicao inicial
    last_target = ()  # posicao target anterior
    pos = ()  # posicao atual
    my_x = 0 # x calculado 
    my_y = 0 # y calculado
    my_pos = () # posicao atual calculada
    prev_out_l = 0   # ultima potencia aplicada na roda esquerda
    prev_out_r = 0   # ultima potencia aplicada na roda direita
    in_power_l = 0   # potencia a aplicar na roda esquerda
    in_power_r = 0   # potencia a aplicar na roda direita
    prev_in_r = 0
    prev_in_l = 0
    target = ()  # posicao target
    orientation = None  # orientacao desde a ultima rotacao
    next_orientation = None  # orientacao objetivo
    visited_positions = set()  # set com todas as posicoes ja visitadas
    not_taken_positions = {}  # set com posicoes conhecidas nao visitadas
    path = []   # lista com as posicoes a percorrer para chegar a uma posicao objetivo
    paredes = dict()
    mapa = dict()
    spots = dict()  # dicionario com as posicoes dos spots encontrados
    turn_priority = "front"
    returning = False   # voltar à posicao inicial

    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'mapping'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop': # and self.measures.start:
                state = stopped_state
                self.initial_pos = (self.measures.x, self.measures.y)
                # print(f"Initial pos -> ({self.initial_pos[0]},{self.initial_pos[1]})")
                self.pos = (0, 0)
                self.last_target = self.pos
                self.orientation = self.direction()
                self.visited_positions.add(self.pos)

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'go_ahead':
                self.pos = round(self.measures.x - self.initial_pos[0], 2), round(self.measures.y - self.initial_pos[1],
                                                                                  2)

                # print(f"\nPosition -> ({self.pos[0]},{self.pos[1]})")
                # print(f"myPosition -> ({self.my_pos[0]},{self.my_pos[1]})")

                # if self.target != ():
                #     print(f"target -> ({self.target[0]},{self.target[1]})")
                
                # ainda nao chegou ao objetivo, anda
                self.go()

                # verificar se ja chegou à posicao objetivo
                if self.has_reached_target(self.my_pos, self.target) or self.measures.irSensor[0] > 2.5:
                    print("\n\n Cheguei ao target")
                    print(f"target: ({self.target[0]},{self.target[1]})")
                    print("front:", self.measures.irSensor[0])
                    self.visited_positions.add(self.target)
                    self.last_target = self.target
                    self.target = ()

                    self.last_state = state
                    state = "mapping"

            elif state == 'rotate_left':
                # print("orientation: " + str(self.orientation))
                # print("compass: " + str(self.measures.compass))
                self.update_in_power(0, 0)

                # nao tem orientacao objetivo, calcula
                if self.next_orientation is None:
                    # somar 90 graus
                    # special case -> 180 + 90 = 270 mas tem de ser -90
                    if self.orientation == 180:
                        self.next_orientation = -90

                    else:
                        self.next_orientation = self.orientation + 90

                    # print("orientation target -> " + str(self.next_orientation))

                #else: # tem orientacao objetivo, roda
                if abs(self.measures.compass - self.next_orientation) <= 30:
                    self.rotate("left", 1)

                else:
                    self.rotate("left", 150)

                # verificar se ja rodou o pretendido
                if self.next_orientation == self.direction():
                    state = "mapping"
                    self.next_orientation = None
                    self.orientation = self.direction()

            elif state == 'rotate_right':
                # print("\norientation: " + str(self.orientation))
                # print("compass: " + str(self.measures.compass))
                self.update_in_power(0, 0)

                # nao tem orientacao objetivo, calcula
                if self.next_orientation is None:
                    # subtrair 90 graus
                    # special case -> -90 - 90 = -180, self.directions() retorna sempre +180
                    if self.orientation == -90:
                        self.next_orientation = 180

                    else:
                        self.next_orientation = self.orientation - 90

                    # print("orientation target -> " + str(self.next_orientation))

                #else:   # tem orientacao objetivo, roda
                # caso especial para ver se esta perto da orientacao objetivo
                if self.next_orientation == 180:
                    if abs(self.next_orientation + self.measures.compass) <= 30:
                        self.rotate("right", 1)

                    else:
                        self.rotate("right", 150)

                else:
                    if abs(self.next_orientation - self.measures.compass) <= 30:
                        self.rotate("right", 1)

                    else:
                        self.rotate("right", 150)

                # verificar se ja rodou o pretendido
                if self.next_orientation == self.direction():
                    state = "mapping"
                    self.next_orientation = None
                    self.orientation = self.direction()

            elif state == 'sbinalla':
                # print("orientation: " + str(self.orientation))
                # print("compass: " + str(self.measures.compass))
                self.update_in_power(0, 0)

                # nao tem orientacao objetivo, calcula
                if self.next_orientation is None:
                    if self.orientation == 90 or self.orientation == -90:
                        self.next_orientation = -self.orientation

                    elif self.orientation == 180:
                        self.next_orientation = 0

                    else:
                        self.next_orientation = 180

                    # print("orientation target -> " + str(self.next_orientation))

                #else:  # tem orientacao objetivo, roda
                if abs(self.measures.compass - self.next_orientation) <= 30:
                    self.rotate("left", 1)

                else:
                    self.rotate("left", 150)

                # verificar se ja rodou o pretendido
                if self.next_orientation == self.direction():
                    state = "mapping"
                    self.next_orientation = None
                    self.orientation = self.direction()

            elif state == "mapping":
                print("\n---MAPPING---")
                self.driveMotors(0, 0)
                self.update_in_power(0, 0)

                print("bMy position: ", self.my_pos)
                self.correct_gps(self.last_target)
                
                self.pos = round(self.measures.x - self.initial_pos[0], 2), round(self.measures.y - self.initial_pos[1],
                                                                                2)
                print("Current cell: ", self.last_target)
                print(f"Position -> ({self.pos[0]},{self.pos[1]})")
                print("aMy position: ", self.my_pos)

                # tem target, vai só
                if self.target != ():
                    print("tem target vai só")
                    state = self.next_move(self.last_target, self.target)

                else:
                    # ja tem caminho
                    if len(self.path) > 0:
                        print("i know the way")
                        self.target = self.path.pop()

                    # nao tem target nem path, calcula target
                    else:
                        print("no target, no path")
                        
                        self.clean_not_taken()
                        # verificar ground sensor
                        self.save_spots()

                        # calcular target
                        if self.last_target in self.not_taken_positions.keys():
                            target = self.not_taken_positions[self.last_target].pop()

                        else:
                            print("calcular next target")
                            target = self.calc_next_target(self.turn_priority)

                        # procurar caminho com astar
                        if target is None:
                            
                            if self.not_taken_positions == {} and self.returning == False:
                                print("Mapa feito..")
                                self.returning = True
                                paths = self.get_paths_between_spots()
                                shortest_path, order = self.calc_shortest_path(paths)
                                self.writePath(shortest_path, order)
                                self.writeMap(order)
                                self.take_me_home()

                            elif self.returning and self.last_target == (0, 0):
                                self.finish()
                                print("\n\nTime: ", self.measures.time)
                                return                                

                            else:
                                self.calc_nearest_not_visited()
                            
                            self.target = self.path.pop()

                        else:
                            self.target = target

                        state = self.next_move(self.last_target, self.target)

                print("next state: " + state)
                self.last_state = state
            
            self.gps()

    def go(self):
        left_sensor = self.measures.irSensor[1]
        right_sensor = self.measures.irSensor[2]

        if left_sensor > 3.1:
            self.update_in_power(0.15, 0.135)
            self.driveMotors(0.15, 0.135)
        
        elif right_sensor > 3.1:
            self.update_in_power(0.135, 0.15)
            self.driveMotors(0.135, 0.15)
        
        else: 
            if self.orientation == 180:  # direcao para esquerda
                if self.measures.compass < 0:
                    self.forwards(0.15, 0.01, self.measures.compass, -180)
                elif self.measures.compass > 0:
                    self.forwards(0.15, 0.01, self.measures.compass, 180)
            else:
                self.forwards(0.15, 0.01, self.measures.compass, self.orientation)

    def update_in_power(self, l_power, r_power):
        # self.prev_in_l = self.in_power_l
        # self.prev_in_r = self.in_power_r
        self.in_power_l = l_power
        self.in_power_r = r_power

    def movement(self, previous_out, in_power):
        return (in_power + previous_out) / 2

    def gps(self):
        # out_l = self.movement(self.prev_in_l, self.in_power_l)
        # out_r = self.movement(self.prev_in_r, self.in_power_r)
        out_l = self.movement(self.prev_out_l, self.in_power_l)
        out_r = self.movement(self.prev_out_r, self.in_power_r)
        
        # translation
        # if self.prev_in_l == 0 and self.prev_in_r == 0:
        #     lin = ((out_l + out_r) / 2) / 2
        # else:
        #     lin = (out_l + out_r) / 2
        
        lin = (out_l + out_r) / 2

        self.prev_out_l = out_l
        self.prev_out_r = out_r
        
        self.my_x = self.my_x + lin * cos(radians(self.direction()))
        self.my_y = self.my_y + lin * sin(radians(self.direction()))

        self.my_pos = round(self.my_x, 1), round(self.my_y, 1)

    def correct_gps(self, cell):
        x, y = cell[0], cell[1]
        walls = self.check_walls()

        front_sensor = self.measures.irSensor[0]
        left_sensor = self.measures.irSensor[1]
        right_sensor = self.measures.irSensor[2]
        back_sensor = self.measures.irSensor[3]

        # print("front sensor: ", front_sensor)
        # print("left sensor: ", left_sensor)
        # print("right sensor: ", right_sensor)

        robot_radius = 0.5
        wall_thickness = 0.1

        if self.orientation == 0:  # direita
            if walls[0] == 1:
                wall_position = (x + 1, y)
                self.my_x = wall_position[0] - self.dist_from_sensor(front_sensor) - robot_radius - wall_thickness

            if walls[1] == 1:
                wall_position = (x, y + 1)
                self.my_y = wall_position[1] - self.dist_from_sensor(left_sensor) - robot_radius - wall_thickness

            elif walls[2] == 1:
                wall_position = (x, y - 1)
                self.my_y = wall_position[1] + self.dist_from_sensor(right_sensor) + robot_radius + wall_thickness

        elif self.orientation == 90:  # cima
            if walls[0] == 1:
                wall_position = (x, y + 1)
                self.my_y = wall_position[1] - self.dist_from_sensor(front_sensor) - robot_radius - wall_thickness

            if walls[1] == 1:
                wall_position = (x - 1, y)
                self.my_x = wall_position[0] + self.dist_from_sensor(left_sensor) + robot_radius + wall_thickness

            elif walls[2] == 1:
                wall_position = (x + 1, y)
                self.my_x = wall_position[0] - self.dist_from_sensor(right_sensor) - robot_radius - wall_thickness

        elif self.orientation == -90:  # baixo
            if walls[0] == 1:
                wall_position = (x, y - 1)
                self.my_y = wall_position[1] + self.dist_from_sensor(front_sensor) + robot_radius + wall_thickness

            if walls[1] == 1:
                wall_position = (x + 1, y)
                self.my_x = wall_position[0] - self.dist_from_sensor(left_sensor) - robot_radius - wall_thickness

            elif walls[2] == 1:
                wall_position = (x - 1, y)
                self.my_x = wall_position[0] + self.dist_from_sensor(right_sensor) + robot_radius + wall_thickness

        elif self.orientation == 180:  # esquerda
            if walls[0] == 1:
                wall_position = (x - 1, y)
                self.my_x = wall_position[0] + self.dist_from_sensor(front_sensor) + robot_radius + wall_thickness

            if walls[1] == 1:
                wall_position = (x, y - 1)
                self.my_y = wall_position[1] + self.dist_from_sensor(left_sensor) + robot_radius + wall_thickness

            elif walls[2] == 1:
                wall_position = (x, y + 1)
                self.my_y = wall_position[1] - self.dist_from_sensor(right_sensor) - robot_radius - wall_thickness

        self.my_pos = round(self.my_x, 1), round(self.my_y, 1)
        
    def get_paths_between_spots(self):
        print("\nSpots: ", self.spots)
        number_of_spots = len(self.spots.keys())
        paths = dict()

        for i in range(number_of_spots - 1):
            for j in range(i + 1, number_of_spots):
                temp = astar(self.spots[i], self.spots[j], self.visited_positions, list(self.paredes.keys()))

                if i in paths.keys():
                    paths[i].add((j, len(temp)))
                else:
                    paths[i] = set()
                    paths[i].add((j, len(temp)))

                if j in paths.keys():
                    paths[j].add((i, len(temp)))
                else:
                    paths[j] = set()
                    paths[j].add((i, len(temp)))

        return paths

    def calc_shortest_path(self, paths):
        order = smallPath(paths, 0)

        shortest_path = [self.spots[0]]

        for i in range(len(order) - 1):
            temp = astar(self.spots[order[i]], self.spots[order[i+1]], self.visited_positions, list(self.paredes.keys()))
            shortest_path.extend(temp[::-1])

        temp = astar(self.spots[order[len(order) - 1]], self.spots[order[0]], self.visited_positions, list(self.paredes.keys()))
        shortest_path.extend(temp[::-1])

        return shortest_path, order

    def save_spots(self):
        if self.measures.ground != -1:
            self.spots[self.measures.ground] = self.last_target

    def calc_nearest_not_visited(self):
        nearest = 99
        goal = None
        min_path = None
        for position in self.not_taken_positions.keys():
            path = astar(self.last_target, position, self.visited_positions, list(self.paredes.keys()))

            if len(path) < nearest:
                goal = position
                nearest = len(path)
                min_path = path

        self.path = min_path

        print("\nGoal: ", goal)
        print("Path: ", self.path)

    def take_me_home(self):
        self.path = astar(self.last_target, (0, 0), self.visited_positions, list(self.paredes.keys()))

    def has_reached_target(self, pos, next_pos):
        dist_to_goal = 0.25
        if self.orientation == 0:
            if next_pos[0] - pos[0] <= dist_to_goal:
                return True

        elif self.orientation == 180:
            if pos[0] - next_pos[0] <= dist_to_goal:
                return True

        elif self.orientation == -90:
            if pos[1] - next_pos[1] <= dist_to_goal:
                return True

        elif self.orientation == 90:
            if next_pos[1] - pos[1] <= dist_to_goal:
                return True

        else:
            return False

    def calc_next_target(self, priority):
        # calcular target com base nos caminhos possiveis, guarda os caminhos possiveis nao escolhidos
        x, y = self.last_target[0], self.last_target[1]
        self.not_taken_positions.setdefault((x, y), set())
        temp = self.possible_targets(self.check_walls())
        target = None

        print("temp: ", temp)

        if priority == "front":
            if temp[0] != () and temp[0] not in self.visited_positions:  # frente
                target = temp[0]

                if temp[1] != () and temp[1] not in self.visited_positions:
                    self.not_taken_positions[(x, y)].add(temp[1])

                if temp[2] != () and temp[2] not in self.visited_positions:
                    self.not_taken_positions[(x, y)].add(temp[2])

                if temp[3] != () and temp[3] not in self.visited_positions:
                    self.not_taken_positions[(x, y)].add(temp[3])

            elif temp[2] != () and temp[2] not in self.visited_positions:  # direita
                target = temp[2]

                if temp[1] != () and temp[1] not in self.visited_positions:
                    self.not_taken_positions[(x, y)].add(temp[1])

                if temp[3] != () and temp[3] not in self.visited_positions:
                    self.not_taken_positions[(x, y)].add(temp[3])

            elif temp[1] != () and temp[1] not in self.visited_positions:  # esquerda
                target = temp[1]

                if temp[3] != () and temp[3] not in self.visited_positions:
                    self.not_taken_positions[(x, y)].add(temp[3])

            elif temp[3] != () and temp[3] not in self.visited_positions:   # trás
                target = temp[3]

        elif priority == "turn":
            if temp[2] != () and temp[2] not in self.visited_positions:  # direita
                target = temp[2]

                if temp[1] != () and temp[1] not in self.visited_positions:
                    self.not_taken_positions[(x, y)].add(temp[1])

                if temp[0] != () and temp[0] not in self.visited_positions:
                    self.not_taken_positions[(x, y)].add(temp[0])

                if temp[3] != () and temp[3] not in self.visited_positions:
                    self.not_taken_positions[(x, y)].add(temp[3])

            elif temp[1] != () and temp[1] not in self.visited_positions:  # esquerda
                target = temp[1]

                if temp[0] != () and temp[0] not in self.visited_positions:
                    self.not_taken_positions[(x, y)].add(temp[0])

                if temp[3] != () and temp[3] not in self.visited_positions:
                    self.not_taken_positions[(x, y)].add(temp[3])

            elif temp[0] != () and temp[0] not in self.visited_positions:  # frente
                target = temp[0]

                if temp[3] != () and temp[3] not in self.visited_positions:
                    self.not_taken_positions[(x, y)].add(temp[3])

            elif temp[3] != () and temp[3] not in self.visited_positions:
                target = temp[3]


        if len(self.not_taken_positions[(x, y)]) == 0:
            self.not_taken_positions.pop((x, y), None)

        if target is not None:
            print(f"\ntarget -> ({target[0]},{target[1]})")

        return target

    def clean_not_taken(self):
        # tirar posicao onde estou dos not_taken
        # apagar posicoes que ja nao tem nenhum caminho que ja nao tenha sido percorrido
        to_remove = []

        for key in self.not_taken_positions.keys():
            if self.last_target in self.not_taken_positions[key]:
                self.not_taken_positions[key].discard(self.last_target)

            if len(self.not_taken_positions[key]) == 0:
                to_remove.append(key)

        print("To remove: ", to_remove)
        for key in to_remove:
            self.not_taken_positions.pop(key, None)

    # ver direção do bicho para calcular next_pos
    def direction(self):
        bussola = self.measures.compass

        if -5 < bussola < 5:
            return 0

        elif 85 < bussola < 95:
            return 90

        elif (bussola < -175) or (bussola > 175):
            return 180

        elif -95 < bussola < -85:
            return -90

        else:
            return bussola

    def next_move(self, position, target):
        # calcular next state com base no target
        diff = position[0] - target[0], position[1] - target[1]

        if self.orientation == 0:  # direita
            if diff[0] == -2 and diff[1] == 0:
                return "go_ahead"

            elif diff[0] == 0 and diff[1] == -2:
                return "rotate_left"

            elif diff[0] == 0 and diff[1] == 2:
                return "rotate_right"

            else:
                return "sbinalla"

        elif self.orientation == 90:  # cima
            if diff[0] == -2 and diff[1] == 0:
                return "rotate_right"

            elif diff[0] == 0 and diff[1] == -2:
                return "go_ahead"

            elif diff[0] == 2 and diff[1] == 0:
                return "rotate_left"

            else:
                return "sbinalla"

        elif self.orientation == -90:  # baixo
            if diff[0] == 0 and diff[1] == 2:
                return "go_ahead"

            elif diff[0] == 2 and diff[1] == 0:
                return "rotate_right"

            elif diff[0] == -2 and diff[1] == 0:
                return "rotate_left"

            else:
                return "sbinalla"

        elif self.orientation == 180:  # esquerda
            if diff[0] == 2 and diff[1] == 0:
                return "go_ahead"

            elif diff[0] == 0 and diff[1] == 2:
                return "rotate_left"

            elif diff[0] == 0 and diff[1] == -2:
                return "rotate_right"

            else:
                return "sbinalla"

        return None

    def possible_targets(self, walls):
        x, y = self.last_target[0], self.last_target[1]
        ways = [(), (), (), ()]
        self.mapa[(28 + x, 14 - y)] = "X"

        if self.orientation == 0:  # direita
            front = (28 + (x + 1), 14 - y)
            behind = (28 + (x - 1), 14 - y)
            left = (28 + x, 14 - (y + 1))
            right = (28 + x, 14 - (y - 1))

            if walls[0] == 0:
                free_pos = (x + 2, y)
                ways[0] = free_pos
                self.mapa[front] = "X"
                self.paredes.pop((x + 1, y), None)
            else:
                self.mapa[front] = "|"
                self.paredes[(x + 1, y)] = "|"

            if walls[1] == 0:
                free_pos = (x, y + 2)
                ways[1] = free_pos
                self.mapa[left] = "X"
                self.paredes.pop((x, y + 1), None)
            else:
                self.mapa[left] = "-"
                self.paredes[(x, y + 1)] = "-"

            if walls[2] == 0:
                free_pos = (x, y - 2)
                ways[2] = free_pos
                self.mapa[right] = "X"
                self.paredes.pop((x, y - 1), None)
            else:
                self.mapa[right] = "-"
                self.paredes[(x, y - 1)] = "-"

            if walls[3] == 0: 
                ways[3] = (x - 2, y)
                self.mapa[behind] = "X"
                self.paredes.pop((x - 1, y), None)
            else:
                self.mapa[behind] = "-"
                self.paredes[(x - 1, y)] = "-"

        elif self.orientation == 90:  # cima
            front = (28 + x, 14 - (y + 1))
            behind = (28 + x, 14 - (y - 1))
            left = (28 + (x - 1), 14 - y)
            right = (28 + (x + 1), 14 - y)

            if walls[0] == 0:
                free_pos = (x, y + 2)
                ways[0] = free_pos
                self.mapa[front] = "X"
                self.paredes.pop((x, y + 1), None)
            else:
                self.mapa[front] = "-"
                self.paredes[(x, y + 1)] = "-"

            if walls[1] == 0:
                free_pos = (x - 2, y)
                ways[1] = free_pos
                self.mapa[left] = "X"
                self.paredes.pop((x - 1, y), None)
            else:
                self.mapa[left] = "|"
                self.paredes[(x - 1, y)] = "|"

            if walls[2] == 0:
                free_pos = (x + 2, y)
                ways[2] = free_pos
                self.mapa[right] = "X"
                self.paredes.pop((x + 1, y), None)
            else:
                self.mapa[right] = "|"
                self.paredes[(x + 1, y)] = "|"

            if walls[3] == 0: 
                ways[3] = (x, y - 2)
                self.mapa[behind] = "X"
                self.paredes.pop((x, y - 1), None)
            else:
                self.mapa[behind] = "-"
                self.paredes[(x, y - 1)] = "-"

        elif self.orientation == -90:  # baixo
            front = (28 + x, 14 - (y - 1))
            behind = (28 + x, 14 - (y + 1))
            left = (28 + (x + 1), 14 - y)
            right = (28 + (x - 1), 14 - y)

            if walls[0] == 0:
                free_pos = (x, y - 2)
                ways[0] = free_pos
                self.mapa[front] = "X"
                self.paredes.pop((x, y - 1), None)
            else:
                self.mapa[front] = "-"
                self.paredes[(x, y - 1)] = "-"

            if walls[1] == 0:
                free_pos = (x + 2, y)
                ways[1] = free_pos
                self.mapa[left] = "X"
                self.paredes.pop((x + 1, y), None)
            else:
                self.mapa[left] = "|"
                self.paredes[(x + 1, y)] = "|"

            if walls[2] == 0:
                free_pos = (x - 2, y)
                ways[2] = free_pos
                self.mapa[right] = "X"
                self.paredes.pop((x - 1, y), None)
            else:
                self.mapa[right] = "|"
                self.paredes[(x - 1, y)] = "|"
            
            if walls[3] == 0: 
                ways[3] = (x, y + 2)
                self.mapa[behind] = "X"
                self.paredes.pop((x, y + 1), None)
            else:
                self.mapa[behind] = "-"
                self.paredes[(x, y + 1)] = "-"

        elif self.orientation == 180:  # esquerda
            front = (28 + (x - 1), 14 - y)
            behind = (28 + (x + 1), 14 - y)
            left = (28 + x, 14 - (y - 1))
            right = (28 + x, 14 - (y + 1))

            if walls[0] == 0:
                free_pos = (x - 2, y)
                ways[0] = free_pos
                self.mapa[front] = "X"
                self.paredes.pop((x - 1, y), None)
            else:
                self.mapa[front] = "|"
                self.paredes[(x - 1, y)] = "|"

            if walls[1] == 0:
                free_pos = (x, y - 2)
                ways[1] = free_pos
                self.mapa[left] = "X"
                self.paredes.pop((x, y - 1), None)
            else:
                self.mapa[left] = "-"
                self.paredes[(x, y - 1)] = "-"

            if walls[2] == 0:
                free_pos = (x, y + 2)
                ways[2] = free_pos
                self.mapa[right] = "X"
                self.paredes.pop((x, y + 1), None)
            else:
                self.mapa[right] = "-"
                self.paredes[(x, y + 1)] = "-"

            if walls[3] == 0: 
                ways[3] = (x + 2, y)
                self.mapa[behind] = "X"
                self.paredes.pop((x + 1, y), None)
            else:
                self.mapa[behind] = "-"
                self.paredes[(x + 1, y)] = "-"

        return ways

    def check_walls(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        front_sensor = self.measures.irSensor[center_id]
        left_sensor = self.measures.irSensor[left_id]
        right_sensor = self.measures.irSensor[right_id]
        back_sensor = self.measures.irSensor[back_id]

        # print("front: ", front_sensor)
        # print("left: ", left_sensor)
        # print("right: ", right_sensor)

        # frente, esquerda, direita, trás
        # 0 - livre
        # 1 - parede
        walls = [0, 0, 0, 0]

        if front_sensor >= 1:
            walls[0] = 1

        if left_sensor >= 1.2:
            walls[1] = 1

        if right_sensor >= 1.2:
            walls[2] = 1

        if back_sensor >= 1:
            walls[3] = 1

        return walls

    def forwards(self, lin, k, m, r):
        rot = k * (m - r)

        r_power = lin - (rot / 2)
        l_power = lin + (rot / 2)

        self.update_in_power(l_power, r_power)

        # print(f'motors({l_power}, {r_power})')
        self.driveMotors(l_power, r_power)

    def rotate(self, direction, vel_factor):
        power = 0.01 * vel_factor

        if direction == "left":
            self.driveMotors(-power, power)

        elif direction == "right":
            self.driveMotors(power, -power)

    def dist_from_sensor(self, sensor_value):
        return (1 / sensor_value)

    def writePath(self, path, order):
        file = open("path.out", 'w')

        for i in range(len(path)):
            if path[i] in self.spots.values():

                for key, value in self.spots.items():
                    if value == path[i]:
                        if key == 0:
                            file.write(str(path[i][0]) + " " + str(path[i][1]))

                        else:
                            file.write(str(path[i][0]) + " " + str(path[i][1]) + " #" + str(key))
                        
                        break

            else:
                file.write(str(path[i][0]) + " " + str(path[i][1]))
            
            file.write('\n')
        file.close()
    
    def writeMap(self, spots_order):
        file = open("map.out", 'w')
        lista = list(self.mapa.keys())

        # posicao inicial
        self.mapa[(28, 14)] = "I"

        # spots
        for key, value in self.spots.items():
            self.mapa[(28 + value[0], 14 - value[1])] = str(key)

        for i in range(1, 28):
            for j in range(1, 56):
                if (j, i) in self.mapa.keys():
                    file.write(self.mapa.get((j, i)))
                else:
                    file.write(' ')
            file.write('\n')
        
        # write beacons of shortest path
        # st = ""
        # for i in spots_order:
        #     st = st + " " + str(i)
        
        # st += " 0"
        # file.write(st)

        file.close()

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS * 2 - 1) for i in range(CELLROWS * 2 - 1)]
        i = 1
        for child in root.iter('Row'):
            line = child.attrib['Pattern']
            row = int(child.attrib['Pos'])
            if row % 2 == 0:  # this line defines vertical lines
                for c in range(len(line)):
                    if (c + 1) % 3 == 0:
                        if line[c] == '|':
                            self.labMap[row][(c + 1) // 3 * 2 - 1] = '|'
                        else:
                            None
            else:  # this line defines horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        if line[c] == '-':
                            self.labMap[row][c // 3 * 2] = '-'
                        else:
                            None

            i = i + 1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv), 2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 180.0], host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
