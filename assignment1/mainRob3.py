import sys

from astar import *
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS = 7
CELLCOLS = 14


class MyRob(CRobLinkAngs):
    initial_pos = ()  # posicao inicial
    last_target = ()  # posicao target anterior
    pos = ()  # posicao atual
    target = ()  # posicao target
    orientation = None  # orientacao desde a ultima rotacao
    next_orientation = None  # orientacao objetivo
    visited_positions = set()  # set com todas as posicoes ja visitadas
    not_taken_positions = {}  # set com posicoes conhecidas nao visitadas
    path = []
    i_know_the_way = True if path != [] and path is not None else False
    paredes = dict()
    mapa = dict()
    spots = dict()

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

            if state == 'stop' and self.measures.start:
                state = stopped_state
                self.initial_pos = (self.measures.x, self.measures.y)
                print(f"Initial pos -> ({self.initial_pos[0]},{self.initial_pos[1]})")
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

                print(f"\nPosition -> ({self.pos[0]},{self.pos[1]})")

                if self.target != ():
                    print(f"target -> ({self.target[0]},{self.target[1]})")

                # ainda nao chegou ao objetivo, anda
                if self.orientation == 180:  # andar para esquerda
                    if self.measures.compass < 0:
                        self.forwards(0.15, 0.01, self.measures.compass, -180)
                    elif self.measures.compass > 0:
                        self.forwards(0.15, 0.01, self.measures.compass, 180)
                else:
                    self.forwards(0.15, 0.01, self.measures.compass, self.orientation)

                # verificar se ja chegou à posicao objetivo
                if self.has_reached_target(self.pos, self.target):
                    print("\n\n Cheguei ao target")

                    self.visited_positions.add(self.target)
                    self.last_target = self.target
                    self.target = ()

                    state = "mapping"

            elif state == 'rotate_left':
                print("orientation: " + str(self.orientation))
                print("compass: " + str(self.measures.compass))

                if self.next_orientation is None:
                    # somar 90 graus
                    # special case -> 180 + 90 = 270 mas tem de ser -90
                    if self.orientation == 180:
                        self.next_orientation = -90

                    else:
                        self.next_orientation = self.orientation + 90

                    print("orientation target -> " + str(self.next_orientation))

                else: # tem orientacao objetivo, roda
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
                print("\norientation: " + str(self.orientation))
                print("compass: " + str(self.measures.compass))

                if self.next_orientation is None:
                    # subtrair 90 graus
                    # special case -> -90 - 90 = -180, self.directions() retorna sempre +180
                    if self.orientation == -90:
                        self.next_orientation = 180

                    else:
                        self.next_orientation = self.orientation - 90

                    print("orientation target -> " + str(self.next_orientation))

                else:   # tem orientacao objetivo, roda
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

                    print('rotate_right function')

            elif state == 'sbinalla':
                print("orientation: " + str(self.orientation))
                print("compass: " + str(self.measures.compass))

                if self.next_orientation is None:
                    if self.orientation == 90 or self.orientation == -90:
                        self.next_orientation = -self.orientation

                    elif self.orientation == 180:
                        self.next_orientation = 0

                    else:
                        self.next_orientation = 180

                    print("orientation target -> " + str(self.next_orientation))

                else:  # tem orientacao objetivo, roda
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
                print("Current position: ", self.last_target)

                self.driveMotors(0, 0)
                self.clean_not_taken()
                # verificar ground sensor
                self.save_spots()

                # tem target, vai só
                if self.target != ():
                    print("tem target vai só")
                    state = self.next_move(self.last_target, self.target)

                else:
                    if self.i_know_the_way:
                        print("i know the way")
                        self.target = self.path.pop()

                    # nao tem target nem path, calcula target
                    else:
                        print("no target, no path")

                        if self.not_taken_positions == {} and len(self.visited_positions) >= 10:
                            paths = self.get_paths_between_spots()
                            shortest_path, order = self.calc_shortest_path(paths)
                            self.writePath(shortest_path, order)
                            print("Time: ", self.measures.time)
                            self.finish()
                            return

                        if self.last_target in self.not_taken_positions.keys():
                            target = self.not_taken_positions[self.last_target].pop()

                        else:
                            print("calcular next target")
                            target = self.calc_next_target()

                        if target is None:  # procurar caminho com astar
                            # a*
                            self.calc_nearest_not_visited()
                            self.target = self.path.pop()

                        else:
                            self.target = target

                        state = self.next_move(self.last_target, self.target)

                print("next state: " + state)

            elif state == 'end':
                self.driveMotors(0, 0)
                print("state end")

    def get_paths_between_spots(self):
        print("\nSpots: ", self.spots)
        number_of_spots = len(self.spots.keys())
        paths = dict()

        for i in range(number_of_spots - 1):
            for j in range(i + 1, number_of_spots):
                temp = astar(self.spots[i], self.spots[j], self.visited_positions, self.paredes)

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
            temp = astar(self.spots[order[i]], self.spots[order[i+1]], self.visited_positions, self.paredes)
            shortest_path.extend(temp[::-1])

        temp = astar(self.spots[order[len(order) - 1]], self.spots[order[0]], self.visited_positions, self.paredes)
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

    def has_reached_target(self, pos, next_pos):
        if self.orientation == 0:
            if next_pos[0] - pos[0] <= 0.35:
                return True

        elif self.orientation == 180:
            if pos[0] - next_pos[0] <= 0.35:
                return True

        elif self.orientation == -90:
            if pos[1] - next_pos[1] <= 0.35:
                return True

        elif self.orientation == 90:
            if next_pos[1] - pos[1] <= 0.35:
                return True

        else:
            return False

    def calc_next_target(self):
        # calcular target com base nos caminhos possiveis, guarda os caminhos possiveis nao escolhidos
        x, y = self.last_target[0], self.last_target[1]
        self.not_taken_positions.setdefault((x, y), set())
        temp = self.possible_targets(self.check_walls())
        target = None

        print("temp: ", temp)

        if temp[0] != () and temp[0] not in self.visited_positions:  # frente
            target = temp[0]

            if temp[1] != () and temp[1] not in self.visited_positions:
                self.not_taken_positions[(x, y)].add(temp[1])

            if temp[2] != () and temp[2] not in self.visited_positions:
                self.not_taken_positions[(x, y)].add(temp[2])

        elif temp[2] != () and temp[2] not in self.visited_positions:  # direita
            target = temp[2]

            if temp[1] != () and temp[1] not in self.visited_positions:
                self.not_taken_positions[(x, y)].add(temp[1])

        elif temp[1] != () and temp[1] not in self.visited_positions:  # esquerda
            target = temp[1]

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
        ways = [(), (), ()]
        self.mapa[(28 + x, 14 - y)] = "X"

        if self.orientation == 0:  # direita
            if walls[0] == 0:
                free_pos = (x + 2, y)
                ways[0] = free_pos
                self.mapa[(28 + x + 1, 14 - y)] = "X"
            else:
                self.mapa[(28 + x + 1, 14 - y)] = "|"
                self.paredes[(x + 1, y)] = "|"

            if walls[1] == 0:
                free_pos = (x, y + 2)
                ways[1] = free_pos
                self.mapa[(28 + x, 14 - (y + 1))] = "X"
            else:
                self.mapa[(28 + x, 14 - (y + 1))] = "-"
                self.paredes[(x, y + 1)] = "-"

            if walls[2] == 0:
                free_pos = (x, y - 2)
                ways[2] = free_pos
                self.mapa[(28 + x, 14 - (y - 1))] = "X"
            else:
                self.mapa[(28 + x, 14 - (y - 1))] = "-"
                self.paredes[(x, y - 1)] = "-"

        elif self.orientation == 90:  # cima
            if walls[0] == 0:
                free_pos = (x, y + 2)
                ways[0] = free_pos
                self.mapa[(28 + x, 14 - (y + 1))] = "X"
            else:
                self.mapa[(28 + x, 14 - (y + 1))] = "-"
                self.paredes[(x, y + 1)] = "-"

            if walls[1] == 0:
                free_pos = (x - 2, y)
                ways[1] = free_pos
                self.mapa[(28 + (x - 1), 14 - y)] = "X"
            else:
                self.mapa[(28 + (x - 1), 14 - y)] = "|"
                self.paredes[(x - 1, y)] = "|"

            if walls[2] == 0:
                free_pos = (x + 2, y)
                ways[2] = free_pos
                self.mapa[(28 + (x + 1), 14 - y)] = "X"
            else:
                self.mapa[(28 + (x + 1), 14 - y)] = "|"
                self.paredes[(x + 1, y)] = "|"

        elif self.orientation == -90:  # baixo
            if walls[0] == 0:
                free_pos = (x, y - 2)
                ways[0] = free_pos
                self.mapa[(28 + x, 14 - (y - 1))] = "X"
            else:
                self.mapa[(28 + x, 14 - (y - 1))] = "-"
                self.paredes[(x, y - 1)] = "-"

            if walls[1] == 0:
                free_pos = (x + 2, y)
                ways[1] = free_pos
                self.mapa[(28 + x + 1, 14 - y)] = "X"
            else:
                self.mapa[(28 + x + 1, 14 - y)] = "|"
                self.paredes[(x + 1, y)] = "|"

            if walls[2] == 0:
                free_pos = (x - 2, y)
                ways[2] = free_pos
                self.mapa[(28 + (x - 1), 14 - y)] = "X"
            else:
                self.mapa[(28 + (x - 1), 14 - y)] = "|"
                self.paredes[(x - 1, y)] = "|"

        elif self.orientation == 180:  # esquerda
            if walls[0] == 0:
                free_pos = (x - 2, y)
                ways[0] = free_pos
                self.mapa[(28 + (x - 1), 14 - y)] = "X"
            else:
                self.mapa[(28 + (x - 1), 14 - y)] = "|"
                self.paredes[(x - 1, y)] = "|"

            if walls[1] == 0:
                free_pos = (x, y - 2)
                ways[1] = free_pos
                self.mapa[(28 + x, 14 - (y - 1))] = "X"
            else:
                self.mapa[(28 + x, 14 - (y - 1))] = "-"
                self.paredes[(x, y - 1)] = "-"

            if walls[2] == 0:
                free_pos = (x, y + 2)
                ways[2] = free_pos
                self.mapa[(28 + x, 14 - (y + 1))] = "X"
            else:
                self.mapa[(28 + x, 14 - (y + 1))] = "-"
                self.paredes[(x, y + 1)] = "-"

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

        if back_sensor >= 1.2:
            walls[3] = 1

        return walls

    def forwards(self, lin, k, m, r):
        rot = k * (m - r)

        r_power = lin - (rot / 2)
        l_power = lin + (rot / 2)

        print(f'motors({l_power}, {r_power})')
        self.driveMotors(l_power, r_power)

    def rotate(self, direction, vel_factor):
        power = 0.01 * vel_factor

        if direction == "left":
            self.driveMotors(-power, power)

        elif direction == "right":
            self.driveMotors(power, -power)

    def writePath(self, path, order):
        file = open("path-c3.out", 'w')

        for i in range(len(path)):
            if path[i] in self.spots.values():

                for key in self.spots.keys():
                    if key == 0:
                        file.write(str(path[i][0]) + " " + str(path[i][1]))
                        file.write('\n')
                    elif self.spots[key] == path[i] and key !=0:
                        file.write(str(path[i][0]) + " " + str(path[i][1]) + " #" + str(key))
                        file.write('\n')
            else:
                file.write(str(path[i][0]) + " " + str(path[i][1]))
                file.write('\n')
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
ficheiro = ""

for i in range(1, len(sys.argv), 2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--file" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        fichiero = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 180.0], host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
