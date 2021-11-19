import sys

from astar import astar
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
    i_know_the_way = True if path != [] else False
    mapa=dict()

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
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state
                self.initial_pos = (self.measures.x, self.measures.y)
                print(f"Initial pos -> ({self.initial_pos[0]},{self.initial_pos[1]})")
                self.pos = self.initial_pos
                self.last_target = self.initial_pos
                self.orientation = self.direction()
                self.visited_positions.add(self.initial_pos)

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'go_ahead':
                self.pos = (self.measures.x, self.measures.y)

                print(f"\nPosition -> ({self.pos[0]},{self.pos[1]})")

                if self.target != ():
                    print(f"target -> ({self.target[0]},{self.target[1]})")

                print("orientation: " + str(self.orientation))
                print("compass: " + str(self.measures.compass))

                # nao tem target, rip
                if self.target == ():
                    print("No target!")
                    state = "end"

                # ainda nao chegou ao objetivo, anda
                else:
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

                        for key, list in self.not_taken_positions.items():
                            for pos in list:
                                if self.target == pos:
                                    print("Removing pos from not taken: ", pos)
                                    self.not_taken_positions[key].remove(pos)

                        self.last_target = self.target
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

                elif self.next_orientation == self.direction():
                    state = "mapping"
                    self.next_orientation = None
                    self.orientation = self.direction()

                else:
                    if abs(self.measures.compass - self.next_orientation) <= 40:
                        self.rotate("left", 2)

                    else:
                        self.rotate("left", 10)

                    print('rotate_left function')

            elif state == 'rotate_right':
                print("orientation: " + str(self.orientation))
                print("compass: " + str(self.measures.compass))

                if self.next_orientation is None:
                    # subtrair 90 graus
                    # special case -> -90 - 90 = -180, self.directions() retorna sempre +180
                    if self.orientation == -90:
                        self.next_orientation = 180

                    else:
                        self.next_orientation = self.orientation - 90

                    print("orientation target -> " + str(self.next_orientation))

                elif self.next_orientation == self.direction():
                    state = "mapping"
                    self.next_orientation = None
                    self.orientation = self.direction()

                else:
                    if abs(self.measures.compass - self.next_orientation) <= 40:
                        self.rotate("right", 2)

                    else:
                        self.rotate("right", 10)

                    print('rotate_right function')

            elif state == "mapping":
                print("\n---MAPPING---")
                self.driveMotors(0, 0)

                target = self.calc_next_target()

                if target is None:
                    # a*
                    goal = self.calc_nearest_not_visited()
                    self.path = astar(self.last_target, goal, self.visited_positions, "walls")
                else:
                    self.target = target
                    state = self.next_move(self.last_target, self.target)
                    print("next state: " + state)

            elif state == 'end':
                self.driveMotors(0, 0)
                print("state end")

            elif state == "the-way":
                self.driveMotors(0, 0)
                print("state the-way")

    def calc_nearest_not_visited(self):
        # TODO calcular posicao nao visitada mais perto
        return self.not_taken_positions.keys()[0]

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

        x, y = self.last_target[0], self.last_target[1]
        self.not_taken_positions.setdefault((x, y), set())

        # calcular target com base nos caminhos possiveis, guarda os caminhos possiveis nao escolhidos
        temp = self.possible_targets(self.check_walls())
        if temp[0] != () and temp[0] not in self.visited_positions:  # frente
            target = temp[0]

            if temp[1] != () and temp[1] not in self.visited_positions:
                self.not_taken_positions[(x, y)].add(temp[1])

            if temp[2] != () and temp[2] not in self.visited_positions:
                self.not_taken_positions[(x, y)].add(temp[2])

        elif temp[1] != () and temp[1] not in self.visited_positions:  # esquerda
            target = temp[1]

            if temp[2] != () and temp[2] not in self.visited_positions:
                self.not_taken_positions[(x, y)].add(temp[2])

        elif temp[2] != () and temp[2] not in self.visited_positions:  # direita
            target = temp[2]

        self.clean_not_taken()

        print(f"\ntarget -> ({target[0]},{target[1]})")
        print("\nNot taken: ", self.not_taken_positions)
        return target

    def clean_not_taken(self):
        to_remove = []
        for key, sett in self.not_taken_positions.items():
            if len(sett) == 0:
                to_remove.append(key)
                continue

            for pos in sett:
                if self.target == pos:
                    print("Removing pos from not taken: ", pos)
                    self.not_taken_positions[key].remove(pos)

        print("To remove: ", to_remove)
        for i in to_remove:
            self.not_taken_positions.pop(i)

    # ver direção do bicho para calcular next_pos
    def direction(self):
        bussola = self.measures.compass

        if -10 < bussola < 10:
            return 0

        elif 80 < bussola < 100:
            return 90

        elif (bussola < -170) or (bussola > 170):
            return 180

        elif -100 < bussola < -80:
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

        elif self.orientation == 90:  # cima
            if diff[0] == -2 and diff[1] == 0:
                return "rotate_right"

            elif diff[0] == 0 and diff[1] == -2:
                return "go_ahead"

            elif diff[0] == 2 and diff[1] == 0:
                return "rotate_left"

        elif self.orientation == -90:  # baixo
            if diff[0] == 0 and diff[1] == 2:
                return "go_ahead"

            elif diff[0] == 2 and diff[1] == 0:
                return "rotate_right"

            elif diff[0] == -2 and diff[1] == 0:
                return "rotate_left"

        elif self.orientation == 180:  # esquerda
            if diff[0] == 2 and diff[1] == 0:
                return "go_ahead"

            elif diff[0] == 0 and diff[1] == 2:
                return "rotate_left"

            elif diff[0] == 0 and diff[1] == -2:
                return "rotate_right"

        return None

    def possible_targets(self, walls):
        x, y = self.last_target[0], self.last_target[1]
        ways = [(), (), ()]

        if self.orientation == 0:  # direita
            if walls[0] == 0:
                free_pos = (x + 2, y)
                ways[0] = free_pos
            else:
                self.mapa[(x+1,y)]="|"

            if walls[1] == 0:
                free_pos = (x, y + 2)
                ways[1] = free_pos
            else:
                self.mapa[(x,y+1)]="-"

            if walls[2] == 0:
                free_pos = (x, y - 2)
                ways[2] = free_pos
            else:
                self.mapa[(x,y-1)]="-"

            if walls[3] == 0:
                pass
            else:
                self.mapa[(x-1,y)]="|"

        elif self.orientation == 90:  # cima
            if walls[0] == 0:
                free_pos = (x, y + 2)
                ways[0] = free_pos
            else:
                self.mapa[(x,y+1)]="-"

            if walls[1] == 0:
                free_pos = (x - 2, y)
                ways[1] = free_pos
            else:
                self.mapa[(x-1,y)]="|"


            if walls[2] == 0:
                free_pos = (x + 2, y)
                ways[2] = free_pos
            else:
                self.mapa[(x+1,y)]="|"

            if walls[3] == 1:
                pass
            else:
                self.mapa[(x,y-1)]="-"

        elif self.orientation == -90:  # baixo
            if walls[0] == 0:
                free_pos = (x, y - 2)
                ways[0] = free_pos
            else:
                self.mapa[(x,y-1)]="-"

            if walls[1] == 0:
                free_pos = (x + 2, y)
                ways[1] = free_pos
            else:
                self.mapa[(x+1,y)]="|"

            if walls[2] == 0:
                free_pos = (x - 2, y)
                ways[2] = free_pos
            else:
                self.mapa[(x-1,y)]="|"

            if walls[3] == 0:
                pass
            else:
                self.mapa[(x,y+1)]="|"

        elif self.orientation == 180:  # esquerda
            if walls[0] == 0:
                free_pos = (x - 2, y)
                ways[0] = free_pos
            else:
                self.mapa[(x-1,y)]="|"

            if walls[1] == 0:
                free_pos = (x, y - 2)
                ways[1] = free_pos
            else:
                self.mapa[(x,y-1)]="-"

            if walls[2] == 0:
                free_pos = (x, y + 2)
                ways[2] = free_pos
            else:
                self.mapa[(x,y+1)]="-"

            if walls[3] == 0:
                pass
            else:
                self.mapa[(x+1,y)]="|"

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

        if front_sensor >= 1.3:
            walls[0] = 1

        if left_sensor >= 1.3:
            walls[1] = 1

        if right_sensor >= 1.3:
            walls[2] = 1

        if back_sensor >= 1.3:
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

    def turnBack(self, bussola):
        while abs((bussola - self.measures.compass) < 160):
            print("A RODAAAAR")
            print(self.driveMotors(-0.3, 0.3))


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
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
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
