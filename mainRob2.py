import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS = 7
CELLCOLS = 14


class MyRob(CRobLinkAngs):
    initial_pos = ()            # posicao inicial
    last_target = ()            # posicao target anterior
    pos = ()                    # posicao atual
    target = ()                 # posicao target
    orientation = None          # orientacao desde a ultima rotacao
    next_orientation = None     # orientacao objetivo
    visited_positions = set()   # set com todas as posicoes ja visitadas

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
        stopped_state = 'go_ahead'

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
                    print(f"next pos -> ({self.target[0]},{self.target[1]})")

                print("orientation: " + str(self.orientation))
                print("compass: " + str(self.measures.compass))

                # nao tem target, calcula
                if self.target == ():
                    self.target = self.calc_next_pos(self.last_target)

                    if self.target is None:
                        print("\n\nCould not calc target")
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

                elif self.next_orientation == self.direction():  # ver caso -180
                    state = "mapping"
                    self.next_orientation = None
                    self.orientation = self.direction()

                else:
                    if abs(self.measures.compass - self.next_orientation) <= 40:
                        self.rotate("right", 2)

                    else:
                        self.rotate("right", 10)

                    print('rotate_right function')

            elif state == 'end':
                self.driveMotors(0, 0)
                print("state end")

            elif state == "mapping":
                print("\n---MAPPING---")
                self.driveMotors(0, 0)
                if self.target != ():
                    self.visited_positions.add(self.target)  # adicionar
                print(self.visited_positions)
                self.target = ()  # reset target

                # TODO mapping das paredes em redor

                state = self.next_state()
                print("next state: " + state)


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

    def calc_next_pos(self, previous_target):
        if self.orientation == 0:
            return (previous_target[0] + 2, previous_target[1])

        elif self.orientation == 180:
            return (previous_target[0] - 2, previous_target[1])

        elif self.orientation == 90:
            return (previous_target[0], previous_target[1] + 2)

        elif self.orientation == -90:  # south
            return (previous_target[0], previous_target[1] - 2)

        else:
            return None  # se nao estiver alinhado com as 4 direcoes esperadas rip :/

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

    def next_state(self):
        # TODO mais tarde -> guardar o caminho que estava livre e nao foi escolhido

        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        front_sensor = self.measures.irSensor[center_id]
        left_sensor = self.measures.irSensor[left_id]
        right_sensor = self.measures.irSensor[right_id]

        print("front_sensor: ", front_sensor)
        print("right_sensor: ", left_sensor)
        print("left_sensor: ", right_sensor)

        print("Walls: ", self.check_walls())

        if self.check_walls()[0] == 0:
            return "go_ahead"
        else:
            print('---Parede em frente, escolher lado---')
            if self.check_walls()[1] == 0:
                return "rotate_left"

            elif self.check_walls()[2] == 0:
                return "rotate_right"

            else:  # voltar para tras
                return "rotate_left"

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
