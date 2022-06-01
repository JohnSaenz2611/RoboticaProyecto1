
import pprint

class File_reader():
    def __init__(self, path):
        self.path = path
        path_file = open(self.path)
        lines = path_file.read().splitlines()
        self.obstacle_list = []

        (self.q0_x, self.q0_y, self.q0_theta) = self.split_line(lines, 1)
        (self.qf_x, self.qf_y, self.qf_theta) = self.split_line(lines, 2)
        (self.qLoc_x, self.qLoc_y, self.qLoc_theta) = self.split_line(lines, 4)
        self.qDerechaF = self.split_line(lines, 3)[0]
        self.dDerechaL = self.split_line(lines, 5)[0]
        self.dFrenteL = self.split_line(lines, 5)[0]
        for index in range(0, len(lines) - 12, 2):
            splited_line = self.split_line(lines, index + 8)
            splited_line[0] = splited_line[0] * 2# x
            splited_line[1] = 9 - splited_line[1] * 2 # Y
            # splited_line = list(map(lambda x,: x*2, splited_line))
            self.obstacle_list.append(splited_line)

        path_file.close()
        pprint.pprint(lines)

    def split_line(self, lines, line_index) -> list:
        desired_line = ''.join(lines[line_index]).split(',')[1:]
        return list(map(float, desired_line))
