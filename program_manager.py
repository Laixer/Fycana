import json
import ik
import ec240cl


class Program:
    solver = ik.InverseKinematics(ec240cl.BOOM_LENGTH, ec240cl.ARM_LENGTH)

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def agles(self):
        return self.solver.solve([self.x, self.y, self.z])


class ProgramManager:
    prog_list = []
    prog_active = 0

    def __init__(self, filename):
        file = open(filename)

        data = json.load(file)

        for point in data['steps']:
            self.prog_list.append(Program(point['parameters'][0], point['parameters'][1], point['parameters'][2]))

        file.close()

    def program(self):
        return self.prog_list[self.prog_active]

    def next(self):
        if len(self.prog_list)-1 > self.prog_active:
            self.prog_active = self.prog_active + 1
