import csv
from matplotlib import pyplot as plt
import numpy as np


servos = []
loads = []


servos2 = []
loads2 = []


def func(a, b, x):
    return a * x + b


class Data(object):
    def __init__(self):
        self.loads = []
        self.load = 0.

    def getLoad(self):
        self.load = sum(self.loads) / len(self.loads)
        return 0


with open("/home/acca/catkin_ws/src/PID/src/datasheet.csv", "r") as csvFile:
    reader = csv.reader(csvFile, delimiter=",")

    data = Data()
    servo = 0

    for row in reader:
        # servos.append(int(row[0]))
        # loads.append(float(row[1]))

        new_servo = int(row[0])
        new_load = float(row[1])
        data.loads.append(new_load)

        if servo != new_servo:

            data.getLoad()

            if (60 <= servo and servo <= 100):
                servos.append(servo)
                loads.append(data.load)

            data = Data()
            servo = new_servo


with open("/home/acca/catkin_ws/src/PID/src/datasheet.csv", "r") as csvFile:
    reader = csv.reader(csvFile, delimiter=",")

    for row in reader:
        # servos.append(int(row[0]))
        # loads.append(float(row[1]))

        new_servo = int(row[0])
        new_load = float(row[1])

        if 60 <= new_servo and new_servo <= 100:

            servos2.append(new_servo)
            loads2.append(new_load)


fp1 = np.polyfit(servos, loads, 1)

a = fp1[0]
b = fp1[1]

print(a, b)

xs = range(int(min(servos2)), int(max(servos2)), 1)
ys = []

for x in xs:
    ys.append(func(a, b, x))

plt.scatter(servos2, loads2, c="r", s=5)
plt.scatter(servos, loads, c="b", s=40)
plt.plot(xs, ys, c="g")

plt.xlabel("load(lbs)")
plt.ylabel("servo(')")

plt.show()
