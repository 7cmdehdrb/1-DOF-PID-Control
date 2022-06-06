import csv
from matplotlib import pyplot as plt
import numpy as np


servos = []
loads = []


with open("/home/acca/catkin_ws/src/PID/src/datasheet.csv", "r") as csvFile:
    reader = csv.reader(csvFile, delimiter=",")

    for row in reader:
        # servos.append(int(row[0]))
        # loads.append(float(row[1]))

        new_servo = int(row[0])
        new_load = float(row[1])

        if 60 <= new_servo and new_servo <= 100:

            servos.append(new_servo)
            loads.append(new_load)


plt.scatter(loads, servos, c="r", s=5)
# plt.plot(xs, ys, c="b")

plt.xlabel("load(lbs)")
plt.ylabel("servo(')")

plt.show()
