from math import *
import emptyWorld
import Robot
import World

# Roboter in einer Welt positionieren:
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 2, 5.5, pi / 2)

# Anzahl Zeitschritte n mit jeweils der Laenge T = 0.1 sec definieren.
# T laesst sich ueber die Methode myRobot.setTimeStep(T) einstellen.
# T = 0.1 sec ist voreingestellt.
n = 1500

# Definiere Folge von Bewegungsbefehle:
motionCircle = [[1, -24 * pi / 180] for i in range(n)]

# Bewege Roboter
for t in range(n):
    # Bewege Roboter
    motion = motionCircle[t]
    print "v = ", motion[0], "omega = ", motion[1]*180/pi
    myRobot.move(motion)

    # Gib Daten vom Distanzsensor aus:
    distanceSensorData = myRobot.sense()
    print "Dist Sensor: ", distanceSensorData

    # Gib Daten vom Boxsensor aus:
    boxSensorData = myRobot.senseBoxes()
    if boxSensorData != None:
        print "Box Sensor: ", boxSensorData


# Simulation schliessen:
myWorld.close()
