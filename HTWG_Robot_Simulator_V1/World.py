# class World.
#
# This class contains methods to define a world and
# to simulate movements and distance sensor measurements of a robot.
# The world consists just of a set of walls (as line segments).
# Only the walls can be seen by the distance sensor.
#
# A set of red boxes can be placed.
# The boxes can be sensed by a box sensor.
#
# Moreover, a polyline (i.e. planed path) can be drawn in the world.
#
# From the world an occupancy grid can be generated.
#
# O. Bittel
# V 1.1; 20.3.2015


# Changelog
# 19.06.2015: (Phil) Added support for landmark detection, added function to draw points
# 20.06.2015: (Phil) Changed draw_points() to draw_particle(), added draw_number()
# 21.06.2015: (Phil) Updated draw_particle() and draw_number(), added functions to directly undraw a single particle /
#                    number
# 24.06.2015: (Ecki) Refactored sense_landmark to sense_landmarks_in_range, created new function for sense_landmarks

from math import *
import numpy as np
from graphics import *
from CursorController import *
from OccupancyGrid import *
from numbers import Number

import time

class World:

    # --------
    # init: creates a an empty world
    # with the given size in [m]
    #
    def __init__(self, width, height):
        # World size:
        self._width = width
        self._height = height

        # Border lines:
        self._xll = 0.0 # x coord of left lower point
        self._yll = 0.0 # y coord of left lower point
        self._lines = [] # List of lines
        ll = Point(self._xll, self._yll) # lower left
        lr = Point(self._xll+width, self._yll) # lower right
        ur = Point(self._xll+width, self._yll+height) # upper right
        ul = Point(self._xll, self._yll+height) # upper left
        self._lines.append(Line(ll,lr))
        self._lines.append(Line(lr,ur))
        self._lines.append(Line(ur,ul))
        self._lines.append(Line(ul,ll))

        # Boxes:
        self._boxes = []
        self._boxSensor = True
        self._boxSensorAngle = 140 * (pi/180) # 140 degree
        self._boxRadius = 0.1 # box radius
        self._boxesSensedDist = [] # Distance to the sensed boxes
        self._boxesSensedAngles = [] # Angles of the sensed boxes

        # Rooms
        self._rooms = []

        # Define graphic window with coordinates and draw borderline
        self._win = GraphWin("HTWG Robot Simulator", int(800.0*width/height), 800, autoflush=False)
        self._win.setCoords(self._xll-0.1, self._yll-0.3, self._xll+width+0.1, self._yll+height+0.1)
        for l in self._lines:
            l.draw(self._win)

        # Robot (initialization is done in setRobot()
        self._robot = None
        self._robotCircle = None # robot is shown as circle with robot position as center
        self._robotTheta = None # robot's global orientation
        self._robotLine = None # local x-axis of the robot

        # Added 08.07.2015 Estimated position
        self._estimation_circle = None
        self._estimation_theta = None
        self._estimation_line = None

        # Sensor values:
        self._sensorShow = True # Sensor values are shown
        self._sensorDist = [] # Distances to obstacles
        self._sensorPoints = [] # Obstacle points
        self._sensorLines = [] # Sensor beams as lines for drawing

        # Clock:
        self._clockTime = 0.0
        p = Point(self._xll+width/2, self._yll-0.1)
        self._clockTimeText = Text(p,"Clock Time %4.2f" % self._clockTime )
        self._clockTimeText.draw(self._win)

        # Occupancy Grid:
        self._grid = None

        # Path history
        self._showPathHistory = True
        self._pathPoints = []
        self._maxPathPoints = 150
        self._path_resolution = 5  # Timesteps per path-point
        self._pathLine = None
        self._path_counter = 0
        self._drivenDistance = 0.0

        # Drawn Polyline:
        self._drawnPolyline = []

        # Drawn Circles:
        self._drawnCircles = []

        # Added 19.06.2015
        self._particles = []  # [[particle1, circle1], [particle2, circle2], ..., [particle_n, circle_n]]
        self._particle_radius = 0.05
        self._drawn_numbers = []

        self._landmarks = []  # [[number1, landmark1, nr1], ..., [number_n, landmark_n, nr_n]]
        self._landmark_positions = []
        self._landmark_radius = 0.2
        self._landmark_sensor = True
        self._landmark_sensor_angle = 360 * (pi/180) # 360 degrees
        self._landmark_sensed_indexes = []  # Number (index) of the sensed landmark
        self._landmark_sensed_dist = []  # Distance to the sensed landmark
        self._landmark_sensed_angles = []  # Angles of the sensed landmark

    # Added 19.0.2015
    def get_size(self):
        return [self._width, self._height]

    def draw_particle(self, particle, color='black', radius=None, number=None):
        """ Draw given particle
        :param particle:    particle_ref
        :param color:       (string) color
        :param radius:      (float) radius, default=None
        :param number:      (int) number, default=None
        :return:            -
        """

        if radius is None:
            p_radius = self._particle_radius
        else:
            p_radius = radius

        p_x, p_y, p_theta = particle.get_pos()
        p = Circle(Point(p_x, p_y),  p_radius)
        p.setFill(color)
        p.draw(self._win)
        if number is not None:
            number = self.draw_number(p_x + 0.2, p_y, number)
        self._particles.append([particle, p, number])

    def undraw_particle(self, particle):
        """ Undraws given particle and removes it from particle-list
        :param particle:    particle_ref
        :return:            -
        """
        for p in self._particles:
            if p[0] == particle:
                p[1].undraw()
                if p[2] is not None:
                    self.undraw_number(p[2])
                self._particles.remove(p)

    def undraw_partciles(self):
        """ Undraw all particles and clear particle list
        :return: -
        """
        if not self._particles:
            return
        else:
            for p in self._particles:
                p[1].undraw()
            self._particles = []

    def draw_number(self, x, y, number, color='black', size=8):
        nr = Text(Point(x, y), str(int(number)))
        nr.setFill(color)
        nr.setSize(size)
        nr.draw(self._win)
        self._drawn_numbers.append(nr)
        return nr

    def undraw_number(self, number):
        if number in self._drawn_numbers:
            number.undraw()
            self._drawn_numbers.remove(number)

    def undraw_numbers(self):
        for number in self._drawn_numbers:
            number.undraw()
        self._drawn_numbers = []

    # --------
    # Draw a polyline.
    #
    def drawPolyline(self, poly, color = 'green'):
        self.undrawPolyline()
        for n in range(len(poly)-1):
            l = Line(Point(poly[n][0],poly[n][1]),Point(poly[n+1][0],poly[n+1][1]))
            l.draw(self._win)
            l.setFill(color)
            l.setWidth(3)
            self._drawnPolyline.append(l)

    # --------
    # Undraw the polyline.
    #
    def undrawPolyline(self):
        if self._drawnPolyline == []:
            return
        for l in self._drawnPolyline:
            l.undraw()
        self._drawnPolyline = []

    # --------
    # Draw a circle.
    #
    def drawCircle(self, (x, y), color='black', **kwargs):
        self.undrawCircle()
        # Set radius
        r = 0.1
        if 'radius' in kwargs:
            if isinstance(kwargs['radius'], Number):
                r = kwargs['radius']
        # Draw circle
        c = Circle(Point(x, y), r)
        c.draw(self._win)
        c.setFill(color)
        self._drawnCircles.append(c)

    # --------
    # Undraw the cirle.
    #
    def undrawCircle(self):
        if self._drawnCircles == []:
            return
        for c in self._drawnCircles:
            c.undraw()
        self._drawnCircles = []

    # --------
    # add new a new line from point (x0,y0) to (x1,y1)
    #
    def addLine(self, x0, y0, x1, y1):
        l = Line(Point(x0,y0),Point(x1,y1))
        self._lines.append(l)
        l.setWidth(5)
        l.setFill('blue')
        l.draw(self._win)

    # --------
    # add new a new round Box at point (x,y).
    #
    def addBox(self, x, y):
        box = Circle(Point(x,y),self._boxRadius)
        box.draw(self._win)
        self._boxes.append(box)

    # --------
    # Define a new room with name n and center position (x,y).
    #
    def defineRoom(self, n, x, y):
        self._rooms.append([n,x,y])
        t = Text(Point(x,y),n)
        t.draw(self._win)

    # --------
    # Return all rooms.
    #
    def getRooms(self):
        return self._rooms

    # Added 19.06.2015
    def draw_landmark(self, x, y, number=None):
        """ Place a landmark in the world at position(x, y)
        :param x:       coord
        :param y:       coord
        :param number:  (int) landmark number, default=None
        :return: -
        """

        landmark = Circle(Point(x, y), self._landmark_radius)
        landmark.draw(self._win)
        self._landmark_positions.append([x, y])
        if number is None:
            number = len(self._landmarks)
        nr = self.draw_number(x, y, number)
        self._landmarks.append([number, landmark, nr])

    def get_landmark_positions(self):
        """ Return positions of all placed landmarks
        :return: list([x1, y1], [x2, y2], [x_n, y_n])
        """

        if self._landmark_positions:
            return self._landmark_positions
        else:
            return None

    # Added 08.07.2015 (Phil)
    def draw_estimated_position(self, x, y, theta):
        """
        :param x:           x_position
        :param y:           y_position
        :param theta:       theta
        :return:            -
        """

        # Undraw old estimated position
        if self._estimation_circle is not None:
            self._estimation_circle.undraw()
        if self._estimation_line is not None:
            self._estimation_line.undraw()

        # Set estimated position and draw estimated position:
        c = Point(x,y)
        r = self._robot.getSize()/2
        self._estimation_circle = Circle(c, r)
        self._estimation_circle.setFill('yellow')
        self._estimation_theta = theta
        p = Point(x+r*cos(theta), y+r*sin(theta))
        self._estimation_line = Line(c, p) # line shows the estimation's orientation
        self._estimation_line.setFill('black')
        self._estimation_circle.draw(self._win)
        self._estimation_line.draw(self._win)
        self._estimation_line.setWidth(3)

    # --------
    # set the robot at pose (x,y,theta) and draw it.
    #
    def setRobot(self, robot, x, y, theta):
        self._robot = robot
        robot.setWorld(self) # the robot must know his world

        # Set robot and draw robot:
        c = Point(x,y)
        r = robot.getSize()/2
        self._robotCircle = Circle(c,r)
        self._robotTheta = theta
        p = Point(x+r*cos(theta),y+r*sin(theta))
        self._robotLine = Line(c,p) # line shows the robot's orientation
        self._robotCircle.draw(self._win)
        self._robotLine.draw(self._win)
        self._robotLine.setWidth(3)

        # Update status bar
        self._clockTimeText.setText("Clock Time: %4.2f Driven Distance: %4.2f Position: %4.2f, %4.2f, %4.2f "
                                    % (self._clockTime, self._drivenDistance, x, y, theta*180/pi))

        # Show all:
        self._udateWindow()
        print "click in window to start"
        self._win.getMouse() # pause for click in window
        #k = self.win.getKey()
        #print "key "

    # --------
    # get the true robot pose (x,y,theta).
    #
    def getTrueRobotPose(self):
        x = self._robotCircle.getCenter().getX()
        y = self._robotCircle.getCenter().getY()
        theta = self._robotTheta
        return [x,y,theta]

    # --------
    # move the robot in the direction of his self._robotTheta + dTheta/2 by the length of d
    # and then change the robot's orientation by dTheta.
    # The movements takes dt in clock time (dt is only used to change clock time output).
    # If the robot's movements is not possible because of obstacles, the movements will be not
    # performed and False is returned.
    #
    def moveRobot(self, d, dTheta, dT):
        c = self._robotCircle.getCenter()
        r = self._robotCircle.getRadius()
        x = c.getX()
        y = c.getY()
        theta = self._robotTheta
        dx = d*cos(theta+0.5*dTheta)
        dy = d*sin(theta+0.5*dTheta)
        nc = Point(x+dx, y+dy)

        if self.getNearestDistance(nc) < r: # movements is not possible because of obstacles
            print "Robot stalled: ", x, y, theta
            # raw_input("Enter: ")
            return False

        # move robot and draw robot:
        self._robotLine.undraw()
        self._robotCircle.move(dx,dy)
        self._robotTheta = (self._robotTheta + dTheta)%(2*pi)
        p = Point(x+dx+r*cos(self._robotTheta),y+dy+r*sin(self._robotTheta))
        self._robotLine = Line(nc,p)
        self._robotLine.draw(self._win)
        self._robotLine.setWidth(3)

        # Path history:
        self._drivenDistance += d
        if self._showPathHistory:
            self._path_counter += 1
            # Draw one point per x timesteps
            if self._path_counter % self._path_resolution == 0:
                # If _pathPoints length is over self._maxPathPoints, undraw the oldest point
                if len(self._pathPoints) > self._maxPathPoints:
                    self._pathPoints[len(self._pathPoints)-1].undraw()
                    # Resize _pathPoints to self._maxPathPoints elements
                    self._pathPoints = self._pathPoints[:self._maxPathPoints]
                # Insert new Position Point in _pathPoints
                self._pathPoints.insert(0, nc)
                # Draw newest path Point
                self._pathPoints[0].setFill('red')
                self._pathPoints[0].draw(self._win)
        #print x+dx, y+dy, self.robotTheta

         # Clear sensor values, compute new sensor values and draw it:
        self._sensorPoints = []
        self._sensorDist = []
        self.sense()
        self._drawSense()
        self._boxesSensedDist = []
        self._boxesSensedAngles = []
        self.senseBox()

        # Added 19.06.2015
        self._landmark_sensed_indexes = []
        self._landmark_sensed_dist = []
        self._landmark_sensed_angles = []
        self.sense_landmarks_in_range()

        # Update clock and status bar
        self._clockTime += dT
        self._clockTimeText.setText("Clock Time: %4.2f Driven Distance: %4.2f Position: %4.2f, %4.2f, %4.2f "
                                    % (self._clockTime, self._drivenDistance, x+dx, y+dy, self._robotTheta*180/pi))

        # show all
        self._udateWindow()
        return True

    # # Added 03.07.2015 (Phil)
    # def move_particle(self, particle, d, d_theta, dt):
    #     """
    #     :param particle:    particle object
    #     :param d:           driven distance
    #     :param d_theta:     relative rotation
    #     :param dt:          time step
    #     :return:            -
    #     """
    #
    #     # Get current particle position
    #     x, y, theta = particle.get_pos()
    #     dx = d * cos(theta + 0.5 * d_theta)
    #     dy = d * sin(theta + 0.5 * d_theta)
    #     theta_new = (theta + d_theta) % (2 * pi)
    #
    #     # Move particle and draw article:
    #     self.undraw_particle(particle)
    #     self._robotCircle.move(dx,dy)
    #     self._robotTheta = (self._robotTheta + dTheta)%(2*pi)
    #     p = Point(x+dx+r*cos(self._robotTheta),y+dy+r*sin(self._robotTheta))
    #     self._robotLine = Line(nc,p)
    #     self._robotLine.draw(self._win)
    #     self._robotLine.setWidth(3)

    # --------
    # Compute distance values in the given direction of the robot sensors
    # If sensorShow = True, sensor beams are displayed.
    #
    def sense(self):
        if self._sensorDist == []:
            alphas = self._robot.getSensorDirections()
            distMax = self._robot.getMaxSenseValue()
            p = self._robotCircle.getCenter()
            for alpha in alphas:
                theta = (self._robotTheta+alpha) % (2*pi)
                q = self.getNearestIntersectionWithBeam(p,theta)
                #print "p: ", p.getX(), p.getY(), theta
                d = World._dist(p,q)
                #print "q: ", q.getX(), q.getY(), d
                if d > distMax:
                    self._sensorDist.append(None)
                    x = p.getX()+distMax*cos(theta)
                    y = p.getY()+distMax*sin(theta)
                    self._sensorPoints.append(Point(x,y))
                    #print "sensorPoint: ", x, y, "\n"
                else:
                    self._sensorDist.append(d)
                    self._sensorPoints.append(q)
                    #print "sensorPoint: ", q.getX(), q.getY(), "\n"
            self._drawSense()
            # print "time: ", self._clockTime, time.clock()

        return self._sensorDist


    # --------
    # Draw sensor beams.
    #
    def _drawSense(self):
        if not self._sensorShow:
            return

        # Undraw sensor beam lines:
        for l in self._sensorLines:
            l.undraw()

        # Draw new sensor beam lines:
        self._sensorLines = []
        p = self._robotCircle.getCenter()
        for q in self._sensorPoints:
            l = Line(p,q)
            l.setFill('red')
            self._sensorLines.append(l)
            l.draw(self._win)
        self._robotCircle.undraw()
        self._robotLine.undraw()
        self._robotCircle.draw(self._win)
        self._robotLine.draw(self._win)
        self._robotLine.setWidth(3)

    # --------
    # If boxSensor = True, try to detect box and
    # compute distance and orientation to the detected boxes.
    #
    def senseBox(self):
        if self._boxSensor == False:
            return None
        if self._boxesSensedDist == []:
            p = self._robotCircle.getCenter()
            for box in self._boxes:
                box.undraw()
                box.setFill('white')
                pb = box.getCenter()
                theta = atan2(pb.getY()-p.getY(), pb.getX()-p.getX())
                # print 'Box check', self._robotTheta, theta
                # angle to box relative to robot's x axis from [-pi,+pi)
                alphaBox = (self._robotTheta - theta + pi) % (2*pi) - pi
                if abs(alphaBox) <= self._boxSensorAngle/2:
                    ip = self.getNearestIntersectionWithBeam(p, theta)
                    d = World._dist(p, pb)
                    if World._dist(p, ip) > d:
                        # Box can be seen:
                        #print 'Box can be seen', d, alphaBox
                        box.setFill('red')
                        self.boxDist = World._dist(p, pb)
                        self._boxesSensedDist.append(d)
                        self._boxesSensedAngles.append(alphaBox)
                box.draw(self._win)
        #print "senseBox: ", self._boxesSensedDist,self._boxesSensedAngles
        return [self._boxesSensedDist,self._boxesSensedAngles]

    # Added 19.06.2015
    def sense_landmarks_in_range(self):
        """ Sense all landmarks in sensor range
        :return: landmark number, dist to landmark, angle to landmark
        """
        if self._landmark_sensor == False:
            return None
        if self._landmark_sensed_dist == []:
            p = self._robotCircle.getCenter()
            for landmark in self._landmarks:
                landmark[1].undraw()
                landmark[2].undraw()
                landmark[1].setFill('white')
                pb = landmark[1].getCenter()
                theta = atan2(pb.getY()-p.getY(), pb.getX()-p.getX())
                # print 'Box check', self._robotTheta, theta
                # angle to box relative to robot's x axis from [-pi,+pi)
                alpha_landmark = (self._robotTheta - theta + pi) % (2*pi) - pi
                if abs(alpha_landmark) <= self._landmark_sensor_angle/2:
                    ip = self.getNearestIntersectionWithBeam(p, theta)
                    d = World._dist(p, pb)
                    if World._dist(p, ip) > d:
                        # landmark can be seen:
                        #print 'landmark can be seen', d, alphaBox
                        landmark[1].setFill('green')
                        self.landmark_dist = World._dist(p, pb)
                        self._landmark_sensed_indexes.append(landmark[0])
                        self._landmark_sensed_dist.append(d)
                        self._landmark_sensed_angles.append(alpha_landmark)
                # Redraw landmark and number
                landmark[1].draw(self._win)
                landmark[2].draw(self._win)
        # print "sense_landmark: ", self._landmark_sensed_dist,self._landmark_sensed_angles
        return [self._landmark_sensed_indexes, self._landmark_sensed_dist, self._landmark_sensed_angles]

    # Added 24.06.2015
    def sense_landmarks(self):
        """
        senses all landmarks in world
        :return: landmark number, dist to landmark, angle to landmark
        """
        if self._landmark_sensor == False:
            return None
        # get robot position
        pos_robot = self._robotCircle.getCenter()
        landmark_sensed_indexes = []
        landmark_sensed_dist = []
        landmark_sensed_angles = []
        for landmark in self._landmarks:
            pos_lmrk = landmark[1].getCenter()
            theta = atan2(pos_lmrk.getY()-pos_robot.getY(), pos_lmrk.getX()-pos_robot.getX())
            # angle to box relative to robot's x axis from [-pi,+pi)
            theta_lmrk = (self._robotTheta - theta + pi) % (2*pi) - pi
            dist = World._dist(pos_robot, pos_lmrk)
            landmark_sensed_indexes.append(landmark[0])
            landmark_sensed_dist.append(dist)
            landmark_sensed_angles.append(theta_lmrk)
        return [landmark_sensed_indexes, landmark_sensed_dist, landmark_sensed_angles]

    def getCursorController(self):
        return CursorController(self._win)

    # --------
    # update and draw the actual window.
    # If simulation runs to fast then delay wth a time.sleep()
    def _udateWindow(self):
        #time.sleep(0.05)
        self._win.update()
        #self.win.getMouse() # pause for click in window


    def close(self, waitForClick = True):
        if waitForClick:
            print "click in window to close"
            self._win.getMouse() # pause for click in window
        self._win.close()


    # --------
    # compute the nearest intersection point between the beam starting at point p in direction of theta
    # and a line of the world
    #
    def getNearestIntersectionWithBeam(self, p, theta):
        #print "\ntheta= ", theta*180/pi
        if len(self._lines) == 0:
            return None
        dmin = float("inf")
        ip = None
        for line in self._lines:
            #print "line: ", line.getP1().getX(),line.getP1().getY(),line.getP2().getX(),line.getP2().getY()
            q = World._intersectSegmentBeam(p,theta,line)
            if q is not None:
                #print "q =", q.getX(), q.getY(), self._dist(p,q)
                d = World._dist(p,q)
                if d < dmin:
                    dmin = d
                    ip = q
        if ip is None:
            return None
        #l = Line(p,ip)
        #l.setFill('red')
        #l.draw(self.win)
        return ip


    # --------
    # compute the distance to the line of the world which is nearest to p.
    #
    def getNearestDistance(self, p):
        if len(self._lines) == 0:
            return None
        dmin = float("inf")
        for l in self._lines:
            d = World._distPointSegment(p,l)
            #print "Distance to Line: ", l.getP1().getX(), l.getP1().getY(), l.getP2().getX(), l.getP2().getY(), d
            if d < dmin:
                dmin = d
        return dmin


    # --------
    # compute the distance between the points p and q.
    #
    @staticmethod
    def _dist(p, q):
        dx = p.getX()-q.getX()
        dy = p.getY()-q.getY()
        return sqrt(dx*dx+dy*dy)


    # --------
    # compute the distance between point p and segment line.
    #
    @staticmethod
    def _distPointSegment(p, line):
        p1 = line.getP1()
        p2 = line.getP2()
        x1 = p1.getX()
        y1 = p1.getY()
        x2 = p2.getX()
        y2 = p2.getY()
        theta = atan2(y2-y1,x2-x1)+pi/2
        ip = World._intersectSegmentBeam(p, theta, line, oppositeDirectionInclusive = True)
        if ip is None:
            # print "ip = None: "
            d1 = World._dist(p,p1)
            d2 = World._dist(p,p2)
            if d1 <= d2:
                return d1
            else:
                return d2
        else:
            # print "ip: ", ip.getX(), ip.getY()
            return World._dist(ip,p)


    # --------
    # Compute the intersection point between segment line and the beam given by p and theta.
    # If oppositeDirectionInclusive = True, then the intersection point between segment line
    # and the line through point p with grade theta is computed.
    #
    @staticmethod
    def _intersectSegmentBeam(p, theta, line, oppositeDirectionInclusive = False):
        x0 = p.getX()
        y0 = p.getY()
        x1 = line.getP1().getX()
        y1 = line.getP1().getY()
        x2 = line.getP2().getX()
        y2 = line.getP2().getY()

        delta = fabs(atan2(y2-y1,x2-x1)-theta)
        if delta < 1.0e-03 or fabs(delta-pi) < 1.0e-03:
            # line and beam are nearly parallel
            # print 'parallel'
            return None

        a = np.array([[x2-x1, -cos(theta)],
                      [y2-y1, -sin(theta)]])
        b = np.array([x0-x1,
                      y0-y1])
        k = np.linalg.solve(a, b)
        k0 = k[0]
        k1 = k[1]
        if k0 < 0 or k0 > 1:
            return None
        if not oppositeDirectionInclusive and k1 < 0:
            return None
        ip = Point(x1 + k0*(x2-x1), y1 + k0*(y2-y1))
        return ip

    def getOccupancyGrid(self, cellSize = 0.1):
        if self._grid is None:
            self._grid = OccupancyGrid(self._xll, self._yll, self._width, self._height, cellSize)
            for l in self._lines:
                x0 = l.getP1().getX()
                y0 = l.getP1().getY()
                x1 = l.getP2().getX()
                y1 = l.getP2().getY()
                self._grid.addLine(x0, y0, x1, y1)
        return self._grid










