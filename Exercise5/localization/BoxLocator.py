__author__ = 'Ecki'

from Exercise5.util import Calculations as Calc


class BoxLocator:

    def __init__(self, robot_location):
        self.robot_loc = robot_location
        self.robot = robot_location.get_robot()
        # tolerance in which a box is defined
        self.box_tol = 0.5
        # list of all box objects
        self.boxes = []

    def sense_box_points(self):
        """
        returns a list with the coordinates of all boxes in sight
        :return:
        """
        boxes = self.robot.senseBoxes()
        if boxes is not None:
            found_boxes = []
            for i in xrange(len(boxes[0])):
                distance = boxes[0][i]
                theta_rel = -boxes[1][i]
                theta = Calc.add_angles(self.robot_loc.get_robot_angle(), theta_rel)
                [x_robot, y_robot] = self.robot_loc.get_robot_point()
                [x_rel, y_rel] = Calc.polar_2_cartesian(distance, theta)
                found_boxes.append([x_robot + x_rel, y_robot + y_rel])
            return found_boxes
        else:
            return None

    def add_point(self, point):
        # if list of boxes is empty, create a new box
        if len(self.boxes) < 1:
            new_box = Box(point)
            self.boxes.append(new_box)
        else:

            matchting_box_found = False
            # go through all defined boxes
            for box in self.boxes:
                # check if point is close to a box
                dist = Calc.get_dist_from_point_to_point(point, box.pos)
                # if so add point to the box and break
                if dist < self.box_tol:
                    box.add_point(point)
                    matchting_box_found = True
                    break

            # when no box is matching, create a new box
            if matchting_box_found == False:
                new_box = Box(point)
                self.boxes.append(new_box)


    def update_boxes(self):
        """
        makes a new measurement and updates all boxes
        :return:
        """
        found_box_points = self.sense_box_points()
        if found_box_points is not None:
            for point in found_box_points:
                self.add_point(point)

    def get_found_box_points(self):
        """
        returns the center coordinates of all found boxes
        :return:
        """
        points = []
        if len(self.boxes) > 0:
            for box in self.boxes:
                points.append(box.pos)
        return points

    def print_found_boxes(self):
        points = self.get_found_box_points()
        print "Found", len(self.boxes), "boxes:"
        number = 0
        for point in points:
            print "Box", number, "at", point
            number += 1



class Box:

    def __init__(self, position):
        self.points = []
        self.points.append(position)
        # the center of mass from all points
        self.pos = position

    def __len__(self):
        return len(self.points)

    def add_point(self, point):
        """
        adds a new measurement point to this box und updates center
        :param point:
        :return:
        """
        [x_point, y_point] = point
        [x_old, y_old] = self.pos
        weight = len(self)
        x_new = (x_old * weight + x_point) / float(weight + 1)
        y_new = (y_old * weight + y_point) / float(weight + 1)
        self.points.append(point)
        self.pos = [x_new, y_new]


