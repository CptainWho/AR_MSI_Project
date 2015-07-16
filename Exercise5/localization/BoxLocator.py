__author__ = 'Ecki'

from Exercise5.util import Calculations as Calc


class BoxLocator:

    def __init__(self, robot_location, world):
        self.robot_loc = robot_location
        self.world = world
        self.grid = world.getOccupancyGrid()
        self.robot = robot_location.get_robot()
        # tolerance in which a box is defined
        self.box_tol = 1.0
        # list of all box objects
        self.boxes = []
        # tolerance in which a found is accepted to be in the room
        self.tol_room_borders = 1.5

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

    def add_point(self, point, room):
        actual_room_name = room.get_name()
        # if list of boxes is empty, create a new box
        if len(self.boxes) < 1:
            new_box = Box(point, actual_room_name)
            self.boxes.append(new_box)
        else:
            matching_box_found = False
            # go through all defined boxes
            for box in self.boxes:
                # check if point is close to a box
                dist = Calc.get_dist_from_point_to_point(point, box.pos)
                if dist < self.box_tol:
                    # next check if the found box is in the same room
                    if box.get_room_name() == actual_room_name:
                        # if so add point to the box and break
                        box.add_point(point)
                        matching_box_found = True
                        break

            # when no box is matching, create a new box
            if matching_box_found == False:
                # only create a new box if point is not too far away from room
                if room.point_inside_borders(point, self.tol_room_borders):
                    new_box = Box(point, actual_room_name)
                    self.boxes.append(new_box)

    def update_boxes(self, room):
        """
        makes a new measurement and updates all boxes
        :return:
        """
        if room is not None:
            found_box_points = self.sense_box_points()
            if found_box_points is not None:
                for point in found_box_points:
                    self.add_point(point, room)

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

    def draw_found_boxes(self):
        points = self.get_found_box_points()
        for point in points:
            self.world.draw_found_box(point)


class Box:

    def __init__(self, position, room_string=None):
        self.points = []
        self.points.append(position)
        # the center of mass from all points
        self.pos = position
        # the room where the box was found
        self.room_string = room_string

    def __len__(self):
        return len(self.points)

    def get_room_name(self):
        return self.room_string

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


