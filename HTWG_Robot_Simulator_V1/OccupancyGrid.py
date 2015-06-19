# -*- coding: utf-8 -*-
# class OccupancyGrid.
#
# O. Bittel
#
# modified by Philipp Lohrer on 16.06.2015
# Changelog:
#   added getters for grid attributes
#   converted grid to numpy array
#   changed draw() to draw an matplotlib-obj instead of creating a graphics-obj
#   added methods set_value_at_index() and get_value_at_index() to set/get values via indexing


from math import *
from graphics import *
import numpy as np
import matplotlib.pyplot as plt

class OccupancyGrid:

    # --------
    # init: creates a an empty occupancy grid with the given cell size
    #
    def __init__(self, xll, yll, width, height, cellSize = 0.1):
        # define grid:
        self.xSize = int((width+cellSize/2)/cellSize) + 1
        self.ySize = int((height+cellSize/2)/cellSize) + 1
        # self.grid = [[0 for x in range(self.ySize)] for x in range(self.xSize)]
        self.grid = np.zeros((self.xSize, self.ySize), dtype=np.float)
        self.width = width
        self.height = height
        self.cellSize = float(cellSize)

        plt.ion()

    def getGridWidth(self):
        return self.width

    def getGridHeight(self):
        return self.height

    def getGridSize(self):
        return [self.xSize, self.ySize]

    def getCellSize(self):
        return self.cellSize

    def getGrid(self):
        return self.grid

    def printGrid(self):
        print "xSize*ySize: ", self.xSize, self.ySize
        for yi in range(self.ySize-1,-1,-1):
            s = ""
            for xi in range(self.xSize):
                s += "%1d" % self.grid[xi][yi]
            print s

    def drawGrid(self):
        # added: figure for plotting
        fig = plt.figure()
        fig.suptitle('OccupancyGrid', fontsize=20)
        plot = plt.imshow(self.grid.transpose(), origin='lower')
        cbar = plt.colorbar(mappable=plot)
        cbar.set_label('Occupancy', fontsize=16)
        plt.show()

        # # define graphic window:
        # win = GraphWin("Occupancy Grid", int(800.0*self.width/self.height), 800, autoflush=False)
        # win.setCoords(-self.cellSize, -self.cellSize, self.width+self.cellSize, self.height+2*self.cellSize)
        #
        # # draw all grid cells:
        # for yi in range(self.ySize):
        #     for xi in range(self.xSize):
        #         if self.grid[xi][yi] == 1:
        #             p1 = Point(xi*self.cellSize, yi*self.cellSize)
        #             p2 = Point((xi+1)*self.cellSize, (yi+1)*self.cellSize)
        #             r = Rectangle(p1, p2)
        #             r.setFill('black')
        #             r.draw(win)
        #         # TODO draw cells with higher values in another color and display occupancy
        #
        # # close window when click:
        # print "click in window to close"
        # win.getMouse() # pause for click in window
        # win.close()

    # --------
    # Add new a new line from point (x0,y0) to (x1,y1) to the occupancy grid.
    # Currently only horizontal or vertical lines ar allowed.
    # To Do: Implement Bresenham algorihm for arbitrary lines.
    #
    def addLine(self, x0, y0, x1, y1, value = 1):
        if x0 != x1 and y0 != y1:
            raise ValueError, 'lines must be horizontal or vertical'
        x0_i = int(x0/self.cellSize + 0.5)
        y0_i = int(y0/self.cellSize + 0.5)
        # print x0, y0, x1, y1
        if x0 == x1:
            y1_i = int(y1/self.cellSize + 0.5)
            if y0_i < y1_i:
                for yi in range(y0_i,y1_i+1):
                    self.grid[x0_i][yi] = value
            else:
                for yi in range(y1_i,y0_i+1):
                    self.grid[x0_i][yi] = value
        else:
            x1_i = int(x1/self.cellSize + 0.5)
            if x0_i < x1_i:
                for xi in range(x0_i,x1_i+1):
                    self.grid[xi][y0_i] = value
            else:
                for xi in range(x1_i,x0_i+1):
                    self.grid[xi][y0_i] = value


    # --------
    # Set grid value at the coordinate (x,y).
    #
    def setValue(self, x, y, value = 1):
        if x < 0 or x > self.width:
            return
        if y < 0 or y > self.width:
            return
        xi = int(x/self.cellSize + 0.5)
        yi = int(y/self.cellSize + 0.5)
        self.grid[xi][yi] = value


    # --------
    # Get grid value at the coordinate (x,y).
    #
    def getValue(self, x, y):
        if x < 0 or x > self.width:
            return
        if y < 0 or y > self.width:
            return
        xi = int(x/self.cellSize + 0.5)
        yi = int(y/self.cellSize + 0.5)
        return self.grid[xi][yi]

    def set_value_at_index(self, index_x, index_y, value=1):
        """ Set given value at specified indexes in grid
        :param index_x:
        :param index_y:
        :param value:   default = 1
        :return:
        """

        # Check boundaries of grid
        if 0 < index_x < self.xSize and 0 < index_y < self.ySize:
            self.grid[index_x, index_y] = value

    def get_value_at_index(self, index_x, index_y):
        """ Return value at given indexes
        :param index_x:
        :param index_y:
        :return:        value at grid[index_x, index_y]
        """

        # Check boundaries of grid
        if 0 < index_x < self.xSize and 0 < index_y < self.ySize:
            return self.grid[index_x, index_y]




def test1():
    myGrid = OccupancyGrid(0, 0, 0.8, 0.5)
    myGrid.setValue(0.0,0.0)
    myGrid.setValue(0.1,0.0)
    myGrid.setValue(0.3,0.0)
    myGrid.setValue(0.4,0.0)
    myGrid.setValue(0.5,0.0)

    myGrid.setValue(0.0,0.2)
    myGrid.setValue(0.1,0.2)
    myGrid.setValue(0.3,0.2)
    myGrid.setValue(0.4,0.2)
    myGrid.setValue(0.5,0.2)

    myGrid.setValue(0.8,0.4)
    myGrid.setValue(0.8,0.5)

    myGrid.printGrid()

def test2():
    myGrid = OccupancyGrid(0, 0, 10.0, 8.0)
    myGrid.addLine(0.1,0.1, 0.7,0.1)
    myGrid.addLine(0.7,0.1, 0.7,0.3)

    myGrid.printGrid()
    myGrid.drawGrid()

def test3():
    myGrid = OccupancyGrid(0, 0, 2, 2, 1)
    myGrid.setValue(0,0)
    myGrid.setValue(0,1)

    myGrid.printGrid()


# test3()