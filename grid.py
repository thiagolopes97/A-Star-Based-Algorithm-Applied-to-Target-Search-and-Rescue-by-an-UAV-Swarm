from constants import *
import pygame as pg
from math import atan2, pi, exp, floor
import random
import copy 
import numpy as np
# importing "heapq" to implement heap queue
import heapq


vec = pg.math.Vector2 



NOT_VISITED = 0      
VISITED = 1
OBSTACLE = 2

class GridField(object):
    def __init__(self, resolution):

        self.cols =int(SCREEN_WIDTH/resolution)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT/resolution)  # Rows of the grid
        self.cells =  np.ndarray((self.rows+1,self.cols+1), dtype=Cell) # grid memory using numpy array
        self.resolution = resolution # Resolution of grid relative to window width and height in pixels
        print(f' Grid created with  col:{self.cols} row:{self.rows}')
        #self.field = [[vec(random.uniform(0,1),random.uniform(0,1)) for col in range(self.cols)] for row in range(self.rows)] # create matrix 
        self.cells_ = {} # Memory using dictionary NOT USED
        self.h_cells = []
        heapq.heapify(self.h_cells)

        self.create_grid_cells()
   

    def create_grid_cells(self):
        '''
            Creates grid with cells according to resolution 
        '''
        blockSize =  self.resolution
        for x in range(0, SCREEN_WIDTH,  blockSize):
            for y in range(0, SCREEN_HEIGHT,  blockSize):
                #self.cells_[f'{int(x/blockSize)},{int(y/blockSize)}'] = Cell(vec(x,y), blockSize)
                #             row                  col       
                self.cells[int(y/blockSize)][int(x/blockSize)] = Cell(vec(x,y), blockSize)
                # priority queue HEAP 
                heapq.heappush( self.h_cells ,  (self.cells[int(y/blockSize)][int(x/blockSize)].state, ( int(y/blockSize),int(x/blockSize) )  ))

    def draw(self, screen):

        blockSize = self.resolution #Set the size of the grid block

        for x in range(0, SCREEN_WIDTH, blockSize):
            for y in range(0, SCREEN_HEIGHT, blockSize):
                rect = pg.Rect(x, y, blockSize, blockSize)
                pg.draw.rect(screen, (120,120,120), rect, 1)
                self.cells[int(y/blockSize)][int(x/blockSize)].draw_center(screen)

    def change_state_cell(self, cell, to_state = VISITED):
        '''
            Cell is visitated
        '''
        try:
            self.cells[cell[1]][cell[0]].change_state(to_state)
        except:
            pass

    def get_state_cell(self, cell):
        '''
            Get if cell was visisted before
            cell: tuple with coordenates
            return: state of the cell 
        '''
        try:
            return self.cells[cell[1]][cell[0]].state
        except:
            return VISITED

    def get_sucessors(self,cell):
        """
            Obtains a list of the 8-connected successors of the node at (i, j).

            :param cell: position cell .
            :type tuple: int.
           
            :return: list of the 8-connected successors.
            :rtype: list of cells.
        """
        i = cell[0]
        j = cell[1]
        successors = []
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if (dx,dy)!= (0,0):
                    x = i + dx
                    y = j + dy
                    # if not visited
                    if x >= 0 and y >= 0:
                        if self.get_state_cell((x,y)) == NOT_VISITED:
                            successors.append(self.get_cell_center((x,y)))  
                            #successors.append((x, y))
        
        #print(successors)
        #input()
        return successors

    def get_all_sucessors(self, cell, step = 1):
        """
            Obtains a list of the 8-connected successors of the node at (i, j).

            :param cell: position cell .
            :type tuple: int.

            :return: list of the 8-connected successors.
            :rtype: list of cells.
        """
        i = cell[0]
        j = cell[1]
        successors = []
        for dx in range(-1*step, 2*step):
            for dy in range(-1*step, 2*step):
                if (dx, dy) != (0, 0):
                    x = i + dx #*step
                    y = j + dy #*step
                    # if not visited
                    if x >= 0 and y >= 0:
                        if x > self.cols:
                            x = self.cols

                        if y > self.rows:
                            y = self.rows
                        try:
                            successors.append(self.get_cell_center((x, y)))
                        except:
                            pass
                            # successors.append((x, y))

        # print(successors)
        # input()
        return successors

    def get_size(self):
        '''
            Returns a tuple containing sizeof the grid :(#col,#row) 
        '''
        return (self.cols, self.rows)

    def get_cell_center(self,cell):
        return self.cells[cell[1]][cell[0]].get_cell_center()   
    
    def get_cell_not_visited(self):
        '''
            This method will return coordenates of a cell that wasnt visited yet
        '''
        return  heapq.heappop( self.h_cells )

class Cell():
    '''
        Represents a cell in the grid
        Every cell represents an area in the map that is being searched
    '''
    def __init__(self, pos, blockSize):
        self.size_block = blockSize
        self.position = pos
        self.state = NOT_VISITED
        self.center_in_coord_global = vec(self.position[0]+ self.size_block/2, self.position[1]+ self.size_block/2)

    def draw_center(self,screen):
        
        if self.state == NOT_VISITED:
            pg.draw.circle(screen, (255,0,0), vec(self.position[0]+ self.size_block/2, self.position[1]+ self.size_block/2), 3)
        if self.state == VISITED: 
            pg.draw.circle(screen, (0,255,0), vec(self.position[0]+ self.size_block/2, self.position[1]+ self.size_block/2), 3)
        if self.state == OBSTACLE:
            pg.draw.circle(screen, (0,0,255), vec(self.position[0]+ self.size_block/2, self.position[1]+ self.size_block/2), 3)

    def change_state(self, state = VISITED):
        if self.state != OBSTACLE:
            self.state = state
    
    def get_cell_center(self):
        return self.center_in_coord_global