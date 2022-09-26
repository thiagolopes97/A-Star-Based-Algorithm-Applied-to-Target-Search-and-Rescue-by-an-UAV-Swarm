import time
import pygame
import math
import csv
from constants import *
from vehicle import Vehicle
from scan import ScanInterface
from state_machine import * #FiniteStateMachine, SeekState, SearchTargetState
from random import uniform
from obstacle import Obstacles
from utils import Npc_target
from grid import GridField
import numpy as np
import matplotlib.pyplot as plt

vec2 = pygame.math.Vector2
##=========================
class RateSimulation(object):
    def __init__(self, in_repetitions, in_num_swarm, in_num_obstacles, in_algorithms):
        self.current_repetition = 0
        self.in_num_swarm = []
        self.in_num_obstacles = []
        self.in_algorithms = []
        
        # Inputs of Rate
        self.in_repetitions = in_repetitions * len(in_num_swarm) * len(in_num_obstacles) * len(in_algorithms)
        
        res = [[i, j, k] for i in enumerate(in_num_swarm) 
                         for j in enumerate(in_num_obstacles) 
                         for k in enumerate(in_algorithms) ]

        for r in enumerate(res):
            print(str(r))
            self.in_num_swarm += [r[1][0][1]] * in_repetitions 
            self.in_num_obstacles += [r[1][1][1]] * in_repetitions 
            self.in_algorithms += [r[1][2][1]] * in_repetitions 
        
        # Outputs of Rate
        self.out_time_mission = []
        self.out_time_target = []
        self.out_num_uav = []
        self.print_plan_rate()

    def set_time_target(self, time_target):
        self.out_time_target.append(time_target)

    def set_out(self, out_time_mission, out_num_uav):
        self.out_time_mission.append(out_time_mission)
        self.out_num_uav.append(out_num_uav)

    def next_simulation(self):
        if self.in_repetitions - 1 == self.current_repetition:
            return False
        else:
            self.current_repetition = self.current_repetition + 1
            self.print_simulation()
            return True

    def print_plan_rate(self):
        for idx in range(0, self.in_repetitions):
            print(f'{idx+1} - num_obstacles: {self.in_num_obstacles[idx]}, num_swarm : {self.in_num_swarm[idx]}, algorithm : {self.in_algorithms[idx].to_string()},')

    def print_simulation(self):
        return f'{self.current_repetition+1} - num_swarm: {self.in_num_swarm[self.current_repetition]}, num_obstacles: {self.in_num_obstacles[self.current_repetition]}, Algorithm: {self.in_algorithms[self.current_repetition].to_string()}'

    def print_simulation_idx(self, idx):
        return f'{idx+1} - Time: {time:.2f}, num_uav: {self.out_num_uav[idx]}, num_swarm: {self.in_num_swarm[idx]}, num_obstacles: {self.in_num_obstacles[idx]}'

    def print_rate(self):
        num_d = []
        time_sim = []
        time_target = []
        num_obst = []
        qntd_drones = []
        idx_sim = []

        for idx in range(0, len(self.out_time_target)):
            print(f'{idx+1} - Time Target: {self.out_time_target[idx]}, Time Mission: {self.out_time_mission[idx]}, num_uav: {self.out_num_uav[idx]}, num_swarm: {self.in_num_swarm[idx]}, num_obstacles: {self.in_num_obstacles[idx]}')
            num_d.append(self.in_num_swarm[idx])
            idx_sim.append(idx+1)
            time_target.append(self.out_time_target[idx])
            time_sim.append(self.out_time_mission[idx])
            qntd_drones.append(self.out_num_uav[idx])
            num_obst.append(self.in_num_obstacles[idx])

        plt.xlabel('Idx')
        plt.ylabel('Time')
        plt.title('History of simulations')
        #plt.plot(idx_sim, time_sim, 'r--', idx_sim,qntd_drones, 'g^' , idx_sim,num_obst,'bs' )
        plt.plot(idx_sim,qntd_drones, 'g^', idx_sim, num_d  )
        plt.show()
        #plt.plot(t, t, 'r--', t, t**2, 'bs', t, t**3, 'g^') 

    def save_csv(self):
        with open('result.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Execution", "N Drones", "N Obstacles", "Algorithm", "Time Found Target", "Time Mission Completed"])
            print(len(self.out_time_mission))
            for idx in range(0, self.in_repetitions):
                #writer.writerow([idx+1, self.in_num_swarm[idx], self.in_num_obstacles[idx], self.in_algorithms[idx], self.out_time_target[idx], self.out_time_mission[idx]])
                writer.writerow([idx+1, self.in_num_swarm[idx], self.in_num_obstacles[idx], self.in_algorithms[idx].to_string(), self.out_time_target[idx], self.out_time_mission[idx]])
            

class ScreenSimulation(object):
    '''
        Class responsable to represent the canvas variables
    ''' 
    def __init__(self):
        pygame.init()
        self.font16 = pygame.font.SysFont(None, 16)
        self.font20 = pygame.font.SysFont(None, 20)
        self.font24 = pygame.font.SysFont(None, 24)
        self.size = SCREEN_WIDTH, SCREEN_HEIGHT 
        self.clock = pygame.time.Clock()
        self.screen = pygame.display.set_mode(self.size)

class Simulation(object):
    
    def __init__(self, screenSimulation,rate:RateSimulation):
        self.target_simulation = None
        self.screenSimulation = screenSimulation
        self.start_watch = 0
        self.stop_watch = 0
        self.rate = rate
        self.time_executing = 0 
        self.found = False
        # variables for obstacles
        self.obstacles = Obstacles(rate.in_num_obstacles[0], (SCREEN_WIDTH,SCREEN_HEIGHT))
        self.list_obst = []
        self.generate_obstacles()

        # Grid
        self.grid_field = GridField(RESOLUTION)

        # state machines for controlling drones
        self.behaviors =[] 
        
        # Current simulations 
        self.swarm = []
        self.targets_search = [] # memory of targets used in simulations

        # npc target 
        self.npc = Npc_target()
        self.all_sprites = pygame.sprite.Group()
        self.all_sprites.add(self.npc)

        self.create_swarm_uav(rate.in_num_swarm[0])

        # target 
        self.target_simulation = self.generate_new_random_target()
        self.targets_search.append(self.target_simulation)

        #self.set_target_using_search_pattern(self.target_simulation)

    def generate_obstacles(self):
        # Generates obstacles
        self.obstacles.generate_obstacles()
        self.list_obst = self.obstacles.get_coordenates()

    def create_swarm_uav(self, num_swarm, search_pattern = 'DefineTargetScan'):
        # Create N simultaneous Drones
        for d in range(0, num_swarm):
            # Seek state: se tem o target inicialmente:
            if search_pattern == 'DefineTargetScan':
                self.behaviors.append( FiniteStateMachine( SeekState() ) ) # Inicial state

            elif search_pattern == 'DefineTargetScan_A_star':
                self.behaviors.append(FiniteStateMachine(A_star()))


            elif search_pattern == 'RowScan':
                self.behaviors.append(FiniteStateMachine(SearchTargetState()))


            elif search_pattern == 'RowScan_AStar':
                self.behaviors.append(FiniteStateMachine(SearchTargetState_A_star()))

            elif search_pattern == 'ColumnScan_A_star':
                self.behaviors.append(FiniteStateMachine(ColumnScan_A_star()))

            elif search_pattern == 'ColumnScan_Seek':
                self.behaviors.append(FiniteStateMachine(ColumnScan_Seek()))

            elif search_pattern == 'ColumnScan_small_A_star':
                self.behaviors.append(FiniteStateMachine(ColumnScan_small_Astar()))

            elif search_pattern == 'ColumnScan_small_Seek':
                self.behaviors.append(FiniteStateMachine(ColumnScan_small_Seek()))

            elif search_pattern == 'TestScan_Seek':
                self.behaviors.append(FiniteStateMachine(TestScan_Seek()))

            elif search_pattern == 'TestScan_A_star':
                self.behaviors.append(FiniteStateMachine(TestScan_A_star()))


            else:
                self.behaviors.append( FiniteStateMachine( SearchTargetState() ) ) # Inicial state


            drone = Vehicle(uniform(0,100), uniform(0,100), self.behaviors[-1], self.screenSimulation.screen)
            self.swarm.append(drone)

    def add_new_uav(self):
        self.behaviors.append( FiniteStateMachine( SeekState() ) )
        drone = Vehicle(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, self.behaviors[-1], self.screenSimulation.screen)

        drone.set_target(vec2(pygame.mouse.get_pos()[0],pygame.mouse.get_pos()[1]))
        self.append_uav(drone)
    
    def append_uav(self, drone):
        self.swarm.append(drone)

    def set_target(self, target, found=False):
        self.target_simulation = target
        for _ in self.swarm:
            _.set_target(target)
            if found == True:
                _.found = True

    def set_time_target(self):
        self.rate.set_time_target(time.time() - self.start_watch)

    def set_target_using_search_pattern(self, target_simulation):
        '''
            IN TEST
            Set target area to be search  
        '''
        # saves global target
        self.target_simulation = target_simulation

        # get #row and #col
        col = self.grid_field.cols
        row = self.grid_field.rows

        table_search = np.zeros((row,col))
        num_drones = len(self.swarm)

        step = math.ceil(row/num_drones)
        r=0
        c=0
        col_ = col 

        while num_drones > 0 :
            # self.swarm.set_target() - argumento é o target referente a celular
            # pegar a posicao do centro da celula
            cell_center = self.grid_field.cells[r][c].get_cell_center()
            drone_target =  vec2(  cell_center[0], cell_center[1] )
            self.swarm[num_drones-1].set_target( drone_target ) 
            self.swarm[num_drones-1].mission_target = vec2(  drone_target )
            
            #print(f'drone: {num_drones} celula: {(r,c,step)} {vec2(  cell_center[0], cell_center[1] )}')

            table_search[r][c]  = num_drones
            num_drones -= 1
            
            # verifica se o passo não vai passar o limite de linhas da matriz
            if r < row - step:
                r += step
            else:
                r = 0 
                col_ = math.floor(col_/2)
                c+= col_
                
        #print(table_search)
        self.table_search = table_search
      
    def draw_obstacles(self):
        # draws the sprites of tree
        for _ in self.list_obst: 
            self.obstacles.all_sprites.draw(self.screenSimulation.screen)
            self.obstacles.all_sprites.update(_,0)
            pygame.draw.circle(self.screenSimulation.screen,(200, 200, 200), _, radius=RADIUS_OBSTACLES, width=1)
            pygame.draw.circle(self.screenSimulation.screen,(200, 200, 200), _, radius=RADIUS_OBSTACLES*1.6 + AVOID_DISTANCE, width=1)

    def draw_target(self):
        # draw target - npc
        if self.target_simulation: 
            self.all_sprites.draw(self.screenSimulation.screen)
            self.all_sprites.update(self.target_simulation,0)
            pygame.draw.circle(self.screenSimulation.screen, LIGHT_BLUE, self.target_simulation, RADIUS_TARGET, 2)

    def draw(self):
        #draw grid of visited celss
        self.grid_field.draw(self.screenSimulation.screen)
        # draw target - npc
        self.draw_target()
        # draw obstacles
        self.draw_obstacles()

    def run_simulation(self):
        # draw grid of visited cels, target and obstacles
        self.draw()

        # Target is Found: pass it to all drones
        if self.found:
            self.set_target(self.target_simulation, found = True)

        if self.start_watch == 0:
            self.start_watch = time.time()

        # for every drone, it will update the collision avoidace, aling the direction and draw current position in simuation
        self.rate.in_algorithms[self.rate.current_repetition].scan(self, self.list_obst)
       
        self.time_executing += SAMPLE_TIME # count time of execution based on the sampling

        # check completition of simulation
        if self.completed_simulation() >= 1 and self.stop_watch == 0 or self.time_executing > TIME_MAX_SIMULATION:
            self.stop_watch = time.time()
            
            if self.rate and self.rate.next_simulation():
                self.rest_simulation()
            else:
                return False

        return True

    def completed_simulation(self):
        count_completed = 0

        if self.target_simulation:
            for _ in self.swarm:
                if _.reached_goal(self.target_simulation):
                    count_completed = count_completed + 1 
        return count_completed/self.rate.in_num_swarm[self.rate.current_repetition]

    def generate_new_random_target(self):
        '''
            Generates valid random targets from a safe distance from obstacles
        '''
        found_valid_target= False 
        while not found_valid_target : 
            # generates new point
            target = vec2(uniform(SCREEN_WIDTH/3,SCREEN_WIDTH), uniform(100,SCREEN_HEIGHT))
            c=0
            #checks if it is valid
            for o in self.list_obst:
                # distance to obstacles
                d = target.distance_to(o)
                # check if ditance is not inside obstacle
                if d < RADIUS_OBSTACLES + RADIUS_TARGET:
                    c += 1
            # check counter
            if c == 0 :
                found_valid_target = True

        return target
        
    def rest_simulation(self):
        # reset grid 
        self.grid_field = GridField(RESOLUTION)

        # new obstacles
        self.obstacles.num_of_obstacles = self.rate.in_num_obstacles[self.rate.current_repetition]
        # Repeat scenario for new number of drones
        num_repet = self.rate.in_repetitions / len(self.rate.in_num_swarm)
        if self.rate.current_repetition > num_repet -1:
            self.obstacles.reset_seed()

        self.generate_obstacles()

        time = self.stop_watch - self.start_watch
        if self.time_executing > TIME_MAX_SIMULATION:
            time = "Goal not reached"
        self.rate.set_out(time, self.completed_simulation())
            
        for _ in self.swarm:
            _.set_target(None)
            del _

        self.swarm = []
        self.start_watch = 0
        self.stop_watch = 0
        self.target_simulation = None
        serch_patter_for_iteration = self.rate.in_algorithms[self.rate.current_repetition].to_string()
        print(f'ITERATION USING: {serch_patter_for_iteration} ')
        self.create_swarm_uav(self.rate.in_num_swarm[self.rate.current_repetition], serch_patter_for_iteration)
        self.time_executing = 0 # Reset timer

        # set new random target for iteration
        target = self.generate_new_random_target()
        self.targets_search.append(target)
        self.set_target(target)

        # Prepare ALGORITHM TO SEARCH PATTERN
        self.rate.in_algorithms[self.rate.current_repetition].prepare_simulation(self, target)
        self.found = False