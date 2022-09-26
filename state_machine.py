import random
from random import choices
from math import pi, atan2, inf
from constants import *
import numpy as np
import pygame
import time
import copy

NOT_VISITED = 0
VISITED = 1
OBSTACLE = 2
vec2 = pygame.math.Vector2

def get_random_state():
    """
        This method will sample random state based on the list of states and their probabilities

        :return: The name of the state
        :rtype: string
    """
    states = ['SeekState','StayAtState', 'Eight2State','OvalState', 'ScanState' ]
    probabilities = [0.05, 0.3, 0.05, 0.3, 0.3]

    x = np.random.choice(states, p =probabilities)
    return x

class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)

    def get_current_state(self):
        return self.state.state_name

class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

class SeekState(State):
    """
        Drone will seek target
    """
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'SeekState'
        self.time_executing = 0 #Variavel para contagem do tempo de execução
        print('Seek')
        self.finished = False
        self.memory_last_position = vec2(inf,inf)
        self.sampling_time = 3
        self.time_blocked = 0

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

     # New target from mouse click
        if agent.get_target():
            self.target = agent.get_target()
            agent.mission_target = agent.get_target()
            agent.set_target(None)


        dist = self.memory_last_position.distance_to(agent.get_position())

        # verifica se chegou
        d = self.target.distance_to(agent.get_position())

        if d <= RADIUS_TARGET and d > 3 :
            self.finished = True
            self.state_name = 'Done'

     # Verifica se terminou a execucao
        if self.finished == True:
            pass

        if dist < 30 and self.finished == False  :
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 2:
                pos_in_grid = agent.position_in_grid
                agent.grid_map.change_state_cell(pos_in_grid, OBSTACLE )

                state_machine.change_state(GoToClosestDroneState())

    def execute(self, agent):
        # logic to move drone to target
        try:
            self.target
        except:
            self.target = agent.mission_target

        agent.arrive(self.target)
        self.time_executing += SAMPLE_TIME

        # Sampling location every T seconds
        if self.time_executing >=  self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = copy.deepcopy(agent.get_position())

class GoToClosestDroneState(State):
    """
        Drone will seek closest drone in swarm
    """
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'GoToClosesDroneState'
        self.time_executing = 0 #Variavel para contagem do tempo de execução
        print('GoToClosesDroneState')
        self.finished = False


    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # New target from mouse click
        if agent.get_target():
            state_machine.change_state(SeekState())

        if self.time_executing > 3:
            state_machine.change_state(RandomTargetState())

        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(SeekState())

    def execute(self, agent):
        # logic to move drone to target

        self.target = agent.get_closest_drone()

        agent.arrive(self.target)
        self.time_executing +=SAMPLE_TIME

        if (self.target - agent.location).length() < SIZE_DRONE*2*1.4 :
            self.finished = True

class RandomTargetState(State):
    """
        Drone will seek a random target to unblock as last resort
    """
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'RandomTargetState'
        self.time_executing = 0 #Variavel para contagem do tempo de execução
        print('RandomTargetState')
        self.finished = False

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # chegou ao waypoint
        if self.finished == True:
                state_machine.change_state(SeekState())


    def execute(self, agent):
        # logic to move drone to target
        try:
            self.target
        except:
            random_position = vec2(random.uniform(-400,400),random.uniform(-400,400))
            self.target = agent.mission_target+ random_position

        agent.arrive(self.target)
        self.time_executing +=SAMPLE_TIME

        if (self.target - agent.location).length() < 10 or self.time_executing > 3:
            self.finished = True

class SearchTargetState(State):
    """
        Drone will seek a 8-connected cells - random target  as last resort
    """
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'SearchTargetState'
        self.time_executing = 0 #Variavel para contagem do tempo de execução
        print('SearchTargetState')
        self.finished = False

        # Map resolution
        self.cols =int(SCREEN_WIDTH/RESOLUTION)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT/RESOLUTION)  # Rows of the grid

        # waypoints to be followed
        self.waypoints = []
        self.next_waypoint = 0

        # for checking if it is blocked
        self.memory_last_position = vec2(inf,inf)
        self.sampling_time = 2
        self.time_blocked = 0
        self.blocked = False

    def generate_waypoints(self):
        waypoints = [ vec2(0,0) ] # initial position
        #size of the grid
        cols = self.grid_map.get_size()

        global_coord = [ vec2(self.target[0],vec2(self.target[0])) ]
        # em coordenadas locais do grid
        # vai até o final
        waypoints.append( ( cols, 0 ) )
        # desce
        #waypoints.append( ( 0, 1 ) )
        # volta
        #waypoints.append( ( - cols , 0 ) )

        # converter para coordenadas globais
        for w in waypoints:
            global_coord.append( vec2(self.grid_map.get_cell_center(w)[0],self.grid_map.get_cell_center(w)[1] ) )

        return global_coord

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(SeekState())

        try:
            #self.state_name = f'TARGET: {self.target}'
            pass
        except:
            pass

     # distancia percorrida desde a amostragem
        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 30 and self.finished == False :
            self.time_blocked += SAMPLE_TIME
            #self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 2:
                self.time_blocked = 0
                #state_machine.change_state(SearchTargetState())
                self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))

    def execute(self, agent):
        # logic to move drone to target
        try: # verifica se o drone já te um target ou seja, uma coluna a cobrir
            self.target

        except: # nao tem, logo:
            self.target = agent.mission_target
            self.waypoints = agent.grid_map.get_sucessors( agent.position_in_grid )
            #print(self.waypoints)

        agent.arrive(self.target)

        self.time_executing +=SAMPLE_TIME

        if (self.target - agent.location).length() < 30 :
            #self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
            self.waypoints = agent.grid_map.get_sucessors( agent.position_in_grid )
            #self.state_name = f'{self.waypoints}'
            if len(self.waypoints) > 0: # enquanto existem celulas nao visitadas na regiao
                targ = random.choice(self.waypoints)
                self.target = targ
            else: # random na tela para buscar ja que todas as celulas foram visitadas
                self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))

            #rint(f'EU IRIA PARA A CELULA : {agent.grid_map.get_cell_not_visited()}')

        # target is found by a drone in the swarm
        if agent.found:
            self.finished = True

        # Sampling location every T seconds
        if self.time_executing >=  self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = copy.deepcopy(agent.get_position())

class RandomSearchState(State):
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'SearchTargetState'
        self.time_executing = 0 #Variavel para contagem do tempo de execução
        print('SearchTargetState')
        self.finished = False

        # Map resolution
        self.cols =int(SCREEN_WIDTH/RESOLUTION)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT/RESOLUTION)  # Rows of the grid

        # waypoints to be followed
        self.waypoints = []
        self.next_waypoint = 0

        # for checking if it is blocked
        self.memory_last_position = vec2(inf,inf)
        self.sampling_time = 2
        self.time_blocked = 0
        self.blocked = False

    def generate_waypoints(self):
        waypoints = [ vec2(0,0) ] # initial position
        #size of the grid
        cols = self.grid_map.get_size()

        global_coord = [ vec2(self.target[0],vec2(self.target[0])) ]
        # em coordenadas locais do grid
        # vai até o final
        waypoints.append( ( cols, 0 ) )
        # desce
        waypoints.append( ( 0, 1 ) )
        # volta
        waypoints.append( ( - cols , 0 ) )

        # converter para coordenadas globais
        for w in waypoints:
            global_coord.append( vec2(self.grid_map.get_cell_center(w)[0],self.grid_map.get_cell_center(w)[1] ) )

        return global_coord

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(SeekState())

        try:
            self.state_name = f'TARGET: {self.target}'
        except:
            pass

     # distancia percorrida desde a amostragem
        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 70 and self.finished == False :
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 1:
                #state_machine.change_state(SearchTargetState())
                #self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
                pass

    def execute(self, agent):
        # logic to move drone to target
        try: # verifica se o drone já te um target ou seja, uma coluna a cobrir
            self.target
        except: # nao tem, logo:
            self.target = agent.mission_target
            agent.grid_map.get_sucessors( agent.position_in_grid )

        agent.arrive(self.target)

        self.time_executing +=SAMPLE_TIME

        if (self.target - agent.location).length() < RADIUS_OBSTACLES*2 :
            self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
            #rint(f'EU IRIA PARA A CELULA : {agent.grid_map.get_cell_not_visited()}')

        # target is found by a drone in the swarm
        if agent.found:
            self.finished = True

        # Sampling location every T seconds
        if self.time_executing >=  self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = copy.deepcopy(agent.get_position())


class SearchTargetState_A_star(State):
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'SearchTargetState_A_star'
        self.time_executing = 0  # Variavel para contagem do tempo de execução
        print('SearchTargetState_A_star')
        self.finished = False

        # Map resolution
        self.cols = int(SCREEN_WIDTH / RESOLUTION)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT / RESOLUTION)  # Rows of the grid

        # waypoints to be followed
        self.waypoints = []
        self.next_waypoint = 0

        # for checking if it is blocked
        self.memory_last_position = vec2(inf, inf)
        self.sampling_time = 2
        self.time_blocked = 0
        self.blocked = False

    def generate_waypoints(self):
        waypoints = [vec2(0, 0)]  # initial position
        # size of the grid
        cols = self.grid_map.get_size()

        global_coord = [vec2(self.target[0], vec2(self.target[0]))]
        # em coordenadas locais do grid
        # vai até o final
        waypoints.append((cols, 0))
        # desce
        waypoints.append((0, 1))
        # volta
        waypoints.append((- cols, 0))

        # converter para coordenadas globais
        for w in waypoints:
            global_coord.append(vec2(self.grid_map.get_cell_center(w)[0], self.grid_map.get_cell_center(w)[1]))

        return global_coord

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(A_star())

        try:
            self.state_name = f'TARGET: {self.target}'
        except:
            pass

        # distancia percorrida desde a amostragem
        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 70 and self.finished == False:
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 1:
                # state_machine.change_state(SearchTargetState())
                # self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
                pass

    def execute(self, agent):
        # logic to move drone to target
        try:  # verifica se o drone já te um target ou seja, uma coluna a cobrir
            self.target
        except:  # nao tem, logo:
            self.target = agent.mission_target
            agent.grid_map.get_sucessors(agent.position_in_grid)

        agent.arrive(self.target)

        self.time_executing += SAMPLE_TIME

        if (self.target - agent.location).length() < RADIUS_OBSTACLES * 2:
            self.target = vec2(random.uniform(0, SCREEN_WIDTH), random.uniform(0, SCREEN_HEIGHT))
            # rint(f'EU IRIA PARA A CELULA : {agent.grid_map.get_cell_not_visited()}')

        # target is found by a drone in the swarm
        if agent.found:
            self.finished = True

        # Sampling location every T seconds
        if self.time_executing >= self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = copy.deepcopy(agent.get_position())

class A_star(State):
    """
        Teste A*
    """

    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'PathAStar'
        self.time_executing = 0  # Variavel para contagem do tempo de execução
        print('PathAStar')
        self.finished = False
        self.memory_last_position = vec2(inf, inf)
        self.sampling_time = 3
        self.time_blocked = 0
        self.final_goal = None
        self.sub_path = False

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # New target from mouse click
        if agent.get_target():
            self.final_goal = agent.get_target()  # Target location
            self.target = self.final_goal  # agent.get_target()
            # agent.mission_target = vec2(random.uniform(100,300),random.uniform(100,300))  # <--- Define a posicao que o drone deve ir... Valor antigo: agent.get_target()
            #agent.set_target(None)

        dist = self.memory_last_position.distance_to(agent.get_position())

        # verifica se chegou
        d = self.final_goal.distance_to(agent.get_position())
        num_swarm = len(agent.positions_drones)

        # dist1 = agent.memory_location
        # print(dist)

        if d > RADIUS_TARGET:
            agent.step = int(d/RESOLUTION)
            if agent.step > 3:
                agent.step = 3

            if self.time_blocked > 2:
                agent.step += 1




        if self.sub_path == False:
            self.sub_path = True

            agent.step = 1
            near_grid = agent.grid_map.get_all_sucessors(agent.position_in_grid, step=agent.step)

            # print(agent.pos_obstacles)
            # print(agent.positions_drones)

            aux_list = []
            close_point = False

            for i in near_grid:
                dist_a = vec2(i[0], i[1]).distance_to(self.final_goal)
                #dist_a += abs(i[0] - self.final_goal[0]) + abs(i[1] - self.final_goal[1])

                for j in agent.pos_obstacles:
                    # print(vec2(j[0],j[1]).distance_to(agent.get_position()))
                    dist_obs = vec2(j[0], j[1]).distance_to(vec2(i[0], i[1]))

                    # print(dist_obs)
                    if dist_obs < AVOID_OBSTACLES + 50:
                        close_point = True
                        break

                    if dist_obs <= RESOLUTION*1.5:
                        dist_a += 100 #1 / dist_obs ** 2 #abs(i[0] - j[0]) + abs(i[1] - j[1]) #

                if close_point == True:
                    close_point = False
                    continue

                point = vec2(i[0], i[1])
                aux_list.append([point, dist_a])

            best_point = None
            best_value = inf

            for p in aux_list:
                point = p[0]
                value = p[1]
                if point in agent.test:
                    continue

                if best_value > value:
                    best_value = value
                    best_point = point

            print(best_point)

            if best_point is None:
                best_point = vec2(agent.get_position()[0] + RESOLUTION/2, agent.get_position()[0] + RESOLUTION/2)

            agent.test.append(best_point)
            agent.mission_target = best_point
        else:
            sub_path_dist = agent.mission_target.distance_to(agent.get_position())
            if num_swarm <= 3:
                alpha = 0
            else:
                alpha = 0

            if sub_path_dist < RESOLUTION/3 + alpha and d >= RADIUS_TARGET:
                self.sub_path = False

        if d <= RADIUS_TARGET and d > 3:
            self.finished = True
            self.state_name = 'Done'

        # Verifica se terminou a execucao
        if self.finished == True:
            pass

        if dist < 30 and self.finished == False:
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'

            if self.time_blocked > 4:
                pos_in_grid = agent.position_in_grid
                agent.grid_map.change_state_cell(pos_in_grid, OBSTACLE)

                self.sub_path = False
                state_machine.change_state(A_star())
                #state_machine.change_state(GoToClosestDroneStateA_star())

        if dist > RESOLUTION:
            self.time_blocked = 0
                # self.target = vec2(random.uniform(agent.get_position()[0] - RESOLUTION/2,agent.get_position()[0] + RESOLUTION/2),
                #                    random.uniform(agent.get_position()[1] - RESOLUTION/2,agent.get_position()[1] + RESOLUTION/2))

    def execute(self, agent):
        # logic to move drone to target
        try:
            self.target
        except:
            self.target = agent.mission_target

        agent.arrive(agent.mission_target)  # self.target)
        self.time_executing += SAMPLE_TIME

        # Sampling location every T seconds
        if self.time_executing >= self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = copy.deepcopy(agent.get_position())


class GoToClosestDroneStateA_star(State):
    """
        Drone will seek closest drone in swarm
    """

    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'GoToClosesDroneState'
        self.time_executing = 0  # Variavel para contagem do tempo de execução
        print('GoToClosesDroneState')
        self.finished = False

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # New target from mouse click
        if agent.get_target():
            state_machine.change_state(A_star())

        if self.time_executing > 3:
            state_machine.change_state(SearchTargetState_A_star())

            # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(A_star())

    def execute(self, agent):
        # logic to move drone to target

        self.target = agent.get_closest_drone()

        agent.arrive(self.target)
        self.time_executing += SAMPLE_TIME

        if (self.target - agent.location).length() < SIZE_DRONE * 2 * 1.4:
            self.finished = True




class ColumnScan_A_star(State):
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'ColumnScan_A_star'
        self.time_executing = 0  # Variavel para contagem do tempo de execução
        print('ColumnScan_A_star')
        self.finished = False

        # Map resolution
        self.cols = int(SCREEN_WIDTH / RESOLUTION)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT / RESOLUTION)  # Rows of the grid

        # waypoints to be followed
        self.waypoints = []
        self.next_waypoint = 0

        # for checking if it is blocked
        self.memory_last_position = vec2(inf, inf)
        self.sampling_time = 2
        self.time_blocked = 0
        self.blocked = False
        self.waypoints_generated = False
        self.actual_waypoint = None

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        if self.waypoints_generated == False:
            waypoints = []  # initial position
            # size of the grid
            cols, rows = agent.grid_map.get_size()
            index = agent.index
            num_swarm = len(agent.positions_drones)
            cols_ratio = int(cols/num_swarm)



            initial_c = index* cols_ratio * RESOLUTION + RESOLUTION / 2
            initial_r = RESOLUTION / 2

            waypoints.append(vec2(initial_c                 , initial_r)) #<-- Centraliza inicial
            waypoints.append(vec2(initial_c                 , SCREEN_HEIGHT - initial_r)) #<-- Desce
            waypoints.append(vec2(initial_c + RESOLUTION    , SCREEN_HEIGHT - initial_r)) #<-- Direita
            waypoints.append(vec2(initial_c + RESOLUTION    , initial_r)) #<-- Sobe
            waypoints.append(vec2(initial_c + 2* RESOLUTION , initial_r)) #<-- Direita

            waypoints.append(vec2(initial_c + 2* RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 3* RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 3* RESOLUTION, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 4* RESOLUTION, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 4 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 5 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 5 * RESOLUTION, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 6 * RESOLUTION, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 6 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 7 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 7 * RESOLUTION, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 8 * RESOLUTION, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 8 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 9 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 9 * RESOLUTION, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 10 * RESOLUTION, initial_r))  # <-- Direita

            self.waypoints_generated = True

            agent.waypoints = waypoints

            agent.mission_target = agent.waypoints.pop(0)

            #print(initial_r)
        # verifica se chegou
        d = agent.mission_target.distance_to(agent.get_position())

        #print(agent.waypoints)

        if d <= 15 and len(agent.waypoints) > 0:
            agent.mission_target = agent.waypoints.pop(0)


        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(A_star())

        try:
            self.state_name = f'TARGET: {self.target}'
        except:
            pass

        # distancia percorrida desde a amostragem
        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 15 and self.finished == False:
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 20:
                #state_machine.change_state(GoToClosestDroneState())
                self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
                pass

    def execute(self, agent):
        # logic to move drone to target
        try:  # verifica se o drone já te um target ou seja, uma coluna a cobrir
            self.target
        except:  # nao tem, logo:
            self.target = agent.mission_target #self.generate_waypoints() #agent.mission_target
            agent.grid_map.get_sucessors(agent.position_in_grid)

        agent.arrive(agent.mission_target)

        self.time_executing += SAMPLE_TIME

        if (self.target - agent.location).length() < RADIUS_OBSTACLES * 2:
            self.target = vec2(random.uniform(0, SCREEN_WIDTH), random.uniform(0, SCREEN_HEIGHT))
            # rint(f'EU IRIA PARA A CELULA : {agent.grid_map.get_cell_not_visited()}')

        # target is found by a drone in the swarm
        if agent.found:
            self.finished = True

        # Sampling location every T seconds
        if self.time_executing >= self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = copy.deepcopy(agent.get_position())


class ColumnScan_Seek(State):
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'ColumnScan_Seek'
        self.time_executing = 0  # Variavel para contagem do tempo de execução
        print('ColumnScan_Seek')
        self.finished = False

        # Map resolution
        self.cols = int(SCREEN_WIDTH / RESOLUTION)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT / RESOLUTION)  # Rows of the grid

        # waypoints to be followed
        self.waypoints = []
        self.next_waypoint = 0

        # for checking if it is blocked
        self.memory_last_position = vec2(inf, inf)
        self.sampling_time = 2
        self.time_blocked = 0
        self.blocked = False
        self.waypoints_generated = False
        self.actual_waypoint = None

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        if self.waypoints_generated == False:
            waypoints = []  # initial position
            # size of the grid
            cols, rows = agent.grid_map.get_size()
            index = agent.index
            num_swarm = len(agent.positions_drones)
            cols_ratio = int(cols/num_swarm)



            initial_c = index* cols_ratio * RESOLUTION + RESOLUTION / 2
            initial_r = RESOLUTION / 2

            waypoints.append(vec2(initial_c, initial_r))  # <-- Centraliza inicial
            waypoints.append(vec2(initial_c, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + RESOLUTION, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 2 * RESOLUTION, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 2 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 3 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 3 * RESOLUTION, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 4 * RESOLUTION, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 4 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 5 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 5 * RESOLUTION, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 6 * RESOLUTION, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 6 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 7 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 7 * RESOLUTION, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 8 * RESOLUTION, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 8 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 9 * RESOLUTION, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 9 * RESOLUTION, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 10 * RESOLUTION, initial_r))  # <-- Direita

            self.waypoints_generated = True

            agent.waypoints = waypoints

            agent.mission_target = agent.waypoints.pop(0)

            #print(initial_r)
        # verifica se chegou
        d = agent.mission_target.distance_to(agent.get_position())

        #print(agent.waypoints)

        if d <= 15 and len(agent.waypoints) > 0:
            agent.mission_target = agent.waypoints.pop(0)


        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(SeekState())

        try:
            self.state_name = f'TARGET: {self.target}'
        except:
            pass

        # distancia percorrida desde a amostragem
        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 15 and self.finished == False:
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 20:
                #state_machine.change_state(GoToClosestDroneState())
                self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
                pass

    def execute(self, agent):
        # logic to move drone to target
        try:  # verifica se o drone já te um target ou seja, uma coluna a cobrir
            self.target
        except:  # nao tem, logo:
            self.target = agent.mission_target #self.generate_waypoints() #agent.mission_target
            agent.grid_map.get_sucessors(agent.position_in_grid)

        agent.arrive(agent.mission_target)

        self.time_executing += SAMPLE_TIME

        if (self.target - agent.location).length() < RADIUS_OBSTACLES * 2:
            self.target = vec2(random.uniform(0, SCREEN_WIDTH), random.uniform(0, SCREEN_HEIGHT))
            # rint(f'EU IRIA PARA A CELULA : {agent.grid_map.get_cell_not_visited()}')

        # target is found by a drone in the swarm
        if agent.found:
            self.finished = True

        # Sampling location every T seconds
        if self.time_executing >= self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = copy.deepcopy(agent.get_position())

class ColumnScan_small_Astar(State):
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'ColumnScan_small_Astar'
        self.time_executing = 0  # Variavel para contagem do tempo de execução
        print('ColumnScan_small_Astar')
        self.finished = False

        # Map resolution
        self.cols = int(SCREEN_WIDTH / RESOLUTION)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT / RESOLUTION)  # Rows of the grid

        # waypoints to be followed
        self.waypoints = []
        self.next_waypoint = 0

        # for checking if it is blocked
        self.memory_last_position = vec2(inf, inf)
        self.sampling_time = 2
        self.time_blocked = 0
        self.blocked = False
        self.waypoints_generated = False
        self.actual_waypoint = None

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        if self.waypoints_generated == False:
            waypoints = []  # initial position
            # size of the grid
            cols, rows = agent.grid_map.get_size()
            index = agent.index
            num_swarm = len(agent.positions_drones)
            cols_ratio = int(cols/num_swarm)

            initial_c = index  * RESOLUTION + RESOLUTION / 2
            initial_r = RESOLUTION / 2

            waypoints.append(vec2(initial_c, initial_r))  # <-- Centraliza inicial
            waypoints.append(vec2(initial_c, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + RESOLUTION*num_swarm, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 2 * RESOLUTION*num_swarm, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 2 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 3 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 3 * RESOLUTION*num_swarm, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 4 * RESOLUTION*num_swarm, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 4 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 5 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 5 * RESOLUTION*num_swarm, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 6 * RESOLUTION*num_swarm, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 6 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 7 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 7 * RESOLUTION*num_swarm, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 8 * RESOLUTION*num_swarm, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 8 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 9 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 9 * RESOLUTION*num_swarm, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 10 * RESOLUTION*num_swarm, initial_r))  # <-- Direita

            self.waypoints_generated = True

            agent.waypoints = waypoints

            agent.mission_target = agent.waypoints.pop(0)

            #print(initial_r)
        # verifica se chegou
        d = agent.mission_target.distance_to(agent.get_position())

        #print(agent.waypoints)

        if d <= RESOLUTION/2 and len(agent.waypoints) > 0:
            agent.mission_target = agent.waypoints.pop(0)


        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(A_star())

        try:
            self.state_name = f'TARGET: {self.target}'
        except:
            pass

        # distancia percorrida desde a amostragem
        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 15 and self.finished == False:
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 20:
                #state_machine.change_state(GoToClosestDroneState())
                self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
                pass

    def execute(self, agent):
        # logic to move drone to target
        try:  # verifica se o drone já te um target ou seja, uma coluna a cobrir
            self.target
        except:  # nao tem, logo:
            self.target = agent.mission_target #self.generate_waypoints() #agent.mission_target
            agent.grid_map.get_sucessors(agent.position_in_grid)

        agent.arrive(agent.mission_target)

        self.time_executing += SAMPLE_TIME

        if (self.target - agent.location).length() < RADIUS_OBSTACLES * 2:
            self.target = vec2(random.uniform(0, SCREEN_WIDTH), random.uniform(0, SCREEN_HEIGHT))
            # rint(f'EU IRIA PARA A CELULA : {agent.grid_map.get_cell_not_visited()}')

        # target is found by a drone in the swarm
        if agent.found:
            self.finished = True

        # Sampling location every T seconds
        if self.time_executing >= self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = copy.deepcopy(agent.get_position())


class ColumnScan_small_Seek(State):
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'ColumnScan_small_Seek'
        self.time_executing = 0  # Variavel para contagem do tempo de execução
        print('ColumnScan_small_Seek')
        self.finished = False

        # Map resolution
        self.cols = int(SCREEN_WIDTH / RESOLUTION)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT / RESOLUTION)  # Rows of the grid

        # waypoints to be followed
        self.waypoints = []
        self.next_waypoint = 0

        # for checking if it is blocked
        self.memory_last_position = vec2(inf, inf)
        self.sampling_time = 2
        self.time_blocked = 0
        self.blocked = False
        self.waypoints_generated = False
        self.actual_waypoint = None

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        if self.waypoints_generated == False:
            waypoints = []  # initial position
            # size of the grid
            cols, rows = agent.grid_map.get_size()
            index = agent.index
            num_swarm = len(agent.positions_drones)
            cols_ratio = int(cols/num_swarm)

            initial_c = index  * RESOLUTION + RESOLUTION / 2
            initial_r = RESOLUTION / 2

            waypoints.append(vec2(initial_c, initial_r))  # <-- Centraliza inicial
            waypoints.append(vec2(initial_c, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + RESOLUTION*num_swarm, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 2 * RESOLUTION*num_swarm, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 2 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 3 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 3 * RESOLUTION*num_swarm, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 4 * RESOLUTION*num_swarm, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 4 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 5 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 5 * RESOLUTION*num_swarm, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 6 * RESOLUTION*num_swarm, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 6 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 7 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 7 * RESOLUTION*num_swarm, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 8 * RESOLUTION*num_swarm, initial_r))  # <-- Direita

            waypoints.append(vec2(initial_c + 8 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Desce
            waypoints.append(vec2(initial_c + 9 * RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Direita
            waypoints.append(vec2(initial_c + 9 * RESOLUTION*num_swarm, initial_r))  # <-- Sobe
            waypoints.append(vec2(initial_c + 10 * RESOLUTION*num_swarm, initial_r))  # <-- Direita

            self.waypoints_generated = True

            agent.waypoints = waypoints

            agent.mission_target = agent.waypoints.pop(0)

            #print(initial_r)
        # verifica se chegou
        d = agent.mission_target.distance_to(agent.get_position())

        #print(agent.waypoints)

        if d <= RESOLUTION/2 and len(agent.waypoints) > 0:
            agent.mission_target = agent.waypoints.pop(0)


        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(SeekState())

        try:
            self.state_name = f'TARGET: {self.target}'
        except:
            pass

        # distancia percorrida desde a amostragem
        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 15 and self.finished == False:
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 20:
                #state_machine.change_state(GoToClosestDroneState())
                self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
                pass

    def execute(self, agent):
        # logic to move drone to target
        try:  # verifica se o drone já te um target ou seja, uma coluna a cobrir
            self.target
        except:  # nao tem, logo:
            self.target = agent.mission_target #self.generate_waypoints() #agent.mission_target
            agent.grid_map.get_sucessors(agent.position_in_grid)

        agent.arrive(agent.mission_target)

        self.time_executing += SAMPLE_TIME

        if (self.target - agent.location).length() < RADIUS_OBSTACLES * 2:
            self.target = vec2(random.uniform(0, SCREEN_WIDTH), random.uniform(0, SCREEN_HEIGHT))
            # rint(f'EU IRIA PARA A CELULA : {agent.grid_map.get_cell_not_visited()}')

        # target is found by a drone in the swarm
        if agent.found:
            self.finished = True

        # Sampling location every T seconds
        if self.time_executing >= self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = copy.deepcopy(agent.get_position())


class TestScan_Seek(State):
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'TestScan_Seek'
        self.time_executing = 0  # Variavel para contagem do tempo de execução
        print('TestScan_Seek')
        self.finished = False

        # Map resolution
        self.cols = int(SCREEN_WIDTH / RESOLUTION)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT / RESOLUTION)  # Rows of the grid

        # waypoints to be followed
        self.waypoints = []
        self.next_waypoint = 0

        # for checking if it is blocked
        self.memory_last_position = vec2(inf, inf)
        self.sampling_time = 2
        self.time_blocked = 0
        self.blocked = False
        self.waypoints_generated = False
        self.actual_waypoint = None

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        if self.waypoints_generated == False:
            waypoints = []  # initial position
            # size of the grid
            cols, rows = agent.grid_map.get_size()
            index = agent.index
            num_swarm = len(agent.positions_drones)
            cols_ratio = int(cols/num_swarm)

            initial_c = index * cols_ratio * RESOLUTION + RESOLUTION / 2
            initial_r = RESOLUTION / 2


            waypoints.append(vec2(initial_c                 , initial_r)) #<-- Centraliza inicial
            waypoints.append(vec2(initial_c + RESOLUTION*cols_ratio, initial_r))
            waypoints.append(vec2(initial_c + RESOLUTION * cols_ratio, SCREEN_HEIGHT - initial_r))
            waypoints.append(vec2(initial_c , SCREEN_HEIGHT - initial_r))

            waypoints.append(vec2(initial_c , initial_r + RESOLUTION))
            waypoints.append(vec2(initial_c + RESOLUTION * (cols_ratio-1), initial_r + RESOLUTION))
            waypoints.append(vec2(initial_c + RESOLUTION * (cols_ratio - 1), SCREEN_HEIGHT - initial_r - RESOLUTION))
            waypoints.append(vec2(initial_c + RESOLUTION, SCREEN_HEIGHT - initial_r - RESOLUTION))


            self.waypoints_generated = True

            agent.waypoints = waypoints

            agent.mission_target = agent.waypoints.pop(0)

            #print(initial_r)
        # verifica se chegou
        d = agent.mission_target.distance_to(agent.get_position())

        #print(agent.waypoints)

        if d <= RESOLUTION/2 and len(agent.waypoints) > 0:
            agent.mission_target = agent.waypoints.pop(0)


        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(SeekState())

        try:
            self.state_name = f'TARGET: {self.target}'
        except:
            pass

        # distancia percorrida desde a amostragem
        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 15 and self.finished == False:
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 10:
                #state_machine.change_state(GoToClosestDroneState())
                self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
                pass

    def execute(self, agent):
        # logic to move drone to target
        try:  # verifica se o drone já te um target ou seja, uma coluna a cobrir
            self.target
        except:  # nao tem, logo:
            self.target = agent.mission_target #self.generate_waypoints() #agent.mission_target
            agent.grid_map.get_sucessors(agent.position_in_grid)

        agent.arrive(agent.mission_target)

        self.time_executing += SAMPLE_TIME

        if (self.target - agent.location).length() < RADIUS_OBSTACLES * 2:
            self.target = vec2(random.uniform(0, SCREEN_WIDTH), random.uniform(0, SCREEN_HEIGHT))
            # rint(f'EU IRIA PARA A CELULA : {agent.grid_map.get_cell_not_visited()}')

        # target is found by a drone in the swarm
        if agent.found:
            self.finished = True

        # Sampling location every T seconds
        if self.time_executing >= self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = copy.deepcopy(agent.get_position())

class TestScan_A_star(State):
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'TestScan_A_star'
        self.time_executing = 0  # Variavel para contagem do tempo de execução
        print('TestScan_A_star')
        self.finished = False

        # Map resolution
        self.cols = int(SCREEN_WIDTH / RESOLUTION)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT / RESOLUTION)  # Rows of the grid

        # waypoints to be followed
        self.waypoints = []
        self.next_waypoint = 0

        # for checking if it is blocked
        self.memory_last_position = vec2(inf, inf)
        self.sampling_time = 2
        self.time_blocked = 0
        self.blocked = False
        self.waypoints_generated = False
        self.actual_waypoint = None

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        if self.waypoints_generated == False:
            waypoints = []  # initial position
            # size of the grid
            cols, rows = agent.grid_map.get_size()
            index = agent.index
            num_swarm = len(agent.positions_drones)
            cols_ratio = int(cols/num_swarm)

            initial_c = index * cols_ratio * RESOLUTION + RESOLUTION / 2
            initial_r = RESOLUTION / 2


            waypoints.append(vec2(initial_c                 , initial_r)) #<-- Centraliza inicial
            waypoints.append(vec2(initial_c + RESOLUTION*cols_ratio, initial_r))
            waypoints.append(vec2(initial_c + RESOLUTION * cols_ratio, SCREEN_HEIGHT - initial_r))
            waypoints.append(vec2(initial_c , SCREEN_HEIGHT - initial_r))

            waypoints.append(vec2(initial_c , initial_r + RESOLUTION))
            waypoints.append(vec2(initial_c + RESOLUTION * (cols_ratio-1), initial_r + RESOLUTION))
            waypoints.append(vec2(initial_c + RESOLUTION * (cols_ratio - 1), SCREEN_HEIGHT - initial_r - RESOLUTION))
            waypoints.append(vec2(initial_c + RESOLUTION, SCREEN_HEIGHT - initial_r - RESOLUTION))


            #waypoints.append(vec2(initial_c + RESOLUTION * cols_ratio, ))
            #waypoints.append(vec2(initial_c, SCREEN_HEIGHT - initial_r))



            # waypoints.append(vec2(initial_c                 , SCREEN_HEIGHT - initial_r)) #<-- Desce
            # waypoints.append(vec2(initial_c + RESOLUTION   *num_swarm , SCREEN_HEIGHT - initial_r)) #<-- Direita
            # waypoints.append(vec2(initial_c + RESOLUTION   *num_swarm, initial_r)) #<-- Sobe
            # waypoints.append(vec2(initial_c + 2*RESOLUTION *num_swarm, initial_r)) #<-- Direita
            #
            # waypoints.append(vec2(initial_c + 2* RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Desce
            # waypoints.append(vec2(initial_c + 3* RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Direita
            # waypoints.append(vec2(initial_c + 3* RESOLUTION*num_swarm, initial_r))  # <-- Sobe
            # waypoints.append(vec2(initial_c + 4* RESOLUTION*num_swarm, initial_r))  # <-- Direita
            #
            # waypoints.append(vec2(initial_c + 4* RESOLUTION*num_swarm, SCREEN_HEIGHT - initial_r))  # <-- Desce

            self.waypoints_generated = True

            agent.waypoints = waypoints

            agent.mission_target = agent.waypoints.pop(0)

            #print(initial_r)
        # verifica se chegou
        d = agent.mission_target.distance_to(agent.get_position())

        #print(agent.waypoints)

        if d <= RESOLUTION/2 and len(agent.waypoints) > 0:
            agent.mission_target = agent.waypoints.pop(0)


        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(A_star())

        try:
            self.state_name = f'TARGET: {self.target}'
        except:
            pass

        # distancia percorrida desde a amostragem
        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 15 and self.finished == False:
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 10:
                #state_machine.change_state(GoToClosestDroneState())
                self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
                pass

    def execute(self, agent):
        # logic to move drone to target
        try:  # verifica se o drone já te um target ou seja, uma coluna a cobrir
            self.target
        except:  # nao tem, logo:
            self.target = agent.mission_target #self.generate_waypoints() #agent.mission_target
            agent.grid_map.get_sucessors(agent.position_in_grid)

        agent.arrive(agent.mission_target)

        self.time_executing += SAMPLE_TIME

        if (self.target - agent.location).length() < RADIUS_OBSTACLES * 2:
            self.target = vec2(random.uniform(0, SCREEN_WIDTH), random.uniform(0, SCREEN_HEIGHT))
            # rint(f'EU IRIA PARA A CELULA : {agent.grid_map.get_cell_not_visited()}')

        # target is found by a drone in the swarm
        if agent.found:
            self.finished = True

        # Sampling location every T seconds
        if self.time_executing >= self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = copy.deepcopy(agent.get_position())


class A_star_teste(State):
    """
        Teste A*
    """

    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'PathAStar'
        self.time_executing = 0  # Variavel para contagem do tempo de execução
        print('PathAStar')
        self.finished = False
        self.memory_last_position = vec2(inf, inf)
        self.sampling_time = 3
        self.time_blocked = 0
        self.final_goal = None
        self.sub_path = False
        self.visited = []

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # New target from mouse click
        if agent.get_target():
            self.final_goal = agent.get_target()  # Target location
            self.target = self.final_goal  # agent.get_target()
            # agent.mission_target = vec2(random.uniform(100,300),random.uniform(100,300))  # <--- Define a posicao que o drone deve ir... Valor antigo: agent.get_target()
            agent.set_target(None)

        dist = self.memory_last_position.distance_to(agent.get_position())

        # verifica se chegou
        d = self.final_goal.distance_to(agent.get_position())

        agent.mission_target = self.final_goal

        if self.sub_path == False:
            self.sub_path = True
            self.visited.append(agent.position_in_grid)

            near_grid = agent.grid_map.get_all_sucessors(agent.position_in_grid, step=agent.step)


            print(near_grid)



        # # dist1 = agent.memory_location
        # # print(dist)
        #
        # if d > RADIUS_TARGET:
        #     agent.step = int(d/RESOLUTION)
        #     if agent.step > 3:
        #         agent.step = 3
        #
        #     if self.time_blocked > 2:
        #         agent.step += 1
        #
        # if self.sub_path == False:
        #     self.sub_path = True
        #     near_grid = agent.grid_map.get_all_sucessors(agent.position_in_grid, step=agent.step)
        #
        #     # print(agent.pos_obstacles)
        #     # print(agent.positions_drones)
        #
        #     aux_list = []
        #     close_point = False
        #
        #     for i in near_grid:
        #         dist_a = vec2(i[0], i[1]).distance_to(self.final_goal)
        #         dist_a += abs(i[0] - self.final_goal[0]) + abs(i[1] - self.final_goal[1])
        #
        #         for j in agent.pos_obstacles:
        #             # print(vec2(j[0],j[1]).distance_to(agent.get_position()))
        #             dist_obs = vec2(j[0], j[1]).distance_to(vec2(i[0], i[1]))
        #
        #             # print(dist_obs)
        #             if dist_obs < AVOID_OBSTACLES + 25:
        #                 close_point = True
        #                 break
        #
        #             if dist_obs <= RESOLUTION*2.5:
        #                 dist_a += 10 #1 / dist_obs ** 2 #abs(i[0] - j[0]) + abs(i[1] - j[1]) #
        #
        #         if close_point == True:
        #             close_point = False
        #             continue
        #
        #         point = vec2(i[0], i[1])
        #         aux_list.append([point, dist_a])
        #
        #     best_point = None
        #     best_value = inf
        #
        #     for p in aux_list:
        #         point = p[0]
        #         value = p[1]
        #         if best_value > value:
        #             best_value = value
        #             best_point = point
        #
        #     if best_point is None:
        #         best_point = vec2(agent.get_position()[0] + BLOCKSIZE/2, agent.get_position()[0] + BLOCKSIZE/2)
        #
        #
        #
        #     agent.mission_target = best_point
        # else:
        #     sub_path_dist = agent.mission_target.distance_to(agent.get_position())
        #     if sub_path_dist < RESOLUTION/2:
        #         self.sub_path = False
        #
        # if d <= RADIUS_TARGET and d > 3:
        #     self.finished = True
        #     self.state_name = 'Done'
        #
        # # Verifica se terminou a execucao
        # if self.finished == True:
        #     pass
        #
        # if dist < 50 and self.finished == False:
        #     self.time_blocked += SAMPLE_TIME
        #     self.state_name = f'Blocked: {self.time_blocked:.2f}'
        #
        #     if self.time_blocked > 4:
        #         pos_in_grid = agent.position_in_grid
        #         agent.grid_map.change_state_cell(pos_in_grid, OBSTACLE)
        #
        #         self.sub_path = False
        #         # state_machine.change_state(PathAStar())
        #         state_machine.change_state(GoToClosestDroneStateA_star())
        #
        # if dist > RESOLUTION:
        #     self.time_blocked = 0
        #         # self.target = vec2(random.uniform(agent.get_position()[0] - RESOLUTION/2,agent.get_position()[0] + RESOLUTION/2),
        #         #                    random.uniform(agent.get_position()[1] - RESOLUTION/2,agent.get_position()[1] + RESOLUTION/2))

    def execute(self, agent):
        # logic to move drone to target
        try:
            self.target
        except:
            self.target = agent.mission_target

        agent.arrive(agent.mission_target)  # self.target)
        self.time_executing += SAMPLE_TIME

        # Sampling location every T seconds
        if self.time_executing >= self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = copy.deepcopy(agent.get_position())