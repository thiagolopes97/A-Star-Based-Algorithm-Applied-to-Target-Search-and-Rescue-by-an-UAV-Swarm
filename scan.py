from constants import *

class ScanInterface:
    def to_string(self) -> str:
        """return the name of Algorithm"""
        pass
    def prepare_simulation(self, simulation, target):
        pass

    def update_drone(self, drone, simulation, list_obst, index ):
        '''
            This method is used to align drone with swarm
            update collison avoidance, 
            updates velocity and position 
            draw drone on screen
        '''
        drone.align_direction_with_swarm(simulation.swarm, index)
        drone.collision_avoidance(simulation.swarm,list_obst,index) 
        drone.update()
        drone.draw(simulation.screenSimulation.screen) 
    
    def draw_legend(self,drone, simulation, index):
        '''
            This method draws legend under drone
            which describes its position and behavior
        '''
        # index to keep track of  drone in the list
            # writes drone id
        img = simulation.screenSimulation.font20.render(f'Drone {index+1}', True, LIGHT_BLUE)
        simulation.screenSimulation.screen.blit(img, drone.get_position()+(0,20))
            # writes drone current behavior
        img = simulation.screenSimulation.font16.render(drone.behavior.get_current_state(), True, LIGHT_BLUE)
        simulation.screenSimulation.screen.blit(img, drone.get_position()+(0,30))
            # writes drone current position in grid
        p = drone.get_position()
        col = int(p.x/RESOLUTION) 
        row = int(p.y/RESOLUTION) 
        img = simulation.screenSimulation.font16.render(f'Pos:{col},{row}', True, LIGHT_BLUE)
        simulation.screenSimulation.screen.blit(img, drone.get_position()+(0,40))

    def update_grid(self, drone, simulation):
            # Discretized position in grid
        p = drone.get_position()
        col = int(p.x/RESOLUTION) 
        row = int(p.y/RESOLUTION) 

            # changes states of cell to visited 
        simulation.grid_field.change_state_cell((col,row))
        drone.set_position_in_grid(col,row)
        drone.save_grid(simulation.grid_field)

    def scan(self, simulation, list_obst):
        """run the scan algorithm"""
        pass

class DefineTargetScan(ScanInterface):
    '''
        This strategy is when swarm know the target
        Not for search, used for reference!
    '''
    def to_string(self) -> str:
        return 'DefineTargetScan'

    def scan(self, simulation, list_obst):
        index = 0 # index is used to track current drone in the simulation list
        for index, _ in enumerate(simulation.swarm):
            # Discretized position, updates grid and send to drone
            self.update_grid( _ , simulation)

            # align, collison avoidance, updates velocity and position and draw drone
            self.update_drone( _ ,simulation, list_obst,index)
            self.draw_legend( _ , simulation, index)


            
            if _.reached_goal(simulation.target_simulation):
                #print(f"Drone {index} atingiu o target")
                simulation.found = True
                simulation.set_time_target()


class DefineTargetScan_A_star(ScanInterface):
    '''
        This strategy is when swarm know the target
        Not for search, used for reference!
    '''

    def to_string(self) -> str:
        return 'DefineTargetScan_A_star'

    def scan(self, simulation, list_obst):
        index = 0  # index is used to track current drone in the simulation list
        for index, _ in enumerate(simulation.swarm):
            # Discretized position, updates grid and send to drone
            self.update_grid(_, simulation)

            # align, collison avoidance, updates velocity and position and draw drone
            self.update_drone(_, simulation, list_obst, index)
            self.draw_legend(_, simulation, index)

            if _.reached_goal(simulation.target_simulation):
                # print(f"Drone {index} atingiu o target")
                simulation.found = True
                simulation.set_time_target()

class RandoWalkScan(ScanInterface):
    def to_string(self) -> str:
        return 'RandoWalkScan'

    def scan(self, simulation, list_obst):
        pass

class SnookerScan(ScanInterface):
    def to_string(self) -> str:
        return 'SnookerScan'

    def scan(self, simulation, list_obst):
        pass

class RowScan(ScanInterface):
    def to_string(self) -> str:
        return 'RowScan'
    
    def prepare_simulation(self, simulation,target):
        simulation.set_target_using_search_pattern(target)

    def scan(self, simulation, list_obst):
        index = 0 # index is used to track current drone in the simulation list
        for index, _ in enumerate(simulation.swarm):
            # Discretized position, updates grid and send to drone
            self.update_grid( _ , simulation)
            # align, collison avoidance, updates velocity and position and draw drone
            self.update_drone( _ ,simulation, list_obst,index)
            self.draw_legend( _ , simulation, index)

            if _.reached_goal(simulation.target_simulation):
                simulation.found = True
                simulation.set_time_target()

    def define_search_area(self):
        pass

class MeshScan(ScanInterface):
    def to_string(self) -> str:
        return 'MeshScan'

    def scan(self, simulation, list_obst):
        pass


class RowScan_AStar(ScanInterface):
    def to_string(self) -> str:
        return 'RowScan_AStar'

    def prepare_simulation(self, simulation, target):
        simulation.set_target_using_search_pattern(target)

    def scan(self, simulation, list_obst):
        index = 0  # index is used to track current drone in the simulation list
        for index, _ in enumerate(simulation.swarm):
            # Discretized position, updates grid and send to drone
            self.update_grid(_, simulation)
            # align, collison avoidance, updates velocity and position and draw drone
            self.update_drone(_, simulation, list_obst, index)
            self.draw_legend(_, simulation, index)

            if _.reached_goal(simulation.target_simulation):
                simulation.found = True
                simulation.set_time_target()

    def define_search_area(self):
        pass

class RowScan_AStar(ScanInterface):
    def to_string(self) -> str:
        return 'RowScan_AStar'

    def prepare_simulation(self, simulation, target):
        simulation.set_target_using_search_pattern(target)

    def scan(self, simulation, list_obst):
        index = 0  # index is used to track current drone in the simulation list
        for index, _ in enumerate(simulation.swarm):
            # Discretized position, updates grid and send to drone
            self.update_grid(_, simulation)
            # align, collison avoidance, updates velocity and position and draw drone
            self.update_drone(_, simulation, list_obst, index)
            self.draw_legend(_, simulation, index)

            if _.reached_goal(simulation.target_simulation):
                simulation.found = True
                simulation.set_time_target()

    def define_search_area(self):
        pass

class ColumnScan_A_star(ScanInterface):
    def to_string(self) -> str:
        return 'ColumnScan_A_star'

    def prepare_simulation(self, simulation, target):
        simulation.set_target_using_search_pattern(target)

    def scan(self, simulation, list_obst):
        index = 0  # index is used to track current drone in the simulation list
        for index, _ in enumerate(simulation.swarm):
            # Discretized position, updates grid and send to drone
            self.update_grid(_, simulation)
            # align, collison avoidance, updates velocity and position and draw drone
            self.update_drone(_, simulation, list_obst, index)
            self.draw_legend(_, simulation, index)

            if _.reached_goal(simulation.target_simulation):
                simulation.found = True
                simulation.set_time_target()

    def define_search_area(self):
        pass

class ColumnScan_Seek(ScanInterface):
    def to_string(self) -> str:
        return 'ColumnScan_Seek'

    def prepare_simulation(self, simulation, target):
        simulation.set_target_using_search_pattern(target)

    def scan(self, simulation, list_obst):
        index = 0  # index is used to track current drone in the simulation list
        for index, _ in enumerate(simulation.swarm):
            # Discretized position, updates grid and send to drone
            self.update_grid(_, simulation)
            # align, collison avoidance, updates velocity and position and draw drone
            self.update_drone(_, simulation, list_obst, index)
            self.draw_legend(_, simulation, index)

            if _.reached_goal(simulation.target_simulation):
                simulation.found = True
                simulation.set_time_target()

    def define_search_area(self):
        pass

class ColumnScan_small_A_star(ScanInterface):
    def to_string(self) -> str:
        return 'ColumnScan_small_A_star'

    def prepare_simulation(self, simulation, target):
        simulation.set_target_using_search_pattern(target)

    def scan(self, simulation, list_obst):
        index = 0  # index is used to track current drone in the simulation list
        for index, _ in enumerate(simulation.swarm):
            # Discretized position, updates grid and send to drone
            self.update_grid(_, simulation)
            # align, collison avoidance, updates velocity and position and draw drone
            self.update_drone(_, simulation, list_obst, index)
            self.draw_legend(_, simulation, index)

            if _.reached_goal(simulation.target_simulation):
                simulation.found = True
                simulation.set_time_target()

    def define_search_area(self):
        pass

class ColumnScan_small_Seek(ScanInterface):
    def to_string(self) -> str:
        return 'ColumnScan_small_Seek'

    def prepare_simulation(self, simulation, target):
        simulation.set_target_using_search_pattern(target)

    def scan(self, simulation, list_obst):
        index = 0  # index is used to track current drone in the simulation list
        for index, _ in enumerate(simulation.swarm):
            # Discretized position, updates grid and send to drone
            self.update_grid(_, simulation)
            # align, collison avoidance, updates velocity and position and draw drone
            self.update_drone(_, simulation, list_obst, index)
            self.draw_legend(_, simulation, index)

            if _.reached_goal(simulation.target_simulation):
                simulation.found = True
                simulation.set_time_target()

    def define_search_area(self):
        pass

class TestScan_Seek(ScanInterface):
    def to_string(self) -> str:
        return 'TestScan_Seek'

    def prepare_simulation(self, simulation, target):
        simulation.set_target_using_search_pattern(target)

    def scan(self, simulation, list_obst):
        index = 0  # index is used to track current drone in the simulation list
        for index, _ in enumerate(simulation.swarm):
            # Discretized position, updates grid and send to drone
            self.update_grid(_, simulation)
            # align, collison avoidance, updates velocity and position and draw drone
            self.update_drone(_, simulation, list_obst, index)
            self.draw_legend(_, simulation, index)

            if _.reached_goal(simulation.target_simulation):
                simulation.found = True
                simulation.set_time_target()

    def define_search_area(self):
        pass

class TestScan_A_star(ScanInterface):
    def to_string(self) -> str:
        return 'TestScan_A_star'

    def prepare_simulation(self, simulation, target):
        simulation.set_target_using_search_pattern(target)

    def scan(self, simulation, list_obst):
        index = 0  # index is used to track current drone in the simulation list
        for index, _ in enumerate(simulation.swarm):
            # Discretized position, updates grid and send to drone
            self.update_grid(_, simulation)
            # align, collison avoidance, updates velocity and position and draw drone
            self.update_drone(_, simulation, list_obst, index)
            self.draw_legend(_, simulation, index)

            if _.reached_goal(simulation.target_simulation):
                simulation.found = True
                simulation.set_time_target()

    def define_search_area(self):
        pass
