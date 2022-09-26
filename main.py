import sys, pygame
from constants import *
import random 
import copy
from scan import * #DefineTargetScan, RowScan
from obstacle import Obstacles
from simulation import Simulation, ScreenSimulation, RateSimulation
from grid import GridField
import matplotlib.pyplot as plt


vec2 = pygame.math.Vector2
##=========================
screenSimulation = ScreenSimulation()

background_image = pygame.image.load("models/texture/camouflage.png").convert()
background_image = pygame.transform.scale(background_image,(SCREEN_WIDTH,SCREEN_HEIGHT))
# defines initial target
target = vec2(random.uniform(0,SCREEN_WIDTH/2), random.uniform(0,SCREEN_HEIGHT/2))

simulation = Simulation(screenSimulation, RateSimulation(1, [2,3,4], [2,3,4,5,6,7,8,9,10,11,12,15,17,20,25], [DefineTargetScan(),DefineTargetScan_A_star()]))
    #  Possible Scan Methods:
    # - DefineTargetScan()
    # - TestScan_Seek()
    # - TestScan_A_star()
    # - ColumnScan_small_A_star()
    # - ColumnScan_small_Seek()
    # - RowScan_AStar()
    # - RowScan()
    # - ColumnScan_A_star()
    # - ColumnScan_Seek()


run = True
while run:
    # Draws at every dt
    screenSimulation.clock.tick(FREQUENCY)

    # Pygame Events 
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()
        
        # Key 'd' pressed
        if event.type == pygame.KEYDOWN and event.key == pygame.K_d:
            for _ in simulation.swarm:
                _.set_debug()

        # Mouse Clicked -> new taget or new Drone 
        if event.type == pygame.MOUSEBUTTONDOWN:
            # left button - New Target
            if pygame.mouse.get_pressed()[0] == True:
                target = vec2(pygame.mouse.get_pos()[0],pygame.mouse.get_pos()[1])
                simulation.set_target(target)

            # right button - New Drone
            if pygame.mouse.get_pressed()[2] == True:
                simulation.add_new_uav()              
                
    # Background
    #screenSimulation.screen.fill(LIGHT_BLUE)
    screenSimulation.screen.blit(background_image, [0, 0])

    # updates and draws all simulations  
    run = simulation.run_simulation()

    for idx, time in enumerate(simulation.rate.out_time_mission):
        try:
            search = simulation.rate.in_algorithms[simulation.rate.current_repetition].to_string()
            img = screenSimulation.font20.render(f'{idx+1} - Search: {search} -  Scan Time: {time:.2f}, {simulation.rate.print_simulation_idx(idx)}', True, LIGHT_BLUE)
        except:
            search = simulation.rate.in_algorithms[idx].to_string()
            img = screenSimulation.font16.render(f'{idx+1} -  Search: {search} - Scan Time: {time}', True, LIGHT_BLUE)
        screenSimulation.screen.blit(img, (20, 20*(idx+2)))
        
    # Writes the App name in screen
    img = screenSimulation.font24.render('Swarm Search using Drones', True, LIGHT_BLUE)
    screenSimulation.screen.blit(img, (20, 20))
    # Writes current Search name in screen

    search = simulation.rate.in_algorithms[simulation.rate.current_repetition].to_string()
    img = screenSimulation.font24.render(f'Search using: {search} ', True, LIGHT_BLUE)
    screenSimulation.screen.blit(img, (800, 20))
    # Debug lines - only to assist the developer
    #img = screenSimulation.font24.render('Debug lines: '+ drone.get_debug(), True, BLUE)
    #screenSimulation.screen.blit(img, (20, 40))

    pygame.display.flip()
    
    if not run:
        pygame.time.wait(1000) 

simulation.rate.save_csv()
simulation.rate.print_rate()



