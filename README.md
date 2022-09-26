# Drone Swarm in Search Target Scenario applying 

This simulator aims to facilitate future simulations using drones swarms for search applications, or any different scenarios, through an easy to use and modify, open-source implementation. 

To run the program it is necessary:

- PYTHON
- VSCode or some other code editor

Dependencies:

- PYGAME -> https://www.pygame.org/wiki/GettingStarted
     - pip install pygame
- NUMPY -> https://numpy.org/
     - pip install numpy


-> To execute the simulator -:: Run the file: main.py ::-

Files Description:

     Main.py - main file
     utils.py - Useful functions used
     constants.py - parameters for the simulation
     state_machine.py - Implementation of the decision making (State Machine and Behaviors)
     vehicle.py - Implementation of the vehicle (drone) logic and its basic movements
     obstacle.py - Obstacle generation logic
     scan.py - Search strategies logic
     grid.py - Discrete map logic

The model folder contains sprites used for drone animation

https://github.com/luizgiacomossi/Search_Drone_Swarms/tree/grid_search
