# A Star Based Algorithm Applied to Target Search and Rescue by an UAV Swarm

Abstract: Drone swarm has been used to improve applications in several areas (e.g., monitoring, surveillance, security, search)
being present in numerous papers due to its potential. This application consists in using a specific number of drones
flying simultaneously and using computational intelligence techniques between the drones to avoid collisions and
increase the effectiveness in conducting search tasks. As a reference, the article Autonomous and Collective
Intelligence for UAV Swarm in Target Search Scenario, presents a proposed solution for coordinating a UAV swarm using
bivariate potential fields with autonomous and distributed intelligence between drones for a cooperative targeting
application. The article Cooperative and Decentralized Decision-Making for Loyal Wingman UAVs applying methods using
decision tree and PSO as an algorithm for parameter optimization, which allow decentralized decision making for
autonomous combat UAVs. The contribution of this work is to propose a technique to coordinate a drone swarm using search
methods based on the A* algorithm and its heuristics (i.e., Euclidean and Manhattan distances). Thus, the communication
between drones is also evaluated and applied to a target search problem on a two-dimensional cost map. The results show
a considerable optimization in the effectiveness of the drone swarm. Thus, fewer drones are blocked at local minima,
which highlights the contribution of this work in optimizing the intelligence performance of drones compared to previous
approaches.

This is a modification of open source Drone Swarm in Search Target Scenario 
(explained video [here](https://www.youtube.com/watch?v=l07YPjrnLNw) and [paper](https://ieeexplore.ieee.org/document/9605450)). 
As a contribution to the original paper and software, new search patterns were developed in order to test different search heuristics 
using a discrete approach based on the A* algorithm.

To run the program it is necessary:
- PYTHON
- PyCharm, VSCode or some other code editor

Dependencies:
- PYGAME -> https://www.pygame.org/wiki/GettingStarted
     - pip install pygame
- NUMPY -> https://numpy.org/
     - pip install numpy
- PANDAS -> https://pandas.pydata.org/
     - pip install pandas
     
-> To execute the simulator -:: Run the file: main.py ::-

Files Description:

     main.py - Main file
     utils.py - Useful functions used
     constants.py - Global parameters for the simulation
     state_machine.py - Implementation of the decision making, search and approach methods
     vehicle.py - Implementation of the vehicle (drone) logic and its basic movements
     obstacle.py - Obstacle generation logic
     scan.py - Search strategies logic
     grid.py - Discrete map logic

The model folder contains sprites used for drone animation

Original simulation github repo: https://github.com/luizgiacomossi/Search_Drone_Swarms/tree/grid_search
