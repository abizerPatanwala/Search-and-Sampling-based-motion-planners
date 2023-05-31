# Search-and-Sampling-based-motion-planners
This project implements various search based planners (**Dijsktra, A star, D star**) and sampling-based planners(**RRT, RRT star and Informed RRT star**).The planners are
implemented on a 2d map. The map is given as input in the .csv format in the matrix form where '0' value corresponds to free location and '1' value 
corresponds to object location. Each of the folder contains *main.py* file which is responsible for executing the code. The output folder contains the output from the algorithm
as the image file showing the map and the resultant path. To run any of the algorithm, download the corresponding folder and run 
*python3 main.py*. The map can be changed by chaning the values in the .csv file. 

## Resultant path from search based planners:
### Dijkstra
![Dijsktra](Astar_dijsktra/output/dijsktra.png)
### A*
![A star](Astar_dijsktra/output/Astar.png)
### D* static path
![D_star static](D_star/output/Dstar_static.png)
### D* dynamic path
![D_star step 1](D_star/output/Dstar_dyn1.png)
## Resultant path from sampling based planners:
### RRT
![RRT star](RRT_RRTstar/output/RRT.png)
### RRT*
![RRT star](RRT_RRTstar/output/RRT_star.png)
<p float="left">
  <img src="PRM_Planners/output/bridge.png" width="320" />
  <img src="PRM_Planners/output/gaussian.png" width="320" /> 

</p>
### PRM bridge planner
![PRM bridge planner](PRM_Planners/output/bridge.png)
### PRM gaussian planner
![PRM gaussian planner](PRM_Planners/output/gaussian.png)
### PRM random planner
![PRM random planner](PRM_Planners/output/random.png)
### PRM uniform planner
![PRM uniform planner](PRM_Planners/output/uniform.png)
### Informed RRT*
![Informed RRT*](informed_RRT/output/Infomed_RRT_star.png)
