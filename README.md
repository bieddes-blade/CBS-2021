# CBS-2021

## Running the code

If you'd like to use this code, please:
1) compile it (for example, ```g++ main.cpp map.cpp agent.cpp ctNode.cpp ctSolution.cpp search.cpp -o try-multirobot -std=c++0x```),
2) specify the parameters in [call-multirobot.py](call-multirobot.py),
3) and run it with python3.

The code uses maps and scenarios from [Moving AI Lab](https://movingai.com/benchmarks/mapf/index.html).

An example of a map can be found in the file [check.map](check.map) (same with examples of scenario and goal files: [check.scen](check.scen), [check.goals](check.goals)). 
In call-multirobot.py, one can choose which optimizations are going to be tested and enter the parameter values. 
The variables include:
- path_to_bin (path to the executable), 
- path_to_map (path to the map),
- paths_to_scen (paths to scenarios), 
- path_to_goals (path to the file with additional goals),
- MAX_AGENTS (the maximum number of agents),
- MAX_SCEN (the maximum number of scenarios), 
- MAX_FAILED (maximum consecutive failures, after which this scenario will not be included in tests with more agents), 
- TIME_OUT (the time limit in seconds), 
- dijkstra_precalc (set true to use exact heuristic precomputation),
- use_CAT (set true to use the conflict avoidance table), 
- heuristic (choose heuristic: normal, normal_diagonal, number_of_conflicts, number_of_conflicting_agents, number_of_pairs, vertex_cover), 
- prioritize_conflicts (set true to prioritize cardinal and semi-cardinal conflicts),
- use_bypass (set true to use the bypass technique),
- use_ecbs (set true to use suboptimal CBS),
- omega (choose suboptimality factor for ECBS),
- use_symm (set true to use symmetry breaking constraints),
- online (choose an online setting with many goals or an offline setting with one goal),
- horizon (conflicts will be ignored after this number of steps),
- replanning (the number of timesteps before replanning),
- print_paths (set true to print paths for each agent).

## More on file formats

All maps begin with the lines:

```
type octile
height y
width x
map
```

where y and x are the respective height and width of the map.

The map data is store as an ASCII grid. The upper-left corner of the map is (0,0). The following characters are possible:

- `. G S W` - passable terrain,
- `@ O T` - out of bounds or unpassable terrain.

A scenario is a text file with a list of start and finish points for each agent. Each line of a scenario has 9 fields: bucket, map, map width, map height, start x-coordinate, start y-coordinate, goal x-coordinate, goal y-coordinate, optimal length. The optimal path length is assuming sqrt(2) diagonal costs.

A goals file contains additional goals for agents in the online setting. Every line contains a list of goal x-coordinates and goal y-coordinates, followed by -1.

## Code structure

This repository contains 
