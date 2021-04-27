import os
import time
import subprocess

'''path_to_bin = "/Users/clarence/Desktop/multirobot/new_clean_code/try-multirobot"
path_to_map = "/Users/clarence/Desktop/multirobot/maps/empty-16-16.map"
paths_to_scen = "/Users/clarence/Desktop/multirobot/maps/empty-16-16-scen-even/empty-16-16-even-"'''

'''path_to_bin = "/Users/clarence/Desktop/multirobot/new_clean_code/try-multirobot"
path_to_map = "/Users/clarence/Desktop/multirobot/maps/berlin_1_256.map"
paths_to_scen = "/Users/clarence/Desktop/multirobot/maps/berlin-scen-even/Berlin_1_256-even-"'''

path_to_bin = "/Users/clarence/Desktop/multirobot/new_clean_code/try-multirobot"
path_to_map = "/Users/clarence/Desktop/multirobot/maps/warehouse-10-20-10-2-2.map"
paths_to_scen = "/Users/clarence/Desktop/multirobot/maps/warehouse-scen-even/warehouse-10-20-10-2-2-even-"

'''path_to_bin = "/Users/clarence/Desktop/multirobot/new_clean_code/try-multirobot"
path_to_map = "/Users/clarence/Desktop/multirobot/maps/brc202d.map"
paths_to_scen = "/Users/clarence/Desktop/multirobot/maps/brc202d-scen-even/brc202d-even-"'''

MAX_AGENTS = 100 # maximum number of agents
MAX_SCEN = 25 # maximum number of scenarios
MAX_FAILED = 4 # maximum consecutive failures
TIME_OUT = 60 # time limit

dijkstra_precalc = "false" # true -> use dijkstra precalc
use_CAT = "true" # true -> use CAT
heuristic = "normal" # heuristic (normal, normal_diagonal, number_of_conflicts, number_of_conflicting_agents, number_of_pairs, vertex_cover)
prioritize_conflicts = "false" # true -> prioritize conflicts
use_bypass = "true" # true -> use bypass
use_ecbs = "false" # true -> use ecbs
omega = "1" # suboptimality factor
print_paths = "false" # true -> paths for each agent are shown, false -> only success rates are shown

success_rates = [] # percent of passed tests for each number of agents
failed = [0] * MAX_SCEN # for counting consecutive failures

for agents in range(40, MAX_AGENTS + 1): # choose a number of agents
    num_of_passed_tests = 0

    for cur_scen in range(0, MAX_SCEN): # choose a scenario
        if (failed[cur_scen] <= MAX_FAILED): # if the number of consecutive failures doesn't exceed MAX_FAILED
            # start the timer and create a subprocess
            timeStarted = time.time()
            test = subprocess.Popen([path_to_bin, path_to_map, paths_to_scen + str(cur_scen + 1) + ".scen", str(agents), dijkstra_precalc, use_CAT, heuristic, prioritize_conflicts, use_bypass, use_ecbs, omega, print_paths])
            
            try:
                test.wait(timeout = TIME_OUT)
            except subprocess.TimeoutExpired:
                test.kill()
                failed[cur_scen] += 1
                timeDelta = time.time() - timeStarted
                print("agents:", agents, "scenario:", cur_scen, "time:", "%.5f" % timeDelta, "result: FAILURE") # time limit exceeded
            else:
                num_of_passed_tests += 1
                timeDelta = time.time() - timeStarted
                print("agents:", agents, "scenario:", cur_scen, "time:", "%.5f" % timeDelta, "result: SUCCESS")
        else:
            print("agents:", agents, "scenario:", cur_scen, "time:", "MAX_FAILED exceeded", "result: FAILURE")

    success_rates.append(round(num_of_passed_tests / MAX_SCEN * 100, 4))
    print("\nsuccess rates for each number of agents:", success_rates, "\n")