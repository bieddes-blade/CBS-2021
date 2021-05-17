import os
import time
import subprocess
from subprocess import Popen, PIPE

path_to_bin = "/multirobot/new_clean_code/try-multirobot"
path_to_map = "/multirobot/maps/warehouse-10-20-10-2-2.map"
paths_to_scen = "/multirobot/maps/warehouse-scen-even/warehouse-10-20-10-2-2-even-"
path_to_goals = "/multirobot/maps/check.goals"

MAX_AGENTS = 100 # maximum number of agents
MAX_SCEN = 25 # maximum number of scenarios
MAX_FAILED = 3 # maximum consecutive failures
TIME_OUT = 300 # time limit

dijkstra_precalc = "false" # set true to use dijkstra precalc
use_CAT = "true" # set true to use CAT
heuristic = "normal" # choose heuristic (normal, normal_diagonal, number_of_conflicts, number_of_conflicting_agents, number_of_pairs, vertex_cover)
prioritize_conflicts = "false" # set true to prioritize conflicts
use_bypass = "true" # set true to use bypass
use_ecbs = "false" # set true to use ecbs
omega = "1" # choose suboptimality factor (only for ecbs)
use_symm = "false" # set true to use symmetry breaking
online = "false" # online setting with many goals or offline setting with one goal
horizon = "100000000" # conflicts will be ignored after this number of steps
replanning = "100000000" # number of timesteps before replanning
print_paths = "false" # set true to print paths for each agent


success_rates = [] # percent of passed tests for each number of agents
failed = [0] * MAX_SCEN # for counting consecutive failures

for agents in range(1, MAX_AGENTS + 1): # choose a number of agents
    num_of_passed_tests = 0

    for cur_scen in range(0, MAX_SCEN): # choose a scenario
        if (failed[cur_scen] <= MAX_FAILED): # if the number of consecutive failures doesn't exceed MAX_FAILED
            # start the timer and create a subprocess
            timeStarted = time.time()
            # uncomment next line to run on multiple tests
            test = subprocess.Popen([path_to_bin, path_to_map, paths_to_scen + str(cur_scen + 1) + ".scen", str(agents), dijkstra_precalc, use_CAT, heuristic, prioritize_conflicts, use_bypass, use_ecbs, omega, use_symm, online, path_to_goals, horizon, replanning, print_paths])
            # uncomment next line to run on one test
            #test = subprocess.Popen([path_to_bin, path_to_map, paths_to_scen, str(agents), dijkstra_precalc, use_CAT, heuristic, prioritize_conflicts, use_bypass, use_ecbs, omega, use_symm, online, path_to_goals, horizon, replanning, print_paths])
            
            try:
                test.wait(timeout = TIME_OUT)
            except subprocess.TimeoutExpired:
                test.kill()
                failed[cur_scen] += 1
                timeDelta = time.time() - timeStarted
                print("agents:", agents, "scenario:", cur_scen, "time:", "%.5f" % timeDelta, "result: FAILURE") # time limit exceeded
            else:
                num_of_passed_tests += 1
                failed[cur_scen] = 0
                timeDelta = time.time() - timeStarted
                print("agents:", agents, "scenario:", cur_scen, "time:", "%.5f" % timeDelta, "result: SUCCESS")
        else:
            print("agents:", agents, "scenario:", cur_scen, "time:", "MAX_FAILED exceeded", "result: FAILURE")

    success_rates.append(round(num_of_passed_tests / MAX_SCEN * 100, 4))
    print("\nsuccess rates for each number of agents:", success_rates, "\n")