import time
import subprocess

import argparse


def build_run_args(args, cur_scen, agents):
    if args.one_run:
        scen = args.scen
    else:
        scen = args.scen + str(cur_scen + 1) + ".scen"
    return [args.bin, args.map, scen, str(agents), args.precalc, args.CAT, args.heuristic, args.prioritize_conflicts,
            args.bypass, args.ecbs, args.omega, args.symmetry, args.online, args.goals, args.horizon, args.replanning,
            args.paths]


def get_parser():
    parser = argparse.ArgumentParser()

    data = parser.add_argument_group("data paths", "contain paths to launch data")
    data.add_argument("-b", "--bin", default="./multirobot", help="solution binary")
    data.add_argument("-m", "--map", default="./check.map", help="map file")
    data.add_argument("-s", "--scen", default="./check.scen", help="scen file or dir")
    data.add_argument("-g", "--goals", default="./check.goals", help="goals file")

    launch = parser.add_argument_group("launch arguments")
    launch.add_argument("--max-agents", default=100, type=int)
    launch.add_argument("--max-scen", default=25, type=int)
    launch.add_argument("--max-failed", default=3, type=int)
    launch.add_argument("--time-out", default=300, type=int)
    launch.add_argument("--one-run", action="store_true", help="run one test for scen file")

    algorithm = parser.add_argument_group("algorithm settings")
    algorithm.add_argument("--precalc", default="false", choices=["false", "true"])
    algorithm.add_argument("--CAT", default="true", choices=["false", "true"])
    algorithm.add_argument("--heuristic", default="true", choices=(
        "normal", "normal_diagonal", "number_of_conflicts", "number_of_conflicting_agents", "number_of_pairs",
        "vertex_cover"))
    algorithm.add_argument("--prioritize-conflicts", default="false", choices=["false", "true"])
    algorithm.add_argument("--bypass", default="true", choices=["false", "true"])
    algorithm.add_argument("--ecbs", default="false", choices=["false", "true"])
    algorithm.add_argument("--omega", default="1", help="only if ecbs")
    algorithm.add_argument("--symmetry", default="false", choices=["false", "true"])
    algorithm.add_argument("--online", default="false", choices=["false", "true"])
    algorithm.add_argument("--horizon", default="100000000",
                           help="conflicts will be ignored after this number of steps")
    algorithm.add_argument("--replanning", default="100000000",
                           help="number of time steps before replanning")
    algorithm.add_argument("--paths", default="false", choices=["false", "true"],
                           help="set true to print paths for each agent")
    return parser


parser = get_parser()
args = parser.parse_args()

success_rates = []  # percent of passed tests for each number of agents
failed = [0] * args.max_scen  # for counting consecutive failures

for agents in range(1, args.max_agents + 1):  # choose a number of agents
    num_of_passed_tests = 0

    for cur_scen in range(0, args.max_scen):  # choose a scenario
        if failed[cur_scen] <= args.max_failed:  # if the number of consecutive failures doesn't exceed MAX_FAILED
            # start the timer and create a subprocess
            timeStarted = time.time()
            test = subprocess.Popen(build_run_args(args, cur_scen, agents))

            try:
                test.wait(timeout=args.time_out)
            except subprocess.TimeoutExpired:
                test.kill()
                failed[cur_scen] += 1
                timeDelta = time.time() - timeStarted
                print("agents:", agents, "scenario:", cur_scen, "time:", "%.5f" % timeDelta,
                      "result: FAILURE")  # time limit exceeded
            else:
                num_of_passed_tests += 1
                failed[cur_scen] = 0
                timeDelta = time.time() - timeStarted
                print("agents:", agents, "scenario:", cur_scen, "time:", "%.5f" % timeDelta, "result: SUCCESS")
        else:
            print("agents:", agents, "scenario:", cur_scen, "time:", "MAX_FAILED exceeded", "result: FAILURE")

    success_rates.append(round(num_of_passed_tests / args.max_scen * 100, 4))
    print("\nsuccess rates for each number of agents:", success_rates, "\n")
