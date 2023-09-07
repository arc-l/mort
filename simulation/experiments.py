import time
import timeit
from pathlib import Path
from statistics import stdev

from rearrange_solver.constants import SimOptions
from simulations import (general_cube_test, pyramid_test_2d,
                         pyramid_test_2d_to_3d, pyramid_test_3d,
                         pyramid_test_3d_to_2d)


def random_tests():
    counts = (5, 10, 20, 30, 40)
    for i in counts:
        for j in range(1, 21):
            s = str(i) + '_' + str(j) + '.txt'
            my_file = Path('./action_sequence_' + s)
            if not my_file.exists():
                general_cube_test(i, 0.3, 'start_' + s, 'goal_' + s, 'action_sequence_' + s)


def ilp_random_experiment():
    counts = (5, 10, 20, 30, 40)
    counts = (40,)
    for i in counts:
        total_time = 0
        for j in range(1, 21):
            s = str(i) + '_' + str(j) + '.txt'
            solver = general_cube_test(i, 0.3, 'start_'+s, 'goal_'+s, 'action_sequence_'+s, SimOptions.DIRECT)
            curr_time = timeit.timeit(solver.ilp_permutation_solver, number=40//i)
            if curr_time > 10:
                print("Test", s, "took", curr_time, "seconds")
            total_time += curr_time
        res = "Average runtime (sec) for "+str(i)+" objects: "+str(total_time/20)
        with open('runtimes.txt', 'a') as file:
            file.write(res+'\n')
        print(res)


def ilp_vs_greedy_experiment(test_type=1, inplace=True, trials=30, output='runtimes.txt'):
    object_counts = []
    time_result_ilp = []
    time_result_ilp_stdev = []
    time_result_greedy = []
    time_result_greedy_stdev = []
    optimality_buff_result_greedy = []
    optimality_buff_result_greedy_stdev = []
    optimality_total_result_greedy = []
    optimality_total_result_greedy_stdev = []
    if test_type == 1 or test_type == 2:
        case = "2D to 2D pyramid" if test_type == 1 else "3D to 2D pyramid"
        for i in range(3, 12):
            m_2d = int(i*(i*2 - i + 1)/2)
            object_counts.append(m_2d)
            times_ilp = [None] * trials
            times_greedy = [None] * trials
            optimality_buff_greedy = [None] * trials
            optimality_total_greedy = [None] * trials
            for j in range(trials):
                if test_type == 1:
                    solver = pyramid_test_2d(i, i, obj_scale=0.3, inplace=inplace)
                elif test_type == 2:
                    solver = pyramid_test_3d_to_2d(i, i, obj_scale=0.3)
                # print(case, 'test for', str(m_2d), 'objects:')

                t0 = time.perf_counter()
                ilp_buff_2d = solver.ilp_permutation_solver()
                t1 = time.perf_counter()
                times_ilp[j] = t1 - t0
                # print("ILP runtime (sec):", str(times_ilp[j]))

                t0 = time.perf_counter()
                greedy_buff_2d = solver.greedy_solver()
                t1 = time.perf_counter()
                times_greedy[j] = t1 - t0
                if ilp_buff_2d == 0:
                    if greedy_buff_2d > 0:
                        print("nonzero divided by 0 error!")
                    optimality_buff_greedy[j] = float(1 + greedy_buff_2d) / (1 + ilp_buff_2d)
                else:
                    optimality_buff_greedy[j] = float(greedy_buff_2d) / ilp_buff_2d
                optimality_total_greedy[j] = float(greedy_buff_2d + m_2d) / (ilp_buff_2d + m_2d)
            time_result_ilp.append(sum(times_ilp) / trials)
            time_result_ilp_stdev.append(stdev(times_ilp))
            time_result_greedy.append(sum(times_greedy) / trials)
            time_result_greedy_stdev.append(stdev(times_greedy))
            optimality_buff_result_greedy.append(sum(optimality_buff_greedy) / trials)
            optimality_buff_result_greedy_stdev.append(stdev(optimality_buff_greedy))
            optimality_total_result_greedy.append(sum(optimality_total_greedy) / trials)
            optimality_total_result_greedy_stdev.append(stdev(optimality_total_greedy))
    elif test_type == 3 or test_type == 4:
        case = "3D to 3D pyramid" if test_type == 3 else "2D to 3D pyramid"
        for i in range(2, 6):
            m_3d = int(i*(i + 1)*(2*i + 1)/6)
            object_counts.append(m_3d)
            times_ilp = [None] * trials
            times_greedy = [None] * trials
            optimality_buff_greedy = [None] * trials
            optimality_total_greedy = [None] * trials
            for j in range(trials):
                if test_type == 3:
                    solver = pyramid_test_3d(i, i, obj_scale=0.3, inplace=inplace)
                elif test_type == 4:
                    solver = pyramid_test_2d_to_3d(i, i, obj_scale=0.3)

                t0 = time.perf_counter()
                ilp_buff_3d = solver.ilp_permutation_solver()
                t1 = time.perf_counter()
                times_ilp[j] = t1 - t0

                t0 = time.perf_counter()
                greedy_buff_3d = solver.greedy_solver()
                t1 = time.perf_counter()
                times_greedy[j] = t1 - t0
                if ilp_buff_3d == 0:
                    if greedy_buff_3d > 0:
                        print("nonzero divided by 0 error!")
                    optimality_buff_greedy[j] = float(1 + greedy_buff_3d) / (1 + ilp_buff_3d)
                else:
                    optimality_buff_greedy[j] = float(greedy_buff_3d) / ilp_buff_3d
                optimality_total_greedy[j] = float(greedy_buff_3d + m_3d) / (ilp_buff_3d + m_3d)
            time_result_ilp.append(sum(times_ilp) / trials)
            time_result_ilp_stdev.append(stdev(times_ilp))
            time_result_greedy.append(sum(times_greedy) / trials)
            time_result_greedy_stdev.append(stdev(times_greedy))
            optimality_buff_result_greedy.append(sum(optimality_buff_greedy) / trials)
            optimality_buff_result_greedy_stdev.append(stdev(optimality_buff_greedy))
            optimality_total_result_greedy.append(sum(optimality_total_greedy) / trials)
            optimality_total_result_greedy_stdev.append(stdev(optimality_total_greedy))
    elif test_type == 5:
        success_rate_ilp = []
        success_rate_greedy = []
        case = "Random Instances"
        if trials > 20:
            trials = 20
        object_counts = [5, 10, 20, 30]
        object_counts = [20]
        unsuccessful_greedy_trials = ['30_1.txt']
        for num_objects in object_counts:
            successful_tests_greedy = trials
            successful_tests_ilp = trials
            comparison_tests = trials
            times_ilp = [None] * trials
            times_greedy = [None] * trials
            optimality_buff_greedy = [None] * trials
            optimality_total_greedy = [None] * trials
            for j in range(trials):
                s = str(num_objects)+'_'+str(j + 1)+'.txt'
                solver = general_cube_test(num_objects, 0.3, 'start_'+s, 'goal_'+s, 'action_sequence_'+s, SimOptions.DIRECT, inplace=inplace)

                try:
                    times_ilp[j] = timeit.timeit(solver.ilp_permutation_solver, number=1)
                    if times_ilp[j] > 10:
                        print("Test", s, "took", times_ilp[j], "seconds for ILP to solve")
                except OSError:  # TimeoutError is a Subclass of OSError, so it is caught here
                    print("Test", s, "timedout")
                    successful_tests_ilp -= 1
                ilp_buff = solver.ilp_buff_count

                times_greedy[j] = timeit.timeit(solver.greedy_solver, number=1)
                greedy_buff = solver.greedy_buff_count
                if s in unsuccessful_greedy_trials or greedy_buff == -1:
                    print("Test", s, "could not be solved by greedy algo!")
                    times_greedy[j] = None
                    successful_tests_greedy -= 1

                if ilp_buff == -1 or greedy_buff == -1:
                    comparison_tests -= 1
                else:
                    print(s, greedy_buff, ilp_buff)
                    if ilp_buff == 0:
                        if greedy_buff > 0:
                            print("nonzero divided by 0 error!")
                        optimality_buff_greedy[j] = float(1 + greedy_buff) / (1 + ilp_buff)
                    else:
                        optimality_buff_greedy[j] = float(greedy_buff) / ilp_buff
                    optimality_total_greedy[j] = float(greedy_buff + num_objects) / (ilp_buff + num_objects)
            time_result_ilp.append(sum(filter(None, times_ilp)) / successful_tests_ilp)
            time_result_ilp_stdev.append(stdev(filter(None, times_ilp)))
            time_result_greedy.append(sum(filter(None, times_greedy)) / successful_tests_greedy)
            time_result_greedy_stdev.append(stdev(filter(None, times_greedy)))
            success_rate_ilp.append(float(successful_tests_ilp) / trials)
            success_rate_greedy.append(float(successful_tests_greedy) / trials)
            optimality_buff_result_greedy.append(sum(filter(None, optimality_buff_greedy)) / comparison_tests)
            optimality_buff_result_greedy_stdev.append(stdev(filter(None, optimality_buff_greedy)))
            optimality_total_result_greedy.append(sum(filter(None, optimality_total_greedy)) / comparison_tests)
            optimality_total_result_greedy_stdev.append(stdev(filter(None, optimality_total_greedy)))
    if not inplace:
        case += " (not in-place)"
    with open(output, 'a') as file:
        output_mode = file.write
        # output_mode = print
        output_mode("\nResults for "+case+":")
        output_mode("\nObjectCounts = "+str(object_counts))
        if test_type == 5:
            output_mode("\nILPSuccessRate = "+str(success_rate_ilp))
            output_mode("\nGreedySuccessRate = "+str(success_rate_greedy))
        output_mode("\nILPComputationTime = "+str(time_result_ilp))
        output_mode("\nILPStdDev = "+str(time_result_ilp_stdev))
        output_mode("\nGreedyComputationTime = "+str(time_result_greedy))
        output_mode("\nGreedyStdDev = "+str(time_result_greedy_stdev))
        output_mode("\nGreedyOptimalityRatioBuff = "+str(optimality_buff_result_greedy))
        output_mode("\nGreedyOptimalityRatioBuffStdDev = "+str(optimality_buff_result_greedy_stdev))
        output_mode("\nGreedyOptimalityRatioMoves = "+str(optimality_total_result_greedy))
        output_mode("\nGreedyOptimalityRatioMovesStdDev = "+str(optimality_total_result_greedy_stdev)+'\n')


def optimality_vs_layers(layers, test_nums, inplace=True, trials=20, output='runtimes.txt', num_objects=20):
    total_over_objs_result_ilp = []
    total_over_objs_result_ilp_stdev = []
    case = "Random Instances total moves/objs for " + str(num_objects)
    for i in test_nums:
        total_over_objs = [None] * trials
        s = str(num_objects)+'_'+str(i)+'.txt'
        for j in range(trials):
            solver = general_cube_test(num_objects, 0.3, 'start_'+s, 'goal_'+s, 'action_sequence_'+s, SimOptions.DIRECT, inplace=inplace, shuffle=True)
            solver.ilp_permutation_solver()
            ilp_buff = solver.ilp_buff_count
            total_over_objs[j] = float(ilp_buff + num_objects) / num_objects
        total_over_objs_result_ilp.append(sum(filter(None, total_over_objs)) / trials)
        total_over_objs_result_ilp_stdev.append(stdev(filter(None, total_over_objs)))
    if not inplace:
        case += " (not in-place)"
    with open(output, 'a') as file:
        output_mode = file.write
        # output_mode = print
        output_mode("\nResults for "+case+":")
        output_mode("\nLayers = "+str(layers))
        output_mode("\nILPTotalMovesOverNumObjs = "+str(total_over_objs_result_ilp))
        output_mode("\nILPStdDev = "+str(total_over_objs_result_ilp_stdev)+'\n')
