import sys
import timeit

sys.path.append("..")
sys.path.append("../simulation")
sys.path.append("../rearrange_solver")
import pybullet as p
from experiments import (ilp_random_experiment, ilp_vs_greedy_experiment,
                         optimality_vs_layers, random_tests)
from rearrange_solver.constants import SimOptions
from rearrange_solver.solver import Solver
from simulations import (check_start_goal_poses, debug, general_cube_test,
                         pyramid_test_2d, pyramid_test_2d_to_3d,
                         pyramid_test_3d, pyramid_test_3d_to_2d, shelf_test)


def opt_v_layers():
    optimality_vs_layers([2, 3, 4], [22, 23, 24], num_objects=10)
    optimality_vs_layers([2, 3, 4, 5, 6, 7], [22, 7, 2, 10, 26, 27], num_objects=20)
    optimality_vs_layers([2, 3, 4, 5, 6, 7], [22, 23, 24, 25, 19, 27], num_objects=30)


def debug_cases():
    # 30 object cases
    for i in range(1, 21):
        debug(30, i)
    # highest optimality ratio (out-of-place)
    debug(20, 11)
    debug(20, 17)
    debug(20, 19)
    # highest total move ratio
    debug(20, 12)
    debug(20, 2)


def find_case():
    """Find case where greedy performs worse than ILP, then display in pybullet."""
    i = 3
    ratio = 1
    while ratio == 1:
        solver = pyramid_test_2d(i, i, obj_scale=0.3, sim=SimOptions.DIRECT)
        ilp_buff_2d = solver.ilp_permutation_solver()
        greedy_buff_2d = solver.greedy_solver()
        ratio = float(greedy_buff_2d) / ilp_buff_2d if (ilp_buff_2d != 0) else 1
        print(ratio)
    check_start_goal_poses(6, 0.3, 'start_coord.txt', 'goal_coord.txt', True, False, solver.obj_to_colors)


def structured_tests():
    solver = shelf_test(5, 5, obj_scale=0.3, sim=SimOptions.GUI)
    solver = pyramid_test_2d(5, 5, obj_scale=0.3, sim=SimOptions.GUI, inplace=True)
    solver = pyramid_test_3d(3, 3, obj_scale=0.3, sim=SimOptions.GUI, inplace=True)
    solver = pyramid_test_2d_to_3d(3, 3, obj_scale=0.3, sim=SimOptions.GUI)
    solver = pyramid_test_3d_to_2d(6, 6, obj_scale=0.3, sim=SimOptions.GUI)
    solver.ilp_permutation_solver()


def run_experiments():
    ilp_vs_greedy_experiment(test_type=1, trials=30, inplace=False)
    ilp_vs_greedy_experiment(test_type=1, trials=30, inplace=True)
    ilp_vs_greedy_experiment(test_type=2, trials=30)
    ilp_vs_greedy_experiment(test_type=3, trials=30, inplace=False)
    ilp_vs_greedy_experiment(test_type=3, trials=30, inplace=True)
    ilp_vs_greedy_experiment(test_type=4, trials=30)
    ilp_vs_greedy_experiment(test_type=5, trials=20, inplace=False)
    ilp_vs_greedy_experiment(test_type=5, trials=20, inplace=True)


def time_simulation(solver: Solver, repeat=5):
    print("Time for simulation:", timeit.timeit(lambda: solver.ilp_permutation_solver(), number=repeat))
    print("Time for simulation:", timeit.timeit(lambda: solver.greedy_solver(), number=repeat))


if __name__ == '__main__':
    cube = p.GEOM_BOX
    cylinder = p.GEOM_CYLINDER
    
    # opt_v_layers()
    # debug_cases()
    # find_case()
    # structured_tests()
    # run_experiments()

    # random_tests()
    # ilp_random_experiment()

    # solver = general_cube_test(4, 0.3, 'start_coord unstable.txt', 'goal_coord unstable.txt', 'action_sequence unstable.txt')
    # solver = general_cube_test(4, 0.3, 'start_coord infeasible.txt', 'goal_coord infeasible.txt', 'action_sequence infeasible.txt')
    # solver = general_cube_test(6, 0.3, 'start_coord wall example.txt', 'goal_coord wall example.txt', 'action_sequence wall example.txt')
    # solver.ilp_permutation_solver()
    # solver.greedy_solver()
    solver = pyramid_test_2d(3, 3, obj_scale=0.3, sim=SimOptions.GUI)
    # solver = pyramid_test_3d(5, 5, obj_scale=0.3, sim=SimOptions.GUI)
    # solver = pyramid_test_3d(5, 5, obj_scale=0.3, sim=SimOptions.GUI_INTERACTIVE)
    time_simulation(solver, 1)
