import math
import random

import pybullet as p
from geometry import *
from rearrange_solver.constants import SimOptions
from rearrange_solver.dag import DAG
from rearrange_solver.solver import Solver
from utils import bfs, check_start_goal_poses, get_colors


def shelf_test(n=5, k=5, obj_scale=0.3, shape=p.GEOM_BOX, sim=SimOptions.NONE):
    """
    k layers of n objects

    :type n: int
    :type k: int
    :type obj_scale: float
    :type shape: int
    :rtype: Solver
    """
    obj_to_start_coords = construct_shelf(n, k, obj_scale)
    m = n * k
    goal = list(range(m))
    random.shuffle(goal)
    with open('start_coord.txt', 'w') as start_file, open('goal_coord.txt', 'w') as goal_file:
        for obj, coord in enumerate(obj_to_start_coords):
            start_file.write(str(obj)+': '+coord+'\n')
            goal_file.write(str(goal[obj])+': '+coord+'\n')
    solver = general_cube_test(m, sim_type=sim)
    solver.init_colors(get_colors(m, goal))
    return solver


def pyramid_test_2d(n=5, k=5, obj_scale=0.3, shape=p.GEOM_BOX, sim=SimOptions.NONE, inplace=True):
    """
    n + (n - 1) + ... + (n - k + 1) objects

    :type n: int
    :type k: int
    :type obj_scale: float
    :type shape: int
    :rtype: Solver
    """
    obj_to_start_coords = construct_2d_pyramid(n, k, obj_scale)
    m = int(k*(n*2 - k + 1)/2)
    goal = list(range(m))
    random.shuffle(goal)
    with open('start_coord.txt', 'w') as start_file, open('goal_coord.txt', 'w') as goal_file:
        for obj, pos in enumerate(obj_to_start_coords):
            start_file.write(str(obj)+': '+', '.join(map(str, pos))+'\n')
            goal_file.write(str(goal[obj])+': '+', '.join(map(str, pos))+'\n')
    solver = general_cube_test(m, sim_type=sim, inplace=inplace)
    solver.init_colors(get_colors(m, goal))
    return solver


def pyramid_test_3d(s=5, k=5, obj_scale=0.3, shape=p.GEOM_BOX, sim=SimOptions.NONE, inplace=True):
    """
    s^2 + (s - 1)^2 + ... + (s - k + 1)^2 objects

    :type s: int
    :type k: int
    :type obj_scale: float
    :type shape: int
    :rtype: Solver
    """
    obj_to_start_coords = construct_3d_pyramid(s, k, obj_scale)
    diff = s - k
    m = int(s*(s + 1)*(2*s + 1)/6 - diff*(diff + 1)*(2*diff + 1)/6)
    goal = list(range(m))
    random.shuffle(goal)
    with open('start_coord.txt', 'w') as start_file, open('goal_coord.txt', 'w') as goal_file:
        for obj, pos in enumerate(obj_to_start_coords):
            start_file.write(str(obj)+': '+', '.join(map(str, pos))+'\n')
            goal_file.write(str(goal[obj])+': '+', '.join(map(str, pos))+'\n')
    solver = general_cube_test(m, sim_type=sim, inplace=inplace)
    solver.init_colors(get_colors(m, goal))
    return solver


def pyramid_test_2d_to_3d(s=5, k_goal=5, obj_scale=0.3, sim=SimOptions.NONE):
    # construct goal
    diff = s - k_goal
    m_goal = int(s*(s + 1)*(2*s + 1)/6 - diff*(diff + 1)*(2*diff + 1)/6)
    obj_to_goal_coords = construct_3d_pyramid(s, k_goal, obj_scale)
    with open('goal_coord.txt', 'w') as goal_file:
        for obj, pos in enumerate(obj_to_goal_coords):
            goal_file.write(str(obj)+': '+', '.join(map(str, pos))+'\n')

    # construct start
    n = s
    k_start = k_goal
    m_start = int(k_start*(n*2 - k_start + 1)/2)
    while m_start < m_goal:
        if k_start < n:
            k_start += 1
        else:
            n += 1
        m_start = int(k_start*(n*2 - k_start + 1)/2)
    side_length = 2*obj_scale
    y = 1.5*side_length*(s//2 + 0.5*((s + 1) % 2))
    obj_to_start_coords = construct_2d_pyramid(n, k_start, obj_scale, y)[:m_goal]
    random.shuffle(obj_to_start_coords)
    with open('start_coord.txt', 'w') as start_file:
        for obj, pos in enumerate(obj_to_start_coords):
            start_file.write(str(obj)+': '+', '.join(map(str, pos))+'\n')
    solver = general_cube_test(m_goal, sim_type=sim)
    solver.init_colors(get_colors(m_goal, range(m_goal)))
    return solver


def pyramid_test_3d_to_2d(n=5, k_goal=5, obj_scale=0.3, sim=SimOptions.NONE):
    # construct goal
    m_goal = int(k_goal*(n*2 - k_goal + 1)/2)
    side_length = 2*obj_scale
    s = int(math.sqrt(n))
    y = 1.5*side_length*(s//2 + 0.5*((s + 1) % 2))
    obj_to_goal_coords = construct_2d_pyramid(n, k_goal, obj_scale, y)
    with open('goal_coord.txt', 'w') as goal_file:
        for obj, pos in enumerate(obj_to_goal_coords):
            goal_file.write(str(obj)+': '+', '.join(map(str, pos))+'\n')

    # construct start
    k_start = s
    diff = s - k_start
    m_start = int(s*(s + 1)*(2*s + 1)/6 - diff*(diff + 1)*(2*diff + 1)/6)
    while m_start < m_goal:
        if k_start < s:
            k_start += 1
        else:
            s += 1
        diff = s - k_start
        m_start = int(s*(s + 1)*(2*s + 1)/6 - diff*(diff + 1)*(2*diff + 1)/6)
    obj_to_start_coords = construct_3d_pyramid(s, k_start, obj_scale)[:m_goal]
    random.shuffle(obj_to_start_coords)
    with open('start_coord.txt', 'w') as start_file:
        for obj, pos in enumerate(obj_to_start_coords):
            start_file.write(str(obj)+': '+', '.join(map(str, pos))+'\n')
    solver = general_cube_test(m_goal, sim_type=sim)
    solver.init_colors(get_colors(m_goal, range(m_goal)))
    return solver


def general_cube_test(count, obj_scale=0.3, s_coords='start_coord.txt', g_coords='goal_coord.txt', output='action_sequence.txt', sim_type=SimOptions.GUI, inplace=True, shuffle=False):
    m = count
    obj_to_start_position, obj_to_goal_position, start_layer_to_obj, goal_layer_to_obj = check_start_goal_poses(m, obj_scale, s_coords, g_coords, inplace, shuffle)

    start_bot, goal_bot = [], []
    for obj in range(m):
        if obj_to_start_position[obj][-1] == obj_scale:
            start_bot.append(obj)
        if obj_to_goal_position[obj][-1] == obj_scale:
            goal_bot.append(obj)

    start_up, goal_up = [[] for _ in range(m)], [[] for _ in range(m)]
    for i in range(m):
        start_obj_pos = obj_to_start_position[i]
        tl1 = (start_obj_pos[0] - obj_scale, start_obj_pos[1] + obj_scale)
        br1 = (start_obj_pos[0] + obj_scale, start_obj_pos[1] - obj_scale)
        for obj in start_layer_to_obj.get(int((start_obj_pos[-1] - obj_scale)/(2*obj_scale)) + 1, []):
            tl2 = (obj_to_start_position[obj][0] - obj_scale, obj_to_start_position[obj][1] + obj_scale)
            br2 = (obj_to_start_position[obj][0] + obj_scale, obj_to_start_position[obj][1] - obj_scale)
            if check_overlap((tl1, br1), (tl2, br2)):
                # print('start_up:', i, 'with', obj)
                start_up[i].append(obj)

        goal_obj_pos = obj_to_goal_position[i]
        tl1 = (goal_obj_pos[0] - obj_scale, goal_obj_pos[1] + obj_scale)
        br1 = (goal_obj_pos[0] + obj_scale, goal_obj_pos[1] - obj_scale)
        for obj in goal_layer_to_obj.get(int((goal_obj_pos[-1] - obj_scale)/(2*obj_scale)) + 1, []):
            tl2 = (obj_to_goal_position[obj][0] - obj_scale, obj_to_goal_position[obj][1] + obj_scale)
            br2 = (obj_to_goal_position[obj][0] + obj_scale, obj_to_goal_position[obj][1] - obj_scale)
            if check_overlap((tl1, br1), (tl2, br2)):
                # print('goal_up:', i, 'with', obj)
                goal_up[i].append(obj)

    goal_start_constr = [[] for _ in range(m)]

    alr_solved = []  # bottom-most objects that are already in goal
    unsolved_goal_obj = set()
    real_goal_bot = set()  # bottom-most objects that are not already in goal
    q = [goal_bot]
    while q:
        curr_lvl = q.pop(0)
        next_lvl = set()
        for obj in curr_lvl:
            if obj in unsolved_goal_obj:
                continue
            if obj_to_goal_position[obj] == obj_to_start_position[obj]:
                alr_solved.append(obj)
                for obj2 in goal_up[obj]:
                    next_lvl.add(obj2)
            else:
                if obj not in real_goal_bot:
                    real_goal_bot.add(obj)
                    unsolved_goal_obj.update(bfs(goal_up, obj))
        if next_lvl:
            q.append(next_lvl)
    # print("real_goal_bot:", real_goal_bot)
    # print("unsolved_goal_obj:", unsolved_goal_obj)
    # print("alr_solved:", alr_solved)

    num_start_layers = len(start_layer_to_obj.keys())
    num_goal_layers = len(goal_layer_to_obj.keys())
    # if num_start_layers == num_goal_layers:
    #     print(s_coords, g_coords)
    #     print("num_start_layers:", num_start_layers)
    #     print(start_layer_to_obj)
    #     print("num_goal_layers:", num_goal_layers)
    #     print(goal_layer_to_obj)
    # for goal_obj in range(m):
    for goal_obj in unsolved_goal_obj:
        goal_pos = obj_to_goal_position[goal_obj]
        tl1 = (goal_pos[0] - obj_scale, goal_pos[1] + obj_scale)
        br1 = (goal_pos[0] + obj_scale, goal_pos[1] - obj_scale)
        rec1 = (tl1, br1)
        goal_layer = int((goal_pos[-1] - obj_scale)/(2*obj_scale))
        for lyr in range(goal_layer, num_start_layers):
            for start_obj in start_layer_to_obj[lyr]:
                start_pos = obj_to_start_position[start_obj]
                tl2 = (start_pos[0] - obj_scale, start_pos[1] + obj_scale)
                br2 = (start_pos[0] + obj_scale, start_pos[1] - obj_scale)
                rec2 = (tl2, br2)
                if check_overlap(rec1, rec2):
                    # print('goal_start_constr:', goal_obj, 'with', start_obj)
                    if goal_obj == start_obj:
                    # if goal_obj == start_obj and lyr == goal_layer:
                        goal_start_constr[goal_obj].extend(start_up[start_obj])
                    else:
                        goal_start_constr[goal_obj].append(start_obj)

    solver_start = DAG(m, start_bot, start_up)
    solver_goal = DAG(m, goal_bot, goal_up)
    solver = Solver(m, solver_start, solver_goal, goal_start_constr, obj_scale, p.GEOM_BOX, obj_to_start_position, obj_to_goal_position, output, sim_type)
    solver.init_alr_solved(alr_solved)
    return solver


def debug(m, test_num):
    """Debug a specific testcase."""
    s = str(m) + '_' + str(test_num) + '.txt'
    solver = general_cube_test(m, 0.3, 'start_' + s, 'goal_' + s, 'action_sequence_' + s, SimOptions.DIRECT, False)
    solver.init_colors(get_colors(m, range(m)))
    # solver.ilp_permutation_solver()
    # solver.greedy_solver()
