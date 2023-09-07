import math
import random
import time

import pybullet as p
import pybullet_data


def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


def get_colors(m, goal, mono=False):
    colors = [None] * m
    if mono:
        for i in range(m):
            j = i/m
            # colors[goal[i]] = (1, j, j, 1)  # red gradient
            # colors[goal[i]] = (j, 1, j, 1)  # green gradient
            colors[goal[i]] = (j, 1, 1, 1)  # blue gradient
    else:
        for i, lst in enumerate(chunks(range(m), math.ceil(m/5))):
            for j, obj in enumerate(lst):
                hue = j / len(lst)
                if i == 0:  # increase green
                    colors[goal[obj]] = (1, hue, 0, 1)
                elif i == 1:  # decrease red
                    colors[goal[obj]] = (1 - hue, 1, 0, 1)
                elif i == 2:  # increase blue
                    colors[goal[obj]] = (0, 1, hue, 1)
                elif i == 3:  # decrease green
                    colors[goal[obj]] = (0, 1 - hue, 1, 1)
                elif i == 4:  # increase red
                    colors[goal[obj]] = (hue, 0, 1, 1)
                # print(str(i)+':', str(obj)+',', colors[goal[obj]])
    return colors


def bfs(graph, source):
    seen = []
    q = [source]
    while q:
        u = q.pop(0)
        if u not in seen: seen.append(u)
        for v in graph[u]:
            q.append(v)
    return seen


def check_stable_state(obj_scale, obj_to_position, colors, camPos=(0, 0, 0)):
    p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    # p.setRealTimeSimulation(True)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(cameraTargetPosition=camPos, cameraDistance=15*obj_scale, cameraPitch=-10, cameraYaw=8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF('plane.urdf')
    obj_to_sim_id = [None] * len(obj_to_position)
    size = [obj_scale, obj_scale, obj_scale]
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size)
    coll_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
    for obj, pos in enumerate(obj_to_position):
        id = p.createMultiBody(baseMass=10, baseCollisionShapeIndex=coll_shape, baseVisualShapeIndex=visual, basePosition=pos)
        obj_to_sim_id[obj] = id
        if colors: p.changeVisualShape(id, -1, rgbaColor=colors[obj])
        p.addUserDebugText(str(obj), [0.4, 0, 0],
                    textColorRGB=[0, 0, 0],
                    textSize=1.5,
                    parentObjectUniqueId=id)
    for _ in range(400):
        p.stepSimulation()
        time.sleep(1./240.)
    # calc distance from expected for each obj, if greater than threshold, then unstable
    for k, position in enumerate(obj_to_position):
        curr_pos = p.getBasePositionAndOrientation(obj_to_sim_id[k])[0]
        dist_from_expected = math.dist(position, curr_pos)
        if dist_from_expected > 0.1:
            p.disconnect()
            return False
    # p.resetSimulation()
    input("Press Enter to close the simulation...")
    p.disconnect()
    return True


def check_start_goal_poses(m, obj_scale, s_coords, g_coords, inplace=True, shuffle=False, sim_check=False, colors=None):
    obj_to_start_position = [None] * m
    obj_to_goal_position = [None] * m
    start_layer_to_obj = {}
    goal_layer_to_obj = {}

    with open(s_coords) as start:
        for line in start:
            line = line.split(': ')
            id = int(line[0])
            start_pos = tuple(float(x) for x in line[1].split(', '))
            obj_to_start_position[id] = start_pos
            if not shuffle:
                layer = int((start_pos[-1] - obj_scale)/(2*obj_scale))
                if layer not in start_layer_to_obj:
                    start_layer_to_obj[layer] = [id]
                else:
                    start_layer_to_obj[layer].append(id)
    with open(g_coords) as goal:
        for line in goal:
            line = line.split(': ')
            id = int(line[0])
            goal_pos = tuple(float(x) for x in line[1].split(', '))
            if not inplace:
                goal_pos = (goal_pos[0], round(-goal_pos[1] - 3*obj_scale, 2), goal_pos[2])
            obj_to_goal_position[id] = goal_pos
            if not shuffle:
                layer = round((goal_pos[-1] - obj_scale)/(2*obj_scale))
                if layer not in goal_layer_to_obj:
                    goal_layer_to_obj[layer] = [id]
                else:
                    goal_layer_to_obj[layer].append(id)
    if shuffle:
        random.shuffle(obj_to_start_position)
        random.shuffle(obj_to_goal_position)
        for id in range(m):
            start_pos = obj_to_start_position[id]
            layer = round((start_pos[-1] - obj_scale)/(2*obj_scale))
            if layer not in start_layer_to_obj:
                start_layer_to_obj[layer] = [id]
            else:
                start_layer_to_obj[layer].append(id)
            goal_pos = obj_to_goal_position[id]
            layer = round((goal_pos[-1] - obj_scale)/(2*obj_scale))
            if layer not in goal_layer_to_obj:
                goal_layer_to_obj[layer] = [id]
            else:
                goal_layer_to_obj[layer].append(id)

    if sim_check:
        if not check_stable_state(obj_scale, obj_to_start_position, colors):
            print('Start state unstable!')
            return
        if not check_stable_state(obj_scale, obj_to_goal_position, colors):
            print('Goal state unstable!')
            return
        print('Start and goal are stable!')
    return obj_to_start_position, obj_to_goal_position, start_layer_to_obj, goal_layer_to_obj
