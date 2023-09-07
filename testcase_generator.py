import math
from simulation import check_overlap
import random
import pybullet as p
import pybullet_data


def generate_test(n, x, y, obj_scale, layers=2, mode=p.DIRECT):
    # Setup bullet world
    # p.connect(p.GUI)
    p.connect(mode)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(cameraTargetPosition=(0, 0, 0), cameraDistance=15*obj_scale, cameraPitch=-10, cameraYaw=8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF('plane.urdf')
    obj_to_sim_id = [None] * n
    size = [obj_scale, obj_scale, obj_scale]
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size)
    coll_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
    square = math.ceil(math.sqrt(n))
    for obj in range(n):
        side_length = 2*obj_scale
        x_coord = round(1.5*side_length*(1 + (obj % square)), 2)
        y_coord = round(1.5*side_length*(1 + obj // square), 2)
        buff_pos = (-x_coord, -y_coord, obj_scale)
        id = p.createMultiBody(baseMass=10, baseCollisionShapeIndex=coll_shape, baseVisualShapeIndex=visual, basePosition=buff_pos)
        obj_to_sim_id[obj] = id
        p.addUserDebugText(str(obj), [0.4, 0, 0],
                    textColorRGB=[0, 0, 0],
                    textSize=1.5,
                    parentObjectUniqueId=id)

    # Generation begins here
    obj_to_position = [None] * n
    for i in range(n):
        prev = p.saveState()
        stable = False
        while not stable:
            x_coord = round(random.uniform(0, x), 6)
            y_coord = round(random.uniform(0, y), 6)
            z_coord = obj_scale
            tl1 = (x_coord - obj_scale, y_coord + obj_scale)
            br1 = (x_coord + obj_scale, y_coord - obj_scale)
            for j in range(i):
                tl2 = (obj_to_position[j][0] - obj_scale, obj_to_position[j][1] + obj_scale)
                br2 = (obj_to_position[j][0] + obj_scale, obj_to_position[j][1] - obj_scale)
                if check_overlap((tl1, br1), (tl2, br2)):
                    z_coord = round(max(z_coord, obj_to_position[j][2] + 2*obj_scale), 2)
            # ensure z does not exceed layer limit
            if int((z_coord - obj_scale)/(2*obj_scale)) + 1 > layers:
                continue
            obj_to_position[i] = (x_coord, y_coord, z_coord)
            curr_orn = p.getBasePositionAndOrientation(obj_to_sim_id[i])[1]
            p.resetBasePositionAndOrientation(obj_to_sim_id[i], obj_to_position[i], curr_orn)
            for _ in range(400):
                p.stepSimulation()
            # calc distance from expected for each obj, if greater than threshold, then unstable
            stable = True
            for k in range(i + 1):
                curr_pos = p.getBasePositionAndOrientation(obj_to_sim_id[k])[0]
                dist_from_expected = math.dist(obj_to_position[k], curr_pos)
                if dist_from_expected > 0.1:
                    stable = False
                    p.restoreState(prev)
                    break
        p.removeState(prev)

    with open('start_'+str(n)+'_'+str(layers)+'layers.txt', 'w') as f:
        for i, pos in enumerate(obj_to_position):
            f.write(str(i)+': '+', '.join(map(str, pos))+'\n')
    # return obj_to_position


if __name__ == '__main__':
    generate_test(30, 3, 3, 0.3, 3)
