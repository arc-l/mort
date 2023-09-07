def check_overlap(rec1, rec2):
    l1_x, l1_y = rec1[0]
    r1_x, r1_y = rec1[1]
    l2_x, l2_y = rec2[0]
    r2_x, r2_y = rec2[1]
    # If one rectangle is on left side of other
    if l1_x > r2_x or l2_x > r1_x:
        return False
    # If one rectangle is above other
    if r1_y > l2_y or r2_y > l1_y:
        return False
    return True


def construct_shelf(n=5, k=5, obj_scale=0.3):
    m = n*k
    result = [None] * m

    cnt = 0
    side_length = 2*obj_scale
    for i in range(k):
        z = round(obj_scale + side_length*i, 2)
        for j in range(n):
            x = round(1.5*side_length*j, 2)
            pos = (x, 0, z)
            # pos = ', '.join(map(str, pos))
            result[cnt] = pos
            cnt += 1
    return result


def construct_2d_pyramid(n=5, k=5, obj_scale=0.3, y=0):
    assert(k <= n)
    m = int(k*(n*2 - k + 1)/2)
    result = [None] * m

    cnt = 0
    side_length = 2*obj_scale
    for i in range(k):
        layer_horizontal_offset = 0.75*side_length*i
        z = round(obj_scale + side_length*i, 2)
        for j in range(n - i):
            x = round(layer_horizontal_offset + 1.5*side_length*j, 2)
            pos = (x, y, z)
            # pos = ', '.join(map(str, pos))
            result[cnt] = pos
            cnt += 1
    return result


def construct_3d_pyramid(s=5, k=5, obj_scale=0.3):
    assert(k <= s)
    diff = s - k
    m = int(s*(s + 1)*(2*s + 1)/6 - diff*(diff + 1)*(2*diff + 1)/6)
    result = [None] * m

    cnt = 0
    side_length = 2*obj_scale
    for i in range(k):
        layer_horizontal_offset = 0.75*side_length*i
        z = round(obj_scale + side_length*i, 2)
        for j in range(s - i):
            y = round(layer_horizontal_offset + 1.5*side_length*j, 2)
            for l in range(s - i):
                x = round(layer_horizontal_offset + 1.5*side_length*l, 2)
                pos = (x, y, z)
                # pos = ', '.join(map(str, pos))
                result[cnt] = pos
                cnt += 1
    return result
