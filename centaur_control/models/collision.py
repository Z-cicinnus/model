"""按已启用的碰撞规则检查几何穿透。"""




def collision_pairs(model, data, tolerance=1e-4):
    """按当前模型启用的碰撞规则筛选穿透，不能覆盖已排除的碰撞对。"""
    pairs = set()
    for contact in data.contact:
        if contact.dist < -tolerance:
            b1, b2 = model.geom_bodyid[contact.geom1], model.geom_bodyid[contact.geom2]
            pairs.add((model.body(int(b1)).name, model.body(int(b2)).name))
    return sorted(pairs)
