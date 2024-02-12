from dm_control import mjcf
from genotype import SpherePart


def translate_genotype_to_phenotype_recursive(model, parent_body, sphere_part, direction, motor_strength_dict):
    if sphere_part.created:
        return
    sphere_part.created = True
    child_body = parent_body.add('body', name=f"{sphere_part.name}", pos=sphere_part.pos)
    child_body.add('geom', type='sphere', size=(sphere_part.size, sphere_part.size, sphere_part.size))
    joint = child_body.add('joint', type=sphere_part.joint_type, axis=direction)
    if sphere_part.frozen_edges[direction]:
        # motor = model.actuator.add('motor', joint=joint, ctrllimited='true', ctrlrange=(0, 0))
        # motor_strength_dict[motor.name] = 0
        pass
    else:
        motor = model.actuator.add('motor', joint=joint, ctrllimited='true', ctrlrange=(-1, 1))
        motor_strength_dict[motor.name] = sphere_part.edge_strength[direction]
    for next_direction, next_child in sphere_part.edges.items():
        if next_child is not None:
            translate_genotype_to_phenotype_recursive(model, child_body, next_child, next_direction, motor_strength_dict)


def translate_genotype_to_phenotype(genotype: SpherePart):
    motor_strength_dict = {}

    model = mjcf.RootElement()
    plane = model.worldbody.add('geom', type='plane', size=(100, 100, 0.1), pos=(0, 0, 1))

    root_body = model.worldbody.add('body', name='body0', pos=(0, 0, 0))
    root_body.add('joint', type='free')
    root_body.add('geom', type='sphere', size=(genotype.size, genotype.size, genotype.size))
    genotype.created = True

    for direction, child in genotype.edges.items():
        if child is not None:
            translate_genotype_to_phenotype_recursive(model, root_body, child, direction, motor_strength_dict)
    
    # adjust the position of the plane to be just below the lowest body
    lowest_body = root_body
    for body in model.find_all('body'):
        if body.pos[2] < lowest_body.pos[2]:
            lowest_body = body
    plane.pos[2] = lowest_body.pos[2] - 2

    return model, motor_strength_dict
