# Written with assistance from Github Co-Pilot
import copy
import time
import random
import dm_control.mujoco
import mujoco.viewer

from genotype import SpherePart, add_node_mutation, get_all_sphere_parts, remove_node_mutation
from genotype import flip_freeze_edge_mutation
from phenotype import translate_genotype_to_phenotype


def get_motor_names(m):
    '''
    Returns a list of the names of all the motors in the model
    '''
    motor_names = []
    for i in range(m.na):
        name = dm_control.mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        motor_names.append(name)
    return motor_names


def simulate(model, motor_strength_dict):
    with open('creature.xml', 'w') as f:
        f.write(model.to_xml_string())
    
    m = dm_control.mujoco.MjModel.from_xml_path("creature.xml")
    d = dm_control.mujoco.MjData(m)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        # Set camera parameters
        # These parameters can be adjusted to change the camera angle and perspective
        viewer.cam.azimuth = 180  # Azimuthal angle (in degrees)
        viewer.cam.elevation = -20  # Elevation angle (in degrees)
        viewer.cam.distance = 15.0  # Distance from the camera to the target
        viewer.cam.lookat[0] = 0.0  # X-coordinate of the target position
        viewer.cam.lookat[1] = 0.0  # Y-coordinate of the target position
        viewer.cam.lookat[2] = 0.0  # Z-coordinate of the target position

        for i in range(3000):
            motor_names = get_motor_names(m)
            for m_name in motor_names:
                i = dm_control.mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, m_name)
                d.ctrl[i] = motor_strength_dict[m_name]
            
            dm_control.mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(1/1000)

        # fitness is the distance from the first body to the origin
        fitness = d.qpos[0] ** 2 + d.qpos[1] ** 2
        print(f"Creature fitness: {fitness:.4f}")
        viewer.close()
    
    return fitness


def main():
    genotype = SpherePart("body0", 0.5, (0, 0, 0))
    for i in range(6):
        all_sphere_parts = get_all_sphere_parts([genotype], genotype)
        random_part = random.choice(all_sphere_parts)
        add_node_mutation(random_part, f"body{i+1}")

    model, motor_strength_dict = translate_genotype_to_phenotype(copy.deepcopy(genotype))
    fitness = simulate(model, motor_strength_dict)

    # Select a random node and run the freeze edge mutation
    all_sphere_parts = get_all_sphere_parts([genotype], genotype)
    all_spheres_minus_root = [sphere for sphere in all_sphere_parts if sphere.name != "body0"]
    random_part = random.choice(all_spheres_minus_root)
    flip_freeze_edge_mutation(random_part)

    model, motor_strength_dict = translate_genotype_to_phenotype(copy.deepcopy(genotype))
    fitness = simulate(model, motor_strength_dict)

    # Select a random node and remove it
    all_sphere_parts = get_all_sphere_parts([genotype], genotype)
    all_spheres_minus_root = [sphere for sphere in all_sphere_parts if sphere.name != "body0"]
    random_part = random.choice(all_spheres_minus_root)
    remove_node_mutation(random_part)

    model, motor_strength_dict = translate_genotype_to_phenotype(copy.deepcopy(genotype))
    fitness = simulate(model, motor_strength_dict)


if __name__ == "__main__":
    main()