import shutil
import numpy as np
import pymesh
import pybullet as p

from glo import *
from utils import *
from .scene import Scene

### Init mesh in pybullet and get stable position, orientation and scaling if possible
def _get_stable_pos_ornquat_for_init_object(sim, wait_time=STABILIZATION_TIME):
        # Start at default [0,0,0] pos and [0,0,0,1] orn quaternion
        # Let it settle for wait_time seconds
        for i in range(int(wait_time/STEP_TO_SECONDS)): sim.p.stepSimulation()
        pos_0, orn_0 = sim.p.getBasePositionAndOrientation(sim.object_id)
        
        # Assert we indeed found a stabilized position
        for i in range(int(1/STEP_TO_SECONDS)): sim.p.stepSimulation()
        pos_1, orn_1 = sim.p.getBasePositionAndOrientation(sim.object_id)
        assert np.allclose(orn_0, orn_1, rtol=TOL_ROT, atol=TOL_ROT), f'orientation did not stay stable, started at {orn_0} ended at {orn_1} after 1 second'
        assert np.allclose(pos_0, pos_1, rtol=TOL_DIST, atol=TOL_DIST), f'position did not stay stable, started at {pos_0} ended at {pos_1} after 1 second'
        
        # Correct the height of the object, but keep the x,y coords and orientation same, so that we land at the
        # previously-seen stable position without ANY tumbling
        stable_orientation_quat = orn_1
        stable_z = pos_1[2]
        return [0,0,stable_z], stable_orientation_quat


def _get_scaling_factor_for_stable_object(sim, stable_pos, stable_orientation_quat):
    
    def get_object_lwh(sim):
        # Get bbox lwh
        lowest_point, highest_point = sim.p.getAABB(bodyUniqueId=sim.object_id)
        return list(np.array(highest_point)-np.array(lowest_point))
        

    # First, let the smallest side be twice the finger length to start big but not too far away
    l, w, h = get_object_lwh(sim)
    initial_fac = GRIPPER_FINGER_HEIGHT / min(l, w, h)
    sim.reset_state(include=NO_GRIPPER, keep_same_object=True, object_scale=initial_fac)

    # Loop
    factor = 1.01
    lwh_old = None

    for _ in range(MAX_TRIES_SCALING):

        # Start at stable pos and orn
        sim.reset_state(include=NO_GRIPPER, keep_same_object=True, object_pos=stable_pos, object_orn=stable_orientation_quat, object_scale=factor*initial_fac)
        
        l, w, h = get_object_lwh(sim)

        # Stagnating?
        if ([l,w,h] == lwh_old): raise Exception('Object not scaling, something weird must be happening.')
        else: lwh_old = [l,w,h]

        def atleast_one_of_len_or_width_fits_within_gripper_width(l, w):
            return (l < GRIPPER_WIDTH - WITHIN_FRAME_TOL) or (w < GRIPPER_WIDTH - WITHIN_FRAME_TOL)

        def height_is_less_than_finger_reach(h):
            return h < GRIPPER_FINGER_HEIGHT - WITHIN_FRAME_TOL

        def atleast_one_of_len_or_width_is_bigger_than(l, w, target):
            return (l > target) or (w > target)

        if (not height_is_less_than_finger_reach(h=h) or not(atleast_one_of_len_or_width_fits_within_gripper_width(l=l, w=w))):
            # for x:
                # If f was > 1, was having to increase, but now went too far, 
                # so now want to scale down such that it is like a smaller-magnitude increase from the starting size
                # so now decrease by a factor larger than the inverse of f, but small enough so that we don't get bigger than where we went after the first try
                # 1/f . x > 1/f, 1/f . x < f ==> 1 < x < f^2
            x = (1+factor**2)/2

            # for m:
                # If f was not > 1, was having to decrease, but we still do not fit the gripper so need to keep decreasing
                # We go conservatively and we want f' < f but use a factor f' . x where x >= 0.75 as we do not want to halve an object which we made smaller already
                # Note than 0.90 was seen as too aggressive            
            m = 0.75

            factor = (1/factor)*x if factor > 1 else factor*m
            continue

        # Degenerate
        elif not atleast_one_of_len_or_width_is_bigger_than(l=l, w=w, target=(GRIPPER_WIDTH - WITHIN_FRAME_TOL) / 3.): 
            # for x:
                # If f was < 1, was having to decrease, but now went too far, 
                # so now want to scale down such that it is like a smaller-magnitude decrease from the starting size
                # so now decrease by a factor larger than the inverse of f, but big enough so that we don't get smaller than where we went after the first try
                # 1/f . x < 1, 1/f . x > f ==> f**2 < x < f
            x = (factor+factor**2)/2

            # for m:
                # If f was not < 1, was having to increase, but we still do not get bigger than degenerate so need to keep increasing
                # We want f' > f and use a factor f' . x where x 1<x<=2 as we do not want to halve an object which we made smaller already
            m = 3
            factor = (1/factor)*x if factor < 1 else factor*m
            continue

        # Return first factor that satisfies contraints
        else: return initial_fac*factor

    # Could not get a factor
    assert False, 'Could not get a factor within the specified number of max_tries'
    return

# Load the simulation and the mesh written in it, and allow mesh to stabilize, and extraxt info about the stable config
def _get_stable_pos_orn_scale_object_from_pybullet(sim):
    # Object urdf is the one hardcoded in the simulation.py file
    stable_pos, stable_orn_quat = _get_stable_pos_ornquat_for_init_object(sim)
    stable_orn_euler = sim.p.getEulerFromQuaternion(stable_orn_quat)
    scaling_factor = _get_scaling_factor_for_stable_object(sim, stable_pos, stable_orn_quat)
    return stable_pos, stable_orn_euler, scaling_factor


def calibrate_mesh_and_save(src, sim, urdf_file_orig):

    object_name = src.split('/')[-1]

    mesh = load_mesh(src)
    # First make the actual bbox center be the designated mesh center in the .obj file
    mesh = move_origin_to_bbox_center(mesh)
    # Save a copy for the simulation's object urdf file, so create sim after this
    dst = os.path.join(TMP_FOLDER_PATH_OBJECTS, object_name)
    save_mesh(mesh, path=dst)

    import pybullet as p
    sim.reset_state(include=NO_GRIPPER, object_urdf_path=urdf_file_orig)
    stable_pos, stable_orn_euler_rad, stable_scale = _get_stable_pos_orn_scale_object_from_pybullet(sim)

    # IMPORTANT: DO OPS IN SAME ORDER As WHEN THEY WERE COMPUTED 
    # AS COMPUTATION WAS CUMULATIVE IN _get_stable_pos_orn_scale_object_from_pybullet
    mesh = get_positioned_and_oriented_mesh(mesh=mesh, translation=stable_pos, orn_euler_rad=stable_orn_euler_rad)

    # Scaling cares about which pt is the origin (center of rotation), so make sure done on proper mesh
    mesh = scale_like_pybullet(mesh, scaling_factor=stable_scale)

    # Important as the same name of the temp file is re-used twice

    save_mesh(mesh, path=dst)
    return dst


#### Locally-used-only utils
def get_bounding_box_center(mesh):
    vertices, edges = pymesh.mesh_to_graph(mesh)
    wire_network = pymesh.WireNetwork.create_from_data(vertices=vertices, edges=edges)
    return wire_network.bbox_center

def move_origin_to_bbox_center(mesh):
    new_vertices = mesh.vertices.copy()
    center = get_bounding_box_center(mesh)
    translator = np.array([-1*center,]*new_vertices.shape[0])
    new_vertices += translator

    mesh_new = pymesh.meshio.form_mesh(new_vertices, mesh.faces, voxels=mesh.voxels)
    return mesh_new

### Testing experiment conditions

# Loads object in actual simulation environment
# Important because the object will get to intersact with the table and gripper

# Find all the grasp angles where the object fits within the gripper
# Each time, the object is rotated, so the simulation tests for stability are triggered

# If ever the object is not stable when rotated, this test will fail from within the simulation code
# If there are not enough grasp angles, this also causes failure of the test

def test_stability_and_get_grasp_angles_fitting_gripper(sim, debug, object_path, urdf_file_to_read): 
    # Sim passed in here should have object initialized at default position and orientation
    sim.reset_state(include=INCLUDE_ALL, object_urdf_path=urdf_file_to_read)
    scene_obj = Scene(sim, debug=debug)

    # Try angles in [0,20,...] (18 total)
    fits = []
    for angle_degrees in list(np.arange(0,360,20)):
        angle_radians = m.radians(angle_degrees)
        
        # Rotate object instead of gripper, this call als tests stability inder rotation
        scene_obj.R_rotate_object_in_anti_gripping_angle(angle_rotation_gripper=angle_radians, 
            post_rot_gripper_h=MIN_Z_ABOVE_GROUND+GRIPPER_FINGER_HEIGHT)
        # Open gripper before the test for fit kicks in
        scene_obj.R_open_gripper()
        # Actual fit test
        if scene_obj.T_can_gripper_be_lowered_without_colliding(): 
            fits.append(angle_degrees)
    return fits
