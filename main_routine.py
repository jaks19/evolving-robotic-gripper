import os
import argparse
import numpy as np

from glo import *
import utils

from main_curate_from_dirs import curate_object_from_path
from parallel_gripper_simulation_pybullet.simulation import Simulation

from scene_code.scene import Scene
from scene_code.routines import routine_gripper_LOW_around_object_grasp_rise_and_shake

def routine_runner(sim, object_path, calibrate, debug, routine_method, grasp_angles=[]):
    object_name = (object_path.split('/')[-1])
    object_urdf_path_tmp = utils.edit_object_name_in_urdf_file(local_object_path=object_path, urdf_path=ORIGINAL_OBJECTS_URDF_PATH)        

    # Center, scale, translate and rotate mesh to stability, for never-processed mesh
    # Will overwrite the object_urdf_path_tmp
    if calibrate: 
        obj_dst, _, _, angles_fit_within_gripper = curate_object_from_path(object_path=object_path, sim=sim, dst_folder=CURATED_FOLDER_PATH, debug=debug)
        assert(len(angles_fit_within_gripper) > 0)
        if len(grasp_angles)==0: grasp_angles = angles_fit_within_gripper
        
        os.remove(object_urdf_path_tmp)
        object_urdf_path_tmp = utils.edit_object_name_in_urdf_file(local_object_path=obj_dst, urdf_path=ORIGINAL_OBJECTS_URDF_PATH)

    # Already calibrated
    else:
        # check_if_object_has_been_calibrated_before
        sim.reset_state(include=INCLUDE_ALL, object_urdf_path=object_urdf_path_tmp)
        sim.p.stepSimulation()
        pos, orn = sim.p.getBasePositionAndOrientation(sim.object_id)
        assert np.allclose([0,0,0], pos, rtol=1e-3, atol=1e-3), f"UNSTABLE: object loaded should settle at pos [0,0,0] but settles at {pos}"
        assert np.allclose([0,0,0,1], orn, rtol=1e-1, atol=1e-1), f"UNSTABLE: object loaded should settle at orn [0,0,0,1] (quat) but settles at {orn}"

        # User did not provide custom grasp angles in degrees
        if len(grasp_angles)==0:
            # Look in database, and if object was curated before, it has some stored angles where object fit within gripper
            _, _, grasp_angles = utils.read_segment_and_angles_from_curated_meshes_info_file(object_name)
    try:
        # Now we are safe to run a grasping routine with a stable object and some grasp angles
        scene_obj = Scene(sim, debug=debug)
        results_before, results_after = routine_method(scene_obj, angles_degrees=grasp_angles)
        if len(results_before.keys()) < len(grasp_angles) or len(results_after.keys()) < len(grasp_angles): 
            print(f"{len(results_before.keys())} grasps worked, provided {len(grasp_angles)} different angles -- ref: {object_path}")

    except TimeoutError:
        # VERY RARE: Object causes gripper to get stuck, maybe because of a weird shape
        print(f'timeout in routine due to violating object: {object_path}')
        results_before, results_after = {}, {}

    except Exception as e:
        print(f'Unaccounted-for exception: Guilty object: {object_path}')
        print(e)
        results_before, results_after = {}, {}
        # raise

    os.remove(object_urdf_path_tmp)
    return results_before, results_after

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run a basic routine')
    parser.add_argument('--obj_path', type=str, required=True)
    parser.add_argument('--calibrate', type=bool, default=False, required=False)
    parser.add_argument('--debug', type=bool, default=False, required=False)
    parser.add_argument('--angles', nargs='*', default=[], required=False)
    args = parser.parse_args()

    utils.create_needed_tmp_folders(TMP_FOLDERS)

    import pybullet as p
    sim = Simulation(p, debug=args.debug)
    if args.angles is not None: args.angles = [float(x) for x in args.angles]

    chosen_routine_method = routine_gripper_LOW_around_object_grasp_rise_and_shake
    results = routine_runner(sim=sim, object_path=args.obj_path, debug=args.debug, calibrate=args.calibrate, grasp_angles=args.angles, routine_method=chosen_routine_method)
    
    print(results)
    sim.p.disconnect()

    utils.delete_needed_tmp_folders(TMP_FOLDERS)

    