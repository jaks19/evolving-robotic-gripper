import numpy as np
import ray
import os

import ES_code.ES_utils as ES_utils
import utils

from scene_code.routines import routine_gripper_LOW_around_object_grasp_rise_and_shake
from parallel_gripper_simulation_pybullet.simulation import Simulation
from main_routine import routine_runner
from glo import *

chosen_routine_method = routine_gripper_LOW_around_object_grasp_rise_and_shake

@ray.remote(num_cpus=1)
class Worker():
    def __init__(self, objects_folder=None, debug=False):
        self.debug = debug
        self.available_object_paths = utils.crawl_folder_for_objs(objects_folder)
        return

    def angle_choosing_policy(self, graspable_angles_degrees, angles_degrees_sorted_by_thickness):
        # return graspable_angles_degrees[:3]
        return utils.angle_choosing_policy(graspable_angles_degrees=graspable_angles_degrees, angles_degrees_sorted_by_thickness=angles_degrees_sorted_by_thickness)

    def try_grasping_available_objects(self, sim, object_paths):
        # loop over objects in folder
        tot_before = 0
        tot_after = 0

        import time;
        start = time.time()
        for j, object_path in enumerate(object_paths):
            # Load object into sim
            object_name = (object_path.split('/')[-1])    

            # Load object's segmentation and angles ranking
            segmentation, angles_degrees_sorted_by_thickness, graspable_angles_degrees = utils.read_segment_and_angles_from_curated_meshes_info_file(mesh_name=object_name)

            # Desired angles = some_policy_to_choose_angles(all_angles)
            angles_degrees_chosen = self.angle_choosing_policy(graspable_angles_degrees=graspable_angles_degrees, angles_degrees_sorted_by_thickness=angles_degrees_sorted_by_thickness)

            # Run routine (but PROVIDE the angles)
            results_before, results_after = routine_runner(sim=sim, object_path=object_path, routine_method=chosen_routine_method, 
                calibrate=False, debug=self.debug, grasp_angles=angles_degrees_chosen)
            
            # If no results received, due to a weird object (extremely rare, an empty dict is returned)
            if len(results_before.keys()) < len(angles_degrees_chosen) or len(results_after.keys()) < len(angles_degrees_chosen): 
                print(f'No result for this object: {object_path}, not accounting for it in stats.')
                continue
                
            tot_before += np.mean(np.array([results_before[k] for k in results_before]))
            tot_after += np.mean(np.array([results_after[k] for k in results_after]))
        
        return tot_before / len(object_paths), tot_after / len(object_paths)

    def evaluate_models(self, batch):
        returned_container = []

        for b in batch:
            model_idx, model = b
            params_left, params_right = model

            import pybullet as p
            edited_mesh_left, edited_mesh_right = ES_utils.vertices_as_params_to_new_meshes(params_left, params_right)
            edited_mesh_left_path, edited_mesh_right_path = utils.save_edited_finger_meshes(edited_mesh_left, edited_mesh_right, folder=EDITED_FINGERS_FOLDER)

            new_arm_urdf_file = utils.edit_finger_names_in_urdf_file(edited_mesh_left_path, edited_mesh_right_path, ORIGINAL_ARM_URDF_FILE)
            sim = Simulation(p, debug=self.debug, arm_urdf=new_arm_urdf_file)
            
            results_before, results_after = self.try_grasping_available_objects(sim, self.available_object_paths)
            returned_container.append([model_idx, results_before, results_after])    
            
            p.disconnect()
            os.remove(new_arm_urdf_file)
            os.remove(edited_mesh_left_path)
            os.remove(edited_mesh_right_path)

        return returned_container

@ray.remote(num_cpus=1)
class Evaluator_Worker():
    def __init__(self, dsets_names_to_paths_dict, debug=False):
        self.debug = debug

        self.dset_names_to_list_object_paths = {}
        for dset in dsets_names_to_paths_dict.keys(): 
            self.dset_names_to_list_object_paths[dset] = utils.crawl_folder_for_objs(dsets_names_to_paths_dict[dset])
        return

    def angle_choosing_policy(self, graspable_angles_degrees, angles_degrees_sorted_by_thickness):
        # return graspable_angles_degrees[:3]
        return utils.angle_choosing_policy(graspable_angles_degrees=graspable_angles_degrees, angles_degrees_sorted_by_thickness=angles_degrees_sorted_by_thickness)

    def try_grasping_available_objects(self, sim, object_paths):
        # loop over objects in folder
        tot_before = 0
        tot_after = 0

        for j, object_path in enumerate(object_paths):
            # Load object into sim
            object_name = (object_path.split('/')[-1])    

            # Load object's segmentation and angles ranking
            segmentation, angles_degrees_sorted_by_thickness, graspable_angles_degrees = utils.read_segment_and_angles_from_curated_meshes_info_file(mesh_name=object_name)

            # Desired angles = some_policy_to_choose_angles(all_angles)
            angles_degrees_chosen = self.angle_choosing_policy(graspable_angles_degrees=graspable_angles_degrees, angles_degrees_sorted_by_thickness=angles_degrees_sorted_by_thickness)

            # Run routine (but PROVIDE the angles)
            results_before, results_after = routine_runner(sim=sim, object_path=object_path, routine_method=chosen_routine_method, 
                calibrate=False, debug=self.debug, grasp_angles=angles_degrees_chosen)
            
            # If no results received, due to a weird object (extremely rare, an empty dict is returned)
            if len(results_before.keys()) < len(angles_degrees_chosen) or len(results_after.keys()) < len(angles_degrees_chosen): 
                print(f'No result for this object: {object_path}, not accounting for it in stats.')
                continue
                
            tot_before += np.mean(np.array([results_before[k] for k in results_before]))
            tot_after += np.mean(np.array([results_after[k] for k in results_after]))
        
        return tot_before / len(object_paths), tot_after / len(object_paths)
        
    # Will receive a batch of ONE model to evaluate, 
    # but keep the batch-way of handling things for consistency with base class
    def evaluate_models(self, batch):
        returned_container = []

        for b in batch:
            model_idx, model = b
            params_left, params_right = model

            import pybullet as p
            edited_mesh_left, edited_mesh_right = ES_utils.vertices_as_params_to_new_meshes(params_left, params_right)
            edited_mesh_left_path, edited_mesh_right_path = utils.save_edited_finger_meshes(edited_mesh_left, edited_mesh_right, folder=EDITED_FINGERS_FOLDER)

            new_arm_urdf_file = utils.edit_finger_names_in_urdf_file(edited_mesh_left_path, edited_mesh_right_path, ORIGINAL_ARM_URDF_FILE)
            sim = Simulation(p, debug=self.debug, arm_urdf=new_arm_urdf_file)
            
            dset_to_results_dict_before = {}
            dset_to_results_dict_after = {}

            for dset, lst_obj_paths in self.dset_names_to_list_object_paths.items(): 
                results_before, results_after = self.try_grasping_available_objects(sim, lst_obj_paths)
                dset_to_results_dict_before[dset] = results_before
                dset_to_results_dict_after[dset] = results_after

            returned_container.append([model_idx, dset_to_results_dict_before, dset_to_results_dict_after])    
            
            p.disconnect()
            os.remove(new_arm_urdf_file)

        return returned_container