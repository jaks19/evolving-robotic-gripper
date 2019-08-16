import os
import sys
import argparse
import shutil
from tqdm import tqdm as tqdm

from glo import *
import utils as utils
from parallel_gripper_simulation_pybullet.simulation import Simulation
from scene_code.calibrate_mesh import calibrate_mesh_and_save, test_stability_and_get_grasp_angles_fitting_gripper
from scene_code.scene import Scene
from scene_code.segmentation import take_segmentation_image_and_get_grasp_angles, display_array, debug_segment_at_angle

# Curate a mesh individually -- calibrate, get segmentation, test stability
# Raises exception if mesh uncurable
def curate_object_from_path(object_path, sim, dst_folder, debug):
    object_name = (object_path.split('/')[-1])
    object_urdf_path_tmp = utils.edit_object_name_in_urdf_file(local_object_path=object_path, urdf_path=ORIGINAL_OBJECTS_URDF_PATH)        

    obj_dst = calibrate_mesh_and_save(sim=sim, src=object_path, urdf_file_orig=object_urdf_path_tmp)
    os.remove(object_urdf_path_tmp)
    object_urdf_path_tmp = utils.edit_object_name_in_urdf_file(local_object_path=obj_dst, urdf_path=ORIGINAL_OBJECTS_URDF_PATH)
    
    seg, all_angles_sorted = take_segmentation_image_and_get_grasp_angles(sim, object_urdf_path_tmp)
    angles_fit_within_gripper = test_stability_and_get_grasp_angles_fitting_gripper(sim, debug, obj_dst, object_urdf_path_tmp)
    
    if DEBUG_PLOTS: 
        display_array(segment_array_to_rgb(seg, id_of_interest=sim.object_id), os.path.join(DEBUG_PLOTS_FOLDER, 'segmentation-{object_name}.png'))
        
        # Angles obtained are for gripper from the vertical and moving anti-clockwise (for +ve)
        # Photo taken above object, pointing camera to [0,0,0], yaw=-90, pitch=270.001
        # Any other gripper initial angle can be used (or even rotate object instead of gripper)
        # Have to add correction to the angles for any other starting gripper config and also multiply by -1 if need to rotate object instead
        if V: print(f'top angle returned is: {all_angles_sorted[0]}, for the gripper along the vertical, which is {all_angles_sorted[0]-90} for the \
            gripper starting along the horizontal and which is in the end {(all_angles_sorted[0]-90)*-1} for rotating the object instead of the gripper')
        debug_segment_at_angle(seg, all_angles_sorted[0], name=f'{DEBUG_PLOTS_FOLDER}/{object_name}_{all_angles_sorted[0]}_deg.png')
    
    os.remove(object_urdf_path_tmp)
    return obj_dst, seg, all_angles_sorted, angles_fit_within_gripper

# Loops over files from a list of folders in a bid to curate the usable .obj meshes
def curate_objects_from_paths_list(paths_list, dst_folder, debug, overwrite):
    success = []
    failure = []

    import pybullet as p
    sim = Simulation(p, debug=debug)

    for object_path in tqdm(paths_list):
        object_name = object_path.split('/')[-1]
        
        if (not utils.check_if_mesh_name_in_processed_meshes_db(mesh_name=object_name)) or overwrite:
            try:    
                object_final_dst, seg, all_angles_sorted, angles_fit_within_gripper = curate_object_from_path(object_path, sim, dst_folder, debug)
                success.append(object_path)
                
                # Since stabilization succeeded, can save mesh and its info
                shutil.copyfile(object_final_dst, os.path.join(dst_folder, object_name))
                utils.log_segment_and_angles_to_curated_meshes_info_file(mesh_name=object_name, segment=seg, all_angles=all_angles_sorted, angles_fit_within_gripper=angles_fit_within_gripper)
            
            except AssertionError as e: 
                print(f'UNSTABILIZABLE OBJECT (printing the caught assert fail): {str(e)}')
                failure.append(object_path)

                # Move errored out mesh instead of deleting, to prevent information loss and also duplicate work if re-launching curation
                shutil.move(object_path, os.path.join(ERROR_CAUSING_MESHES_FOLDER, object_name))
                
            # Latest code commit works perfectly and should only raise an exception if the object is malformed or unstable
            # However, this is helpful when doing any code change
            except Exception as e:
                print(f'Unknown exception from object at {object_path}, text: {e}')
                raise
        else: 
            continue

    sim.p.disconnect()
    return success, failure

def main_curate_from_dirs(args):    
    paths = utils.crawl_folder_for_objs(args.src_folder)
    s, f = curate_objects_from_paths_list(debug=args.debug, paths_list=paths, dst_folder=args.dst_folder, overwrite=args.overwrite)
    print(f'{len(s)} succeeded! {len(f)} failed!')
    
    if V: print('Those failed: {f}')
    return

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Gripper morphology using ES')
    parser.add_argument('--src_folder', type=str, required=True, help="Folder with raw, unprocessed meshes to be read.")
    parser.add_argument('--dst_folder', type=str, required=True, help="Destination folder for processed meshes.")
    parser.add_argument('--debug', type=bool, default=False, help="Visualize the curation process if True.")
    parser.add_argument('--overwrite', type=bool, default=False, help="Overwrite if already in info")
    args = parser.parse_args()

    utils.create_needed_tmp_folders(TMP_FOLDERS)

    # Folders have to exist. We do not want to accidentally overwrite data that took hours to process.
    assert os.path.exists(args.src_folder), f'The source folder {args.src_folder} asked to be traversed does not exist.'
    assert os.path.exists(args.dst_folder), f'The destination folder {args.dst_folder} provided does not exist.'
    if not os.path.exists(ERROR_CAUSING_MESHES_FOLDER): os.makedirs(ERROR_CAUSING_MESHES_FOLDER)
    
    print(f'Debug mode: {args.debug}')
    main_curate_from_dirs(args=args)

    utils.delete_needed_tmp_folders(TMP_FOLDERS)