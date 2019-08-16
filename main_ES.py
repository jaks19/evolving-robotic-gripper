import os
import shutil
import ray
from tqdm import tqdm
import argparse
import pickle
from tensorboardX import SummaryWriter
import psutil

from glo import *
from ES_code.worker import *
from ES_code.solver import *

import utils as utils
import ES_code.ES_utils as ES_utils

def setup_test_folders_dict(args):
    all_test_folders = [args.train_folder]+args.test_folders
    assert len(all_test_folders) > 0

    test_dsets_names_to_paths_dict = {}
    for pth in all_test_folders:
        folder_name = utils.get_folder_basename_from_path(pth)
        test_dsets_names_to_paths_dict[folder_name] = pth
    return test_dsets_names_to_paths_dict

def log_evaluator_results_dict_to_tboard(cur_perf_before, cur_perf_after, writer, i):
    for dset_name, dset_perf in cur_perf_before.items():
        writer.add_scalar(f'performance on {dset_name} BEFORE shaking', dset_perf, i)
        print(f'Perf on {dset_name} BEFORE shaking: {dset_perf}')

    for dset_name, dset_perf in cur_perf_after.items():
        writer.add_scalar(f'performance on {dset_name} AFTER shaking', dset_perf, i)
        print(f'Perf on {dset_name} AFTER shaking: {dset_perf}')
    return

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Gripper morphology using ES')
    parser.add_argument('--num_cores', type=int, default=6)
    parser.add_argument('--npop', type=int, default=50)
    parser.add_argument('--noise_sigma', type=str, default=0.001)
    parser.add_argument('--checkpoint', type=str, default=None)
    
    parser.add_argument('--train_folder', type=str)
    parser.add_argument('--test_folders', nargs='+', required=True)

    parser.add_argument('--log_dir', type=str)
    parser.add_argument('--debug', type=bool, default=False)
    args = parser.parse_args()

    utils.create_needed_tmp_folders(TMP_FOLDERS)
    if not os.path.exists(args.log_dir): os.makedirs(args.log_dir)
    if not os.path.exists(os.path.join(args.log_dir, 'models')): os.makedirs(os.path.join(args.log_dir, 'models'))
    if not os.path.exists(os.path.join(args.log_dir, 'runs')): os.makedirs(os.path.join(args.log_dir, 'runs'))
    writer = SummaryWriter(log_dir=os.path.join(args.log_dir,'runs'))

    print('num_cores available on machine: ', psutil.cpu_count())
    print('num_cores chosen: ', args.num_cores)

    if os.path.exists(TMP_FOLDER_PATH_RAY_LOGS): shutil.rmtree(TMP_FOLDER_PATH_RAY_LOGS)
    os.makedirs(TMP_FOLDER_PATH_RAY_LOGS)

    ray.init(plasma_directory=TMP_FOLDER_PATH_RAY_LOGS, configure_logging=False, num_cpus=args.num_cores)
    num_workers = args.num_cores-1

    # Our model (the starting gripper)
    if args.checkpoint is not None: vertices_left, vertices_right = pickle.load(open(args.checkpoint, 'rb'))
    else: vertices_left, vertices_right = ES_utils.get_editable_vertices_from_original_finger_meshes()
    if not os.path.exists(EDITED_FINGERS_FOLDER): os.makedirs(EDITED_FINGERS_FOLDER)

    model = [
        # Clip the y-direction so that the gripper does not protrude beyond the allowed (when rotated, y axis-->z axis in the simulation)
        Parameter(initial_value=vertices_left, noise_sigma=float(args.noise_sigma), noise_mu=0, lb=[ES_CUTOFF_X_MIN, ES_CUTOFF_Y_MIN, ES_CUTOFF_Z_MIN], ub=[ES_CUTOFF_X_MAX, ES_CUTOFF_Y_MAX, ES_CUTOFF_Z_MAX]), 
        Parameter(initial_value=vertices_right, noise_sigma=float(args.noise_sigma), noise_mu=0, lb=[ES_CUTOFF_X_MIN, ES_CUTOFF_Y_MIN, ES_CUTOFF_Z_MIN], ub=[ES_CUTOFF_X_MAX, ES_CUTOFF_Y_MAX, ES_CUTOFF_Z_MAX])
    ]

    # Perform evol search
    solver = ES_parallel(params=model, npop=args.npop, sigma=0.1, alpha=0.03, num_workers=num_workers)

    # Start
    workers = [Worker.remote(debug=args.debug, objects_folder=args.train_folder) for i in range(num_workers)]
    
    # Will evaluate on the training set + cross-evaluation on a collection of test datasets
    test_dsets_names_to_paths_dict = setup_test_folders_dict(args=args)
    evaluator = Evaluator_Worker.remote(debug=args.debug, dsets_names_to_paths_dict=test_dsets_names_to_paths_dict)
    cur_evaluation_id = evaluator.evaluate_models.remote([[-1, model]])

    if args.checkpoint is not None: num_done_iters = int(args.checkpoint.split("/")[-1].split("-")[0].split("_")[1])
    else: num_done_iters = 0

    for i in tqdm(range(10000)):  
        i = i + num_done_iters
        
        latest_model = solver.run_one_iteration(workers)
        
        # (2): Record the previous iter's model performance here, as evaluator worker from (1)
        # got enough time to finish running, while solver.run_one_iteration(workers) above was running,
        # to boost parallelism
        if cur_evaluation_id is not None and i%3==0:
            # Results are in the form of a dict of dset name to perf
            cur_perf_dict_before, cur_perf_dict_after = ray.get(cur_evaluation_id)[0][1:]
            # Hand-pick perf on the training dataset itself to keep track of progress of ES algorithm
            cur_perf_before = cur_perf_dict_before[utils.get_folder_basename_from_path(args.train_folder)]
            cur_perf_after = cur_perf_dict_after[utils.get_folder_basename_from_path(args.train_folder)]

            if i == 0: print(f'Starting performance: before --> {cur_perf_before} after --> {cur_perf_after}')
            else: print(f'iter {i}, cur_perf: before --> {cur_perf_before} after --> {cur_perf_after}')

            # Log perf on all datasets
            log_evaluator_results_dict_to_tboard(cur_perf_dict_before, cur_perf_dict_after, writer, i)

            # (1): Launch evaluator worker on latest model, let it run in the background
            cur_evaluation_id = evaluator.evaluate_models.remote([[-1, latest_model]])

            # Save model
            dumping = [x.value for x in latest_model]
            pickle.dump(dumping, open(os.path.join(args.log_dir, 'models', f'iter_{i}.p'), 'wb'))

            # Also save the mesh file for easy visualization of progress
            edited_mesh_left, edited_mesh_right = ES_utils.vertices_as_params_to_new_meshes(params_left=latest_model[0], params_right=latest_model[1])
            edited_mesh_left_path, edited_mesh_right_path = utils.save_edited_finger_meshes(edited_mesh_left, edited_mesh_right, folder=args.log_dir, iternum=i)

    utils.delete_needed_tmp_folders(TMP_FOLDERS)
