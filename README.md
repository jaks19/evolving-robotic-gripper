# evolving-robotic-gripper

# gripper-adaptable-morphology

##### python main_curate_from_dirs.py --src_folder [path to uncurated raw meshes] --dst_folder [path to destination folder] --debug True

Note that --dst_folder must exist already. We do not create it to avoid overwriting files inadvertently.

##### python main_routine.py --obj_path [path to an object mesh] --calibrate True --debug True --angles 20 170 330

##### python main_ES.py --objects_folder [path to CURATED meshes] --num_cores 51 --npop 50 --log_dir [path to log folder for checkpts]

##### python main_GEN.py --train_folder [folder with CURATED meshes to train on] --test_folders [folder with CURATED meshes to test on] --log_dir [path to log dir]
