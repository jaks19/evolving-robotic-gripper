# evolving-robotic-gripper

In this work we use our own WSG-32 parallel-jaw gripper simulation which we [open-sourced](https://github.com/jaks19/parallel_gripper_simulation_pybullet), and collect a dataset of objects from distinct categories (e.g. bottles or cones or adversarial objects...)  on which we would like our *optimized gripper morphologies* (see second animation) to have better grasp success than the out-of-the-box WSG-32 gripper configuration.

![GUI wrapper preview](https://github.com/jaks19/evolving-robotic-gripper/blob/master/gifs/gif_sim.png)

![GUI wrapper preview](https://github.com/jaks19/evolving-robotic-gripper/blob/master/gifs/gif_optimizing.png)

*Note*: Anyone using this repo simply has to collect a few .obj meshes of objects on which they would like their gripper to perform well. We provide a few meshes in raw_meshes folder to start.

We search the space of gripper morphologies using augmented random search, and propose changes to the base morphology at each iteration by: randomly adding noise to the 3D finger mesh coordinates. 

*Note*: We have also implemented deformation proposals using computer graphics methods for mesh deformation, such as radial-basis function interpolation or free-form deformation. If interested, please email the author for the code, which is still in experimental mode.

We find that: 
* optimizing the shape of the robotic fingers increases grasp performance
* restricting our evaluation set of objects to a specific type (e.g. bottles or cones or adversarial objects...) reflects on the salient visual features of the emergent gripper finger morphologies

## Example commands for how to use the code:
*Note*: We have the commands in order below and reading them in order will give you a good idea on how to use the code to process meshes and then train on them.

##### python main_curate_from_dirs.py --src_folder [path to uncurated raw meshes] --dst_folder [path to destination folder] --debug True

Note that --dst_folder must exist already. We do not create it to avoid overwriting files inadvertently.

##### python main_routine.py --obj_path [path to an object mesh] --calibrate True --debug True --angles 20 170 330

##### python main_ES.py --objects_folder [path to CURATED meshes] --num_cores 51 --npop 50 --log_dir [path to log folder for checkpts]

##### python main_GEN.py --train_folder [folder with CURATED meshes to train on] --test_folders [folder with CURATED meshes to test on] --log_dir [path to log dir]
