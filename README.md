# evolving-robotic-gripper

In this work we use our own WSG-32 parallel-jaw gripper simulation which we [open-sourced](https://github.com/jaks19/parallel_gripper_simulation_pybullet), and collect a dataset of objects from distinct categories (e.g. bottles or cones or adversarial objects...)  on which we would like our *optimized gripper morphologies* (see second animation) to have better grasp success than the out-of-the-box WSG-32 gripper configuration.

![GUI wrapper preview](https://github.com/jaks19/evolving-robotic-gripper/blob/master/gifs/gif_sim.gif)

![GUI wrapper preview](https://github.com/jaks19/evolving-robotic-gripper/blob/master/gifs/gif_optimizing.gif)

*Note*: Anyone using this repo simply has to collect a few .obj meshes of objects on which they would like their gripper to perform well. We provide a few meshes in raw_meshes folder to start.
Some other sources are: [princeton ModelNet dataset](https://modelnet.cs.princeton.edu/) and [DexNet dataset](https://berkeley.app.box.com/s/6mnb2bzi5zfa7qpwyn7uq5atb7vbztng_) from Berkeley.

We search the space of gripper morphologies using augmented random search, and propose changes to the base morphology at each iteration by: randomly adding noise to the 3D finger mesh coordinates. 

*Note*: We have also implemented deformation proposals using computer graphics methods for mesh deformation, such as radial-basis function interpolation or free-form deformation. If interested, please email the author for the code, which is still in experimental mode.

We find that: 
* optimizing the shape of the robotic fingers increases grasp performance
* restricting our evaluation set of objects to a specific type (e.g. bottles or cones or adversarial objects...) reflects on the salient visual features of the emergent gripper finger morphologies

## Example commands for how to use the code:
*Note*: We have the commands in below in a specific order such that reading them in that order tells the story of how to use our code and how different modules interact.

### First any mesh needs to be processed
Need to alter object meshes' positions and orientations so they appear right on the center of our table and are ready for grasping. This is important because we use a top-view segmentation of the objects to know where to grasp.

```python main_curate_from_dirs.py --src_folder [path to uncurated raw meshes] --dst_folder [path to destination folder] --debug True```

Note 1) that --dst_folder must exist already. We do not create it to avoid overwriting files inadvertently.

Note 2) that curation will create a curated replica of each object mesh into the --dst_folder

Note 3) that the info folder in the root project folder should always be there. When you first curate an object, a database will be created and everytime you curate some mesh, its name and its stable point of contact, stable orientation etc (found during calibration) will be written to this database file. When you carry out experiments, every time an object is loaded, info from this database is used so that the object appears stably in the center of the table.

Note 4) That some meshes (rarely) just cannot be calibrated for our experiments, when perhaps they have a faulty design etc and these will not be saved in --dst_folder. At the end of curation of a whole folder, you will see printed: "n succeeded! k failed!" if your --src_folder size was n+k.


### To get an idea about how our grasping routine works, you could visualize a few grasps
```python main_routine.py --obj_path [path to an object mesh] --calibrate True --debug True --angles 20 170 330```

Essentially this command runs an example grasp on a chosen object and you can view it using debug=True and you can choose which angles to grasp the object at.

Note 1) that actually for visualizing the grasp routine, you could use raw meshes and simply use the calibrate=True option above to calibrate them on the fly. But to run batch experiments eventually, you should perform the step before this one.

2) It will print "({20.0: 1, 170.0: 1, 330.0: 1}, {20.0: 1, 170.0: 0, 330.0: 0})" which is how successful each grasp was at each angle) and here it is doing two prints per angle *because we shake objects after we grasp them and raise them* so there is a *score before and after shaking*.

### IMPORTANT! Never re-curate an already-curated mesh as it corrupts it, which is why we have distinct folders for raw and curated meshes. If a mesh already exists in dst_folder, and you run another curation and a mesh of the same name is being curated, it will overwrite the existing curated one!

### Running augmented random search on the gripper finger meshes morphologies' with evaluation on some chosen dataset as metric for how good morphologies are performing

You are essentially ready to carry out morphology optimization of the gripper finger meshes. Here you need curated/processed meshes for the training to run.

```
python main_ES.py 
--train_folder [path to CURATED meshes] 
--num_cores 51 
--npop 50 
--log_dir [path to log folder for checkpts] 
--test_folders [path to CURATED meshes which are unseen during training, to see degree of overfitting, and you may include several such folders]
```

Essentially this command evaluates your initial gripper on the folder of objects you chose for objects_folder and proposes random modifications to the finger 3D meshes and keep iterating, to increase grasp performance. After a fixed number of epochs, the grippers are tested on unseen objects from --test_folders. 

Bonus: Your gripper fingers are saved automatically in the log folder and we implemented the procedure to read any saved finger morphologies easily using `--checkpoint` arg which should point to any desired saved gripper fingers checkpoint file.
