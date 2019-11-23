import os
import shutil
import pybullet as p

### Verbose-ness level
DEBUG_PLOTS = False
V = False

### Paths
_root_dir = os.path.dirname(os.path.realpath(__file__))

CURATED_FOLDER_PATH = os.path.join(_root_dir, './curated_meshes/')
DEBUG_PLOTS_FOLDER = os.path.join(_root_dir, './debug_plots/')

ROOT_URDFS_FOLDER = os.path.join(_root_dir, './urdfs/')
ORIGINAL_OBJECTS_URDF_PATH = os.path.join(ROOT_URDFS_FOLDER, 'object.urdf')
ORIGINAL_ARM_URDF_FILE = os.path.join(ROOT_URDFS_FOLDER, 'wsg_32.urdf')

ES_FOLDER = os.path.join(_root_dir, "./ES_code")

TMP_FOLDER_PATH_OBJECTS = os.path.join(_root_dir, './tmp/objects/')
TMP_FOLDER_PATH_FINGERS = os.path.join(_root_dir, './tmp/fingers/')
TMP_FOLDER_PATH_URDFS = os.path.join(_root_dir, './tmp/urdfs/')
TMP_FOLDER_PATH_RAY_LOGS = os.path.join(_root_dir, './tmp/ray/')

TMP_FOLDERS = [
	TMP_FOLDER_PATH_OBJECTS,
	TMP_FOLDER_PATH_FINGERS,
	TMP_FOLDER_PATH_URDFS,
	TMP_FOLDER_PATH_RAY_LOGS
]

CURATED_MESHES_INFO_FILE = './info/meshes_info'
ERROR_CAUSING_MESHES_FOLDER = os.path.join(_root_dir, "./error_causing_meshes/.")

EDITED_FINGERS_FOLDER = TMP_FOLDER_PATH_FINGERS

### Simulation quantities
SUSPENSION_HEIGHT = 0.4
GRIPPER_WIDTH = 0.068
GRIPPER_FINGER_HEIGHT = 0.063
ADD_TO_Z_FINGERTIP_TO_GET_Z_GRIPPER = 0.2-0.06344508
OBJECT_MASS = 0.1
MIN_Z_ABOVE_GROUND = 0.137

INCLUDE_ALL = ['object', 'gripper', 'plane']
NO_GRIPPER = ['object', 'plane']

STEP_TO_SECONDS = 1/240 # pybullet's default

GRASP_TIMEOUT = 5


### Curation (Stabilization of objects)
STABILIZATION_TIME = 60 # seconds
WITHIN_FRAME_TOL = 0.002
BELOW_FINGER_CAMERA_TOL = 0.002
assert(abs(WITHIN_FRAME_TOL) >= abs(BELOW_FINGER_CAMERA_TOL))

TOL_DIST = 0.005
TOL_ROT = 1
MAX_TRIES_SCALING = 15
ROTATION_TEST_WAIT_TIME = 0.1

SEG_DISTANCE = GRIPPER_FINGER_HEIGHT-BELOW_FINGER_CAMERA_TOL
SEG_YAW = -90
SEG_PITCH = 270.001
SEG_POSITION = [0,0,0]


### Finger editing ES
ORIG_LEFT_FINGER_PATH = os.path.join(_root_dir, "./arm_meshes/left_finger.stl")
ORIG_RIGHT_FINGER_PATH = os.path.join(_root_dir, "./arm_meshes/right_finger.stl")

ES_CUTOFF_X_MIN = 0.001
ES_CUTOFF_X_MAX = 0.017
ES_CUTOFF_Y_MIN = 0.0021
ES_CUTOFF_Y_MAX = 0.058
ES_CUTOFF_Z_MIN = -0.03
ES_CUTOFF_Z_MAX = 0.03

ES_FINGER_CLIP_DICT = {'min_x':ES_CUTOFF_X_MIN, 'max_x':ES_CUTOFF_X_MAX, 'min_y':ES_CUTOFF_Y_MIN, 'max_y':ES_CUTOFF_Y_MAX, 'min_z':ES_CUTOFF_Z_MIN, 'max_z':ES_CUTOFF_Z_MAX}

### Randomization
NUMPY_SEED = 10

### GENERAL Experiments
NUM_GRASPS_PER_OBJECT = 3
