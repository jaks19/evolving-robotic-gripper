## The Simulation class creates a pybullet environment
## Loads the gripper, object and plane urdf files
## Sets quantities for referencing by any wrapper

from glo import *

class Simulation(object):
    def __init__(self, p, arm_urdf=None, debug=False):
        self.p = p

        if not debug: self.p.connect(self.p.DIRECT)
        else: self.p.connect(self.p.GUI)

        self.p.configureDebugVisualizer(self.p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0)
        self.p.configureDebugVisualizer(self.p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)

        self.set_constants()
        self.arm_urdf = arm_urdf if arm_urdf is not None else ORIGINAL_ARM_URDF_FILE
        
        return

    def reset_camera(self):
        self.p.resetDebugVisualizerCamera(cameraDistance=SEG_DISTANCE, cameraYaw=SEG_YAW, cameraPitch=SEG_PITCH, cameraTargetPosition=SEG_POSITION)
        self.cam_distance_yaw_pitch = [SEG_DISTANCE, SEG_YAW, SEG_PITCH]
        self.cam_pos = SEG_POSITION
        return

    def set_constants(self):
        # Joints tally
        self.num_joints = 9
        self.gripper_joint_indices = [5, 7]
        self.controlled_joints = [0, 1, 2, 3] + self.gripper_joint_indices

        # Limits
        self.ll = [-10, -10, -10, -10, 0, -100, 0, -100, 0]
        self.ul = [10, 10, 10, 10, 0, 100, 0, 100, 0]
        return

    def reset_state(self, include=[], object_pos=[0,0,0], object_orn=[0,0,0,1], object_mass=OBJECT_MASS, object_scale=1, gripper_h=SUSPENSION_HEIGHT, object_urdf_path=None, keep_same_object=False):
        # Restart the simulation engine
        self.p.resetSimulation()

        # Camera
        self.reset_camera()

        # Init simulation variables
        self.p.setRealTimeSimulation(False)
        self.gravity = -10
        self.p.setGravity(0, 0, self.gravity)

        # Objects
        self.load_designs(include, object_pos, object_orn, object_mass, object_scale, gripper_h, object_urdf_path, keep_same_object)

        # Torques
        self.gripper_torque_vec = [0]*2
        return

    def load_designs(self, include, object_pos, object_orn, object_mass, object_scale, gripper_h, object_urdf_path, keep_same_object):
        if 'gripper' in include:
            self.gripper_susp_h = gripper_h
            # Weiss SG-32 hand
            self.arm_id = self.p.loadURDF(fileName=self.arm_urdf, basePosition=[0, 0, gripper_h], useFixedBase=True)
            # Initial position of gripper, needed for debugger sliders to work
            self.pos = [0,0,gripper_h,0]
            # Open gripper from the start, wait 100 steps to allow opening due to position control
            self.p.setJointMotorControlArray(bodyIndex=self.arm_id, jointIndices=self.gripper_joint_indices, controlMode=self.p.VELOCITY_CONTROL, forces=[0]*len(self.gripper_joint_indices))
            self.p.setJointMotorControlArray(bodyIndex=self.arm_id, jointIndices=self.gripper_joint_indices, controlMode=self.p.POSITION_CONTROL, targetPositions=[-100,100])
            for i in range(100): self.p.stepSimulation()

        if 'plane' in include:
            # Surface to land on
            self.plane_id = self.p.loadURDF(os.path.join(ROOT_URDFS_FOLDER, "plane.urdf"), [0, 0, 0], useFixedBase=True)

        if 'object' in include:
            if not keep_same_object: self.object_urdf_path = object_urdf_path
            # Object to be grasped
            self.object_id = self.p.loadURDF(fileName=self.object_urdf_path, basePosition=object_pos, baseOrientation=object_orn, 
            useFixedBase=False, globalScaling=object_scale)
            
            self.p.changeDynamics(self.object_id, -1, mass=object_mass)
            
        self.bodies = include
        return