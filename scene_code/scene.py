import numpy as np
import math as m
import time

from glo import *

# Everything with origin at pybullet's 0,0,0 
# except when putting position into setJointMotorControlArray with p.POSITION_CONTROL (it is at object original center of mass)
class Scene:
    def __init__(self, sim, debug):
        self.debug = debug
        # Provided sim must be reset to any desired state before passing it to here
        self.sim = sim
        self.step_to_sec = 1/240
        self.torques = [0]*self.sim.num_joints

    def advance_time(self, seconds):
        assert(seconds >= self.step_to_sec)
        num_steps = int((1 / (self.step_to_sec))*seconds)
        
        for i in range(num_steps): 
            self.apply_torques()
            self.sim.p.stepSimulation()
        return

    def apply_torques(self):
        self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id, 
                                    jointIndices=list(range(self.sim.num_joints)[4:]), 
                                    controlMode=self.sim.p.VELOCITY_CONTROL, 
                                    forces=([0]*self.sim.num_joints)[4:])

        self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id,
                                    jointIndices=list(range(self.sim.num_joints)),
                                    controlMode=self.sim.p.TORQUE_CONTROL,
                                    forces=self.torques)
        return

    #### Setters change the sim wrapper state but do not move time
    def set_gripper_open(self, force):
        self.torques[self.sim.gripper_joint_indices[0]] = -force
        self.torques[self.sim.gripper_joint_indices[1]] = force
        return

    def set_gripper_close(self, force=100):
        self.torques[self.sim.gripper_joint_indices[0]] = force
        self.torques[self.sim.gripper_joint_indices[1]] = -force
        return


    #### Routines: they advance time and use sub-routines
    def R_rotate_object_in_anti_gripping_angle(self, angle_rotation_gripper, post_rot_gripper_h):
        # Angle is for gripper along vertical but here need correction of -90
        angle = angle_rotation_gripper-(m.pi/2)
        if V: print('Gripper angle: ', angle)

        # Object moves in opposite direction, instead of rotating gripper which is kept fixed
        rotation = -1*angle
        desired_orientation_quat = self.sim.p.getQuaternionFromEuler(np.array([0, 0, rotation]))
        self.sim.reset_state(object_orn=desired_orientation_quat, gripper_h=post_rot_gripper_h, include=INCLUDE_ALL, keep_same_object=True)
    
        # Match sim's default setting of starting with gripper open, BEFORE advancing time at all 
        # (never allow to close! by setting torques before the first advance_time call)
        self.R_open_gripper()    
        self.sim.p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=90, cameraPitch=193, cameraTargetPosition=[0,0,0])     

        # 1s
        self.advance_time(0.1)
        pos, orn = self.sim.p.getBasePositionAndOrientation(self.sim.object_id)
        orn_eul = self.sim.p.getEulerFromQuaternion(orn)
        
        # Assert we started indeed in a stabilized position (Any collision between gripper and object etc will cause this to fail)
        assert np.allclose([0,0,0], pos, rtol=TOL_DIST, atol=TOL_DIST), f'rotating the object to mimic rotating the gripper disrupts the stable COORDINATES: x,y,z -- want {[0,0,0]}, got {pos}'

        # Angle also stable (first two euler angles=0 and angle of rotation matched desired, and can differ by k*2*m.pi for any k -- here check up to one turn away)
        angle_stability_chk_1 = abs(orn_eul[0])<=TOL_ROT and abs(orn_eul[1])<=TOL_ROT
        angle_stability_chk_2 = any([abs(rotation-orn_eul[2])-x<=TOL_ROT for x in [2*m.pi*k for k in [0,1]]])
        assert angle_stability_chk_1 and angle_stability_chk_2, f'rotating the object to mimic rotating the gripper disrupts the other ORIENTATIONS: rx, ry -- want {[0,0,rotation]}, got {orn_eul}'
        return

    def R_shake_gripper_and_see_if_still_holding(self):
            # First metric: grasped BEFORE shaking?
            grasped_before = self.sim.object_id in self.get_objects_touching_with_gripper()

            self.R_shake_gripper()
            self.advance_time(0.1) # For making sure all contact is erased if dropped

            # Second metric: grasped AFTER shaking?
            grasped_after = self.sim.object_id in self.get_objects_touching_with_gripper()

            return [grasped_before, grasped_after]


    def R_move_up_by(self, d_up, discretization):
        start_time = time.time()

        ht_now = self.get_live_position_gripper()[2]
        while self.get_live_position_gripper()[2] < ht_now+d_up:
            self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id,
                                    jointIndices=[0,1,2],
                                    controlMode=self.sim.p.POSITION_CONTROL,
                                    targetPositions=np.array([0,0,self.get_live_position_gripper()[2]+discretization])-np.array([0,0,self.sim.gripper_susp_h]),
                                    targetVelocities=[1e-8]*3)
            self.advance_time(self.step_to_sec)

            if time.time()-start_time > GRASP_TIMEOUT: raise TimeoutError
        return

    def R_close_gripper_grasp_and_move_back_up(self):
        h = self.sim.gripper_susp_h
        self.sim.p.resetDebugVisualizerCamera(cameraDistance=0.1, cameraYaw=90, cameraPitch=193, cameraTargetPosition=[0,0,0])
        self.R_move_up_by(d_up=0.001, discretization=0.001)

        force = 40
        for i in np.arange(0,force,1):
            self.set_gripper_close(force=i)
            self.advance_time(self.step_to_sec)

        if V: print(f'Closed gripper.')

        d_up = 0.1
        discretization = 0.02

        self.R_move_up_by(d_up=d_up, discretization=discretization)
        self.sim.p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=90, cameraPitch=193, cameraTargetPosition=[0,0,0])
        return

    #### SUB-Routines: they advance time but will not be called in the run() function directly, instead called by routines
    def R_open_gripper(self):
        self.set_gripper_open(force=100)
        self.advance_time(0.05)
        return

    def R_shake_gripper(self, amp=0.5):
        shaking_width = 0.02
        discretization = 0.02

        z_always = self.get_live_position_gripper()[2]

        for i in range(6):
            y_now = self.get_live_position_gripper()[1]
            
            if i == 0: w = shaking_width/2
            elif i % 2 != 0: w = -shaking_width
            else: w = shaking_width

            while (self.get_live_position_gripper()[1] < y_now+w and i%2==0) or \
                (self.get_live_position_gripper()[1] > y_now+w and i%2!=0):
                
                d = discretization if w > 0 else -discretization

                self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id,
                                        jointIndices=[0,1,2],
                                        controlMode=self.sim.p.POSITION_CONTROL,
                                        targetPositions=np.array([0,self.get_live_position_gripper()[1]+d,z_always-self.sim.gripper_susp_h]),
                                        targetVelocities=[1e-8]*3)
                self.advance_time(self.step_to_sec)
        return


    #### Tests
    def T_is_the_gripper_here_yet(self, dst, tol=1e-5):
        return np.allclose(self.get_live_position_gripper(), dst, rtol=tol, atol=tol)

    def T_can_gripper_be_lowered_without_colliding(self):
        # Position of the two fingers
        xs = [list(self.sim.p.getLinkState(self.sim.arm_id, i)[4]) for i in [6,8]]
        # Adjust the right finger
        xs[1][1] += 0.010
        
        for x in xs:
            f = list(x)
            t = list(x)
            
            # Ray from above plane to below it
            f[2]=GRIPPER_FINGER_HEIGHT
            t[2]=0

            # Span the whole thickness of the finger (along x-axis)
            froms = [list(f)+np.array([delta_x,0,0]) for delta_x in np.arange(-0.015,0.015, 0.001)]
            tos = [list(t)+np.array([delta_x,0,0]) for delta_x in np.arange(-0.015,0.015, 0.001)]
            
            if self.debug:
                for i in range(len(froms)): self.sim.p.addUserDebugLine(lineFromXYZ=froms[i], lineToXYZ=tos[i], lineColorRGB=[0.5,0.5,0.5])

            # Check if object id in collided objects
            results = self.sim.p.rayTestBatch(rayFromPositions=froms, rayToPositions=tos)

            for tup in results:
                if tup[0] == self.sim.object_id: return False

        return True


    ##### Helpers
    def get_live_position_gripper(self):
        return [self.sim.p.getLinkState(bodyUniqueId=self.sim.arm_id, linkIndex=i)[0][i] for i in range(3)]

    def get_objects_touching_with_gripper(self):
        pts = self.sim.p.getContactPoints(bodyA=self.sim.arm_id)
        return list(set([p[2] for p in pts]))

    def get_objects_touching_with_object(self):
        pts = self.sim.p.getContactPoints(bodyA=self.sim.object_id)
        return list(set([p[2] for p in pts]))
