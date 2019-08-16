import math as m

from glo import *

### Routine methods take in a Scene, wrap around it, and manipulate the Scene 
### in designated ways by calling functions found within the Scene

def routine_gripper_LOW_around_object_grasp_rise_and_shake(scene_obj, angles_degrees=None):
    assert len(angles_degrees) > 0
    scene_obj.sim.reset_state(include=INCLUDE_ALL, keep_same_object=True)

    results_before = {}
    results_after = {}

    for idx, angle_degrees in enumerate(angles_degrees):
        angle_radians = m.radians(angle_degrees)
        scene_obj.R_rotate_object_in_anti_gripping_angle(angle_rotation_gripper=angle_radians, 
            post_rot_gripper_h=MIN_Z_ABOVE_GROUND)

        scene_obj.R_close_gripper_grasp_and_move_back_up()
        
        grasped_before, grasped_after = scene_obj.R_shake_gripper_and_see_if_still_holding()
        
        if grasped_before: results_before[angle_degrees] = +1
        else: results_before[angle_degrees] = 0

        if grasped_after: results_after[angle_degrees] = +1
        else: results_after[angle_degrees] = 0

        # Very rare edge cases
        if grasped_after and scene_obj.sim.plane_id in scene_obj.get_objects_touching_with_object(): 
            raise Exception('Success AND touching with ground at the same time!')
        
    return results_before, results_after