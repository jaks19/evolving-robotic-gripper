import pymesh
import os
import numpy as np
import random

from glo import *
import utils

def get_editable_vertices_from_original_finger_meshes():
    # Left and right fingers are the same for original finger in ES method
    unedited_finger = utils.get_unedited_finger_meshes()
    overall_mask = utils.get_editable_mask_from_mesh_given_clip_dict(mesh=unedited_finger, clip_dict=ES_FINGER_CLIP_DICT)

    editable_vertices_left = unedited_finger.vertices[overall_mask]
    editable_vertices_right = editable_vertices_left.copy()
    return editable_vertices_left, editable_vertices_right

def vertices_as_params_to_new_meshes(params_left, params_right):
    # Original fingers are the same in ES method for left and right
    unedited_finger = utils.get_unedited_finger_meshes()
    editable_mask = utils.get_editable_mask_from_mesh_given_clip_dict(mesh=unedited_finger, clip_dict=ES_FINGER_CLIP_DICT)
    editable_vertices_indices = np.array(editable_mask).nonzero()[0]

    final_vertices_left = unedited_finger.vertices.copy()
    final_vertices_left[editable_vertices_indices] = params_left.value

    final_vertices_right = unedited_finger.vertices.copy()
    final_vertices_right[editable_vertices_indices] = params_right.value
    
    edited_mesh_left = pymesh.meshio.form_mesh(vertices=final_vertices_left, faces=unedited_finger.faces, voxels=unedited_finger.voxels)
    edited_mesh_right = pymesh.meshio.form_mesh(vertices=final_vertices_right, faces=unedited_finger.faces, voxels=unedited_finger.voxels)

    return edited_mesh_left, edited_mesh_right