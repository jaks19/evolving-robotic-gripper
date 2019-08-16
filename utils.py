import pymesh
import os
import numpy as np
import math as m
import random
import shutil
import h5py

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

from glo import *

### Editing urdfs on the fly

def get_new_urdf_path_tmp(urdf_path_orig):
    split = urdf_path_orig.split('/')
    urdf_name = split[-1]
    urdf_name_with_idx = "".join([str(x) for x in [random.randint(0,9) for i in range(10)]]) + urdf_name
    return os.path.join(TMP_FOLDER_PATH_URDFS, urdf_name_with_idx)

# Change the loaded object in the urdf file text and save the urdf file to the given dest
def edit_object_name_in_urdf_file(local_object_path, urdf_path):
    # An .obj file
    assert(len(local_object_path.split('.'))>1 and 'obj' in local_object_path.split('.'))

    # An .urdf file
    assert(len(urdf_path.split('.'))>1 and 'urdf' in urdf_path.split('.'))
    urdf_new_path = get_new_urdf_path_tmp(urdf_path_orig=urdf_path)

    import xml.etree.ElementTree as ET

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    for mesh in root.iter("mesh"):
        mesh.set('filename', local_object_path)

    # Cannot write to other path using ET directly
    tree.write(urdf_new_path)
    return urdf_new_path

def edit_finger_names_in_urdf_file(left_finger_path, right_finger_path, urdf_path):
    # An .obj file
    assert(len(left_finger_path.split('.'))>1 and 'stl' in left_finger_path.split('.'))
    assert(len(right_finger_path.split('.'))>1 and 'stl' in right_finger_path.split('.')) 

    # An .urdf file
    assert(len(urdf_path.split('.'))>1 and 'urdf' in urdf_path.split('.'))
    urdf_new_path = get_new_urdf_path_tmp(urdf_path_orig=urdf_path)

    import xml.etree.ElementTree as ET

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    for mesh in root.iter("mesh"):
        if 'left_finger' in mesh.get('filename'): mesh.set('filename', left_finger_path)
        if 'right_finger' in mesh.get('filename'): mesh.set('filename', right_finger_path)

    tree.write(urdf_new_path)
    return urdf_new_path

def get_unedited_finger_meshes():
    # Meshes are the same for original finger (right and left)
    return load_mesh(ORIG_LEFT_FINGER_PATH)

# Meshes must be in raw position to be loaded by pybullet from stl file
def save_edited_finger_meshes(edited_mesh_left, edited_mesh_right, folder, iternum=None):
    if iternum is None: iternum = "".join([str(x) for x in [random.randint(0,9) for i in range(10)]])

    pth_left = os.path.join(folder, f"{iternum}_left_finger.stl")
    pth_right = os.path.join(folder, f"{iternum}_right_finger.stl")

    save_mesh(mesh=edited_mesh_left, path=pth_left)
    save_mesh(mesh=edited_mesh_right, path=pth_right)
    return pth_left, pth_right
### Basic mesh ops

def load_mesh(path):
    mesh = pymesh.load_mesh(path)
    pymesh.remove_duplicated_vertices(mesh, tol=1e-12, importance=None)
    return mesh

def save_mesh(mesh, path):
    if os.path.exists(path): os.remove(path)
    pymesh.save_mesh(path, mesh)
    return

def visualize(pymesh_mesh=None, save_path=None):
    # Create a new plot
    figure = plt.figure()
    axes = mplot3d.Axes3D(figure)
    
    from stl import mesh as npmesh
    
    def convert_pymesh_to_npmesh(pymesh_mesh):
        vertices = pymesh_mesh.vertices
        faces = pymesh_mesh.faces

        mesh_np = npmesh.Mesh(np.zeros(faces.shape[0], dtype=npmesh.Mesh.dtype))
        for count, face in enumerate(faces):
            for axis in range(3):
                mesh_np.vectors[count][axis] = vertices[face[axis],:]
        return mesh_np

    # Load the STL files and add the vectors to the plot
    your_mesh = convert_pymesh_to_npmesh(pymesh_mesh)
    axes.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors))

    # Auto scale to the mesh size
    scale = your_mesh.points.flatten(-1)
    axes.auto_scale_xyz(scale, scale, scale)

    axes.view_init(0, -180)

    # Save plot as name 'path'
    plt.savefig(save_path)
    return

# Rotation: euler rad
def rotate_like_pybullet(mesh, orn_euler_rad, orn_inverse=False):
    new_v = mesh.vertices.copy()
    axes = [np.array(ax) for ax in [[1,0,0],[0,1,0],[0,0,1]]]
    rots = [pymesh.Quaternion.fromAxisAngle(axes[i], orn_euler_rad[i]) for i in range(len(axes))] 
    rots = [rot.to_matrix() for rot in rots]
    
    if orn_inverse: 
        rots.reverse()
        rots = [np.linalg.inv(rot) for rot in rots]
        
    for rot in rots: new_v = np.dot(rot, new_v.T).T
    return pymesh.meshio.form_mesh(new_v, mesh.faces, mesh.voxels)

def translate_like_pybullet(mesh, translation):
    new_v = mesh.vertices.copy()
    new_v += np.array(translation)
    return pymesh.meshio.form_mesh(new_v, mesh.faces, mesh.voxels)

def scale_like_pybullet(mesh, scaling_factor=1.0):
    new_v = mesh.vertices.copy()*scaling_factor
    return pymesh.meshio.form_mesh(new_v, mesh.faces, mesh.voxels)

def get_positioned_and_oriented_mesh(mesh, translation, orn_euler_rad):
    # Forward direction: rotate then translate
    mesh = rotate_like_pybullet(mesh, orn_euler_rad)
    mesh = translate_like_pybullet(mesh, translation)
    return mesh

def REVERSE_get_positioned_and_oriented_mesh(mesh, translation, orn_euler_rad):
    # Reverse direction: translate then rotate
    mesh = translate_like_pybullet(mesh, list(-1*np.array(translation)))
    mesh = rotate_like_pybullet(mesh, orn_euler_rad, orn_inverse=True)
    return mesh

def get_editable_mask_from_mesh_given_clip_dict(mesh, clip_dict):
    V = mesh.vertices.copy()
    
    masks = [V[:,0] > clip_dict['min_x'], V[:,0] < clip_dict['max_x'], \
        V[:,1] > clip_dict['min_y'], V[:,1] < clip_dict['max_y'], \
        V[:,2] > clip_dict['min_z'], V[:,2] < clip_dict['max_z']]
    
    overall_mask = elementwise_AND(masks)
    return overall_mask


### Vertices and Edges operations

def get_vertices_and_edges(obj_mesh):
    vertices, edges = pymesh.mesh_to_graph(obj_mesh)
    return vertices, edges

def make_edge_set_bidirectional(edge_set):
    assert edge_set.shape[1]==2
    
    new_rows = []
    for r in edge_set:
        new_rows.append([r[1],r[0]])
    
    bidir_edges = np.vstack([edge_set, np.vstack(new_rows)])
    return np.unique(bidir_edges, axis=0)

# A mesh from pymesh has a set V and E, where E refers to indices of vertices
# When only a subset of the vertices is kept, this method gets the corresponding subset of edges left
# The mask is a list with booleans for ALL original vertices, indicating whether a vertex is kept (True) or not (False)
def get_edges_subset_given_all_edges_and_vertices_mask(all_edges, vertices_mask):
    map_old_to_new_V_indices = {}
    counted = 0
    for ind, x in enumerate(vertices_mask):
        if x: 
            map_old_to_new_V_indices[ind] = counted
            counted += 1

    final_edges = []
    for e in all_edges:
        if vertices_mask[e[0]] and vertices_mask[e[1]]:
            final_edges.append([map_old_to_new_V_indices[e[0]], map_old_to_new_V_indices[e[1]]])
    return np.array(final_edges)

# Get the superset of vertices and re-organize edge set which uses indices
def merge_two_sets_vertices_and_edges(V_1, E_1, V_2, E_2):
    assert(V_1.shape == V_2.shape)
    assert(E_1.shape == E_2.shape)
    
    final_V = np.vstack([V_1, V_2])

    shift = V_1.shape[0]
    edit_E_2 = E_2.copy()
    for edge in edit_E_2:
        edge[0] += shift
        edge[1] += shift
    
    final_E = np.vstack([E_1.copy(), edit_E_2])
    assert(final_E.shape[0]==E_1.shape[0]+E_2.shape[0])
    
    return final_V, final_E
    

### Tmp folders

def create_needed_tmp_folders(folders):
    for f in folders:
        if not os.path.exists(f): 
            os.makedirs(f)
    return

def delete_needed_tmp_folders(folders):
    for f in folders:
        if os.path.exists(f): 
            shutil.rmtree(f)
    return

def remove_old_files(folder):
    for the_file in os.listdir(folder):
        file_path = os.path.join(folder, the_file)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
        except Exception as e:
            print(e)
    return

### Directories / directory names...

def crawl_folder_for_objs(folder, size_limit_mb=20):
    matches = []
    for root, dirnames, filenames in os.walk(folder):
        for filename in filenames:
            if filename.endswith('.obj'):
                path = os.path.join(root, filename)
                # if larger than 5 MB
                if os.stat(path).st_size/1e6 < size_limit_mb:
                    matches.append(path)
    return matches           

def get_folder_basename_from_path(path):
    split = path.split('/')
    folder_name = split[-1] if split[-1] != "" else split[-2]
    return folder_name

### Reading/writing to db

def log_segment_and_angles_to_curated_meshes_info_file(mesh_name, segment, all_angles, angles_fit_within_gripper):
    # overwrites existing entries so handle filtering before calling this fn
    with h5py.File(CURATED_MESHES_INFO_FILE, 'a') as f:
        try: del f[mesh_name]
        except KeyError: pass

        grp=f.create_group(mesh_name)
        grp.create_dataset('segmentation', data=segment)
        grp.create_dataset('all_angles', data=all_angles)
        grp.create_dataset('angles_that_fit_gripper', data=angles_fit_within_gripper)
    return

def read_segment_and_angles_from_curated_meshes_info_file(mesh_name):
    with h5py.File(CURATED_MESHES_INFO_FILE, 'r') as f:
        segmentation = np.array(f[mesh_name]['segmentation'])
        all_angles_degrees = np.array(f[mesh_name]['all_angles'])
        angles_that_fit_gripper = np.array(f[mesh_name]['angles_that_fit_gripper'])
    return segmentation, all_angles_degrees, angles_that_fit_gripper

def check_if_mesh_name_in_processed_meshes_db(mesh_name):
    # Try accessing that mesh name from the global db of meshes
    try:
        with h5py.File(CURATED_MESHES_INFO_FILE, 'r') as f: f[mesh_name]
    # Not found
    except: return False
    # Found, no problem
    return True

### MISC

# Given a list of numpy filters (or generally, a list of lists of bools)
# Returns elementwise AND (assumes all lists of same length)
def elementwise_AND(list_of_lists):
    lst_length = len(list_of_lists[0])
    for lst in list_of_lists: assert len(lst)==lst_length, f'expected list of length {lst_length}, got {len(lst)}'
    return [all([lst[i] for lst in list_of_lists]) for i in range(lst_length)]

def angle_choosing_policy(graspable_angles_degrees, angles_degrees_sorted_by_thickness):
    # Graspable angles are at 20 degrees interval i.e. [0,20,40,60,...], with angles where object exceeds gripper width REMOVED from the list
    # Example of a real case: [0,60,80,100,160,240,...]

    # Pre-proxify the list of graspable angles because
    # angles_degrees_sorted_by_thickness does not have angles beyond 180 degrees as same thickness as its angle-180 counterpart
    pre_proxies_graspable_angles_degrees = [a if a <=180 else a-180 for a in graspable_angles_degrees]

    # Not all angles: [0,1,2,3,4,...] found in angles_degrees_sorted_by_thickness
    # At some angles, method of finding grasp widths based on segmentation could not figure out width
    # (Intersection of line with outline method)

    # So if need to read width of grasp at angle a from sorted list and it is not present,
    # read width at b* where b* = argmin_a'{|a'-a|}
    # Actual graps will be done at a but b* gives a proxy to the thickness
    proxies_graspable_angles_degrees = []
    for a in pre_proxies_graspable_angles_degrees:
        diffs = np.array(angles_degrees_sorted_by_thickness)-a
        sorted_diffs_neglecting_sign = sorted(diffs, key=lambda diff: abs(diff))
        proxies_graspable_angles_degrees.append(a+sorted_diffs_neglecting_sign[0])

    # Want to ONLY GRASP AT GRASPABLE ANGLES but want to do so at angles from this set where the object width is the SMALLEST
    # Being at 20 degrees interval, it does not affect variety of angles used 
    # (compared to a more fine-grained method where say the narrowest angles turn out to be [17,18,19...])
    
    #### Will use angles sorted by thickness to meta-sort proxies_graspable_angles_degrees
    
    # Python program to meta-sort a second list using the sort pattern on a base list
    def sort_list(list_to_meta_sort, base_list_to_sort): 
        zipped_pairs = zip(base_list_to_sort, list_to_meta_sort) 
        z = [x for _, x in sorted(zipped_pairs)] 
        return z 

    # Where are the proxy graspable angles in terms of rank in sorted thickness angles?
    indices = [list(angles_degrees_sorted_by_thickness).index(gad) for gad in proxies_graspable_angles_degrees]

    # Lower index means THINNER grasp so sort indices in ascending order and sort graspable angles accordingly
    # Use the fact that graspable_angles_degrees and pre_proxies_graspable_angles_degrees and proxies_graspable_angles_degrees ALL agree on INDICES
    # Grasps actually done at original angles from graspable_angles_degrees
    # Not using values from pre_proxies_graspable_angles_degrees allows us to expose BOTH fingers to all sides of object
    graspable_angles_degrees = sort_list(list_to_meta_sort=graspable_angles_degrees, base_list_to_sort=indices)

    # Retain the top 'NUM_GRASPS_PER_OBJECT' thinnest grasps
    return graspable_angles_degrees[:NUM_GRASPS_PER_OBJECT]

    # Example:

    # graspable_angles_degrees = [ 80 100 120 260 280 300]

    # angles_degrees_sorted_by_thickness = [ 72.  73.  74.  75.  76.  77.  78.  79.  80.  81.  82.  83.  84.  85.
    #   86.  87.  88.  89.  90.  91.  92.  93.  94.  95.  96.  97.  98.  99.
    #  100. 101. 102. 103. 104. 173. 174. 175. 176.  64.  65.  66.  67.  68.
    #   69.  70.  71. 109. 110. 111.  54.  55.  56. 105. 106. 107. 108.  57.
    #   58.  59.  60.  61.  62.  63. 117. 118. 119. 120.  46.  47.  48.  49.
    #   50.  51.  52.  53. 127. 128. 112. 113. 114. 115. 116. 121. 122. 123.
    #  124. 125. 126. 129. 130. 131. 132. 133. 134. 135. 169.  40.  41.  42.
    #   43.  44.  45.  30.  31.  32.  33. 136. 137. 138. 139. 140.  34.  35.
    #   36.  37.  38.  39. 170. 171.  27.  28.  29. 141. 142. 143. 144. 172.
    #   24.  25.  26. 145. 146. 147. 158. 148. 149. 150.  22.  23. 165. 166.
    #  167. 159. 160. 151. 152. 153. 154. 155.  20.  21. 161. 162. 156. 157.
    #   19. 168. 163. 164.  13.  14.  15.  16.  17.  18.   4.   5.   6.   7.
    #    8.   9.  10.  11.  12.   0.   1.   2.   3. 177. 178. 179. 180.]

    # pre_proxies_graspable_angles_degrees = [80, 100, 120, 80, 100, 120]
    # proxies_graspable_angles_degrees = [80.0, 100.0, 120.0, 80.0, 100.0, 120.0]

    # return [80, 260, 100, 280, 120, 300]
