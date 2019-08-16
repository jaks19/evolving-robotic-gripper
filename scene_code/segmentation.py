import numpy as np
import math as m
from PIL import Image
import matplotlib
from matplotlib import pyplot as plt

from glo import *

def distance(a, b):
    return m.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

def get_pixel_generators_x_y_theta():
    
    def convert_theta_to_eqn():
        def f(x, theta): return (m.sin(m.radians(theta)) / m.cos(m.radians(theta)))*x
        def g(y, theta): return y/(m.sin(m.radians(theta)) / m.cos(m.radians(theta)))
        return f, g
    
    f, g = convert_theta_to_eqn()
    def F(x, theta): return int(f(x, theta))
    def G(y, theta): return int(g(y, theta))
    return F, G

def get_pixel_generator_for_all_thetas_from_image(image_array):
    assert(image_array.shape[0]%2==1 and image_array.shape[1]%2==1)
    
    # So far work with origin at center of image, although in the end, in numpy array,
    # origin will be the top left pixel
    x_lim = [-int((image_array.shape[0]-1)/2),int(image_array.shape[0]/2)]
    y_lim = [-int((image_array.shape[1]-1)/2),int(image_array.shape[1]/2)]
    
    def get_pixels_angle_theta(theta, x_lim, y_lim):
        y_from_x, _ = get_pixel_generators_x_y_theta()
        pixels = [ [x, y_from_x(x, theta)] for x in np.arange(x_lim[0], x_lim[1], 1) ]
        return [ p for p in pixels if p[1]>=y_lim[0] and p[1]<=y_lim[1] ]
    
    return lambda theta: get_pixels_angle_theta(theta, x_lim, y_lim)

def move_center_to_top_left(pixels_center_middle, image):
    mid_pt_distances = [int((image.shape[0]-1)/2), int((image.shape[1]-1)/2)]
    
    ps = pixels_center_middle.copy()
    for i in range(len(ps)): ps[i]=[ps[i][0]+mid_pt_distances[0], ps[i][1]+mid_pt_distances[1]]
    return ps

def segment_array_to_rgb(segment, id_of_interest=2):
    l, w = segment.shape
    rgb = np.zeros((l, w, 3))

    wh = np.argwhere(segment==id_of_interest)
    rows = wh[:,0]
    cols = wh[:,1]
    rgb[rows, cols]=[255, 255, 255]
    return rgb

def get_radial_pixels(segment, delta_deg, id_of_interest):
    rgb = segment_array_to_rgb(segment, id_of_interest)
    pixels_generator_all_thetas = get_pixel_generator_for_all_thetas_from_image(rgb)
    
    pixels_radial = {}

    for i in range(int(180/delta_deg)+1):
        # Origin at pybullet center (center of image)
        pixels = pixels_generator_all_thetas(i*delta_deg)
        # Origin at top left corner
        pixels_radial[i*delta_deg] = move_center_to_top_left(pixels, rgb)
    
    return pixels_radial

def get_radial_start_and_end_points_for_object(radial_pixels_dict, segment):
    radial_endpts = {}
    
    for angle in radial_pixels_dict.keys():

        pixels_this_angle = radial_pixels_dict[angle]
        endpts = [None, None]

        # Sort in x first, then y
        pixels_this_angle = sorted(pixels_this_angle, key=lambda x: x[0])
        pixels_this_angle = sorted(pixels_this_angle, key=lambda x: x[1])

        # Move in order and stop once body is seen (note coords)
        # Do same from other direction note where body ends (note coords)
        # Note that this fails to get narrower gripes e.g. on inner part of steering wheel (will always get outer)

        for pixel in pixels_this_angle:            
            if segment[pixel[0], pixel[1]]==2:
                endpts[0] = [pixel[0], pixel[1]]
                break
                
        for pixel in reversed(pixels_this_angle):            
            if segment[pixel[0], pixel[1]]==2:
                endpts[1] = [pixel[0], pixel[1]]
                break

        radial_endpts[angle] = endpts

    return radial_endpts

# Angles are measured from vertical and positive in anti-clockwise direction, when viewed from default camera
# If want to see gripper horizontal, use -90 degrees angle correction to the gripping angles
# Correction not applied here, will be applied at the end of the pipeline, wherever angles are used
def convert_segment_array_to_grasp_angles_sorted_by_thickness(segment, delta_deg, id_of_interest, angle_deg_min=0, angle_deg_max=360):
    # For debugging: prints the exact pixels on the angles 0, 75 and 135
    # for a in [0,75,135]: debug_segment_at_angle(segment, a, name=f'./{DEBUG_PLOTS_FOLDER}/debug_angle_{a}_deg.png')

    radial_pixels_dict = get_radial_pixels(segment=segment, delta_deg=delta_deg, id_of_interest=id_of_interest)
    for k in list(radial_pixels_dict.keys()): 
        if k < angle_deg_min or k > angle_deg_max: del radial_pixels_dict[k]

    radial_endpts = get_radial_start_and_end_points_for_object(radial_pixels_dict, segment)

    distances = {}
    for k in radial_endpts.keys(): 
        if radial_endpts[k][0] is not None and radial_endpts[k][1] is not None:
            distances[k] = distance(radial_endpts[k][0], radial_endpts[k][1])
    
    angles_sorted = sorted(distances, key=lambda k: distances[k])
    return angles_sorted

def take_segmentation_image_and_get_grasp_angles(sim, object_urdf_path_tmp):
    sim.reset_state(include=INCLUDE_ALL, object_urdf_path=object_urdf_path_tmp)
    _, _, _, _, seg = sim.p.getCameraImage(width=1001, height=1001, shadow=0)

    angles_degrees = convert_segment_array_to_grasp_angles_sorted_by_thickness(segment=seg, delta_deg=1, id_of_interest=sim.arm_id)
    return seg, angles_degrees

#### Debug
def display_array(arr, savename):
    from scipy.misc import imsave
    imsave(f'{savename}', arr)
    return

def get_pixels_at_specific_angle(segment, theta):
    rgb = segment_array_to_rgb(segment, 2)
    pixels_generator_all_thetas = get_pixel_generator_for_all_thetas_from_image(segment)

    # Origin at pybullet center (center of image)
    pixels = pixels_generator_all_thetas(theta)
    # Origin at top left corner
    pixels = move_center_to_top_left(pixels, rgb)
    return pixels

def make_specific_pixels_specific_color(all_pixels_np_arr, specific_pixels_list, color_rgb_np_arr):
    for x, y in specific_pixels_list:
        all_pixels_np_arr[x,y]=color_rgb_np_arr
    return all_pixels_np_arr

def debug_segment_at_angle(segment, angle, name=None):
    pixels = get_pixels_at_specific_angle(segment, angle)
    rgb = segment_array_to_rgb(segment, id_of_interest=2)
    rgb = make_specific_pixels_specific_color(rgb, pixels, np.array([255,0,0]))

    name = name if name is not None else f'./{DEBUG_PLOTS_FOLDER}/{angle}.png'
    display_array(rgb, name)
    return