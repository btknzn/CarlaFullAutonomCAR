import open3d as o3d
import numpy as np
import numpy
import math
import numpy
from numpy.matlib import repmat

import open3d as o3d
import numpy as np
import numpy
import math
import numpy
from numpy.matlib import repmat

from skimage.measure import label
import skimage
from carla.depthtopointcloud import depthtopointcloud
from carla import image_converter

pedesterian = [220, 20, 60]
vehicle =   [[0, 0, 255]]
width = 180
height = 320

def object_detection (image_depth,image_segment):

    point_cloud = depthtopointcloud(image_depth)
    segmented_image = image_converter.labels_to_cityscapes_palette(image_segment)

    vehicle3d= segmented_image == vehicle
    pedesterian3d= segmented_image == pedesterian 
    vehicle_det = (vehicle3d[:,:,0]*vehicle3d[:,:,1]*vehicle3d[:,:,2])*255
    pedesterian_det = (pedesterian3d[:,:,0]*pedesterian3d[:,:,1]*pedesterian3d[:,:,2])*255
    label_vehicle = label(vehicle_det, connectivity = vehicle_det.ndim)
    props_vehicle = skimage.measure.regionprops(label_vehicle, intensity_image = None, cache = True)
    vehicle_cen = np.zeros([len(props_vehicle),2])
    for i in range(len(props_vehicle)):
        vehicle_cen[i,:] = np.round(props_vehicle[i].centroid, 0)
    label_pedesterian = label(pedesterian_det, connectivity = pedesterian_det.ndim)
    props_pedesterian = skimage.measure.regionprops(label_pedesterian, intensity_image = None, cache = True)
    Pedestrian_cen = np.zeros([len(props_pedesterian),2])
    for i in range(len(props_pedesterian)):
        Pedestrian_cen[i,:] = np.round(props_pedesterian[i].centroid, 0)

    objects = []

    ##for i in  Pedestrian_cen:
        ##location = i[1]+ i[0]*320
        ##objects.append(point_cloud[0,int(location)])

    for i in  vehicle_cen:
        location = i[1] + i[0]*320
        objects.append(point_cloud[:int(location)])
    
    print(objects)
    print("------------------------------\n!")
    return objects