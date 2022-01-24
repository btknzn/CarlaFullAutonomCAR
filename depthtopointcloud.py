from carla.image_converter import depth_to_array
import math
import numpy
from numpy.matlib import repmat


def depthtopointcloud(image, color=None, max_depth=0.9):
    """
    Convert an image containing CARLA encoded depth-map to a 2D array containing
    the 3D position (relative to the camera) of each pixel and its corresponding
    RGB color of an array.
    "max_depth" is used to omit the points that are far enough.
    """
    far = 1000.0  # max depth in meters.
    normalized_depth = depth_to_array(image)

    # (Intrinsic) K Matrix
    k = numpy.identity(3)
    k[0, 2] = image.width / 2.0
    k[1, 2] = image.height / 2.0
    k[0, 0] = k[1, 1] = image.width / \
        (2.0 * math.tan(image.fov * math.pi / 360.0))

    # 2d pixel coordinates
    pixel_length = image.width * image.height
    u_coord = repmat(numpy.r_[image.width-1:-1:-1],
                     image.height, 1).reshape(pixel_length)
    v_coord = repmat(numpy.c_[image.height-1:-1:-1],
                     1, image.width).reshape(pixel_length)
    if color is not None:
        color = color.reshape(pixel_length, 3)
    normalized_depth = numpy.reshape(normalized_depth, pixel_length)

    # Search for pixels where the depth is greater than max_depth to
    # delete them
    max_depth_indexes = numpy.where(normalized_depth > max_depth)
    #normalized_depth = numpy.delete(normalized_depth, max_depth_indexes)
    #u_coord = numpy.delete(u_coord, max_depth_indexes)
    #v_coord = numpy.delete(v_coord, max_depth_indexes)



    # pd2 = [u,v,1]
    p2d = numpy.array([u_coord, v_coord, numpy.ones_like(u_coord)])

    # P = [X,Y,Z]
    p3d = numpy.dot(numpy.linalg.inv(k), p2d)
    p3d *= normalized_depth * far

    return p3d