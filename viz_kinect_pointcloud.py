#!/usr/bin/python
import os
import numpy as np
import argparse
from PIL import Image

# 3d pointcloud viewer
import pptk


"""
author:songjiaxina@gmail.com
data:2019/04/28
visualize pointcloud from kinect depth image and rgb using pptk
"""

# camera intrinsics.  modify them
fx = 520.9
cx = 325.1
fy = 521.0
cy = 249.7
scale = 5000.0


def generate_pointcloud(depth_image, rgb_image=None):

    res = np.array([[0, 0, 0]])
    print(np.shape(res))
    n_count_valid = np.count_nonzero(np.array(depth_image))
    print("valid depth point numbers is: " + str(n_count_valid))
    res = np.zeros((n_count_valid, 3))

    if rgb_image is not None:
        rgb_array = np.zeros((n_count_valid, 3))

    i = 0
    for x in range(0, depth_image.width):
        for y in range(0, depth_image.height):
            # fecth the real depth
            d = depth_image.getpixel((x, y))
            if d == 0:
                continue
            else:
                # print("current depth is: " + str(d))
                d = d / scale
                # if d > 5.0:
                #     continue
                X = ((float(x) - cx) / fx) * d
                Y = ((float(y) - cy) / fy) * d
                row = np.array([X, Y, d])
                res[i] = row
                if rgb_image is not None:
                    rgb_value = rgb_image.getpixel((x, y))
                    R = rgb_value[0] / 255.0
                    G = rgb_value[1] / 255.0
                    B = rgb_value[2] / 255.0
                    rgb_array[i] = np.array([R, G, B])

                i += 1

    if rgb_image is not None:
        return res, rgb_array
    else:
        return res



if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--depth", required=True, type=str,
                        help="Necessary! provide the depth image file!!")
    parser.add_argument(
        "--rgb", type=str, help="Optional!! provide the rgb image. if not given the pointcloud is single color")
    parser.add_argument("--save", type=str,
                        help="Optional!! save the pointcloud as a ply file")

    args = parser.parse_args()

    have_rgb = False
    if args.rgb:
        print("have rgb image")
        if not os.path.exists(args.rgb):
            print("warning!! drgb image file not exits!! it will run in no rgb mode")
        else:
            have_rgb = True
            print("detected valid rgb image!! it will run in rgb mode")

    if not os.path.exists(args.depth):
        print("fatal error!! depth image file not exits!! make sure the path is correct!!")
        exit(-1)

    print("processing depth image: : " + args.depth + "\nPlease waiting~~~~~~~")
    depth = Image.open(args.depth)

    point_cloud = None
    if have_rgb:
        rgb = Image.open(args.rgb)
        point_cloud, rgb_array = generate_pointcloud(depth, rgb)
    else:
        point_cloud = generate_pointcloud(depth)

    print("processing finished!!")

    if have_rgb:
        v = pptk.viewer(point_cloud, rgb_array)
    else:
        v = pptk.viewer(point_cloud)

    if have_rgb:
        rgb.show()
