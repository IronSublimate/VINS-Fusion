#!/usr/bin/env python3

import cv2
import numpy as np
import argparse
import os

DESC = "Show kitti stereo picture"


def main():
    parser = argparse.ArgumentParser(description=DESC)
    parser.add_argument("sequence_file", help="sequence file to KITTI dataset, e.g. "
                                              "'/kitti/raw/2011_09_30_drive_0028_sync'")

    args = parser.parse_args()

    folder_left = os.path.join(args.sequence_file, "image_02/data/")
    folder_right = os.path.join(args.sequence_file, "image_03/data/")

    cv2.namedWindow("image", cv2.WINDOW_NORMAL)
    t = 0

    for l_str, r_str in zip(os.listdir(folder_left), os.listdir(folder_right)):
        assert l_str == r_str, "left image name is not as same as right image"
        left_img = cv2.imread(os.path.join(folder_left, l_str))
        right_img = cv2.imread(os.path.join(folder_right, r_str))

        img = np.concatenate((left_img, right_img), axis=1)
        cv2.imshow("image", img)
        key = cv2.waitKey(t)
        if key == 27:
            break
        elif key == ord(' '):
            t = 1 - t


if __name__ == '__main__':
    main()
