import glob
import os
import cv2
import numpy as np
import argparse


def main(input_path, output_path):
    img_names = glob.glob(os.path.join(input_path, "*"))
    img_sample = cv2.imread(img_names[0])
    gray = cv2.cvtColor(img_sample, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 150, 255, cv2.CV_8UC1)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    largest_contour = contours[0]
    for con in contours:
        if cv2.contourArea(con) > cv2.contourArea(largest_contour):
            largest_contour = con
    center, radius = cv2.minEnclosingCircle(largest_contour)
    center = tuple(np.asarray(center).astype(np.int32))
    radius = np.around(radius, 0).astype(np.int32)
    output_dir = output_path
    os.makedirs(output_dir, exist_ok=True)
    for img_n in img_names:
        img = cv2.imread(img_n)
        img_cropped = img[center[1]-radius:center[1] +
                          radius, center[0]-radius:center[0]+radius]
        filepath = os.path.relpath(os.path.join(
            output_dir, os.path.basename(img_n)))
        cv2.imwrite(filepath, img_cropped)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--src", "-s", "--source",
                        type=str,
                        default="../input",
                        help="image source path")
    parser.add_argument("--out", "-o", "--output",
                        type=str,
                        default="../input_cropped",
                        help="output path")
    opts = parser.parse_args()
    main(opts.src, opts.out)
