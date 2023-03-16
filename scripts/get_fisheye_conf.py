import cv2
import argparse
import yaml


def fisheye_config(img_sample, yaml_file):
    """
    Get fisheye circle radius and center (with GUI)
    """
    width, height = img_sample.shape[1], img_sample.shape[0]
    WINDOW_NAME = "image"
    scale = 1 if width <= 1000 else 2
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, width // scale + 100, height // scale + 100)
    N = 2 * max(img_sample.shape)
    cv2.createTrackbar("radius", WINDOW_NAME, 0, N, on_trackbar)
    cv2.createTrackbar("Cx", WINDOW_NAME, N // 2, N, on_trackbar)
    cv2.createTrackbar("Cy", WINDOW_NAME, N // 2, N, on_trackbar)

    radius = 0
    cx, cy = (0, 0)
    while True:
        if True:
            if scale != 1:
                frame = cv2.resize(img_sample, (width // scale, height // scale), cv2.INTER_AREA)
            else:
                frame = img_sample.copy()
            radius = cv2.getTrackbarPos("radius", WINDOW_NAME)
            cx = cv2.getTrackbarPos("Cx", WINDOW_NAME)
            cy = cv2.getTrackbarPos("Cy", WINDOW_NAME)
            frame = cv2.circle(frame, (cx // scale, cy // scale), radius // scale, (0, 200, 0), 2)
            cv2.imshow(WINDOW_NAME, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()
    circle_info = {"center": {"cx": cx, "cy": cy}, "radius": radius}
    with open(yaml_file, "w+") as f:
        yaml.dump(circle_info, f)
        f.close()


def on_trackbar(x):
    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--img", "-i", default="test.png")
    parser.add_argument("--yaml", "-y", default="default.yaml")
    opts = parser.parse_args()
    img = cv2.imread(opts.img)
    fisheye_config(img, opts.yaml)


if __name__ == "__main__":
    main()
