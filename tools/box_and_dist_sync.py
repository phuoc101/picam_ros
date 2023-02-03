from glob import glob
import os
import os.path as osp
import argparse
from loguru import logger


class BoxDistSyncer(object):
    def __init__(self, opts):
        self.box_label_files = sorted(glob(osp.join(opts.box_labels, "*.txt")))
        self.tracker_label_files = sorted(glob(osp.join(opts.tracker_labels, "*.txt")))
        self.synced_labels = osp.abspath(
            osp.join(opts.tracker_labels, "..", "box_dist_labels")
        )
        os.makedirs(self.synced_labels, exist_ok=True)
        self.pairs = dict()
        for pair in opts.pairs:
            self.pairs[pair[0]] = pair[1]
        self.box_labels = []
        self.tracker_labels = []
        for box_lbl, track_lbl in zip(self.box_label_files, self.tracker_label_files):
            self.box_labels.append(osp.abspath(box_lbl))
            self.tracker_labels.append(osp.abspath(track_lbl))
        logger.debug(f"Box Labels: {len(self.box_labels)}")
        logger.debug(f"Tracker Labels {len(self.tracker_labels)}")

    def sync(self):
        for box_file, tracker_file in zip(self.box_labels, self.tracker_labels):
            box_lines = []
            tracker_lines = dict()
            with open(box_file, "r") as fb:
                for line in fb.readlines():
                    box_lines.append(line.strip().split(","))
                fb.close()
            with open(tracker_file, "r") as ft:
                for line in ft.readlines():
                    line = line.replace("\n", "").split(" ")
                    tracker_lines[line[0]] = line[1]
                ft.close()
            # Format: [track_id, class_id, x, y, w, h, dist]
            anno = []
            for topic in self.pairs:
                for line in box_lines:
                    frameid, track_id, class_id, x, y, w, h, _, _, _, _ = line
                    if track_id == self.pairs[topic]:
                        anno.append(
                            "{},{},{},{},{},{},{}\n".format(
                                track_id, class_id, x, y, w, h, tracker_lines[topic]
                            )
                        )
            with open(
                osp.join(self.synced_labels, osp.basename(box_file)), "w"
            ) as f:
                f.writelines(anno)


def main(opts):
    box_dist_syncer = BoxDistSyncer(opts)
    box_dist_syncer.sync()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--box-labels",
        default="../extracted_data/Tierankatu_nodrone/labels/box_labels",
        help="Labels for bounding boxes",
    )
    parser.add_argument(
        "--tracker-labels",
        default="../extracted_data/Tierankatu_nodrone/labels/tracker_labels",
        help="Labels for distance to picam",
    )
    parser.add_argument(
        "--pairs",
        action="append",
        nargs="+",
        help="Pairs of optitrack dist and track ID",
    )
    opts = parser.parse_args()
    logger.info(opts)
    main(opts)
