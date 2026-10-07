"""Rule-based pest-sign detector: colour and shape rules, tuned on the simulated crawlspace.

It finds three kinds of pest sign in a camera frame:

  termite_mud_tube   thin, vertical, mud-brown lines on walls and piers
  rodent_droppings   a cluster of small near-black specks on the ground
  rodent_burrow      a near-black elliptical hole in the ground

It's deliberately simple. Anything with the same detect(bgr_image) -> [Detection]
interface can replace it, e.g. a trained model, without changing the rest of the
pipeline. Works with OpenCV 3 (Melodic) and 4.
"""
from collections import namedtuple

import cv2
import numpy as np

# label: one of LABELS; score: confidence 0-1; box: (x, y, w, h) pixels;
# point: (u, v) pixel to locate the sign in 3D (on its surface)
Detection = namedtuple("Detection", "label score box point")

LABELS = ("termite_mud_tube", "rodent_droppings", "rodent_burrow")


def _contours(mask):
    return cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  # OpenCV 3 and 4


class RuleBasedDetector(object):
    def __init__(self, dark_value=18, tube_contrast=28, tube_saturation=100, tube_min_height=40,
                 speck_link_px=40, min_specks=4):
        self.dark_value = dark_value  # 0-255: holes and droppings are darker than this; damp dirt isn't
        self.tube_contrast = tube_contrast  # how much darker than the surface either side a tube must be
        self.tube_saturation = tube_saturation  # mud is saturated brown; grey edges and steel aren't
        self.tube_min_height = tube_min_height  # pixels
        self.speck_link_px = speck_link_px  # specks closer than this belong to one cluster
        self.min_specks = min_specks

    def detect(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        hue, sat, val = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        return self._mud_tubes(hue, sat, val) + self._ground_marks(sat, val)

    def _mud_tubes(self, hue, sat, val):
        # Black-hat along each row: how much darker a pixel is than the surface within ~12 px
        # either side. Thin dark lines stand out on bright and dim walls alike, while wide
        # dark areas (the dirt floor) and step edges (pier corners) don't.
        contrast = cv2.morphologyEx(val, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (25, 1)))
        mud = (sat > self.tube_saturation) & (hue >= 5) & (hue <= 30)  # orange-brown, not grey or pink
        mask = ((contrast > self.tube_contrast) & mud).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 9)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (1, 20)))  # tall only
        found = []
        for c in _contours(mask):
            x, y, w, h = cv2.boundingRect(c)
            if h >= self.tube_min_height and h >= 4 * w:
                found.append(Detection("termite_mud_tube", min(1.0, h / 150.0), (x, y, w, h), (x + w // 2, y + h // 2)))
        return found

    def _ground_marks(self, sat, val):
        dark = (val < self.dark_value).astype(np.uint8) * 255
        rows, cols = val.shape
        found, specks = [], []
        for c in _contours(dark):
            area = cv2.contourArea(c)
            x, y, w, h = cv2.boundingRect(c)
            if x == 0 or y == 0 or x + w == cols or y + h == rows:
                continue  # cut off by the frame edge: can't judge its shape
            fill = area / max(float(w * h), 1.0)
            # A hole seen at a shallow angle is an ellipse 2-7x wider than tall that fills
            # roughly half to 80% of its box; the access opening is a rectangle (~100%).
            if 50 <= area <= 0.08 * rows * cols and w >= 20 and 2 * h <= w <= 7 * h and 0.45 <= fill <= 0.9:
                found.append(Detection("rodent_burrow", round(min(1.0, w / 50.0), 2), (x, y, w, h),
                                       (x + w // 2, y + h // 2)))
            elif w <= 16 and h <= 10 and self._on_dirt(sat, val, x + w // 2, y + h // 2, max(w, h) // 2 + 5):
                specks.append((x + w / 2.0, y + h / 2.0, x, y, w, h))
        burrows = [d.box for d in found]
        for cluster in self._clusters(specks):
            cx, cy = np.mean([s[0] for s in cluster]), np.mean([s[1] for s in cluster])
            on_burrow = any(bx - bw * 0.15 <= cx <= bx + bw * 1.15 and by - bh * 0.5 <= cy <= by + bh * 1.5
                            for bx, by, bw, bh in burrows)  # fragments around a hole's edge
            if len(cluster) >= self.min_specks and not on_burrow:
                xs = [s[2] for s in cluster] + [s[2] + s[4] for s in cluster]
                ys = [s[3] for s in cluster] + [s[3] + s[5] for s in cluster]
                box = (min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))
                point = (int(np.mean([s[0] for s in cluster])), int(np.mean([s[1] for s in cluster])))
                found.append(Detection("rodent_droppings", min(1.0, len(cluster) / 8.0), box, point))
        return found

    def _on_dirt(self, sat, val, cx, cy, d):
        """Is the speck at (cx, cy) a dark mark on dirt? At least 3 of the 4 points d pixels
        away must be saturated brown, and clearly brighter than the speck (by 20+). This
        rejects specks along the line where the dirt meets a pale wall, and dark pixels in
        the very damp, dark dirt at the foot of a wall."""
        rows, cols = val.shape
        ring = [(sat[py, px], val[py, px]) for px, py in ((cx, cy - d), (cx, cy + d), (cx - d, cy), (cx + d, cy))
                if 0 <= px < cols and 0 <= py < rows]
        dirt = [v for s, v in ring if s > 90 and v < 170]
        return len(dirt) >= 3 and np.median(dirt) >= val[cy, cx] + 20

    def _clusters(self, specks):
        """Group specks that chain together within speck_link_px of each other."""
        groups, unseen = [], list(specks)
        while unseen:
            group, frontier = [], [unseen.pop()]
            while frontier:
                s = frontier.pop()
                group.append(s)
                near = [t for t in unseen if (t[0] - s[0]) ** 2 + (t[1] - s[1]) ** 2 <= self.speck_link_px ** 2]
                for t in near:
                    unseen.remove(t)
                frontier += near
            groups.append(group)
        return groups
