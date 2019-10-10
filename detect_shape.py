#!/usr/bin/env python
''' TAKEN FROM:
https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/comp2/shape_detect.py
Github:
https://github.com/nwoeanhinnogaehr/412-W19-G5-public
'''
import cv2
import imutils
import numpy as np
from enum import Enum


class Shapes(Enum):
    unknown = -1
    triangle = 3
    square = 4
    pentagon = 5
    circle = 9


def classify(c):
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
    # if the shape is a triangle, it will have 3 vertices
    if len(approx) == 3:
        shape = "triangle"

    elif len(approx) == 4:
        # compute the bounding box of the contour and use the
        # bounding box to compute the aspect ratio
        (x, y, w, h) = cv2.boundingRect(approx)
        ar = w / float(h)

        # a square will have an aspect ratio that is approximately
        # equal to one, otherwise, the shape is a rectangle
        #shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
        shape = "square"

    # otherwise, we assume the shape is a circle
    else:
        shape = "circle"

    # return the name of the shape
    return shape


def count_objects(mask, threshold=1000, canvas=None):
    """Count the number of distinct objects in the boolean image."""
    _, contours, _ = cv2.findContours(mask, 1, 2)
    moments = [cv2.moments(cont) for cont in contours]
    big_moments = [m for m in moments if m["m00"] > threshold]
    if canvas != None:
        for moment in big_moments:
            cx = int(moment["m10"] / moment["m00"])
            cy = int(moment["m01"] / moment["m00"])
            cv2.circle(canvas, (cx, cy), 20, (0, 0, 255), -1)
    return len(big_moments)


def detect_shape(mask, canvas=None, threshold=100):
    """Detect a shape contained in an image.

    Adappted from: https://stackoverflow.com/questions/11424002/how-to-detect-simple-geometric-shapes-using-opencv
    """
    detected_shapes = []
    moments = []
    _, contours, _ = cv2.findContours(mask, 1, 2)
    for cnt in contours:
        if cv2.moments(cnt)["m00"] > threshold:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            if len(approx) == 3:
                if canvas != None:
                    cv2.drawContours(canvas, [cnt], 0, (0, 255, 0), -1)
                detected_shapes.append(Shapes.triangle)
                moments.append(cv2.moments(cnt))

            elif len(approx) == 4:
                if canvas != None:
                    cv2.drawContours(canvas, [cnt], 0, (0, 0, 255), -1)
                detected_shapes.append(Shapes.square)
                moments.append(cv2.moments(cnt))
            elif len(approx) == 5:
                if canvas != None:
                    cv2.drawContours(canvas, [cnt], 0, 255, -1)
                detected_shapes.append(Shapes.pentagon)
                moments.append(cv2.moments(cnt))
            elif len(approx) > 9:
                if canvas != None:
                    cv2.drawContours(canvas, [cnt], 0, (0, 255, 255), -1)
                detected_shapes.append(Shapes.circle)
                moments.append(cv2.moments(cnt))
            else:
                detected_shapes.append(Shapes.unknown)
                moments.append(cv2.moments(cnt))
    return detected_shapes, moments