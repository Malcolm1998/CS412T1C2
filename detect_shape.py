''' TAKEN FROM:
https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/comp2/shape_detect.py
Github:
https://github.com/nwoeanhinnogaehr/412-W19-G5-public
'''
import cv2
import imutils
import numpy as np

def classify(c):
    # initialize the shape name and approximate the contour
    shape = "unidentified"
    # cv.ArcLength(curve, slice=CV_WHOLE_SEQ, isClosed=-1)
    # arcLength Parameters:
    # curve – Input vector of 2D points, stored in std::vector or Mat.
    # closed – Flag indicating whether the curve is closed or not.
    # The function computes a curve length or a closed contour perimeter
    peri = cv2.arcLength(c, True)
    # cv2.approxPolyDP(curve, epsilon, closed[, approxCurve])
    # approxPolyDP Parameters:
    # curve -Input vector of a 2D point stored in Nx2 numpy array
    # approxCurve – Result of the approximation. The type should match the type of the input curve.
    # epsilon – Parameter specifying the approximation accuracy. This is the maximum distance between the original curve and its approximation.
    # closed – If true, the approximated curve is closed (its first and last vertices are connected). Otherwise, it is not closed.
    # The function returns the result of the approximation
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
    # if the shape is a triangle, it will have 3 vertices
    if len(approx) == 3:
        shape = "triangle"

    # if the shape has 4 vertices, it is either a square or
    # a rectangle
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


# color should be "red" or "green"
# cutoff is # of pixels in region for it to be considered a shape
# returns: array of names of detected shapes, either
#   "triangle", "square" or "circle"
def detect(image, color, cutoff=7000):
    if color == "green":
        lower = np.array([50, 50, 0]) # HSV
        upper = np.array([100, 1000, 1000])
        mask = cv2.inRange(image, lower, upper)
    elif color == "red":
        lower = np.array([0, 150, 0]) # HSV
        upper = np.array([20, 258, 258])
        lower2 = np.array([160, 150, 0]) # HSV
        upper2 = np.array([258, 258, 258])
        mask1 = cv2.inRange(image, lower, upper)
        mask2 = cv2.inRange(image, lower2, upper2)
        mask = mask1 | mask2
    else:
        raise "Invalid color"
    resized = mask
    #resized = imutils.resize(image, width=300)
    ratio = image.shape[0] / float(resized.shape[0])

    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    #gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(resized, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    shapes = []

    # loop over the contours
    for c in cnts:
        # compute the center of the contour, then classify the name of the
        # shape using only the contour
        M = cv2.moments(c)
        if M["m00"] > cutoff:
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            shape = classify(c)

            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            #cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            #cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            shapes.append(shape)

    # show the output image
    #cv2.imshow(color, thresh)

    return shapes
