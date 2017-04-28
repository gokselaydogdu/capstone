# 28.04.2017 last change


import cv2
import numpy
from pyfirmata import Arduino, util
import time
import math

board = Arduino('/dev/ttyUSB0')

servo = board.get_pin('d:9:s')
forward = board.get_pin('d:11:p')
servo.write(42.5)

# Global parameters

right_m, left_m = 0, 0
left_b, right_b = 0, 0

right_x1, right_x2 = 0, 0
left_x1, left_x2 = 0, 0
y1, y2 = 0, 0

SERVO_CONSTANT = 42.5

# Gaussian smoothing
kernel_size = 3
# Canny Edge Detector
low_threshold = 50
high_threshold = 150

# Region-of-interest vertices
# We want a trapezoid shape, with bottom edge at the bottom of the image
trap_bottom_width = 0.95  # width of bottom edge of trapezoid, expressed as percentage of image width
trap_top_width = 0.4  # ditto for top edge of trapezoid
trap_height = 0.4  # height of the trapezoid expressed as percentage of image height

# Hough Transform
rho = 1  # distance resolution in pixels of the Hough grid
theta = 1 * numpy.pi / 180  # angular resolution in radians of the Hough grid
threshold = 5  # minimum number of votes (intersections in Hough grid cell)
min_line_length = 1  # minimum number of pixels making up a line
max_line_gap = 1  # maximum gap in pixels between connectable line segments


forward.write(0.47)


def grayscale(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def canny(img, low_threshold, high_threshold):
    return cv2.Canny(img, low_threshold, high_threshold)


def gaussian_blur(img, kernel_size):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


def region_of_interest(img, vertices):
    mask = numpy.zeros_like(img)

    # defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    # filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, vertices, ignore_mask_color)

    # returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def draw_lines(img, lines, color=[255, 255, 0], thickness=3):
    """
    NOTE: this is the function you might want to use as a starting point once you want to
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).

    Think about things like separating line segments by their
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of
    the lines and extrapolate to the top and bottom of the lane.

    This function draws `lines` with `color` and `thickness`.
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
    global right_m, left_m, left_m, left_b, right_b
    global right_x1, right_x2
    global left_x1, left_x2
    global y1, y2

    # In case of error, don't draw the line(s)
    if lines is None:
        return
    if len(lines) == 0:
        return

    # Find slopes of all lines
    # But only care about lines where abs(slope) > slope_threshold
    slope_threshold = 0.1
    slopes = []
    new_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]  # line = [[x1, y1, x2, y2]]

        # Calculate slope
        if x2 - x1 == 0.:  # corner case, avoiding division by 0
            slope = 999.  # practically infinite slope
        else:
            slope = (y2 - y1) / (x2 - x1)

        # Filter lines based on slope
        if abs(slope) > slope_threshold:
            slopes.append(slope)
            new_lines.append(line)

    lines = new_lines

    # Split lines into right_lines and left_lines, representing the right and left lane lines
    # Right/left lane lines must have positive/negative slope, and be on the right/left half of the image
    right_lines = []
    left_lines = []
    for i, line in enumerate(lines):
        x1, y1, x2, y2 = line[0]
        img_x_center = img.shape[1] / 2  # x coordinate of center of image
        if slopes[i] > 0 and x1 > img_x_center and x2 > img_x_center:
            right_lines.append(line)
        elif slopes[i] < 0 and x1 < img_x_center and x2 < img_x_center:
            left_lines.append(line)

    # Run linear regression to find best fit line for right and left lane lines
    # Right lane lines
    right_lines_x = []
    right_lines_y = []

    for line in right_lines:
        x1, y1, x2, y2 = line[0]

        right_lines_x.append(x1)
        right_lines_x.append(x2)

        right_lines_y.append(y1)
        right_lines_y.append(y2)

    if len(right_lines_x) > 0:
        right_m, right_b = numpy.polyfit(right_lines_x, right_lines_y, 1)  # y = m*x + b
        draw_right = True
    else:
        right_x2, right_x1 = 0, 0
        draw_right = False

    # Left lane lines
    left_lines_x = []
    left_lines_y = []

    for line in left_lines:
        x1, y1, x2, y2 = line[0]

        left_lines_x.append(x1)
        left_lines_x.append(x2)

        left_lines_y.append(y1)
        left_lines_y.append(y2)

    if len(left_lines_x) > 0:
        left_m, left_b = numpy.polyfit(left_lines_x, left_lines_y, 1)  # y = m*x + b
        draw_left = True
    else:
        left_x2, left_x1 = 0, 0
        draw_left = False

    # Find 2 end points for right and left lines, used for drawing the line
    # y = m*x + b --> x = (y - b)/m
    y1 = img.shape[0] * (23 / 24)
    y2 = img.shape[0] * (19 / 24)

    if draw_right:
        right_x1 = (y1 - right_b) / right_m
        right_x2 = (y2 - right_b) / right_m
        cv2.line(img, (int(right_x1), int(y1)), (int(right_x2), int(y2)), color, thickness)

    if draw_left:
        left_x1 = (y1 - left_b) / left_m
        left_x2 = (y2 - left_b) / left_m
        cv2.line(img, (int(left_x1), int(y1)), (int(left_x2), int(y2)), color, thickness)


def weighted_img(img, initial_img, α=0.8, β=1., λ=0.):
    return cv2.addWeighted(initial_img, α, img, β, λ)


def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):

    lines = cv2.HoughLinesP(img, rho, theta, threshold, numpy.array([]), minLineLength=min_line_len,
                            maxLineGap=max_line_gap)
    line_img = numpy.zeros(
        (
            img.shape[0],
            img.shape[1],
            3
        ),
        dtype=numpy.uint8,
    )

    draw_lines(line_img, lines)
    return line_img


cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)
try:
    while cap.isOpened():

        ret, frame = cap.read()
        gray = grayscale(frame)
        blur_gray = gaussian_blur(gray, kernel_size)
        edges = canny(blur_gray, low_threshold, high_threshold)
        imshape = frame.shape
        vertices = numpy.array([[[40, 190], [40, 230], [280, 230], [280, 190], ]], dtype=numpy.int32)
        masked_edges = region_of_interest(edges, vertices)

        # Run Hough on edge detected image
        try:
            line_image = hough_lines(masked_edges, rho, theta, threshold, min_line_length, max_line_gap)
        except OverflowError:
            continue
        # Draw lane lines on the original image
        initial_image = frame.astype('uint8')
        annotated_image = weighted_img(line_image, initial_image)

        # print('right_m', right_m)
        # print('left_m', abs(left_m))

        # case of straight lane
        if (left_x2 and right_x2) > 0:

            length_road = 330
            middle_of_the_road = (right_x2 + left_x2) / 2
            middle_of_the_cam = frame.shape[1] / 2
            vehicle_offset = (middle_of_the_cam - middle_of_the_road)*(132/135)
            label_str = 'offset: %.1f mm' % vehicle_offset
            #result = cv2.putText(annotated_image, label_str, (30, 30), 0, 1, (0, 255, 0), 1, cv2.LINE_AA)

            #angle for our servo
            a = math.atan(vehicle_offset/ length_road)
            servo_angle = math.degrees(a)
            label_str2 = 'angle: %.1f degree' % servo_angle
            result = cv2.putText(annotated_image, label_str2, (30, 30), 0, 1, (0, 255, 0), 1, cv2.LINE_AA)
            print('angle_servo', servo_angle)

            if -10 < servo_angle < 10:
                servo.write(SERVO_CONSTANT - (1.3 * servo_angle))

        # turning right
        if right_x2 is 0:
            length_road = 330
            middle_of_the_cam = frame.shape[1] / 2
            distance_from_left_apex = left_x2 + (100 / 2)
            vehicle_offset = (distance_from_left_apex - middle_of_the_cam) * (132 / 135)
            # label_str = 'offset: %.1f mm' % vehicle_offset
            # result = cv2.putText(annotated_image, label_str, (30, 30), 0, 1, (0, 255, 0), 1, cv2.LINE_AA)

            a = math.atan(vehicle_offset / length_road)
            servo_angle = math.degrees(a)
            label_str2 = 'angle: %.1f degree' % servo_angle
            result = cv2.putText(annotated_image, label_str2, (30, 30), 0, 1, (0, 255, 0), 1, cv2.LINE_AA)
            print('angle_servo', servo_angle)

            if -10 < servo_angle < 10:
                servo.write(SERVO_CONSTANT + (2 * servo_angle))

        # turning left
        if left_x2 is 0:
            length_road = 330
            middle_of_the_cam = frame.shape[1] / 2
            distance_from_right_apex = right_x2 - (100 / 2)
            vehicle_offset = (distance_from_right_apex - middle_of_the_cam) * (132 / 135)

            a = math.atan(vehicle_offset / length_road)
            servo_angle = math.degrees(a)
            label_str2 = 'angle: %.1f degree' % servo_angle
            result = cv2.putText(annotated_image, label_str2, (30, 30), 0, 1, (0, 255, 0), 1, cv2.LINE_AA)
            print('angle_servo', servo_angle)

            if -10 < servo_angle < 10:
                servo.write(SERVO_CONSTANT + (2 * servo_angle))


        cv2.imshow('edges', masked_edges)
        cv2.imshow('img', annotated_image)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            forward.write(0)
            break

except (TypeError, UnboundLocalError, KeyboardInterrupt, ZeroDivisionError):
    raise


cap.release()
cv2.destroyAllWindows()
forward.write(0)
