#!/usr/bin/env python3

"""@package docstring
Loads in a .tflite model downloaded from colab and uses the model to read whatever clue is currently in the robots frame
In order to use this module in another python file just load the module (it will take some time to load) and call read_clueboard(cv_img) in order get clue answer
"""


import tensorflow as tf
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from pathlib import Path
import time

#define constants
uh = 130
us = 255
uv = 255
lh = 110
ls = 120
lv = 50
lvr = 30
lower_hsv = np.array([lh,ls,lv])
upper_hsv = np.array([uh,us,uv])
lower_hsv_rl = np.array([0, ls, lv])
upper_hsv_rl = np.array([30, us, uv])
lower_hsv_rh = np.array([330, ls, lv])
upper_hsv_rh = np.array([360, us, uv])
characters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"


def char_to_one_hot(char_of_interest):
  label_num = characters.find(char_of_interest)
  return np.eye(36)[label_num]

def one_hot_to_char(prediction):
  predicted_index = np.argmax(prediction, axis=0)
  return characters[predicted_index]

def find_bounding_box(labels, good_labels, index):
  component_mask = np.uint8(labels == good_labels[index][0]) * 255

  #Find 4 corners of clueboard
  contours, _ = cv2.findContours(component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

  if contours:
      cnt = contours[0]
      epsilon = 0.02 * cv2.arcLength(cnt, True)
      approx = cv2.approxPolyDP(cnt, epsilon, True)

      if len(approx) != 4:
          print("No bounding rectangle found, trying next contour")
          return None
      else:
          print("Rectangle corners found!")
          return approx

def extract_clue(frame):
  height, width = frame.shape[:2]

  # Threshold the HSV image to get only blue colors
  frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  mask = cv2.inRange(frame_hsv, lower_hsv, upper_hsv)
  inverted_mask = cv2.bitwise_not(mask)

  #find clueboard
  num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(inverted_mask)
  good_labels = []
  for i in range(1, num_labels):
          x, y, w, h, area = stats[i, :5]

          if width*height/1000 < area:
              y1, y2 = max(0, y), min(height, y+h)
              x1, x2 = max(0, x), min(width, x+w)

              if area < width*height/2:
                  good_labels.append((i,area))
                #   component_mask = np.uint8(labels == i) * 255
                #   plt.imshow(component_mask)
                #   plt.show()
                #   print(area)

  if len(good_labels) == 0:
      return None
  good_labels.sort(key=lambda x: x[1], reverse=True)
  index = 0

  approx = None
  while (approx is None and index<len(good_labels)):
    approx = find_bounding_box(labels, good_labels, index)
    index +=1
  
  if approx is None:
    return None


  src_pts = approx.reshape(4, 2).astype(np.float32)

  #From Gemini with some modification:
  # Important: You may need to sort these points to ensure they match dst_pts
  # A common trick is to sum/diff coordinates to find corners
  s = src_pts.sum(axis=1)
  diff = np.diff(src_pts, axis=1)

  ordered_src = np.zeros((4, 2), dtype="float32")
  ordered_src[0] = src_pts[np.argmin(s)]     # Top-left (min sum)
  ordered_src[2] = src_pts[np.argmax(s)]     # Bottom-right (max sum)
  ordered_src[1] = src_pts[np.argmin(diff)]  # Top-right (min difference)
  ordered_src[3] = src_pts[np.argmax(diff)]  # Bottom-left (max difference)

  dest_w = 272*2
  dest_h = 180*2
  dst_pts = np.float32([
      [0, 0],
      [dest_w, 0],
      [dest_w, dest_h],
      [0, dest_h]
  ])

  #apply homography to clueboard

  M = cv2.getPerspectiveTransform(ordered_src, dst_pts)
  warped_colour = cv2.warpPerspective(frame, M, (dest_w, dest_h))
  # plt.imshow(warped_colour)
  # plt.show()

  clue_colour = warped_colour[200:-5, 5:-5]

  clue_hsv = cv2.cvtColor(clue_colour, cv2.COLOR_BGR2HSV)

  # Threshold the HSV image to get only blue colors
  clue_mask = cv2.inRange(clue_hsv, lower_hsv, upper_hsv)

  # plt.imshow(clue_mask)
  # plt.show()

  return clue_mask

# TODO: Update extract letters to identify spaces if needed
def extract_letters(clue):

  kernel = np.ones((3, 3), np.uint8)
  eroded_clue = cv2.erode(clue, kernel, iterations = 1)

  # plt.imshow(eroded_clue)
  # plt.show()
  height, width = eroded_clue.shape[:2]
  num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(eroded_clue)

  characters = []
  target_w, target_h = 30, 50

  CHAR_MIN_AREA_THRESH = 100
  CHAR_MAX_AREA_THRESH = 1200

  for i in range(1, num_labels):
    x, y, w, h, area = stats[i, :5]
    # print(area)

    if CHAR_MIN_AREA_THRESH < area:
        # Define safe crop coordinates with padding
        y1, y2 = max(0, y-10), min(height, y+h+10)
        x1, x2 = max(0, x-10), min(width, x+w+10)

        if area < CHAR_MAX_AREA_THRESH:
            roi = clue[y1:y2, x1:x2]
            if roi.size > 0: # Check if crop is actually valid
                letter = cv2.resize(roi, (target_w, target_h), interpolation=cv2.INTER_CUBIC)
                characters.append([np.array(letter), x])
    else:
        print("Area too large")
        print(area)
        
    #   else:
    #       # Splitting logic for large areas (assumes only two connected letters, not sure if there is a better way to do this...)
    #       wp = int(w/2)
    #       # Split 1
    #       x1_s1, x2_s1 = max(0, x-10), min(width, x+wp+5)
    #       roi1 = clue[y1:y2, x1_s1:x2_s1]
    #       # Split 2
    #       x1_s2, x2_s2 = max(0, x+wp-5), min(width, x+w+10)
    #       roi2 = clue[y1:y2, x1_s2:x2_s2]

    #       for roi in [roi1, roi2]:
    #           if roi.size > 0:
    #               letter = cv2.resize(roi, (target_w, target_h), interpolation=cv2.INTER_CUBIC)
    #               characters.append([np.array(letter), x])

  sorted_characters = sorted(characters, key=lambda row: row[1])
  clue_letters = [data[0] for data in sorted_characters]
  return clue_letters

#return CNN's prediction of character given input image
def read_letter(let_img):
    input_tensor = np.expand_dims(let_img, axis=(0,-1)).astype(np.float32)
    interpreter.set_tensor(input_details[0]['index'], input_tensor)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])
    return one_hot_to_char(output_data[0])

#take a CV image frame and return clue answer
def read_clueboard(cv_image):
    clue_img = extract_clue(cv_image)
    if clue_img is None:
        return ""
    clue_letters_img = extract_letters(clue_img)

    clue_ans = []

    for let_img in clue_letters_img:
        clue_ans.append(read_letter(let_img))
    
    return ''.join(clue_ans)


def extract_clue8(frame):
  height, width = frame.shape[:2]

  # Threshold the HSV image to get only blue colors
  frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  mask_blue = cv2.inRange(frame_hsv, lower_hsv, upper_hsv)

  mask_low_red = cv2.inRange(frame_hsv, lower_hsv_rl, upper_hsv_rl)
  # mask_high_red = cv2.inRange(frame_hsv, lower_hsv_rh, upper_hsv_rh)
  # plt.imshow(mask_high_red)
  # plt.show()
  mask = cv2.bitwise_or(mask_blue, mask_low_red)

  inverted_mask = cv2.bitwise_not(mask)

  num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(inverted_mask)
  good_labels = []
  for i in range(1, num_labels):
          x, y, w, h, area = stats[i, :5]

          if width*height/1000 < area:
              y1, y2 = max(0, y), min(height, y+h)
              x1, x2 = max(0, x), min(width, x+w)

              if area < width*height/2:
                  good_labels.append((i,area))
  
  if len(good_labels) == 0:
      return None
  good_labels.sort(key=lambda x: x[1], reverse=True)

  component_mask = np.uint8(labels == good_labels[0][0]) * 255

  #Find 4 corners of clueboard
  contours, _ = cv2.findContours(component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

  if contours:
      cnt = contours[0]
      approx = cv2.approxPolyN(cnt, 4, -1)
  
  src_pts = approx.reshape(4, 2).astype(np.float32)

  #From Gemini with some modification:
  # Important: You may need to sort these points to ensure they match dst_pts
  # A common trick is to sum/diff coordinates to find corners
  s = src_pts.sum(axis=1)
  diff = np.diff(src_pts, axis=1)

  ordered_src = np.zeros((4, 2), dtype="float32")
  ordered_src[0] = src_pts[np.argmin(s)]     # Top-left (min sum)
  ordered_src[2] = src_pts[np.argmax(s)]     # Bottom-right (max sum)
  ordered_src[1] = src_pts[np.argmin(diff)]  # Top-right (min difference)
  ordered_src[3] = src_pts[np.argmax(diff)]  # Bottom-left (max difference)

  dest_w = 272*2
  dest_h = 180*2
  dst_pts = np.float32([
      [0, 0],
      [dest_w, 0],
      [dest_w, dest_h],
      [0, dest_h]
  ])

  #apply homography to clueboard

  M = cv2.getPerspectiveTransform(ordered_src, dst_pts)
  warped_colour = cv2.warpPerspective(frame, M, (dest_w, dest_h))
  # plt.imshow(warped_colour)
  # plt.show()

  clue_colour = warped_colour[200:-5, 5:-5]

  clue_hsv = cv2.cvtColor(clue_colour, cv2.COLOR_BGR2HSV)

  # Threshold the HSV image to get only blue colors
  clue_mask = cv2.inRange(clue_hsv, lower_hsv, upper_hsv)

  # plt.imshow(clue_mask)
  # plt.show()

  return clue_mask
  

def read_clueboard_8(cv_image):
    clue_img = extract_clue8(cv_image)
    if clue_img is None:
        return ""
    clue_letters_img = extract_letters(clue_img)

    clue_ans = []

    for let_img in clue_letters_img:
        clue_ans.append(read_letter(let_img))
    
    return ''.join(clue_ans)




print("Instantiating interpreter...")
#setup tensorflow
model_path = Path(__file__).parent / 'conv_model_FE_retrain.tflite'
interpreter = tf.lite.Interpreter(model_path=str(model_path))

# Allocate tensors (necessary to prepare the interpreter for inference)
interpreter.allocate_tensors()

# Get input and output tensor details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

print("Interpreter ready for inference")



if __name__ == '__main__':
    test_img_path = "/home/fizzer/ros_ws/src/team6/data/run_outputs_09_04_2026_00_06_56/image_car7.png"
    
    # 1. Initialize CvBridge
    bridge = CvBridge()

    # 2. Load the image using OpenCV
    cv_img = cv2.imread(test_img_path)

    if cv_img is None:
        print(f"Error: Could not load image at {test_img_path}")
    else:
        try:
            # 3. Simulate converting CV2 image to ROS Image message
            ros_image_msg = bridge.cv2_to_imgmsg(cv_img, "bgr8")
            print("Successfully simulated ROS Image message.")

            # 4. Convert it back to CV2 format (as your read_clueboard function expects a CV array)
            # This mimics how your subscriber callback would receive and process the data
            input_frame = bridge.imgmsg_to_cv2(ros_image_msg, "bgr8")

            # 5. Run the detection pipeline
            start_time = time.time()
            clue_text = read_clueboard_8(input_frame)
            end_time = time.time()

            # 6. Output results
            print("-" * 30)
            print(f"Detected Clue: {clue_text}")
            print(f"Inference time: {end_time - start_time:.4f} seconds")
            print("-" * 30)

        except Exception as e:
            print(f"An error occurred during processing: {e}")