#!/usr/bin/env python
from inspect import modulesbyfile
import sys
from tempfile import TemporaryFile
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sara_arm.srv import *

class image_converter:

  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/color/image_raw",Image,self.callback)
    self.click_pub = rospy.Publisher("click_input",PointStamped,queue_size=10)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    # cv_image_new = self.contour(cv_image)

    cv2.imshow("Image window", cv_image)
    cv2.setMouseCallback("Image window", self.click)
    cv2.waitKey(3)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)


  # Image segmentation using KMEANS
  def kmeans(self, cv_image):
    img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
    twoDimage = img.reshape((-1,3))
    twoDimage = np.float32(twoDimage)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_MAX_ITER, 10, 1.0)
    K=3
    attempts = 10
    ret,label,center=cv2.kmeans(twoDimage,K,None,criteria,attempts,cv2.KMEANS_PP_CENTERS)
    center = np.uint8(center)
    res = center[label.flatten()]
    result_image = res.reshape((img.shape))
    return result_image

  # Image segmentation using CONTOURING
  def contour(self, cv_image):
      img = cv2.resize(cv_image,(256,256))
      gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
      _,thresh = cv2.threshold(gray, np.mean(gray), 255, cv2.THRESH_BINARY_INV)
      edges = cv2.dilate(cv2.Canny(thresh,0,255),None)
      cnt = sorted(cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2], key=cv2.contourArea)[-1]
      mask = np.zeros((256,256), np.uint8)
      masked = cv2.drawContours(mask, [cnt], -1, 255, -1)
      dst = cv2.bitwise_and(img, img, mask=masked)
      segmented = cv2.cvtColor(dst, cv2.COLOR_BGR2RGB)
      return segmented

  def click(self, event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
      print(type(x))
      try:
        cloud_point = rospy.ServiceProxy('image_to_cloud_point', image_to_cloud_point)
        resp1 = cloud_point(x,y)
      except rospy.ServiceException as e:
        print("service call failed: %s"%e)

      self.click_pub.publish(resp1.cloud_input)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)