#!/usr/bin/env python
from inspect import modulesbyfile
import sys
from tempfile import TemporaryFile
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sara_arm.srv import *
from screeninfo import get_monitors

class image_converter:

  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

    self.click_pub = rospy.Publisher("button_pose",PoseStamped,queue_size=10)
    self.type_pub = rospy.Publisher("plan_type",UInt8,queue_size=10)

    self.pcp_service = rospy.ServiceProxy('image_to_cloud_point', image_to_cloud_point)

    self.button_h = 50

    self.selection = None

    self.monitors = get_monitors()
    self.monitor_h = self.monitors[0].height
    self.monitor_w = self.monitors[0].width
    self.dim = (self.monitor_w-4*self.button_h, self.monitor_h-4*self.button_h)

  def callback(self,data):
    try:
      msg_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.shape_og = msg_img.shape

    gui_img = self.build_gui(msg_img)

    # DISPLAY SELECTION
    if not self.selection is None:
      cv2.line(gui_img, (self.selection[0]-12,self.selection[1]), (self.selection[0]+12,self.selection[1]), (0, 0, 0), 6)
      cv2.line(gui_img, (self.selection[0],self.selection[1]-12), (self.selection[0],self.selection[1]+12), (0, 0, 0), 6)
      cv2.line(gui_img, (self.selection[0]-10,self.selection[1]), (self.selection[0]+10,self.selection[1]), (0, 255, 0), 2)
      cv2.line(gui_img, (self.selection[0],self.selection[1]-10), (self.selection[0],self.selection[1]+10), (0, 255, 0), 2)

    cv2.namedWindow("Image window", cv2.WND_PROP_FULLSCREEN)
    # cv2.setWindowProperty("Image window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    cv2.imshow("Image window", gui_img)
    # cv2.setMouseCallback("Image window", self.click, msg_img.shape)
    cv2.setMouseCallback("Image window", self.click, (self.dim[1],self.dim[0]))
    cv2.waitKey(3)

  # Image segmentation using KMEANS
  def kmeans(self, cv_img):
    img = cv2.cvtColor(cv_img,cv2.COLOR_BGR2RGB)
    twoDimage = img.reshape((-1,3))
    twoDimage = np.float32(twoDimage)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_MAX_ITER, 10, 1.0)
    K=2
    attempts = 10
    ret,label,center=cv2.kmeans(twoDimage,K,None,criteria,attempts,cv2.KMEANS_PP_CENTERS)
    center = np.uint8(center)
    res = center[label.flatten()]
    result_image = res.reshape((img.shape))
    return result_image

  def click(self, event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
      img_h = param[0]
      img_w = param[1]

      # TOP LEFT BUTTON (SQUARE)
      if x < 2*self.button_h and y < 2*self.button_h:
        self.type_pub.publish(1)
        self.send_command()
      
      # TOP RIGHT BUTTON (DIAMOND)
      elif x > 2*self.button_h + img_w and y < 2*self.button_h:
        self.type_pub.publish(2)
        self.send_command()

      # BOTTOM LEFT BUTTON (PLUS)
      elif x < 2*self.button_h and y > 2*self.button_h + img_h:
        self.type_pub.publish(3)
        self.send_command()
      
      # BOTTOM RIGHT BUTTON (CIRCLE)
      elif x > 2*self.button_h + img_w and y > 2*self.button_h + img_h:
        self.type_pub.publish(4)
        self.send_command()

      # IMAGE SELECTION
      elif x > 2*self.button_h and x < 2*self.button_h+img_w and y > 2*self.button_h and y <2*self.button_h+img_h:
        self.selection = (x,y)
      
      # MOVE LEFT FAST
      elif x<self.button_h and y > 2*self.button_h and y <2*self.button_h+img_h and not self.selection is None:
        self.selection = (max(self.selection[0]-10,2*self.button_h),self.selection[1])
      
      # MOVE LEFT SLOW
      elif x>self.button_h and x<2*self.button_h and y > 2*self.button_h and y <2*self.button_h+img_h and not self.selection is None:
        self.selection = (max(self.selection[0]-1,2*self.button_h),self.selection[1])

      # MOVE RIGHT SLOW
      elif x>2*self.button_h+img_w and x<3*self.button_h+img_w and y > 2*self.button_h and y <2*self.button_h+img_h and not self.selection is None:
        self.selection = (min(self.selection[0]+1,2*self.button_h+img_w),self.selection[1])
      
      # MOVE RIGHT FAST
      elif x>3*self.button_h+img_w and y > 2*self.button_h and y <2*self.button_h+img_h and not self.selection is None:
        self.selection = (min(self.selection[0]+10,2*self.button_h+img_w),self.selection[1])

      # MOVE UP FAST
      elif x > 2*self.button_h and x < 2*self.button_h+img_w and y < self.button_h and not self.selection is None:
        self.selection = (self.selection[0],max(self.selection[1]-10,2*self.button_h))

      # MOVE UP SLOW
      elif x > 2*self.button_h and x < 2*self.button_h+img_w and y > self.button_h and y < 2*self.button_h and not self.selection is None:
        self.selection = (self.selection[0],max(self.selection[1]-1,2*self.button_h))

      # MOVE DOWN SLOW
      elif x > 2*self.button_h and x < 2*self.button_h+img_w and y > 2*self.button_h+img_h and y < 3*self.button_h+img_h and not self.selection is None:
        self.selection = (self.selection[0],min(self.selection[1]+1,2*self.button_h+img_h))

      # MOVE DOWN FAST
      elif x > 2*self.button_h and x < 2*self.button_h+img_w and y > 3*self.button_h+img_h and not self.selection is None:
        self.selection = (self.selection[0],min(self.selection[1]+10,2*self.button_h+img_h))

  def send_command(self):
      if not self.selection is None: 
        try:
          shift_selection = self.selection[0]-2*self.button_h,self.selection[1]-2*self.button_h
          transformed_selection = (round(shift_selection[0] * (self.shape_og[1] / self.dim[0])), round(shift_selection[1] * (self.shape_og[0] / self.dim[1])))
          resp1 = self.pcp_service(transformed_selection[0],transformed_selection[1])
          self.click_pub.publish(resp1.cloud_in_pose)

        except rospy.ServiceException as e:
          print("service call failed: %s"%e)
      
  def build_gui(self, msg_img):
    border = 2*self.button_h
    msg_img = cv2.resize(msg_img, self.dim, interpolation=cv2.INTER_AREA)
    img_h = msg_img.shape[0]
    img_w = msg_img.shape[1]

    # CREATE EMPTY IMAGE FOR GUI
    gui = np.zeros((img_h+2*border, img_w+2*border,3), np.uint8)

    # PUT IMAGE MSG IN THE MIDDLE OF GUI
    gui[border:img_h+border,border:img_w+border] = msg_img

    # BUTTONS
    edge_color = (70,70,70)

    cv2.rectangle(gui, (0,border), (self.button_h,border+img_h),edge_color, 3)
    cv2.arrowedLine(gui,(int(self.button_h/4),border+int(img_h/2)),(int(self.button_h/4)-1,border+int(img_h/2)),edge_color,8,tipLength=30)

    cv2.rectangle(gui, (self.button_h,border), (border,border+img_h),edge_color, 3)
    cv2.arrowedLine(gui,(self.button_h+int(self.button_h/4),border+int(img_h/2)),(self.button_h+int(self.button_h/4)-1,border+int(img_h/2)),edge_color,4,tipLength=30)

    cv2.rectangle(gui, (border,0), (border+img_w,self.button_h),edge_color, 3)
    cv2.arrowedLine(gui,(border+int(img_w/2),int(self.button_h/4)),(border+int(img_w/2),int(self.button_h/4)-1),edge_color,8,tipLength=30)

    cv2.rectangle(gui, (border,self.button_h), (border+img_w,border),edge_color, 3)
    cv2.arrowedLine(gui,(border+int(img_w/2),self.button_h+int(self.button_h/4)),(border+int(img_w/2),self.button_h+int(self.button_h/4)-1),edge_color,4,tipLength=30)

    cv2.rectangle(gui, (border+img_w,border), (border+img_w+self.button_h,border+img_h),edge_color, 3)
    cv2.arrowedLine(gui,(border+img_w+int(3*self.button_h/4),border+int(img_h/2)),(border+img_w+int(3*self.button_h/4)+1,border+int(img_h/2)),edge_color,4,tipLength=30)

    cv2.rectangle(gui, (border+img_w+self.button_h,border), (border+img_w+border,border+img_h),edge_color, 3)
    cv2.arrowedLine(gui,(self.button_h+border+img_w+int(3*self.button_h/4),border+int(img_h/2)),(self.button_h+border+img_w+int(3*self.button_h/4)+1,border+int(img_h/2)),edge_color,8,tipLength=30)

    cv2.rectangle(gui, (border,border+img_h), (border+img_w,border+img_h+self.button_h),edge_color, 3)
    cv2.arrowedLine(gui,(border+int(img_w/2),border+img_h+int(3*self.button_h/4)),(border+int(img_w/2),border+img_h+int(3*self.button_h/4)+1),edge_color,4,tipLength=30)

    cv2.rectangle(gui, (border,border+img_h+self.button_h), (border+img_w,border+img_h+border),edge_color, 3)
    cv2.arrowedLine(gui,(border+int(img_w/2),self.button_h+border+img_h+int(3*self.button_h/4)),(border+int(img_w/2),self.button_h+border+img_h+int(3*self.button_h/4)+1),edge_color,8,tipLength=30)

    cv2.rectangle(gui, (int(border/4),int(border/4)), (int(3*border/4),int(3*border/4)),(255,102,102), 3)

    cv2.circle(gui, (3*self.button_h+img_w,3*self.button_h+img_h),30,(64,226,160), 3)

    width = 20
    height = 60
    middle = 20
    cv2.line(gui,(3*self.button_h+img_w,int(self.button_h/2)-10),(3*self.button_h+img_w-width, int(self.button_h/2)+middle),(124,178,232), 3)
    cv2.line(gui,(3*self.button_h+img_w,int(self.button_h/2)-10),(3*self.button_h+img_w+width, int(self.button_h/2)+middle),(124,178,232), 3)
    cv2.line(gui,(3*self.button_h+img_w,int(self.button_h/2)+height),(3*self.button_h+img_w-width, int(self.button_h/2)+middle),(124,178,232), 3)
    cv2.line(gui,(3*self.button_h+img_w,int(self.button_h/2)+height),(3*self.button_h+img_w+width, int(self.button_h/2)+middle),(124,178,232), 3)

    cv2.line(gui,(self.button_h-30, 3*self.button_h+img_h),(self.button_h+30, 3*self.button_h+img_h),(255,105,248), 3)
    cv2.line(gui,(self.button_h, 3*self.button_h+img_h-30),(self.button_h, 3*self.button_h+img_h+30),(255,105,248), 3)

    return gui

  # Image segmentation using CONTOURING
  def contour(self, cv_img):
    img = cv2.resize(cv_img,(256,256))
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    _,thresh = cv2.threshold(gray, np.mean(gray), 255, cv2.THRESH_BINARY_INV)
    edges = cv2.dilate(cv2.Canny(thresh,0,255),None)
    cnt = sorted(cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2], key=cv2.contourArea)[-1]
    mask = np.zeros((256,256), np.uint8)
    masked = cv2.drawContours(mask, [cnt], -1, 255, -1)
    dst = cv2.bitwise_and(img, img, mask=masked)
    segmented = cv2.cvtColor(dst, cv2.COLOR_BGR2RGB)
    return segmented

  def canny_edge(self, cv_img):
    gray = cv2.cvtColor(cv_img,cv2.COLOR_BGR2GRAY)
    # blur = cv2.GaussianBlur(gray,(1,1),0)
    edges = cv2.Canny(image = gray, threshold1=100, threshold2=200)
    return edges

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