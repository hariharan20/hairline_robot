#!/usr/bin/env python  

import rospy
from sensor_msgs.msg import Image, PointCloud2
import tensorflow as tf
import cv2
import numpy as np
import ros_numpy
import albumentations as A 
import keras
import segmentation_models as sm
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import sys
try:
    sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
except:
    pass

#import tf
def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),dtype=dtype, buffer=img_msg.data)
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv



rospy.loginfo('Import Successful')
#bridge = CvBridge()
rospy.init_node('hair_line_detection')
BACKBONE = 'efficientnetb3'
BATCH_SIZE = 2
CLASSES = ['hair']
preprocess_input = sm.get_preprocessing(BACKBONE)
n_classes = 1
activation = 'sigmoid'
model = sm.Unet(BACKBONE, classes=n_classes, activation=activation)
model.load_weights('ele_ws/src/codes_/best_model_1202.h5')
rospy.loginfo('model Loaded Successfully')
data = rospy.wait_for_message('kinect2/hd/image_color' , Image)
data_3d = rospy.wait_for_message('kinect2/hd/points' , PointCloud2)
data_3d_np = ros_numpy.numpify(data_3d)
img = imgmsg_to_cv2(data)
print(img.shape)
#img = cv2.resize(img, (1024, 1024))
img = img[0:1024 , 0 :1024] 
img = cv2.cvtColor(img , cv2.COLOR_BGR2RGB)
preprocess_input = sm.get_preprocessing(BACKBONE)
transform = A.Compose([A.PadIfNeeded(384 , 480) ,A.Lambda(image=preprocess_input)])
img = transform(image = img)['image']
img = np.expand_dims(img , axis=0)
pr_mask = model.predict(img).round()
pr_mask = pr_mask.astype(np.uint8)
pr_mask = pr_mask.squeeze()
import matplotlib.pyplot as plt
plt.imshow(pr_mask)
plt.show()
contours, hierarchy = cv2.findContours(pr_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
old_len = 0

for contour in contours:
    if(old_len < len(contour)):
        print(len(contour))
        rc = contour
        old_len = len(contour)

pr_mask_with_cntr = cv2.drawContours(pr_mask, rc, -1, (255, 255, 255), 1)
plt.imshow(cv2.drawContours(pr_mask, rc, -1, (255, 255, 255), 3))
plt.show()

from geometry_msgs.msg import PointStamped

pub = rospy.Publisher('hair_line_pointcloud' , PointCloud2 , queue_size=10)
points = np.zeros((len(rc), 3))
points = np.zeros(len(rc) , dtype=[('x' , np.float32) ,('y' , np.float32), ('z' , np.float32)])

#print(data_3d_np['x'].shape)
for i in range(len(rc)):
    #print(rc[i][0])
    #p = PointStamped()
    #p.header.frame_id = 'kinect2_link'
    #p_1 = l.transformPoint('joint1' , p)
    #points['x'][i] = p_1.x
    #points['y'][i] = p_1.y
    #points['z'][i] = p_1.z
    points['x'][i] = data_3d_np['x'][rc[i][0][1]][rc[i][0][0]]
    points['y'][i] = data_3d_np['y'][rc[i][0][1]][rc[i][0][0]]
    points['z'][i] = data_3d_np['z'][rc[i][0][1]][rc[i][0][0]]
pc = ros_numpy.msgify(PointCloud2 , points)
pc.header.frame_id = 'kinect2_link'
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(pc)
    rate.sleep()
