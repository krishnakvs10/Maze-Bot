#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import matplotlib.pylab as plt
from skimage.morphology import skeletonize
import numpy as np 
import cv2
from cv_bridge import CvBridge
from ros_service.srv import path, pathResponse

def process_image(data):
    global image_message
    image_message = data

def main(message):
    global image_message
    rospy.Subscriber(message.img_topic, Image, process_image)
    bridge = CvBridge()
    img_name = bridge.imgmsg_to_cv2(image_message, "bgr8")
    rgb_img = plt.imread(img_name)
    gray_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY) 
    plt.figure(figsize=(7, 7)) 
    plt.imshow(gray_img)  
    h=rgb_img.shape[0]
    w=rgb_img.shape[1]
    print(h)
    print(w)

    x0, y0 = 20, 18
    x1, y1 = 300, 300 
    plt.figure(figsize=(7,7))
    plt.imshow(gray_img)
    plt.plot(x0, y0, 'gx', markersize=14)
    plt.plot(x1, y1, 'rx', markersize=14)  

    th, thr_img = cv2.threshold(gray_img, 127, 255, cv2.THRESH_BINARY)
    plt.figure(figsize=(7,7))
    plt.imshow(thr_img)
    plt.show()

    skeleton = skeletonize(thr_img/255)
    print(skeleton.shape)
    print(skeleton[21,22])
    print(y1,x1)

    print(type(skeleton))
    plt.figure(figsize=(7,7))
    plt.imshow(skeleton)

    mapT = ~skeleton
    plt.figure(figsize=(7,7))
    plt.imshow(mapT)
    plt.show()

    _mapt = np.copy(mapT)
    print(_mapt)
    boxr = 30

    if y1 < boxr: y1 = boxr
    if x1 < boxr: x1 = boxr

    cpys, cpxs = np.where(_mapt[y1 - boxr:y1 + boxr, x1 - boxr:x1 + boxr] == 0)
    print(cpys, cpxs)

    cpys += (y1 - boxr)
    cpxs += (x1 - boxr)

    idx = np.argmin(np.sqrt((cpys - y1) ** 2 + (cpxs - x1) ** 2))
    y, x = cpys[idx], cpxs[idx]

    pts_x = [x]
    pts_y = [y]
    pts_c = [0]

    xmesh, ymesh = np.meshgrid(np.arange(-1, 2), np.arange(-1, 2))
    ymesh = ymesh.reshape(-1)
    xmesh = xmesh.reshape(-1)

    dst = np.zeros((thr_img.shape))
    # Breath first algorithm exploring a tree
    while (True):
        idc = np.argmin(pts_c)
        ct = pts_c.pop(idc)
        x = pts_x.pop(idc)
        y = pts_y.pop(idc)
        
        ys, xs = np.where(_mapt[y - 1:y + 2, x - 1:x + 2] == 0)

        _mapt[ys + y - 1, xs + x - 1] = ct
        _mapt[y, x] = 9999999

        dst[ys + y - 1, xs + x - 1] = ct + 1

        pts_x.extend(xs + x - 1)
        pts_y.extend(ys + y - 1)
        pts_c.extend([ct + 1] * xs.shape[0])

        if pts_x == []:
            break
        if np.sqrt((x - x0) ** 2 + (y - y0) ** 2) < boxr:
            edx = x
            edy = y
            break
            
    plt.figure(figsize=(14, 14))
    plt.imshow(dst)
    plt.show()

    path_x = []
    path_y = []

    y = edy
    x = edx
    while (True):
        nbh = dst[y - 1:y + 2, x - 1:x + 2]
        nbh[1, 1] = 9999999
        nbh[nbh == 0] = 9999999

        if np.min(nbh) == 9999999:
            break

        idx = np.argmin(nbh)
        
        y += ymesh[idx]
        x += xmesh[idx]

        if np.sqrt((x - x1) ** 2 + (y - y1) ** 2) < boxr:
            print('Optimum route found.')
            break
        path_y.append(y)
        path_x.append(x)

    plt.figure(figsize=(15,15))
    plt.imshow(rgb_img)
    plt.plot(path_x, path_y, 'b-', linewidth=5)
    plt.show()
    n=len(path_x)
    path=[]
    for i in range(n):
        path.append([path_x[i],path_y[i]])

    #path contains the list of all the [x,y] coordinates of the shortest path
    SPC=np.array(path)
    np.save('shortest_path.npy',SPC)

    return  pathResponse('Done')



if __name__ == '__main__':
    try:
        rospy.init_node('service_respond')
        service=rospy.Service('service_for_path',path,main)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass