#!/usr/bin/env python
# encoding=utf-8

import os
import PySpin
import sys
import matplotlib.pyplot as plt
import keyboard
import time
import cv2
import imutils  # 图像处理方法
import numpy as np
import open3d as o3d
from os import path
from PIL import Image
import copy
import math


np.set_printoptions(threshold=np.inf)

#在代码同级创建文件夹“out”
save_path = os.path.join(os.getcwd(), "out")
if not os.path.exists(save_path):
    os.mkdir(save_path)

NUM_IMAGES = 1
image = cv2.imread("out/0.jpg")
cimg2 = image

# 调用2D相机拍照
# 输出：opencv格式的图像
def capture():
    global cimg2
    result = True
    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()
    num_cameras = cam_list.GetSize()
    print('找到 %d 台设备' % num_cameras)
    if num_cameras == 0:
        cam_list.Clear()
        system.ReleaseInstance()
        return False
    for i,cam in enumerate(cam_list):
        try:
            nodemap_tldevice = cam.GetTLDeviceNodeMap()
            cam.Init()
            nodemap = cam.GetNodeMap()
            try:
                result = True
                node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode('AcquisitionMode'))
                if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
                    print("无法设置相机为连续拍照模式(无法写入节点)")
                    return 0
                node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
                if not PySpin.IsAvailable(node_acquisition_mode_continuous) or not PySpin.IsReadable(node_acquisition_mode_continuous):
                    print("无法设置相机为连续拍照模式(无法读取节点)")
                    return 0
                # Retrieve integer value from entry node
                acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()
                # Set integer value from entry node as new value of enumeration node
                node_acquisition_mode.SetIntValue(acquisition_mode_continuous)
                cam.BeginAcquisition()
                for i in range(NUM_IMAGES):
                    try:
                        image_result = cam.GetNextImage(200)
                        # 获取传输层接收到的下一个图像。此函数将阻塞1s直到图像到达
                        if image_result.IsIncomplete():
                            print('图像缺失.状态 %d ...' % image_result.GetImageStatus())
                        else:
                            image_data = image_result.GetNDArray()
                            cimg2 = cv2.cvtColor(np.asarray(image_data),cv2.COLOR_RGB2BGR) 
                            cimg2 = cv2.resize(cimg2, (1824, 1216))
                            # cv2.namedWindow('tmp', cv2.WINDOW_KEEPRATIO)
                            # cv2.imshow('tmp',cimg2)  #显示图像
                            # cv2.waitKey()
                            image_result.Release()
                            # 一旦缓存里的图像被存储，或者抛弃，需要马上从缓存里清理掉，防止缓存炸了
                    except PySpin.SpinnakerException as ex:
                        print('错误: %s' % ex)
                        return 0
                cam.EndAcquisition()
            except PySpin.SpinnakerException as ex:
                print('错误: %s' % ex)
            # cam.DeInit()
        except PySpin.SpinnakerException as ex:
            print('错误: %s' % ex)
            del cam
            cam_list.Clear()
            # system.ReleaseInstance()
            return 0
    del cam
    cam_list.Clear()
    system.ReleaseInstance()
    return cimg2

# 模板匹配模块
# 输入：模板图像和目标图像
# 输出：匹配度分数
def findchar(name,imagename):
    template = cv2.imread(name)
    template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    template = cv2.Canny(template, 50, 200)
    (tH, tW) = template.shape[:2]
    #初始化用于追踪匹配区域的簿记变量
    image = cv2.imread(imagename)
    gray = image#cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found = None
    # 遍历图像尺寸
    for scale in np.linspace(0.15, 1.0, 10)[::-1]:#(0.2, 1.0, 20)
        # 根据scale比例缩放图像，并保持其宽高比
        resized = imutils.resize(gray, width=int(gray.shape[1] * scale))
        r = gray.shape[1] / float(resized.shape[1])
        # 缩放到图像比模板小，则终止
        if resized.shape[0] < tH or resized.shape[1] < tW:
            break
        edged = cv2.Canny(resized, 50, 200)
        result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
        (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)

        if 1:
            clone = np.dstack([edged, edged, edged])
            cv2.rectangle(clone, (maxLoc[0], maxLoc[1]),(maxLoc[0]+tW, maxLoc[1]+tH), (0,0,255),2)
            cv2.namedWindow('Visualize', cv2.WINDOW_KEEPRATIO)
            cv2.imshow("Visualize", clone)
            cv2.waitKey(1)

        if found is None or maxVal > found[0]:
            found = (maxVal, maxLoc, r)
    return(found)

# 调用OPEN3d进行3D识别
# 输入：上/下摘钩种类
def plane(type):
    
    # start = time.time()
    distance=0
    while distance==0:
        os.system('./PointCloud')   

        pcd = o3d.io.read_point_cloud("out/0.ply")
        pcd_down = pcd.voxel_down_sample(voxel_size=0.004)

        points = np.asarray(pcd_down.points)
        pcd_down = pcd_down.select_by_index(np.where(points[:, 2] > 0.8)[0])

        plane_model, inliers = pcd_down.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=800)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.6f}x + {b:.6f}y + {c:.6f}z + {d:.6f} = 0")
        aa=(a-0.232566)/2
        bb=(b-0.599264)/2
        cc=(c+0.766026)/2
        dd=d+1.111197
        distance=-dd/(math.sqrt(aa*aa+bb*bb+cc*cc))
        if abs(distance) < 0.9:
            print("车车平移距离：",distance)
            with open("out/result.txt","w") as f:
                f.write(str(type)+", "+str(distance))

        inlier_cloud = pcd_down.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = pcd_down.select_by_index(inliers, invert=True)
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)

        o3d.visualization.draw_geometries([outlier_cloud,mesh])

        points = np.asarray(outlier_cloud.points)
        outlier_cloud = outlier_cloud.select_by_index(np.where(points[:, 0] < 0.5)[0])

        #删除平面外的点
        points = np.asarray(outlier_cloud.points)
        mask = a*points[:, 0] + b*points[:, 1] + c*points[:, 2] + d >0
        outlier_cloud.points = o3d.utility.Vector3dVector(points[mask])

        # 半径离群值移除
        cl, ind = outlier_cloud.remove_radius_outlier(nb_points=5, radius=0.02)
        outlier_remove = outlier_cloud.select_by_index(ind)
        o3d.visualization.draw_geometries([outlier_remove,mesh])

        o3d.io.write_point_cloud('out/clear.pcd', outlier_remove)
        os.system('./cylinder')

# 使用光流检测车辆驶入
# 输出1:发现车辆由运动到停止
def figure_stream():

    feature_params = dict( maxCorners = 100, qualityLevel = 0.3,
                           minDistance = 7, blockSize = 7 )
    # lucas kanade光流法参数
    lk_params = dict( winSize  = (15,15),maxLevel = 2,
                      criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    # 创建随机颜色
    color = np.random.randint(0,255,(100,3))
    
    # 获取第一帧，找到角点
    # old_frame = cv2.imread("out/0.jpg")
    old_frame = capture()
    # old_frame = cv2.resize(old_frame, (1824, 1216))# 双线性插值压缩
    # x, y = old_frame.shape[0:2]
    old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
    # 创建蒙版用来画轨迹
    # mask = np.zeros_like(old_frame)
    #获取图像中的角点，返回到p0中
    p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
    flag0 = 0
    flag1 = 0
    flag2 = 0#记录最近3次循环的光流结果
    mood = 0 #记录光流的4种情况：0由静变动，1由动变静
    while(1):
        
        #读取图像帧
        frame = capture()
        # frame = cv2.resize(old_frame, (1824, 1216))
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #灰度化
        # 计算光流
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        # 选取好的跟踪点
        good_new = p1[st==1]
        good_old = p0[st==1]

        # 画出轨迹
        distance = 0
        counter = 0
        for i,(new,old) in enumerate(zip(good_new,good_old)):
            a,b = new.ravel()#多维数据转一维,将坐标转换后赋值给a，b
            c,d = old.ravel()
            if abs(d-b) < 200:#只记录沿水平方向的移动，容错阈值：200像素
                counter = counter+1
                distance = distance+(c-a)*(c-a)+(d-b)*(d-b)
            # mask = cv2.line(mask, (int(a),int(b)),(int(c),int(d)), color[i].tolist(), 2)#画直线
            frame = cv2.circle(frame,(int(a),int(b)),5,color[i].tolist(),-1)#画点
        average = distance/(counter)
        print("平均移动距离:",average)
        flag2 = flag1
        flag1 = flag0
        if average >= 7000:
            flag0 = 1#运动帧记为1
        else:
            flag0 = 0#静止帧记为0
        if flag0 == 1 and flag1 == 0 and flag2 == 0:
            mood =0
            # break
            print("发现车辆驶入或再次启动")
        elif flag0 == 0 and flag1 == 1 and flag2 == 1:
            mood =1
            print("发现车辆停止，开始识别车号")
            break
        img = frame#cv2.add(frame,mask) # 将画出的线条进行图像叠加
        cv2.namedWindow('frame', cv2.WINDOW_KEEPRATIO)
        cv2.imshow('frame',img)  #显示图像
        cv2.waitKey(5)

        # 更新上一帧的图像和追踪点
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1,1,2)

    return mood




if __name__ == '__main__':
    
    start = time.time()
    if  figure_stream():#如果车由动变静

        image = capture()
        # image = cv2.resize(image, (1824, 1216))
        (score1, maxLoc1, r1) = findchar("out/C64.jpg","out/0.jpg")
        (score2, maxLoc2, r2) = findchar("out/C70.jpg","out/0.jpg")
        print(score1)
        print(score2)
        tH=80#模板高度
        tW=124#模板长度
        type=0
        (hight, width) = image.shape[:2]
        if int(maxLoc1[1] * r1) < hight/2:
            if int(maxLoc2[1] * r1) < hight/2:
                if score1 > score2:
                    (startX, startY) = (int(maxLoc1[0] * r1), int(maxLoc1[1] * r1))
                    (endX, endY) = (int((maxLoc1[0] + tW) * r1), int((maxLoc1[1] + tH) * r1))
                    print("C64")
                    type=1
                else:
                    (startX, startY) = (int(maxLoc2[0] * r2), int(maxLoc2[1] * r2))
                    (endX, endY) = (int((maxLoc2[0] + tW) * r2), int((maxLoc2[1] + tH) * r2))
                    print("C70")
                    type=0
            else:
                (startX, startY) = (int(maxLoc1[0] * r1), int(maxLoc1[1] * r1))
                (endX, endY) = (int((maxLoc1[0] + tW) * r1), int((maxLoc1[1] + tH) * r1))
                print("C64")
                type=1
        else:
            (startX, startY) = (int(maxLoc2[0] * r2), int(maxLoc2[1] * r2))
            (endX, endY) = (int((maxLoc2[0] + tW) * r2), int((maxLoc2[1] + tH) * r2))
            print("C70")
            type=0
        print("用时 %.3f 秒." % (time.time() - start))    
        cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 5)
        cv2.namedWindow('Visualize', cv2.WINDOW_KEEPRATIO)
        cv2.imshow("Visualize", image)
        cv2.waitKey(0)
        with open("out/result.txt","w") as f:
                f.write(str(type))# +", "+str(distance)
        os.system('./PointCloud')   
        sys.exit(0)
    # else:
    #     print("拍照失败！")
    #     sys.exit(1)
    sys.exit(0)
    
