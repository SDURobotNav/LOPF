#!/usr/bin/env python
import pcl
import glob
import cv2 as cv
import math
import tensorflow as tf
import numpy as np
pedestrian_paths = glob.glob('./pedestrian/*')
def extractFeature(temp_pcd):

    feature_list = []

    # feature1 number of cluster
    # ori_pcd = pcl.load(filename)
    ori_pcd = temp_pcd
    feature_list.append(ori_pcd.size)
    # feature2 distance from pointcloud to sensor
    min_dis = 1000
    for i in ori_pcd:
        dis = math.sqrt(float(i[0])**2+float(i[1])**2+float(i[2])**2)
        if dis < min_dis:
            min_dis = dis

    feature_list.append(min_dis)
    # feature3-7 PCA
    data = np.array(ori_pcd)
    data_mean = np.mean(data,axis=0)

    normalize_data = data-data_mean

    H = np.dot(normalize_data.T,normalize_data)
    eigenvectors, eigenvalues, eigenvectors_T = np.linalg.svd(H)
    sort_id = eigenvalues.argsort()[::-1]
    eigenvectors = eigenvectors[:,sort_id]
    # print(eigenvalues)
    # print(eigenvectors)

    for i in histogram2d(normalize_data,eigenvectors[0],eigenvectors[1],14,7):
        for j in i:
            feature_list.append(j)
    # histogram2d(ori_pcd,eigenvectors[0],eigenvectors[1],14,7)
    for i in histogram2d(normalize_data, eigenvectors[0], eigenvectors[2], 9, 5):
        for j in i:
            feature_list.append(j)

    return feature_list
# def sliceFeature(centered_cloud,e1,e2,e3,slice_n,feature):

def histogram2d(centered_cloud,e1,e2,x_num,y_num,feature=None):
    pts2d = np.zeros((centered_cloud.shape[0],2))
    # print(centered_cloud.shape[0])
    for i in range(centered_cloud.shape[0]):
        # print(centered_cloud.shape[0])

        pts2d[i, 0] = np.dot(np.array(centered_cloud[i]), e1)
        pts2d[i, 1] = np.dot(np.array(centered_cloud[i]), e2)

    x_min = min(pts2d[:,0])
    x_max = max(pts2d[:,0])
    y_min = min(pts2d[:,1])
    y_max = max(pts2d[:,1])

    det_x = (x_max-x_min)/(x_num-0.01)
    det_y = (y_max-y_min)/(y_num-0.01)
    hist = np.zeros((x_num,y_num))
    for i in pts2d:
        hist[int((i[0]-x_min)/det_x),int((i[1]-y_min)/det_y)]+=1
    if feature!=None:
        feature.append(hist)
    return hist
if __name__ == '__main__':


    extractFeature(pedestrian_paths[4])