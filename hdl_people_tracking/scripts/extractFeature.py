#!/usr/bin/env python
import pcl
import glob
import cv2 as cv
import math
# import tensorflow as tf
from geometry_msgs.msg import Point
import numpy as np
# from sklearn.neural_network import MLPRegressor

pedestrian_paths = glob.glob('/home/chen/tracking_ws/data_process/car/*')

# print(pedestrian_paths)
class pedestrianFeature():
    def __init__(self, temp_cloud):
        self.feature = []
        self.pointcloud = temp_cloud

        data = np.array(self.pointcloud)
        self.data_mean = np.mean(data, axis=0)

        self.centered_cloud = data - self.data_mean
        self.feature1 = None
        self.feature2 = None
        self.gridFeature = None
        self.cluster_msg = []
        self.max_pt = Point()
        self.min_pt = Point()


    def feature_1(self):
        self.feature1 = self.pointcloud.size
        return self.feature1

    def feature_2(self):
        min_dis = 1000
        max_dis = 0
        for i in self.pointcloud:
            dis = math.sqrt(float(i[0]) ** 2 + float(i[1]) ** 2 + float(i[2]) ** 2)
            if dis < min_dis:
                min_dis = dis
                self.min_pt.x = float(i[0])
                self.min_pt.y = float(i[1])
                self.min_pt.z = float(i[2])
            if dis > max_dis:
                max_dis = dis
                self.max_pt.x = float(i[0])
                self.max_pt.y = float(i[1])
                self.max_pt.z = float(i[2])

        self.feature2 = min_dis
        return min_dis

    def feature_3(self, pcd):
        data = np.array(pcd)
        data_mean = np.mean(data, axis=0)
        normalize_data = data - data_mean
        H = np.dot(normalize_data.T, normalize_data)
        eigenvectors, eigenvalues, eigenvectors_T = np.linalg.svd(H)
        sort_id = eigenvalues.argsort()[::-1]
        eigenvectors = eigenvectors[:, sort_id]
        # print(eigenvalues)
        # print(eigenvectors)
        feature = np.resize(self.histogram2d(pcd, eigenvectors[0], eigenvectors[1], 14, 7), (1, 14 * 7))
        return feature

    def extractFeature(self):
        feature_list = []

        # feature1 number of cluster
        feature_list.append(self.pointcloud.size)
        # feature2 distance from pointcloud to sensor
        min_dis = self.feature_2()
        feature_list.append(min_dis)
        # feature3-7 PCA
        H = np.dot(self.centered_cloud.T, self.centered_cloud)
        eigenvectors, eigenvalues, eigenvectors_T = np.linalg.svd(H)
        sort_id = eigenvalues.argsort()[::-1]
        eigenvectors = eigenvectors[:, sort_id]
        templist=self.histogram2d(eigenvectors[0], eigenvectors[1], 14, 7)

        feature_list+=templist[0]
        feature_list += self.histogram2d(eigenvectors[0], eigenvectors[2], 9, 5)[0]
        # print (feature_list)
        # feature8

        #

        return feature_list

    def grid_feature(self, frame_num=1, x=8, y=8, z=4):
        feature = np.zeros((frame_num, x, y, z))
        min_x, min_y, min_z = self.centered_cloud.min(axis=0)
        max_x, max_y, max_z = self.centered_cloud.max(axis=0)
        d_x = (max_x - 0.0001 - min_x) / x
        d_y = (max_y - 0.0001 - min_y) / y
        d_z = (max_z - 0.0001 - min_z) / z
        for j in range(frame_num):
            for i in self.centered_cloud:
                feature[j, int(i[0] / d_x), int(i[1] / d_y), int(i[2] / d_z)] += 1
        self.gridFeature = feature
        self.cluster_msg = [self.data_mean[0], self.data_mean[1], self.data_mean[2], max_x-min_x, max_y-min_y,max_z-min_z]
        return feature


    def sliceFeature(centered_cloud, e1, e2, e3, slice_n):
        eigenspace = np.array([e1.T, e2.T, e3.T])

        e_min = 9999
        e_max = -999
        aligned_cloud = []
        for i in centered_cloud:
            temp_value = np.dot(eigenspace, centered_cloud[i])
            aligned_cloud.append(temp_value)
            e_min = min(e_min, temp_value[0])
            e_max = max(e_max, temp_value[0])

        height = e_max - e_min
        scale = (slice_n - 0.1) / height

        min_pts = []
        max_pts = []
        pass

    def histogram2d(self, e1, e2, x_num, y_num, feature=None):
        # pts2d = np.zeros((cloud.shape[0], 2))
        # for i in range(cloud.shape[0]):
        #     pts2d[i, 0] = np.dot(np.array(cloud[i]), e1)
        #     pts2d[i, 1] = np.dot(np.array(cloud[i]), e2)
        pts2d = np.zeros((self.centered_cloud.shape[0], 2))
        for i in range(self.centered_cloud.shape[0]):
            pts2d[i, 0] = np.dot(np.array(self.centered_cloud[i]), e1)
            pts2d[i, 1] = np.dot(np.array(self.centered_cloud[i]), e2)
        x_min = min(pts2d[:, 0])
        x_max = max(pts2d[:, 0])
        y_min = min(pts2d[:, 1])
        y_max = max(pts2d[:, 1])
        det_x = (x_max - x_min) / (x_num - 0.01)
        det_y = (y_max - y_min) / (y_num - 0.01)
        hist = np.zeros((x_num, y_num))
        for i in pts2d:
            hist[int((i[0] - x_min) / det_x), int((i[1] - y_min) / det_y)] += 1
        if feature != None:
            feature.append(hist)
        return np.resize(hist,(1,hist.shape[0]*hist.shape[1])).tolist()


if __name__ == '__main__':
    for car in pedestrian_paths[0:100:10]:
        cloud = pcl.load(car)
        a = pedestrianFeature(cloud)
        print(len(a.extractFeature()))
