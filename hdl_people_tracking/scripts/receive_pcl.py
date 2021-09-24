#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math

import pcl
import rospy
import geometry_msgs.msg._Pose
from rospy.timer import sleep
import std_msgs.msg
import visualization_msgs.msg
import tensorflow as tf
import extractFeature as eff
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from hdl_people_tracking.msg import *
import os
import threading
from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)


model_file_path = ""
model_shape = [1, 8,8, 4]
z_thre = 2.0
z_m_thre = -1.5
r_thre = 5.0
ped_thre = 0.65
rospy.get_param
class receive_pcl():

    def __init__(self, model_path=None):
        # self.test_pcd = pcl.load()
        # rospy.init_node("person_publisher", anonymous=False)
        temp_time = rospy.Time.now()
        pcl_rev = rospy.Subscriber("/pubClusters", myClusters, callback=self.handle_pcl)
        flag_rev = rospy.Subscriber("/record_time_python", std_msgs.msg.Bool, callback=self.handle_time)

        pub_markers = rospy.Publisher("Pedestrain", visualization_msgs.msg.MarkerArray, queue_size=1000)
        pub_cluster = rospy.Publisher("Clusters", PointCloud2, queue_size=1000)
        pub_marker = rospy.Publisher("Marker_Pub", visualization_msgs.msg.Marker, latch = True,queue_size=1000)
        pub_myClusters = rospy.Publisher("myClusters", ClusterArray, queue_size=1000)
        self.pub_markers = pub_markers
        self.pub_marker = pub_marker
        self.pub_cluster = pub_cluster
        # self.model = tf.keras.models.load_model(model_path)

        self.model = tf.keras.models.load_model(model_path)
        # self.model.load_weights("/home/chen/tracking_ws/src/hdl_people_tracking/scripts/mymodel7W.h5")
        test_feature_0 = np.array([np.zeros((1,8, 8, 4))]).astype(np.float64)
        test_feature_1 = np.array([np.zeros((145,1))]).astype(np.float64)
        self.model.predict({"input1": test_feature_0, "input2": test_feature_1})
        # self.model.
        # tf.global_variables_initializer()
        # tf.local_variables_initializer()
        self.cluster = None
        self.frame_id = None
        self.timeStamp = None
        self.markers = MarkerArray()
        self.rev_num = 0
        self.pub_myClusters = pub_myClusters
        self.myClusters = ClusterArray()
        self.myClusters.header.frame_id = "velodyne"
        self.id_num = 0
        self.time_list = []
        self.total_cluster_num = 0
        self.max_dis = [10.0]
        print("receive_pcl finished initialize!!!")

    def create_model(self):
        IMAGE_ORDERING = "channels_first"
        input1 = tf.keras.Input(shape=(1, 8, 8, 16), name="input1")
        # cov1
        conv_1 = tf.keras.layers.Conv3D(filters=4, kernel_size=(3, 3, 3), padding='same', activation='relu',
                                        name="CONV3D_1",
                                        dilation_rate=(2, 2, 2), data_format=IMAGE_ORDERING)(input1)
        maxpool_1 = tf.keras.layers.MaxPool3D(name="MAXPOOL3D_1", data_format=IMAGE_ORDERING)(conv_1)

        conv_2 = tf.keras.layers.Conv3D(filters=32, kernel_size=(3, 3, 3), padding='same', activation='relu',
                                        name="CONV3D_2",
                                        dilation_rate=(2, 2, 2), data_format=IMAGE_ORDERING)(maxpool_1)
        maxpool_2 = tf.keras.layers.MaxPool3D(name="MAXPOOL3D_2", data_format=IMAGE_ORDERING)(conv_2)

        conv_3 = tf.keras.layers.Conv3D(filters=32, kernel_size=(3, 3, 3), padding='same', activation='relu',
                                        name="CONV3D_3",
                                        dilation_rate=(2, 2, 2), data_format=IMAGE_ORDERING)(maxpool_2)

        convt_1 = tf.keras.layers.Conv3DTranspose(16, kernel_size=(2, 2, 2), strides=(2, 2, 2), name="CONV3DT_1",
                                                  activation='relu',
                                                  data_format=IMAGE_ORDERING)(conv_3)
        concat_1 = tf.keras.layers.Concatenate(axis=1)([convt_1, conv_2])
        conv_4 = tf.keras.layers.Conv3D(filters=16, kernel_size=(3, 3, 3), padding='same', activation='relu',
                                        name="CONV3D_4",
                                        data_format=IMAGE_ORDERING)(concat_1)
        convt_2 = tf.keras.layers.Conv3DTranspose(4, kernel_size=(2, 2, 2), strides=(2, 2, 2), name="CONV3DT_2",
                                                  activation='relu',
                                                  data_format=IMAGE_ORDERING)(conv_4)
        concat_2 = tf.keras.layers.Concatenate(axis=1)([convt_2, conv_1])

        conv_5 = tf.keras.layers.Conv3D(filters=1, kernel_size=(3, 3, 3), padding='same', activation='sigmoid',
                                        name="CONV3D_5",
                                        data_format=IMAGE_ORDERING)(concat_2)
        out = tf.keras.layers.Flatten()(conv_5)
        out = tf.keras.layers.Dense(512, activation="relu", name="fullc-1")(out)
        # out = tf.keras.layers.Dense(2,activation="softmax",name="fullc-2")(out)
        # model = tf.keras.models.Sequential([
        #     tf.keras.layers.Dense(300, activation='relu', input_shape=(145,)),
        #     tf.keras.layers.Dropout(0.2),
        #     tf.keras.layers.Dense(2, activation='softmax')
        # ])
        # python3
        # input2 = tf.keras.layers.Input(shape=(145,),name="input2")
        # fullc_2d_1 = tf.keras.layers.Dense(512,activation="relu",name="fullc_2d_1")(f_input)
        # # fullc_2d_2 = tf.keras.layers.Dense(2, activation="softmax",name="fullc_2d_2")(fullc_2d_1)
        # x = tf.keras.layers.concatenate([out,fullc_2d_1])
        # final_out = tf.keras.layers.Dense(2, activation="softmax",name="output")(x)
        # python2
        input2 = tf.keras.layers.Input(shape=(145, 1), name="input2")
        f_input = tf.keras.layers.Flatten()(input2)
        fullc_2d_1 = tf.keras.layers.Dense(512, activation="relu", name="fullc_2d_1")(f_input)
        # maxpool_2d_1 = tf.keras.layers.MaxPool1D(name="MAXPOOL1D_1")(fullc_2d_1)
        fullc_2d_2 = tf.keras.layers.Dense(256, activation="softmax", name="fullc_2d_2")(fullc_2d_1)
        x = tf.keras.layers.concatenate([out, fullc_2d_2])
        final_out = tf.keras.layers.Dense(2, activation="softmax", name="output")(x)

        # cov_2d_1 = tf.keras.layers.Conv2D()(input2)
        # max_pool_2d = tf.keras.layers.MaxPooling2D()(cov_2d_1)

        model = tf.keras.Model(inputs=[input1, input2], outputs=[final_out])
        model.summary()
        # tf.keras.utils.plot_model(model, "./image/my_model.png", show_shapes=True)
        return model

    class myCluster():
        def __init__(self, x, y, z, xx, yy, zz):
            self.x = x
            self.y = y
            self.z = z
            self.xx = xx
            self.yy = yy
            self.zz = zz

    def make_cluster(self, x, y, z, xx, yy, zz):
        return self.myCluster(x, y, z, xx, yy, zz)

    def make_cluster_from_list(self, cluster_msg):
        return self.myCluster(cluster_msg[0], cluster_msg[1], cluster_msg[2], cluster_msg[3], cluster_msg[4],
                              cluster_msg[5])

    def handle_pcl(self, msg):
        self.markers = MarkerArray()
        self.timeStamp = msg.Header.stamp

        self.frame_id = msg.Header.frame_id
        # print("Clusters len:{}".format(len(msg.clusters)))
        begin_time = rospy.Time.now()
        print("begin_time is :{}".format(begin_time.to_sec()))
        print("Time stamp is :{}".format(msg.Header.stamp.to_sec()))
        if(math.fabs(begin_time.to_sec()-msg.Header.stamp.to_sec()>0.25)):
            print("Time correction!!!!!!")
            return False
        for cluster in msg.clusters:
            self.handle_pointcloud(cluster)
            self.total_cluster_num +=1
            print ("total_cluster_num is :{}".format(self.total_cluster_num))
        # end_time = rospy.Time.now().nsecs
        # self.time_list.append(end_time-begin_time)
        if self.pub_markers.get_num_connections():

            # print("{} time used!!".format((end_time - begin_time)))

            if self.markers.markers != []:
                # self.pub_markers.publish(self.markers)
                self.id_num = 0
        self.pub_myClusters.publish(self.myClusters)
        # print(len(self.myClusters.clusters))
        self.myClusters.header.stamp = self.timeStamp
        self.myClusters.clusters = []

    def handle_pointcloud(self, cloud):
        pc = pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
        pc_list = []
        for p in pc:
            pc_list.append([p[0], p[1], p[2]])
        temp_cloud = pcl.PointCloud()
        temp_cloud.from_list(pc_list)
        self.FeaturePredict(temp_cloud)


    def person_publisher(self):

        pass

    def drawMarker(self, myCluster):
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = self.timeStamp
        marker.header.frame_id = "velodyne"
        marker.type = visualization_msgs.msg.Marker.CUBE
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.lifetime = rospy.Duration(2)

        marker.pose.position.x = myCluster.x
        marker.pose.position.y = myCluster.y
        marker.pose.position.z = myCluster.z
        marker.pose.orientation.w = 1.0

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.4

        marker.scale.x = myCluster.xx
        marker.scale.y = myCluster.yy
        marker.scale.z = myCluster.zz
        marker.id = self.id_num
        self.id_num += 1
        self.markers.markers.append(marker)


        self.pub_markers.publish(self.markers)


    def FeaturePredict(self, temp_cloud):
        eff_obj = eff.pedestrianFeature(temp_cloud)
        dist = math.sqrt(eff_obj.data_mean[0] ** 2 + eff_obj.data_mean[1] ** 2)
        if dist>self.max_dis[-1]:
            self.max_dis.append(dist)
        print("Max distance :{}".format(self.max_dis))
        # Set some threshold to accelerate the procedure
        if eff_obj.data_mean[2] < z_thre and eff_obj.data_mean[2]>z_m_thre and dist < r_thre:
            test_feature_0_ori = [eff_obj.grid_feature(model_shape[0], model_shape[1], model_shape[2], model_shape[3])]
            test_feature_0 = np.array(test_feature_0_ori)
            test_feature_1_ori = eff_obj.extractFeature()
            test_feature_1 = np.array(test_feature_1_ori)

            # test_feature_1 = np.array([ef_obj])
            test_feature_1 = np.resize(test_feature_1, (1, test_feature_1.shape[0], 1))

            temp_feature = [test_feature_0, test_feature_1]
            begin_time = rospy.Time.now().nsecs
            # global graph


            predict = self.model.predict(temp_feature)[0]
                # predict = self.model.predict({"input1": test_feature_0, "input2": test_feature_1})[0]
            # print("Predict:{}".format(predict))
            end_time = rospy.Time.now().nsecs
            # print ("Time of Prediction:{}".format(end_time-begin_time))

            temp_cluster_feature = self.make_cluster_from_list(eff_obj.cluster_msg)
            # self.drawMarker(temp_cluster_feature)
            if predict[1] > predict[0] and predict[1] > ped_thre:
                self.drawMarker(temp_cluster_feature)
                temp_cluster = Cluster()
                temp_cluster.feature = test_feature_1_ori

                temp_cluster.is_human = True
                temp_cluster.min_pt = eff_obj.min_pt
                temp_cluster.max_pt = eff_obj.max_pt
                temp_cluster.size.x = temp_cluster_feature.xx
                temp_cluster.size.y = temp_cluster_feature.yy
                temp_cluster.size.z = temp_cluster_feature.zz
                temp_cluster.centroid.x = temp_cluster_feature.x
                temp_cluster.centroid.y = temp_cluster_feature.y
                temp_cluster.centroid.z = temp_cluster_feature.z
                self.myClusters.clusters.append(temp_cluster)
                self.pub_markers.publish(self.markers)
                # print("add clusters")
                return True
            else:
                return False
        else:
            print ("Cluster is out of range!!!!!!!!")
            return False

    def init_Clusters(self):
        self.myClusters = ClusterArray()
        self.myClusters.header.frame_id = "velodyne"
        self.myClusters.header.stamp = self.timeStamp

    def handle_time(self,msg):
        # print("Saving Python Result!!!!")
        with open("result_python.txt",'a') as file:
            for i in self.time_list:
                print(i)
                file.write(str(i)+' ')
            file.close()
        os.system("pwd")

        

        


def main():
    # model = create_model()
    model_path = "/home/chen/mytracking_ws/src/85/hdl_people_tracking/model/mymodel4.h5"
    rospy.init_node("person_publisher", anonymous=False)

    receive_pcl(model_path)
    print("receive_pcl finished initialize!!!")

    rospy.spin()


if __name__ == "__main__":
    main()
