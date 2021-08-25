import numpy as np
import pcl
import tensorflow as tf
import extractF as ef
import extractFeature as eff
import glob
import numpy as np
from sklearn.metrics import confusion_matrix
from sklearn.metrics import accuracy_score
import os

os.environ['TF_FORCE_GPU_ALLOW_GROWTH'] = 'true'
physical_devices = tf.config.experimental.list_physical_devices('GPU')
if len(physical_devices) > 0:
    tf.config.experimental.set_memory_growth(physical_devices[0], True)

gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)

    except RuntimeError as e:
        print(e)

car_pcd = glob.glob('car/*')
pedestrian = glob.glob('pedestrian/*')
mnist = tf.keras.datasets.mnist
(x_train, y_train), (x_test, y_test) = mnist.load_data()
x_train, x_test = x_train / 255.0, x_test / 255.0


# physical_devices = tf.config.experimental.list_physical_devices('GPU')
# assert len(physical_devices) > 0, "Not enough GPU hardware devices available"
# tf.config.experimental.set_memory_growth(physical_devices[0], True)


def evaluate_model(model, x_train, y_train, x_test, y_test):
    model.compile(optimizer='adam',
                  loss='sparse_categorical_crossentropy',
                  metrics=['accuracy'])
    print(y_train.shape)
    # print(x_train)
    model.fit({"input1":x_train[0],"input2":x_train[1]}, {"output":y_train}, epochs=20)
    # print(x_test)
    # print(y_train)
    checkpoint_dir = "./models"
    checkpoint_prefix = os.path.join(checkpoint_dir,"ckpt")
    checkpoint = tf.train.Checkpoint()
    checkpoint.save(file_prefix=checkpoint_prefix)
    predicts = model.predict(x_test)
    model.evaluate(x_test, y_test)
    result = []
    for i in predicts:
        if i[0] > i[1]:
            result.append(0)
        else:
            result.append(1)
    print(confusion_matrix(y_test, result))
    print(accuracy_score(y_test, result))


def model_thresholding():
    IMAGE_ORDERING = "channels_first"
    input1 = tf.keras.Input(shape=(1, 16, 16, 8),name="input1")

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
    out = tf.keras.layers.Dense(512,activation="relu",name="fullc-1")(out)
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
    input2 = tf.keras.layers.Input(shape=(145,1),name="input2")
    f_input = tf.keras.layers.Flatten()(input2)
    fullc_2d_1 = tf.keras.layers.Dense(512,activation="relu",name="fullc_2d_1")(f_input)
    
    fullc_2d_2 = tf.keras.layers.Dense(256, activation="softmax",name="fullc_2d_2")(fullc_2d_1)
    x = tf.keras.layers.concatenate([out,fullc_2d_1])
    final_out = tf.keras.layers.Dense(2, activation="softmax",name="output")(x)

    # cov_2d_1 = tf.keras.layers.Conv2D()(input2)
    # max_pool_2d = tf.keras.layers.MaxPooling2D()(cov_2d_1)




    model = tf.keras.Model(inputs=[input1,input2], outputs=[final_out])
    model.summary()
    tf.keras.utils.plot_model(model, "./image/my_model.png", show_shapes=True)
    return model


if __name__ == "__main__":

    x_train = []
    y_train = []
    x_test = []
    y_test = []
    Conv3d_Input = []
    x_conv_3d_train = []
    y_conv_3d_train = []
    x_conv_3d_test = []
    y_conv_3d_test = []
    # ef.pedestrianFeature()
    # print(ef.extractFeature(car_pcd[1]))
    for path in car_pcd[50:]:
        temp_cloud = pcl.load(path)
        x_train.append(ef.extractFeature(temp_cloud))
        y_train.append(0)
        x_conv_3d_train.append(eff.pedestrianFeature(temp_cloud).grid_feature())
        y_conv_3d_train.append(0)
    for path in car_pcd[:50]:
        temp_cloud = pcl.load(path)
        x_test.append(ef.extractFeature(temp_cloud))
        y_test.append(0)
        x_conv_3d_test.append(eff.pedestrianFeature(temp_cloud).grid_feature())
        y_conv_3d_test.append(0)
    for path in pedestrian[3000:]:
        temp_cloud = pcl.load(path)
        x_train.append(ef.extractFeature(temp_cloud))
        y_train.append(1)
        x_conv_3d_train.append(eff.pedestrianFeature(temp_cloud).grid_feature())
        y_conv_3d_train.append(1)
    for path in pedestrian[:300]:
        temp_cloud = pcl.load(path)
        x_test.append(ef.extractFeature(temp_cloud))
        y_test.append(1)
        x_conv_3d_test.append(eff.pedestrianFeature(temp_cloud).grid_feature())
        y_conv_3d_test.append(1)
    x_train = np.array(x_train)

    x_test = np.array(x_test)

    x_train = np.resize(x_train,(x_train.shape[0],145,1))
    x_test = np.resize(x_test,(x_test.shape[0],145,1))
    y_train = np.array(y_train)
    y_test = np.array(y_test)

    x_conv_3d_train = np.array(x_conv_3d_train)
    x_conv_3d_test = np.array(x_conv_3d_test)

    y_conv_3d_train = np.array(y_conv_3d_train)
    y_conv_3d_train = np.resize(y_conv_3d_train,(y_conv_3d_train.shape[0],1))
    print(y_conv_3d_train.shape)
    y_conv_3d_test = np.array(y_conv_3d_test)
    y_conv_3d_test = np.resize(y_conv_3d_test,(y_conv_3d_test.shape[0],1))

    # final feature = [conv_3d, 2d]
    x_final_train = [x_conv_3d_train,x_train]
    x_final_test = [x_conv_3d_test,x_test]
    model = model_thresholding()
    print(x_final_train[1].shape)
    evaluate_model(model, x_final_train, y_conv_3d_train, x_final_test, y_conv_3d_test)
    # print(x_train.shape)
    # model = tf.keras.models.Sequential([
    #     tf.keras.layers.Dense(300, activation='relu', input_shape=(145,)),
    #     tf.keras.layers.Dropout(0.2),
    #     tf.keras.layers.Dense(2, activation='softmax')
    # ])
    print(model.predict([eff.pedestrianFeature(temp_cloud).grid_feature(),ef.extractFeature(temp_cloud)]))
    print("finish")
