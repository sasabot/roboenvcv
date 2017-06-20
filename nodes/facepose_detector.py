#!/usr/bin/env python
import struct
import rospy
import rospkg
from std_msgs.msg import *
from geometry_msgs.msg import *
from roboenvcv.msg import *

import chainer
import cv2
import numpy as np
import hyperfacemodel

import threading

header_size = 18

def solve(field, img):
    size = img.shape[0:2]
    img = img.astype(np.float32) / 255.0
    img = cv2.resize(img, (227, 227))
    img = cv2.normalize(img, None, -0.5, 0.5, cv2.NORM_MINMAX)
    imgs = xp.asarray([np.transpose(img, (2, 0, 1))])
    x = chainer.Variable(imgs, volatile=True)
    y = models(x)

    msg = Person()
    msg.sensor_id = struct.unpack('i', bytearray(field[12:14]) + '\x00\x00')[0]
    msg.position3d = Point(struct.unpack('f', bytearray(field[0:4]))[0],
                           struct.unpack('f', bytearray(field[4:8]))[0],
                           struct.unpack('f', bytearray(field[8:12]))[0])
    pose = (y['pose'].data)[0]
    msg.roll = pose[0]
    msg.pitch = pose[1]
    msg.yaw = pose[2]

    pubmutex.acquire()
    pub_result.publish(msg)
    pubmutex.release()


def backthread(data):
    global onrun

    # parse msg
    num_faces = struct.unpack('i', data[0:1] + '\x00\x00\x00')[0]
    if num_faces == 0:
        runmutex.acquire()
        try:
            onrun = False
        finally:
            runmutex.release()
        return
    imgs = [None for x in range(num_faces)]
    rest = [[] for x in range(num_faces)]
    at = 1
    for i in range(0, num_faces):
        width = struct.unpack('i', data[at:at + 2] + '\x00\x00')[0]
        height = struct.unpack('i', data[at + 2:at + 4] + '\x00\x00')[0]
        rest[i][:] = data[at + 4:at + header_size]
        at += header_size
        raw_array = np.fromstring(data[at:at + width * height * 3], np.uint8)
        imgs[i] = np.reshape(raw_array, (height, width, 3))
        at += width * height * 3

    # conduct face analysis
    threads = [None for x in range(len(imgs))]
    for idx, img in enumerate(imgs):
        threads[idx] = threading.Thread(target=solve, args=(rest[idx], img))
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    runmutex.acquire()
    try:
        onrun = False
    finally:
        runmutex.release()

def on_subscribe(msg):
    global onrun

    # run on backthread to avoid delays
    runmutex.acquire()
    try:
        if not onrun:
            onrun = True
            thread = threading.Thread(target=backthread, args=(msg.data,))
            thread.start()
    finally:
        runmutex.release()

if __name__ == '__main__':
    rospy.init_node('face_pose_extractor')

    pub_result = rospy.Publisher('/roboenvcv/personcoordinate/local', Person, queue_size=100)
    pubmutex = threading.Lock()

    rospack = rospkg.RosPack()
    pkgpath = rospack.get_path('roboenvcv')

    models = hyperfacemodel.HyperFaceModel()
    chainer.serializers.load_npz(pkgpath + '/nodes/model_epoch_190', models)

    use_gpu = rospy.get_param('~gpu')

    if use_gpu:
        print 'using gpu'
        chainer.cuda.cudnn_enabled = False
        chainer.cuda.check_cuda_available()
        chainer.cuda.get_device(0).use()
        models.to_gpu()
        xp = chainer.cuda.cupy
    else:
        xp = np

    onrun = False
    runmutex = threading.Lock()

    sub = rospy.Subscriber('/roboenvcv/cropped/images', UInt8MultiArray, on_subscribe)

    rospy.spin()
