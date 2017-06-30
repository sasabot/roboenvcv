#!/usr/bin/env python
import chainer
import chainer.functions as F
import chainer.links as L

class HyperFaceModel(chainer.Chain):

    def __init__(self):
        super(HyperFaceModel, self).__init__(
            conv1  = L.Convolution2D(3,   96, 11, stride=4, pad=0),
            conv1a = L.Convolution2D(96,  256, 4, stride=4, pad=0),
            conv2  = L.Convolution2D(96,  256, 5, stride=1, pad=2),
            conv3  = L.Convolution2D(256, 384, 3, stride=1, pad=1),
            conv3a = L.Convolution2D(384, 256, 2, stride=2, pad=0),
            conv4  = L.Convolution2D(384, 384, 3, stride=1, pad=1),
            conv5  = L.Convolution2D(384, 256, 3, stride=1, pad=1),
            conv_all = L.Convolution2D(768, 192, 1, stride=1, pad=0),
            fc_full  = L.Linear(6 * 6 * 192, 3072),
            fc_detection1  = L.Linear(3072, 512),
            fc_detection2  = L.Linear(512,  2),
            fc_landmarks1  = L.Linear(3072, 512),
            fc_landmarks2  = L.Linear(512,  42),
            fc_visibility1 = L.Linear(3072, 512),
            fc_visibility2 = L.Linear(512,  21),
            fc_pose1       = L.Linear(3072, 512),
            fc_pose2       = L.Linear(512,  3),
            fc_gender1     = L.Linear(3072, 512),
            fc_gender2     = L.Linear(512,  2),
        )

    def __call__(self, x):
        c1 = F.relu(self.conv1(x))
        m1 = F.max_pooling_2d(c1, 3, stride=2, pad=0)
        m1_n = F.local_response_normalization(m1)
        c1a = F.relu(self.conv1a(m1_n))
        c2 = F.relu(self.conv2(m1_n))
        m2 = F.max_pooling_2d(c2, 3, stride=2, pad=0)
        m2_n = F.local_response_normalization(m2)
        c3 = F.relu(self.conv3(m2_n))
        c3a = F.relu(self.conv3a(c3))
        c4 = F.relu(self.conv4(c3))
        c5 = F.relu(self.conv5(c4))
        m5 = F.max_pooling_2d(c5, 3, stride=2, pad=0)

        c = F.concat((c1a, c3a, m5))

        c_all = F.relu(self.conv_all(c))
        fc = F.relu(self.fc_full(c_all))

        detection = F.relu(self.fc_detection1(fc))
        detection = self.fc_detection2(detection)
        detection = F.softmax(detection)
        landmark = F.relu(self.fc_landmarks1(fc))
        landmark = self.fc_landmarks2(landmark)
        visibility = F.relu(self.fc_visibility1(fc))
        visibility = self.fc_visibility2(visibility)
        pose = F.relu(self.fc_pose1(fc))
        pose = self.fc_pose2(pose)
        gender = F.relu(self.fc_gender1(fc))
        gender = self.fc_gender2(gender)
        gender = F.softmax(gender)

        return {'detection': detection,
                'landmark': landmark,
                'visibility': visibility,
                'gender': gender,
                'pose': pose}
