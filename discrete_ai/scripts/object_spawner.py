#!/usr/bin/env python2

import sys
import os
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    Pose,
    Point,
)

import numpy as np

def gazebo_models_cb(data):
    global box_location 
    box_location = [data.pose[-2].position.x, data.pose[-2].position.y, data.pose[-2].position.z] 


def load_gazebo_models():
    reference_frame = "world"
    model_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../demo_object_models/models/"))

    block_xml = ''
    with open (model_path + "/block/model1.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')

    paper_box_xml = ''
    with open(model_path + "/boxes//paper_box.urdf", "r") as paper_box_file:
        paper_box_xml = paper_box_file.read().replace('\n', '')

    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    #x_v = np.random.normal(-1, 0.20, 1)
    #y_v = np.random.normal(-5.75, 0.10, 1)
 
    # My mod
    x_v = 0.35
    y_v = 0.2
    #z_v = 0.7825
    z_v = 0.65

    # In basket
    x_v = 1.4
    y_v = 0.9
    #z_v = 0.7825
    z_v = 0.6

    x_p = 3.85
    y_p = 0.8
    # z_p = 0.7825
    z_p = 0.79

    block_pose = Pose(position=Point(x=x_v+0.025, y=y_v+0.025, z=z_v))
    paper_box_pose = Pose(position=Point(x=x_p, y=y_p, z=z_p))
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block1", block_xml, "/",
                               block_pose, reference_frame)
        resp2_urdf = spawn_urdf("paper_box", paper_box_xml, "/",
                               paper_box_pose, reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


def delete_gazebo_models():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("block1")
        resp2_delete = delete_model("paper_box")

    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


def main():
    rospy.init_node("object_spawner")
    # Remove models from the scene on shutdown
    # gazebo_model_subscriber = rospy.Subscriber("gazebo/model_states", ModelStates, gazebo_models_cb)
    rospy.on_shutdown(delete_gazebo_models)
    load_gazebo_models()
    while not rospy.is_shutdown():
        rospy.sleep(1.0)
        #print("Box location", box_location)
    return 0


if __name__ == '__main__':
    # box_location = [-100, -100, -100]
    sys.exit(main())
