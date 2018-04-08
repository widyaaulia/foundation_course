#!/usr/bin/env python

import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    with open("$GAZEBO_MODEL_PATH/product_0/model.sdf", "r") as f:
        product_xml = f.read()

    orient = Quaternion(tf.transformations.quaternion_from_euler(0,0,0))

    for num in xrange(0,12):
        item_name = "product_{0}_0".format(num)
        print("Deleting model:%s", item_name)
        delete_model(item_name)

    for num in xrange(0,12):
        bin_y   =   2.8 *   (num    /   6)  -   1.4 
        bin_x   =   0.5 *   (num    %   6)  -   1.5
        item_name   =   "product_{0}_0".format(num)
        print("Spawning model:%s", item_name)
        item_pose   =   Pose(Point(x=bin_x, y=bin_y,    z=2),   orient)
        spawn_model(item_name, product_xml, "", item_pose, "world")