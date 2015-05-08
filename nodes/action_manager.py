#!/usr/bin/env python
import os
from os import listdir
from os.path import isfile, join
from roboflow.srv import GetSavedRoboflowActions, SaveRoboflowAction, GetSavedRoboflowActionsResponse
import rospy

import roslib

roslib.load_manifest('roboflow')


def save_roboflow_action(req):
    data_directory = rospy.get_param('/roboflow/actionsRoot')
    file_extension = rospy.get_param('/roboflow/fileExtension', '.json')
    file_name = data_directory + str(req.action_id) + file_extension
    if not os.path.exists(data_directory):
        os.makedirs(data_directory)
    act_file = open(file_name, 'w')
    act_file.write(req.action_str)
    act_file.close()
    pass


def get_saved_roboflow_actions(dummy):
    actions = []
    data_directory = rospy.get_param('/roboflow/actionsRoot')
    file_extension = rospy.get_param('/roboflow/fileExtension', '.json')
    if not os.path.exists(data_directory):
        return []
    for f in listdir(data_directory):
        file_path = join(data_directory, f)
        if isfile(file_path) and file_path.endswith(file_extension):
            with open(file_path, 'r') as content_file:
                actions.append(content_file.read())
    return GetSavedRoboflowActionsResponse(actions)


if __name__ == '__main__':
    rospy.init_node('roboflow_action_manager')
    rospy.Service('get_saved_roboflow_actions', GetSavedRoboflowActions, get_saved_roboflow_actions)
    rospy.Service('save_roboflow_action', SaveRoboflowAction, save_roboflow_action)
    rospy.spin()
