#!/usr/bin/env python
import requests
import json
import numpy as np

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from mico_ml.srv import QueryML, QueryMLResponse, QueryMLRequest
from mico_ml.srv import QueryMLVel, QueryMLVelResponse, QueryMLVelRequest
from mico_ml.srv import Policy, PolicyResponse, PolicyRequest




url = "http://stephaniiema.pythonanywhere.com/"

class web_client_node():
    def __init__(self):
        # self.pub = rospy.Publisher('next_pose', Pose, queue_size=10)
        self.rate = rospy.Rate(1)
        self.next_position = None
        self.next_quat = None

        self.policy = None

        rospy.Service("web_client_node/get_next_position", QueryMLVel, self.query_position)
        rospy.Service("web_client_node/get_next_pose", QueryMLVel, self.query_pose)
        rospy.Service("web_client_node/get_start_pose", QueryMLVel, self.get_start)
        rospy.Service("web_client_node/set_start_pose", QueryMLVel, self.set_start)
        rospy.Service("web_client_node/get_policy", Policy, self.get_policy)
        rospy.Service("web_client_node/set_policy", Policy, self.set_policy)

    def set_policy(self, policy):
        self.policy = policy.input
        result = PolicyResponse()
        result.output = ""
        return result

    def get_policy(self, policy):
        result = PolicyResponse()
        result.output = self.policy
        return result

    # obs is a 3 elem array
    def query_position(self, queryobs):
        queryobs = queryobs.robot_pose.position
        obs = [queryobs.x, queryobs.y, queryobs.z]
        r = requests.put(url+"query_position", json=self.construct_json_query(obs))
        r = json.loads(r.content) # turns string back into json
        self.next_position = Point(float(r["x"]), float(r["y"]), float(r["z"]))

        result = QueryMLVelResponse()
        result.next.position = self.next_position
        result.next.orientation = self.next_quat # returns previously stored orientation
        return result
    
    def query_pose(self, queryobs):
        queryobs = queryobs.robot_pose
        obs = [queryobs.position.x, queryobs.position.y, queryobs.position.z, queryobs.orientation.x, queryobs.orientation.y, queryobs.orientation.z, queryobs.orientation.w]
        r = requests.put(url + "query", json=self.construct_json_query(obs))
        r = json.loads(r.content) # turns string back into json
        self.next_position = Point(float(r["x"]), float(r["y"]), float(r["z"]))
        self.next_quat = Quaternion(float(r["x1"]), float(r["y1"]), float(r["z1"]), float(r["w"]))

        result = QueryMLVelResponse()
        result.next.position = self.next_position
        result.next.orientation = self.next_quat
        return result

    def get_start(self, pose):
        traj = 1
        body = {"data": {
            "policy": self.policy,
            "traj": traj
        }}
        r = requests.get(url+"start_pose", json=body)
        r = json.loads(r.content)

        result = QueryMLVelResponse()
        result.next.position = Point(float(r["x"]), float(r["y"]), float(r["z"]))
        result.next.orientation = Quaternion(float(r["x1"]), float(r["y1"]), float(r["z1"]), float(r["w"]))
        return result
    
    def set_start(self, pose):
        self.next_position = pose.robot_pose.position
        self.next_quat = pose.robot_pose.orientation

        # echos input values as response
        result = QueryMLVelResponse()
        result.next.position = self.next_position
        result.next.orientation = self.next_quat
        return result
        

    # obs is a 7-elem numpy array
    # returns obs as a json body ready to send
    def construct_json_query(self, obs):
        if self.policy is None:
            raise Exception("no policy set")
        if len(obs) == 7:
            result = {"data": { "x": obs[0],
                            "y": obs[1],
                            "z": obs[2],
                            "x1": obs[3],
                            "y1": obs[4],
                            "z1": obs[5],
                            "w": obs[6]},
                    "policy": self.policy}
        elif len(obs)==3:
            result = {"data": { "x": obs[0],
                           "y": obs[1],
                           "z": obs[2]},
                    "policy": self.policy}
        else:
            raise Exception("invalid obs input to construct json query")
        return result

if __name__ == '__main__':
    rospy.init_node("web_client_node")
    n = web_client_node()
    rospy.spin()
