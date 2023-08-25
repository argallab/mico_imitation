from flask import Flask, request
import json
import numpy as np

# import imitation_class
import imitation_node
from imitation_node import imitation_class
# import pathlib

app = Flask(__name__)

@app.route('/')
def hello_world():
    a = imitation_node.query_next_pose()[0]
    result = {"x": str(a[0]),
    "y": str(a[1]),
    "z": str(a[2]),
    "x1": str(a[3]),
    "y1": str(a[4]),
    "z1": str(a[5]),
    "w": str(a[6])}
    return json.dumps(result)

@app.route('/query', methods=["PUT"])
def query():
    data = request.json['data']
    policy = request.json['policy']
    obs = np.array([float(data['x']), float(data['y']), float(data['z']), float(data['x1']), float(data['y1']), float(data ['z1']), float(data['w'])])

    a = imitation_node.query_next_pose(obs, policy=policy)[0]
    result = {"x": str(a[0]),
    "y": str(a[1]),
    "z": str(a[2]),
    "x1": str(a[3]),
    "y1": str(a[4]),
    "z1": str(a[5]),
    "w": str(a[6])}
    return json.dumps(result)

@app.route('/query_position', methods=["PUT"])
def query_position():
    data = request.json['data']
    policy = request.json['policy']
    obs = np.array([float(data['x']), float(data['y']), float(data['z'])])

    a = imitation_node.query_next_pose(obs, position=True, policy=policy)[0]
    result = {"x": str(a[0]),
    "y": str(a[1]),
    "z": str(a[2])}
    return json.dumps(result)

@app.route('/start_pose', methods=["GET"])
def get_start_pose():
    data = request.json['data']
    policy = str(data['policy'])
    traj = int(data['traj'])

    n = imitation_class(load=True, position=False, policy=policy)
    a = n.trajectories[traj].obs[0,:]
    result = {"x": str(a[0]),
    "y": str(a[1]),
    "z": str(a[2]),
    "x1": str(a[3]),
    "y1": str(a[4]),
    "z1": str(a[5]),
    "w": str(a[6])}
    return json.dumps(result)


