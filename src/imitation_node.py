#!/usr/bin/env python3

import pandas as pd
import numpy as np
import imitation
from imitation.data import types
from imitation.algorithms import bc
import os
import pathlib
import gymnasium as gym

class imitation_node:
    def __init__(self, load=False) -> None:
        self.obs_space= gym.spaces.Box(low=-1.0, high=1.0, shape=(7,), dtype=np.float32)
        self.act_space = gym.spaces.Box(low=-1.5, high=1.5, shape=(7,), dtype=np.float32)
        self.trajectories = self.create_trajectories()
        rng = np.random.default_rng()
        self.bc_trainer = bc.BC(
            observation_space=self.obs_space,
            action_space=self.act_space,
            demonstrations=self.trajectories,
            rng=rng,
            policy= self.load_model() if load else None
        )
        
        if not load:
            # rospy.loginfo("Training...")
            self.bc_trainer.train(n_epochs=50) # more epochs = better, generally
            #save model
            self.bc_trainer.save_policy("bc_policy")
            # rospy.loginfo("Done training.")

    # create trajectories from raw data to train on
    def create_trajectories(self):
        trajectories = []
        for folder in pathlib.Path("raw_data", "reach").iterdir():
            if folder.is_dir():
                for file in folder.iterdir():
                    # if file is csv
                    if file.suffix == ".csv":
                        print(file)
                        df = pd.read_csv(file, header=0)
                        df = df.drop(columns="position")
                        df = df.drop(columns="orientation")

                        # clip beginning and end so only the actual trajectory is left
                        # while first row == second row, drop first row, ignore first column
                        while df.iloc[0,1:].equals(df.iloc[1,1:]):
                            df = df.drop(df.index[0])
                        # while last row == second last row, drop last row, ignore first column
                        while df.iloc[-1,1:].equals(df.iloc[-2,1:]):
                            df = df.drop(df.index[-1])

                        df.drop(columns="rosbagTimestamp", inplace=True)

                        obs = df.to_numpy()
                        acts = np.zeros((obs.shape[0]-1, obs.shape[1]))
                        for i in range(obs.shape[0]-1):
                            acts[i,:] = obs[i+1,:] # actions = next position
                            # acts[i,:] = obs[i+1,:] - obs[i,:] # actions = velocity = change in position

                        infos = np.full((obs.shape[0], 1), {})
                        dones = np.full((obs.shape[0],), False)
                        dones[-1] = True

                        traj = types.Trajectory(obs, acts, infos = None, terminal=True)
                        trajectories.append(traj)
        return trajectories
    
    def step(self, obs):
        return self.bc_trainer.policy.predict(obs, deterministic=True)
    
    def predict_pose_cb(self, req):
        # repackage obs into correct format here
        act, state = self.step(req.observation)

    def load_model(self):
        return imitation.algorithms.bc.reconstruct_policy("bc_policy")
    
# provide obs as a numpy array
def query_next_pose(obs):
    n = imitation_node(load=True)
    return n.step(obs)
    
if __name__ == "__main__":
    # rospy.init_node("imitation_node")
    n = imitation_node(load=True)

    # rospy.spin()

    ###
    ### TESTING / SANITY CHECKING
    ###

    for i in range(len(n.trajectories)):
    # i = 1
        test = np.array(n.trajectories[i].obs[0,:])
        expect = np.array(n.trajectories[i].acts[0,:])
        print("prediction")
        act, state = n.step(test)
        np.set_printoptions(suppress=True)
        # print(act)
        # print(expect)
        print(act-expect)






