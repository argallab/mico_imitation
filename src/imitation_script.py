#!/usr/bin/env python3
'''
Adapted from
    Train an Agent Using Behavior Cloning
    https://imitation.readthedocs.io/en/latest/tutorials/1_train_bc.html
'''
import pandas as pd
import numpy as np
import imitation
from imitation.data import types
from imitation.algorithms import bc
import pathlib
import gymnasium as gym

class imitation_class:
    def __init__(self, load=False, prefix="", position=False, policy="bc_policy") -> None:
        self.obs_space= gym.spaces.Box(low=-1.0, high=1.0, shape=(7,), dtype=np.float32)
        self.act_space = gym.spaces.Box(low=-1.5, high=1.5, shape=(7,), dtype=np.float32)

        if position:
            self.obs_space= gym.spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
            self.act_space = gym.spaces.Box(low=-1.5, high=1.5, shape=(3,), dtype=np.float32)

        self.trajectories = self.create_trajectories(prefix, position)
        rng = np.random.default_rng()
        self.bc_trainer = bc.BC(
            observation_space=self.obs_space,
            action_space=self.act_space,
            demonstrations=self.trajectories,
            rng=rng,
            policy= self.load_model(pathlib.Path("policy", policy)) if load else None
        )

        if not load:
            self.bc_trainer.train(n_epochs=50)
            #save model
            self.bc_trainer.save_policy(pathlib.Path("policy", policy))

    # create trajectories from raw data to train on
    def create_trajectories(self, prefix, position=False):
        if prefix=="": prefix="reach"
        trajectories = []
        for folder in pathlib.Path("raw_data", prefix).iterdir():
            if folder.is_dir():
                for file in folder.iterdir():
                    # if file is csv
                    if file.suffix == ".csv":
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

                        if position: # drop orientation columns
                            df.drop(columns=["x.1", "y.1", "z.1", "w"], inplace=True)

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

    def load_model(self, policy="bc_policy"):
        policy_path = pathlib.Path("policy", policy)
        return imitation.algorithms.bc.reconstruct_policy(str(policy_path))

# provide obs as a numpy array
def query_next_pose(obs=None, position=False, policy="bc_policy"):
    n = imitation_class(load=True, position=position, policy=policy)

    return n.step(obs)

if __name__ == "__main__":
    n = imitation_class(load=False, prefix="fall", position=True, policy="fall")
