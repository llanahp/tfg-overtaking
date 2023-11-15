import gym
import torch as th
from torch import nn

import os
import numpy as np

import argparse
from datetime import datetime

from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.callbacks import BaseCallback

class CustomCombinedExtractorMultiState(BaseFeaturesExtractor):
    def __init__(self, observation_space: gym.spaces.Dict):
        # We do not know features-dim here before going over all the items,
        # so put something dummy for now. PyTorch requires calling
        # nn.Module.__init__ before adding modules
        super(CustomCombinedExtractorMultiState, self).__init__(observation_space, features_dim=1)

        extractors = {}

        total_concat_size = 0
        num_layers = 1
        hidden_states = 3

        n_keys = 0

        # We need to know size of the output of this extractor,
        # so go over all the spaces and compute output feature sizes
        for key, subspace in observation_space.spaces.items():
            # Conv and relu
            extractors[key] = nn.LSTM(input_size=subspace.shape[1], hidden_size=subspace.shape[1]*hidden_states, num_layers=num_layers)
            total_concat_size += subspace.shape[0] * hidden_states * num_layers
            n_keys += 1

        # self.h0 = tuple(th.randn(num_layers, hidden_states).cuda() for _ in range(n_keys))
        self.h0 = [th.randn(num_layers, hidden_states).cuda() for _ in range(n_keys)]
        # self.c0 = tuple(th.randn(num_layers, hidden_states).cuda() for _ in range(n_keys))
        self.c0 = [th.randn(num_layers, hidden_states).cuda() for _ in range(n_keys)]

        self.extractors = nn.ModuleDict(extractors)

        # Update the features dim manually
        # self._features_dim = total_concat_size
        self._features_dim = 4

    def forward(self, observations) -> th.Tensor:
        encoded_tensor_list = []
        n_keys = 0

        # self.extractors contain nn.Modules that do all the processing.
        for key, extractor in self.extractors.items():
            obs = observations[key][0]
            output = extractor(obs, (self.h0[n_keys], self.c0[n_keys]))
            self.h0[n_keys], self.c0[n_keys] = output[1]
            self.h0[n_keys].detach_
            self.h0[n_keys] = self.h0[n_keys].detach()
            self.c0[n_keys].detach_
            self.c0[n_keys] = self.c0[n_keys].detach()
            tensor = output[0]
            tensor = tensor.reshape([tensor.size()[0] * tensor.size()[1]])
            encoded_tensor_list.append(tensor)
            n_keys += 1
        # Return a (B, self._features_dim) PyTorch tensor, where B is batch dimension.
        tensor = th.cat(encoded_tensor_list)
        net_mlp = nn.Linear(in_features=360, out_features=4).cuda()
        tensor = net_mlp(tensor)
        
        # return th.cat(encoded_tensor_list)
        return tensor

class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model (the check is done every ``check_freq`` steps)
    based on the training reward (in practice, we recommend using ``EvalCallback``).

    :param check_freq:
    :param log_dir: Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: Verbosity level.
    """
    def __init__(self, check_freq: int, log_dir: str, save_path: str,verbose: int = 1):
        super(SaveOnBestTrainingRewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = save_path
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        # Create folder if needed
        # if self.save_path is not None:
        #     os.makedirs(self.save_path, exist_ok=True)
        pass

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:

          # Retrieve training reward
          x, y = ts2xy(load_results(self.log_dir), 'timesteps')
          if len(x) > 0:
              # Mean training reward over the last 100 episodes
              mean_reward = np.mean(y[-100:])
            #   if self.verbose > 0:
            #     print(f"Num timesteps: {self.num_timesteps}")
            #     print(f"Best mean reward: {self.best_mean_reward:.2f} - Last mean reward per episode: {mean_reward:.2f}")

              # New best model, you could save the agent here
              if mean_reward > self.best_mean_reward:
                  self.best_mean_reward = mean_reward
                  # Example for saving best model
                  if self.verbose > 0:
                    print(f"Saving new best model to {self.save_path}")
                  self.model.save(self.save_path)

        return True