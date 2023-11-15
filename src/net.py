
import torch as th 
from torch import nn
import numpy as np
from gym.spaces import Discrete, Box, Dict

# class GetLSTMOutput(nn.Module):
#     "Return the final hidden state for each element in the sequence"
#     def forward(self, x):
#         _ , out = x
#         return out[0]

tensor_list = []

features = 1
adversaries = 4
n_states = 30

observation_space = Dict({'d1':Box(0,1,shape=(n_states, features)), \
                          'd2':Box(0,1,shape=(n_states, features)), \
                          'd3':Box(0,1,shape=(n_states, features)), \
                          'd4':Box(0,1,shape=(n_states, features))})
observation_space["d1"] = np.ones((n_states, features))
subspace = observation_space['d1']
npsubspace = np.array(subspace)
# print(npsubspace)
npsubspace[0]=[2]
# print("1era entrada: ", npsubspace)
npsubspace = npsubspace[([n_states-1] + list(range(n_states-1))), :]
npsubspace[0]=[3]
# print("2a entrada: ", npsubspace)
npsubspace = npsubspace[([n_states-1] + list(range(n_states-1))), :]
npsubspace[0]=[4]
# print("3a entrada: ", npsubspace)

num_layers = 1
hidden_states = 3

total_concat_size = 0

extractors = {}
print(subspace.shape)

for key, subspace in observation_space.spaces.items():
        # Conv and relu
        extractors[key] = nn.LSTM(input_size=subspace.shape[1], hidden_size=subspace.shape[1]*hidden_states, num_layers=num_layers, batch_first=True)
        total_concat_size += hidden_states

encoded_tensor_list = []

h0 = th.randn(1, 1, 3)
c0 = th.randn(1, 1, 3)

for key, extractor in extractors.items():
    tensor = th.rand(1,n_states,features)
    print(tensor.size())
    # tensor = th.permute(tensor,(0,2,1))
    # print(tensor.size())
    tensor,  (hn, cn) = extractor(tensor, (h0, c0))
    print(tensor.size())
    encoded_tensor_list.append(tensor)
# Return a (B, self._features_dim) PyTorch tensor, where B is batch dimension.

print(th.cat(encoded_tensor_list, dim=1).size())