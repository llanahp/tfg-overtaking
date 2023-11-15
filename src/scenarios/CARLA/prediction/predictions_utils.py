import numpy as np
import sys
import os
import csv
import pdb

sys.path.append("/home/docker_robesafe/tesis/src/catkin_ws/src/ad_perdevkit/src/perception/gt_generator")

from objects2gt import Objects2GT
from gt_publisher import GTPublisher
from gt2csv import GT2CSV
from store_data import StoreData
from av2.datasets.motion_forecasting.data_schema import ObjectType

sys.path.append("/home/docker_robesafe/argo2goalmp")

from get_prediction_model import get_prediction_model
from plot_qualitative_results_simulation import plot_actor_tracks, plot_predictions, get_object_type
from data import from_numpy

class Motion_Predictor():
    def __init__(self):

        self.OBS_LEN = 50
        self.PRED_LEN = 60
        self.required_variables = 5 # id, obs_num, x, y, padding
        self.TINY_PREDICTION = True
        
        self.THRESHOLD_NUM_OBSERVATIONS = 1 # Minimum number of observations out of self.OBS_LEN (e.g. 20 out of 50),
                                  # to start predicting an agent
        self.NUM_STEPS = 10 # To obtain predictions every n-th STEP
        self.NUM_PREDICTED_POSES = 4 # e.g. t+0, t+STEP, t+2*STEP, t+3*STEP
        
        self.prediction_network = get_prediction_model()

    def preprocess_trackers(self, trajectories):
        """
        """
        
        agents_info_array = np.zeros([0, self.required_variables])

        for key, value in trajectories.items():
            for num_obs in range(self.OBS_LEN):
                agent_info = np.array([key,
                                        num_obs,
                                        value[num_obs,0],
                                        value[num_obs,1],
                                        value[num_obs,2]])

                agents_info_array = np.vstack((agents_info_array, agent_info))

        # Avoid storing full-padded agents
        
        agents_id = np.unique(agents_info_array[:, 0], axis=0)

        valid_agents_info = []
        valid_agents_id = []

        for agent_id in agents_id:
            agent_info = agents_info_array[agents_info_array[:, 0] == agent_id]

            # if not (agent_info[:, -1] == 1).all():  # Avoid storing full-padded agents
            if np.sum(agent_info[:,-1] == 1) >= self.THRESHOLD_NUM_OBSERVATIONS: # Only consider those that have at least 
                                                                      # self.THRESHOLD_STEPS of observations
                valid_agents_info.append(agent_info)
                valid_agents_id.append(int(agent_info[0,0]))
                      
        return valid_agents_info, valid_agents_id

    def predict_agents(self, valid_agents_info, file_id):
        """
        """
             
        # Get agents of the scene (we assume the first one represents our ego-vehicle)
        # (N agents * 50) x 5 (track_id, timestep, x, y, padding)
        # Preprocess agents (relative displacements, orientation, etc.)

        trajs, steps, track_ids, object_types = [], [], [], []
        final_predictions, final_confidences = [], []

        # TODO: Write here the corresponding object type, though at this moment it is not used
        
        object_type = ObjectType.VEHICLE

        # agent_info: 0 = track_id, 1 = timestep, 2 = x, 3 = y, 4 = padding

        for agent_info in valid_agents_info:
            non_padded_agent_info = agent_info[agent_info[:, -1] == 1]
            trajs.append(non_padded_agent_info[:, 2:4])
            steps.append(non_padded_agent_info[:, 1].astype(np.int64))
            
            track_ids.append(non_padded_agent_info[0, 0])
            object_types.append(get_object_type(object_type))

        # Our ego-vehicle is always the first agent of the scenario

        if trajs[0].shape[0] > 1:

            current_step_index = steps[0].tolist().index(self.OBS_LEN-1)
            pre_current_step_index = current_step_index-1

            orig = trajs[0][current_step_index][:2].copy().astype(np.float32)

            curr_pos = trajs[0][current_step_index][:2] - orig
            pre_pos = trajs[0][current_step_index-1][:2] - orig

            theta = np.arctan2(curr_pos[1]-pre_pos[1], curr_pos[0]-pre_pos[0])

            rot = np.asarray([[np.cos(theta), -np.sin(theta)],
                              [np.sin(theta),  np.cos(theta)]], np.float32)

            feats, ctrs, valid_track_ids, valid_object_types = [], [], [], []

            for traj, step, track_id, object_type in zip(trajs, steps, track_ids, object_types):

                if self.OBS_LEN-1 not in step:

                    continue

                valid_track_ids.append(track_id)
                valid_object_types.append(object_type)

                obs_mask = step < self.OBS_LEN
                step = step[obs_mask]
                traj = traj[obs_mask]
                idcs = step.argsort()
                step = step[idcs]
                traj = traj[idcs]
                feat = np.zeros((self.OBS_LEN, 3), np.float32)

                feat[step, :2] = np.matmul(
                    rot, (traj[:, :2] - orig.reshape(-1, 2)).T).T
                feat[step, 2] = 1.0

                ctrs.append(feat[-1, :2].copy())
                feat[1:, :2] -= feat[:-1, :2]

                feat[step[0], :2] = 0
                feats.append(feat)

            feats = np.asarray(feats, np.float32)
            ctrs = np.asarray(ctrs, np.float32)
            data = dict()

            # OBS: Our network must receive a list (batch) per value of the dictionary. In this case, we only want
            # to analyze a single scenario, so the values must be introduced as lists of 1 element, indicating
            # batch_size = 1

            data['scenario_id'] = [file_id]
            data['track_ids'] = [valid_track_ids]
            data['object_types'] = [np.asarray(valid_object_types, np.float32)]
            data['feats'] = [feats]
            data['ctrs'] = [ctrs]
            data['orig'] = [orig]
            data['theta'] = [theta]
            data['rot'] = [rot]

            # Recursively transform numpy.ndarray to torch.Tensor
            data = from_numpy(data)

            output = self.prediction_network(data)

            for agent_index in range(feats.shape[0]):
                agent_mm_pred = output["reg"][0][agent_index,
                                                 :, :, :].cpu().data.numpy()
                agent_cls = output["cls"][0][agent_index, :].cpu().data.numpy()

                if self.TINY_PREDICTION: # Unimodal prediction (60 x 2)
                    most_probable_mode_index = np.argmax(agent_cls)
                    agent_um_pred = agent_mm_pred[most_probable_mode_index, :, :]
                    agent_um_pred = agent_um_pred[::self.NUM_STEPS,:][:self.NUM_PREDICTED_POSES, :]
                    final_predictions.append(agent_um_pred)

                else:
                    final_predictions.append(agent_mm_pred)
                    final_confidences.append(agent_cls)

        return final_predictions, final_confidences

    def write_csv(self, obs, timestamp):
        results_path = f"./scenarios/SMARTS/poses"

        if not os.path.exists(results_path):
            print("Create results path folder: ", results_path)
            os.makedirs(results_path)

        with open(f'{results_path}/poses_{timestamp}.csv', 'w', newline='') as file:
            writer = csv.writer(file, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)

            for key in obs.keys():
                    for num_obs in range(len(obs[key])):
                        writer.writerow([key, 
                                        num_obs, 
                                        obs[key][num_obs][0],
                                        obs[key][num_obs][1],
                                        obs[key][num_obs][2]])