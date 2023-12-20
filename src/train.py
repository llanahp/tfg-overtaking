import os
import sys

import argparse
from datetime import datetime

from stable_baselines3 import DDPG, PPO,A2C, SAC, DQN
from stable_baselines3.common.monitor import Monitor

from sb3_contrib import RecurrentPPO, TRPO

from config.train_config_one_state import CustomCombinedExtractorOneState, SaveOnBestTrainingRewardCallback
from config.train_config_smarts import CustomCombinedExtractorSmarts
from config.train_config_smarts_multi import CustomCombinedExtractorSmartsMulti
from config.train_config_multi_state import CustomCombinedExtractorMultiState

import importlib


def setUpArgs():
	argparser = argparse.ArgumentParser(
		description='Train Agent')
	argparser.add_argument(
		'--model',
		metavar='NAME',
		default='PPO',
		help='model (default: "PPO")')
	argparser.add_argument(
		'--env',
		metavar='NAME',
		default='overtaking',
		help='env (default: "overtaking")')
	argparser.add_argument(
		'--pretrained',
		metavar='NAME',
		default='no',
		help='pretrained (default: "no")')
	argparser.add_argument(
		'--timesteps',
		metavar='N',
		type=int,
		default=100000000,
		help='timesteps (default: 1000)')
	argparser.add_argument(
		'--verbose',
		metavar='NAME',
		default='False',
		help='verbose (default: "False")')
	argparser.add_argument(
		'--compPretrained',
		metavar='NAME',
		default='no',
		help='encompPretrained (default: "no")')
		
	return argparser.parse_args()

def defineLogs(args):
	log_dir = os.path.join('Training', 'Logs')
	model_dir = os.path.join('Training', 'Models')
	save_path = os.path.join(model_dir, '{}_{}_{}'.format(args.env, args.model, datetime.now()))
	print("----->save_path: ",save_path)
	print(log_dir)
	return log_dir, model_dir, save_path

def createEnv(args, model_dir):
	diccionario = {
    "Highway": "scenarios.Highway.HighwaySumoEnvFeatures",
    "overtakingOneLine": "scenarios.overtakingOneLine.overtakingOneLine",
	}

	if args.env in diccionario:
		# Import the env and hide the output (Zen of Python)
		with open(os.devnull, 'w') as null_file:
			old_stdout = sys.stdout
			sys.stdout = null_file
			CustomEnv = importlib.import_module(diccionario[args.env]).CustomEnv
			sys.stdout = old_stdout
	else:
		print("Env not found")

	if args.env == "Highway":
		if (args.model == "PPO"):
			policy_kwargs = dict(
			features_extractor_class=CustomCombinedExtractorOneState,
			net_arch = dict(pi=[256, 256], vf=[256, 256])
			)
		elif (args.model == "A2C"):
			policy_kwargs = dict(
			features_extractor_class=CustomCombinedExtractorOneState,
			net_arch = dict(pi=[128, 128], vf=[128, 128], qf=[128, 128])
			)
		elif (args.model == "SAC"):
			policy_kwargs = dict(
			features_extractor_class=CustomCombinedExtractorOneState,
			net_arch = dict(pi=[128, 128], vf=[128, 128], qf=[128, 128])
			)
		elif (args.model == "DQN"):
			policy_kwargs = dict(
			features_extractor_class=CustomCombinedExtractorOneState,
			net_arch = [64, 128, 64]
			)
		elif (args.model == "TRPO"):
			policy_kwargs = dict(
			features_extractor_class=CustomCombinedExtractorOneState,
			net_arch = dict(pi=[512, 512], vf=[512, 512])
			)
		policy = "MultiInputPolicy"
	elif args.env == "overtakingOneLine":
		if (args.model == "PPO"):
			policy_kwargs = dict(
			features_extractor_class=CustomCombinedExtractorOneState,
			net_arch = dict(pi=[256, 256], vf=[256, 256])
			)
		elif (args.model == "A2C"):
			policy_kwargs = dict(
			features_extractor_class=CustomCombinedExtractorOneState,
			net_arch = dict(pi=[256, 256], vf=[256, 256])
			)
		elif (args.model == "SAC"):
			policy_kwargs = dict(
			features_extractor_class=CustomCombinedExtractorOneState,
			net_arch = dict(pi=[128, 128], vf=[128, 128], qf=[128, 128])
			)
		elif (args.model == "DQN"):
			policy_kwargs = dict(
			features_extractor_class=CustomCombinedExtractorOneState,
			net_arch = [256, 256]
			)
		elif (args.model == "TRPO"):
			policy_kwargs = dict(
			features_extractor_class=CustomCombinedExtractorOneState,
			net_arch = dict(pi=[256, 256], vf=[256, 256])
			)
		policy = "MultiInputPolicy"
	
	env = CustomEnv(render=False)
	env = Monitor(env, model_dir)
	return policy, policy_kwargs, env

def info(args):
	print("\n----->Model:",args.model)
	print("----->Network: ",args.env)
	print("----->Pretrained: ",args.pretrained)
	print("----->CompPretrained: ",args.compPretrained)
	print()

def createModel(args, policy, policy_kwargs, env, log_dir, model_dir):
	if args.model == "PPO":
		if args.pretrained == "no":
			model = PPO(policy, env, verbose = 1, policy_kwargs=policy_kwargs, tensorboard_log=log_dir)
		elif args.pretrained != "no":
			model = PPO(policy, env, verbose = 1, policy_kwargs=policy_kwargs, tensorboard_log=log_dir)
		elif args.compPretrained != "no":
			path = args.compPretrained
			print("pretrained: ",path)
			model = PPO.load(path, env=env)
		else:
			path = os.path.join(model_dir, '{}_{}_'.format(args.env, args.model)) + args.pretrained
			print("pretrained: ",path)
			model = PPO.load(path, env=env)

	elif args.model == "A2C":
		if args.pretrained == "no":
			model = A2C(policy, env, verbose = 1, policy_kwargs=policy_kwargs, tensorboard_log=log_dir)
		elif args.pretrained != "no":
			model = A2C(policy, env, verbose = 1, policy_kwargs=policy_kwargs, tensorboard_log=log_dir)
		elif args.compPretrained != "no":
			path = args.compPretrained
			print("pretrained: ",path)
			model = A2C.load(path, env=env)
		else:
			path = os.path.join(model_dir, '{}_{}_'.format(args.env, args.model)) + args.pretrained
			print("pretrained: ",path)
			model = A2C.load(path, env=env)
	
	elif args.model == "SAC":
		if args.pretrained == "no":
			model = SAC(policy, env, verbose = 1, policy_kwargs=policy_kwargs, tensorboard_log=log_dir)
		elif args.pretrained != "no":
			model = SAC(policy, env, verbose = 1, policy_kwargs=policy_kwargs, tensorboard_log=log_dir)
		elif args.compPretrained != "no":
			path = args.compPretrained
			print("pretrained: ",path)
			model = SAC.load(path, env=env)
		else:
			path = os.path.join(model_dir, '{}_{}_'.format(args.env, args.model)) + args.pretrained
			print("pretrained: ",path)
			model = SAC.load(path, env=env)

	elif args.model == "DQN":
		if args.pretrained == "no":
			model = DQN(policy, env, verbose = 1, policy_kwargs=policy_kwargs, tensorboard_log=log_dir)
		elif args.pretrained != "no":
			model = DQN(policy, env, verbose = 1, policy_kwargs=policy_kwargs, tensorboard_log=log_dir,
			learning_rate=0.01,
			gamma=0.95,
			batch_size=32,
			)
		elif args.compPretrained != "no":
			path = args.compPretrained
			print("pretrained: ",path)
			model = DQN.load(path, env=env)
		else:
			path = os.path.join(model_dir, '{}_{}_'.format(args.env, args.model)) + args.pretrained
			print("pretrained: ",path)
			model = DQN.load(path, env=env)

	elif args.model == "TRPO":
		if args.pretrained == "no":
			model = TRPO(policy, env, verbose = 1, policy_kwargs=policy_kwargs, tensorboard_log=log_dir)
		elif args.pretrained != "no":
			model = TRPO(policy, env, verbose = 1, policy_kwargs=policy_kwargs, tensorboard_log=log_dir)
			print("pretrained--->", args.pretrained)
		elif args.compPretrained != "no":
			path = os.path.join(model_dir,args.compPretrained)
			print("compPretrained: ",path)
			model = TRPO.load(path, env=env)
		else:
			path = os.path.join(model_dir, '{}_{}_'.format(args.env, args.model)) + args.pretrained
			print("pretrained: ",path)
			model = TRPO.load(path, env=env)

	return model
		
def main():
	args = setUpArgs()

	log_dir, model_dir, save_path = defineLogs(args)

	policy, policy_kwargs, env = createEnv(args, model_dir)

	# Callback definition
	callback = SaveOnBestTrainingRewardCallback(check_freq=20, log_dir=model_dir, save_path=save_path)

	info(args)

	model = createModel(args, policy, policy_kwargs, env, log_dir, model_dir)

	if	args.verbose == "True":
		print("Model Policy:\n",model.policy)
	
	model.learn(total_timesteps=args.timesteps, callback=callback)
	model.save(os.path.join(model_dir, 'last'))

if __name__ == "__main__":
	main()
