import os
import sys
import argparse
import random
from stable_baselines3 import PPO, DDPG, A2C, SAC, DQN
from sb3_contrib import RecurrentPPO
import importlib

def displayResults(success, collision, timeout, t, episodes):
	t = t/episodes
	success = success/episodes
	collision = collision/episodes
	timeout = timeout/episodes
	print("\n-----------------------------")
	print("Results:")
	print("\n-----------------------------")
	print("percentage success: ",success)
	print("percentage collision: ",collision)
	print("percentage timeout: ",timeout)
	print("time", t)
	print("-----------------------------\n")

def setUpArgs():
	argparser = argparse.ArgumentParser(
		description='Test Agent')
	argparser.add_argument(
		'--model',
		metavar='NAME',
		default='PPO',
		help='model (default: "PPO")')
	argparser.add_argument(
		'--scenario',
		metavar='NAME',
		default='',
		help='env (default: "")')
	argparser.add_argument(
		'--env',
		metavar='NAME',
		default='overtaking',
		help='env (default: "overtaking")')
	argparser.add_argument(
		'--eval',
		metavar='NAME',
		default='False',
		help='eval (default: "False")')
	argparser.add_argument(
		'--render',
		metavar='NAME',
		default='True',
		help='render (default: "True")')
	argparser.add_argument(
		'--verbose',
		metavar='NAME',
		default='False',
		help='verbose (default: "False")')
	return argparser.parse_args()

def createEnviroment(args):
	# --env=SOMETHING
	diccionario = {
    "Intersection": "scenarios.Intersection.IntersectionSumoEnv",
    "IntersectionFeaturesOne": "scenarios.Intersection.IntersectionSumoEnvFeaturesOneState",
    "IntersectionFeaturesMulti": "scenarios.Intersection.IntersectionSumoEnvFeaturesMultiState",
    "IntersectionCarlaFeaturesOne": "scenarios.Intersection.IntersectionCarlaEnvFeaturesOneState",
    "IntersectionCarlaFeaturesMulti": "scenarios.Intersection.IntersectionCarlaEnvFeaturesMultiState",
    "Highway": "scenarios.Highway.HighwaySumoEnvFeatures",
    "IntersectionRecurrent": "scenarios.Intersection.IntersectionSumoEnvFeaturesMultiState",
    "SMARTS": "scenarios.SMARTS.smartsEnv",
    "MultiSMARTS": "scenarios.SMARTS.smartsEnvMulti",
    "TransformersSMARTS": "scenarios.SMARTS.smartsEnvTransformers",
    "MultiCARLA": "scenarios.CARLA.carlaEnvMulti",
    "overtaking": "scenarios.overtaking.IntersectionSumoEnvFeaturesOneState",
	"overtakingLine": "scenarios.overtakingLine.IntersectionSumoEnvFeaturesOneState",
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


	env = CustomEnv(render=True)
	if args.render == "False":
		env = CustomEnv(render=False)
	return env

def defineModel(args, env):
	if args.eval == "False":
		# Define the model paths
		Path = os.path.join('Training', 'Models', '{}_{}_{}'.format(args.env, args.model, args.scenario))
		print("Path: ",Path)
		print("args.model: ",args.model)
		if args.model == "PPO":
			model = PPO.load(Path, env=env)
		elif args.model == "DDPG":
			model = DDPG.load(Path, env=env)
		elif args.model == "A2C":
			model = A2C.load(Path, env=env)
		elif args.model == "SAC":
			model = SAC.load(Path, env=env)
		elif args.model == "DQN":
			model = DQN.load(Path, env=env)
		elif args.model == "RecurrentPPO":
			model = RecurrentPPO.load(Path, env=env)
	if (model == None):
		return 
	return model

def testAgent(env, args, model, episodes):
	success = 0
	collision = 0
	timeout = 0
	t = 0

	for episode in range(1, episodes+1):
		obs = env.reset()
		done = False
		score = 0

		while not done:
			action = 2 #random.randint(0, 2)
			if args.eval == "False":
				action, _ = model.predict(obs)
			if (args.verbose == "True"):
				print("action: ",action)
				print("obs: ",obs)
			obs, reward, done, info = env.step(action)
			score += reward
		
		t = t + info['time']
		
		if(score >= 0):
			success +=1 
			if (args.verbose == "True"):
				print("Episode: ", episode, " -- succeed --")
		elif(score < 0):
			collision +=1 
			if (args.verbose == "True"):
				print("Episode: ", episode, " -- collided --")
		else:
			timeout +=1
			if (args.verbose == "True"):
				print("Episode: ", episode, " -- timeout --")
	return success, collision, timeout, t

def main():
	args = setUpArgs()

	env = createEnviroment(args)

	print("\n----->Model:",args.model)
	print("----->Network: ",args.env)
	print("----->scenario: ",args.scenario)
	print()

	model = None
	if args.eval == "False":
		model = defineModel(args, env)

	episodes = 20
	success, collision, timeout, t = testAgent(env, args, model, episodes)
	displayResults(success, collision, timeout, t, episodes)

if __name__ == "__main__":
	main()
	