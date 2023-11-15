from IntersectionSumoEnvFeaturesOneState import CustomEnv

env = CustomEnv()

episodes = 10
success = 0
collision = 0

for episode in range(1, episodes+1):
    obs = env.reset()
    done = False
    score = 0

    while not done:
        action = 1
        # print("action: ",action)
        print("obs: ",obs)
        obs, reward, done, info = env.step(action)
        score += reward
    print(info)
    
    if(score > 0):
        success +=1 
    elif(score <= 0):
        collision +=1 

success = success/episodes
collision = collision/episodes
print("percentage success: ",success)
print("percentage collision: ",collision)