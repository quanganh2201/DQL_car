from env import Env
import numpy as np
import model5
import torch
env = Env() #khởi tạo môi trường
EPISODES = 5 # số vòng chạy
steps = 500 # số bước chạy
model_filepath = ...#tên file đã lưu
score = 0
num_states = 6
num_actions = 3
pre_action = None
policy_dqn = model5.DeepQNetwork(input_dims=num_states, fc1_dims=64, fc2_dims=64,
                                      n_actions=num_actions)
policy_dqn.load_state_dict(torch.load(model_filepath))
policy_dqn.eval()
for e in range(EPISODES):
    done = False
    state = env.reset() # khởi tạo ngẫu nhiên giá trị state
    while not done:
        with torch.no_grad():
            action = policy_dqn(model5.state_to_dqn_input(state)).argmax().item()
        reward, done = env.setReward(state, pre_action, action) # nhận reward
        next_state = env.step(action) #chọn giá trị mới theo step
        pre_action = action
        score = +reward
        state = next_state
        if done:
            print("episodes: %d done; total_score %d; total_step: %d", e, score,t)
            break
quit()

