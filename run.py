from env import Env
import numpy as np
env = Env() #khởi tạo môi trường
EPISODES = 10 # số vòng chạy
steps = 1000 # số bước chạy
for e in range(EPISODES):
    done = False
    state = env.reset() # khởi tạo ngẫu nhiên giá trị state
    score = 0
    pre_action = None
    for t in range(steps):
        action = np.random.choice([0,1,2]) # chọn ngẫu nhiên action
        reward = env.setReward(state, pre_action, action) # nhận reward
        next_state, done = env.step(action) #chọn giá trị mới theo step
        pre_action = action 
        score = +reward
        state = next_state
        if t%5 == 0:
            print("episodes: %d; score %d; step: %d", e, score,t)
        if t == 1000 or score <= -30 or score >= 30:
            done = True
            print("episodes: %d done; total_score %d; total_step: %d", e, score,t)
            break
quit()

