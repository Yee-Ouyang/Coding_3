# Importing necessary libraries
import numpy as np
import random
import matplotlib.pyplot as plt

# Setting the discount factor for future rewards 设置未来奖励的折扣因子
gamma = 0.8

# Defining the reward matrix for the environment 定义环境的奖励矩阵
# States are represented as rows: 0=start, 1=right, 2=snake-pit, 3=treasure 状态表示为行：0=起点，1=右边，2=蛇坑，3=宝藏
# Actions are represented as columns in order: U=Up, D=Down, L=Left, R=Right, N=No action 动作按顺序表示为列：U=上，D=下，L=左，R=右，N=无动作
reward = np.array([
    [0, -10, 0, -1, -1],  # Rewards for actions from state 0 如在state1开始，向下蛇坑-10，向右空的-1，最后一个为待在原地不动的奖励
    [0, 10, -1, 0, -1],    # Rewards for actions from state 1
    [-1, 0, 0, 10, -1],    # Rewards for actions from state 2
    [0, -10, 0, 0, 10]    # Rewards for actions from state 3
])

# Initializing the Q-value matrix with zeros
# 用0初始化矩阵
q_matrix = np.zeros((4,5))

# Defining valid and invalid transitions using a transition matrix
# -1 indicates an invalid transition
# 使用转移矩阵定义有效和无效的转移
# -1 表示无效转移
transition_matrix = np.array([
    [-1, 2, -1, 1, 1], #在state0中，执行“上”动作（U）：无效（-1），执行“下”动作（D）：转移到状态2，执行“左”动作（L）：无效（-1），执行“右”动作（R）：转移到状态1，执行“无动作”（N）：保持在状态0（或可以理解为转移到自己）。
    [-1, 3, 0, -1, 2],
    [0, -1, -1, 3, 3],
    [1, -1, 2, -1, 4]
])

# Mapping valid actions for each state to simplify the action selection process
# 映射每个状态的有效动作，以简化动作选择过程
# The actions are encoded as 0=Up, 1=Down, 2=Left, 3=Right, 4=No action
# 动作编码：0=上，1=下，2=左，3=右，4=无动作
valid_actions = np.array([
    [1, 3, 4],  # Valid actions from state 0
    [1, 2, 4],  # Valid actions from state 1
    [0, 3, 4],  # Valid actions from state 2
    [0, 2, 4]   # Valid actions from state 3
])

# Running 1000 episodes for training
for i in range(1000):
    # Start from the initial state 从初始状态开始
    start_state = 0
    current_state = start_state

    # Continue until the goal state is reached 续直到达到目标状态
    while current_state != 3:
        # Select an action randomly from the valid actions for the current state 从当前状态的有效动作中随机选择一个动作
        action = random.choice(valid_actions[current_state])
        # Determine the next state from the transition matrix 从转移矩阵确定下一个状态
        next_state = transition_matrix[current_state][action]

        # Calculate the future rewards from the next state 计算下一个状态的未来奖励
        future_rewards = []
        for action_nxt in valid_actions[next_state]:
            future_rewards.append(q_matrix[next_state][action_nxt])
        
        # Update the Q-value for the current state and action 更新当前状态和动作的Q值
        q_state = reward[current_state][action] + gamma * max(future_rewards)
        q_matrix[current_state][action] = q_state

        # Move to the next state 移动到下一个状态
        current_state = next_state
        
        # Check if the goal state is reached 检查是否达到目标状态
        if current_state == 3:
            print('Goal state reached')

# Display the final Q-matrix after training
print('Final Q-matrix: ')
print(q_matrix)