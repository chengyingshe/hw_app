物对应的机器人轨迹
    # for robot, trajectory in robot_trajectories.items():
    #     if trajectory:  # 确保轨迹非空
    #         goods_pos_str = f" to goods at {assignments[robot]['good']}" if robot in assignments and 'good' in assignments[robot] else ""
    #         print(f"Robot starting from {robot}{goods_pos_str} trajectory:")
    #         trajectory_str = " -> ".join(map(str, [robot] + trajectory))  # 包含起始点
    #         print(trajectory_str)
    #         if 'ship' in assignments[robot]:
    #             print(f" and then to ship at {assignments[robot]['ship']}")
    #         print('Finished\n')
