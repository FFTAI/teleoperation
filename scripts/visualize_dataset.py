import typer
import matplotlib.pyplot as plt
import h5py
import numpy as np

def visualize_dataset(file_path: str):
    with h5py.File(file_path, 'r') as f:
        robot_action = f["action"]["robot"] # N x 32
        robot_state = f["state"]["robot"] # N x 32
        hand_action = f["action"]["hand"] # N x 12
        hand_state = f["state"]["hand"] # N x 12
        time = f["timestamp"] # N

        time = np.array(time).reshape((-1, 1))

        f, axes = plt.subplots(4, 7, sharex='all', sharey='all', figsize=(30, 10))

        # plot robot action and state for each joint
        for i in range(14):
            axes[i//7, i%7].plot(time, robot_action[:, 32-14+i], label="robot action")
            axes[i//7, i%7].plot(time, robot_state[:, 32-14+i], label="robot state")
            axes[i//7, i%7].legend()

        for i in range(12):
            axes[2 + i//6, i%6].plot(time, hand_action[:, i], label="hand action")
            axes[2 + i//6, i%6].plot(time, hand_state[:, i], label="hand state")
            axes[2 + i//6, i%6].legend()


        plt.show()

if __name__ == "__main__":
    typer.run(visualize_dataset)
