import numpy as np
import matplotlib.pyplot as plt


trajs = np.load("./31_obs1_test.npy")

plt.figure()

for i, t in enumerate(trajs):
    plt.scatter(t[:,0], t[:,1], label = f"{i}")
plt.legend()
plt.show()