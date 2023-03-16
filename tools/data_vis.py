import pickle
import numpy as np
import matplotlib.pyplot as plt

files = [
    "../extracted_data/Tierankatu_nodrone/labels/box_dist_labels/data.pkl",
    "../extracted_data/Tierankatu_tello/labels/box_dist_labels/data.pkl",
]
data = dict()
data["X"] = np.array([])
data["y"] = np.array([])
for file in files:
    with open(file, "rb") as f:
        tmp = pickle.load(f)
        if data["X"].size != 0:
            data["X"] = np.vstack((data["X"], tmp["X"]))
            data["y"] = np.concatenate((data["y"], tmp["y"]))
        else:
            data["X"] = tmp["X"]
            data["y"] = tmp["y"]


X, y = data["X"], data["y"]
print(X.shape)
print(y.shape)

area = X[:, 2] * X[:, 3]
idx2del = []
for i, a in enumerate(area):
    if a < 0.15 and y[i] < 0.8:
        idx2del.append(i)
        print(i)

X = np.delete(X, obj=idx2del, axis=0)
y = np.delete(y, obj=idx2del, axis=0)
area = X[:, 2] * X[:, 3]

fig = plt.figure()
# 2D
plt.title("Relationship between bounding box area and distance")
plt.scatter(area, y, s=1)
plt.xlabel("Normalized bounding box area (0-1)")
plt.ylabel("Distance (m)")
plt.show()
# # 3D
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(X[:, 2], X[:, 3])
