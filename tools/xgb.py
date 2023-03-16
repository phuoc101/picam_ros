import pickle
from xgboost import XGBRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error
import numpy as np

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


X_train, X_test, y_train, y_test = train_test_split(data["X"], data["y"], test_size=0.5)
# X_train, y_train = data["X"], data["y"]
print(X_train.shape)
print(y_train.shape)
print(X_test.shape)
print(y_test.shape)

model = XGBRegressor()
model.fit(X_train, y_train)
# model.load_model("./xgb_dist.txt")
y_pred = model.predict(X_test)
mae = mean_absolute_error(y_test, y_pred)
print(f"MAE: {mae}")
# model.save_model("xgb_dist.txt")
