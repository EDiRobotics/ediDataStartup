import os
import pdb
from data_collection.data import StepLMDBDatasetV2

"""
Work on pc@192.168.1.240
"""

dataset_directory = "/home/pc/ediController/dataset/"
dataset_config = "test_nav.csv"
csv_path = os.path.join(dataset_directory, dataset_config)
print("csv path", csv_path)
dataset = StepLMDBDatasetV2(csv_path)
print(len(dataset))

pdb.set_trace()
obs, action, instruct = dataset[0]
print(f"obs: {obs}")
print(f"action: {action}")
print(f"instruct: {instruct}")

print("Total Length", len(dataset))

pdb.set_trace()

for i, (obs, action, instruct) in enumerate(dataset):
    for item in (obs, action, instruct):
        if item is None:
            print(f"{i} is None")
