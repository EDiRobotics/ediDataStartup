import os

from data_collection.data import StepLMDBDatasetV2


"""
Work on pc@192.168.1.240
"""

dataset_directory = "~/ediControlloer/dataset/"
dataset_config = "test2.csv"

dataset = StepLMDBDatasetV2(os.path.join(dataset_directory, dataset_config))
print(len(dataset))

obs, action, instruct = dataset[0]
print(f"obs: {obs}")
print(f"action: {action}")
print(f"instruct: {instruct}")

for i, (obs, action, instruct) in enumerate(dataset):
    for item in (obs, action, instruct):
        if item is None:
            print(f"{i} is None")
