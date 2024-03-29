import gzip
import pickle
import os

import numpy as np


def save_gzip_pickle(data, filename):
    """
    Save the given data using gzip/pickle.
    """
    with gzip.open(filename, "w") as f:
        pickle.dump(data, f)


def load_gzip_pickle(filename):
    """
    Load from file using gzip/pickle.
    """
    with gzip.open(filename, "rb") as f:
        data = pickle.load(f)
    return data


def load_pickle(filename):
    """
    Load from file using pickle.
    """
    with open(filename, "rb") as f:
        data = pickle.load(f)
    return data


def make_dir(directory):
    """
    If the provided directory does not exist, make it.
    """
    if not os.path.exists(directory):
        os.makedirs(directory)


def generate_splits_indices(dataset_length, train_split=0.8, val_split=0.1):
    """
    Generate split indices for a given dataset size.
    """
    assert train_split + val_split <= 1.0

    num_train = int(train_split * dataset_length)
    num_val = int(val_split * dataset_length)

    # Shuffle trial numbers.
    example_ids = np.array(list(range(dataset_length)))
    np.random.shuffle(example_ids)

    return example_ids[:num_train], example_ids[num_train: num_train + num_val], example_ids[num_train + num_val:]
