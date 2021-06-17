import gzip
import pickle


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
