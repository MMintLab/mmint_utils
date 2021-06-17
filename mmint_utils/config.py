import yaml


def load_cfg(cfg_file):
    """
    Load configuration file for project.
    """
    cfg = yaml.load(open(cfg_file), Loader=yaml.SafeLoader)
    return cfg


def dump_cfg(filename, cfg):
    """
    Dump configuration file for project to file.
    """
    with open(filename, 'w') as f:
        yaml.dump(cfg, f)


def combine_cfg(cfg_1, cfg_2):
    """
    Combine config dicts to single config dict.
    """
    for k, v in cfg_2.items():
        if k not in cfg_1:
            cfg_1[k] = dict()
        if isinstance(v, dict):
            combine_cfg(cfg_1[k], v)
        else:
            cfg_1[k] = v
    return cfg_1
