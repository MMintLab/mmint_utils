import yaml


def load_cfg(cfg_file, default_path=None):
    """
    Load configuration file for project.

    If default_path is provided, resulting config will inherit
    configs from default_path. Configs in cfg_file overwrite
    defaults.
    """
    with open(cfg_file) as f:
        cfg = yaml.load(f, Loader=yaml.SafeLoader)

    if default_path is not None:
        default_cfg = load_cfg(default_path)
        cfg = combine_dict(default_cfg, cfg)

    return cfg


def dump_cfg(filename, cfg):
    """
    Dump configuration file for project to file.
    """
    with open(filename, 'w') as f:
        yaml.dump(cfg, f)


def combine_dict(dict_1: dict, dict_2: dict):
    """
    Combine config dicts to single config dict.
    """
    for k, v in dict_2.items():
        if k not in dict_1:
            dict_1[k] = dict()
        if isinstance(v, dict):
            combine_dict(dict_1[k], v)
        else:
            dict_1[k] = v
    return dict_1
