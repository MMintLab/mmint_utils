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
        cfg = combine_cfg(default_cfg, cfg)

    return cfg


def dump_cfg(filename, cfg):
    """
    Dump configuration file for project to file.
    """
    with open(filename, 'w') as f:
        yaml.dump(cfg, f)


def combine_cfg(cfg_1: dict, cfg_2: dict):
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
