# MMint Utils

Utility functions and helpers for MMint lab.

## Usage

Clone repo, then run `pip install .` to install.

Then, you can import and use helpers.

```
import mmint_utils

cfg = mmint_utils.load_cfg("config.yaml")
cfg["example"] = 100
mmint_utils.dump_cfg("config_2.yaml", cfg)
```
