import unittest
import mmint_utils


class TestConfigMethods(unittest.TestCase):

    def test_load_config(self):
        cfg = mmint_utils.load_cfg("test.yaml")
        gt_cfg = {
            "int": 4,
            "bool": True,
            "list": [1, 2, 3, 4],
            "dict": {
                "abc": "d",
            },
        }
        self.assertDictEqual(cfg, gt_cfg)

    def test_save_config(self):
        gt_cfg = {
            "foo": "bar",
            "test": 42,
            "names": ["Han Solo", "Chewbacca", "Luke Skywalker"],
            "dictionary": {
                "Millenium Falcon": 199,
            }
        }
        mmint_utils.dump_cfg("/tmp/test.yaml", gt_cfg)
        cfg = mmint_utils.load_cfg("/tmp/test.yaml")
        self.assertDictEqual(cfg, gt_cfg)

    def test_combine_config(self):
        cfg_1 = {
            "test_1": 1,
        }
        cfg_2 = {
            "test_2": 2,
        }
        gt_cfg = {
            "test_1": 1,
            "test_2": 2,
        }
        cfg = mmint_utils.combine_cfg(cfg_1, cfg_2)
        self.assertDictEqual(cfg, gt_cfg)

    def test_combine_config_overwrite(self):
        cfg_1 = {
            "test_1": 1,
            "abc": "d"
        }
        cfg_2 = {
            "test_2": 2,
            "abc": "test"
        }
        gt_cfg = {
            "test_1": 1,
            "test_2": 2,
            "abc": "test",
        }
        cfg = mmint_utils.combine_cfg(cfg_1, cfg_2)
        self.assertDictEqual(cfg, gt_cfg)

    def test_default_file_load(self):
        gt_cfg = {
            "int": 4,
            "bool": True,
            "list": [1, 2, 3, 4],
            "dict": {
                "abc": "d",
            },
            "testing_default": True,
            "random": "strings",
        }
        cfg = mmint_utils.load_cfg("test.yaml", default_path="default.yaml")
        self.assertDictEqual(cfg, gt_cfg)


if __name__ == '__main__':
    unittest.main()
