import pyyaml


def load_config(config_path):
    with open(config_path, "r") as f:
        config = pyyaml.safe_load(f)
    return config