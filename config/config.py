import yaml
import os

def load_config():
    config_file = os.path.join(get_package_share_directory('ros2_project'), 'config', 'config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    return config
