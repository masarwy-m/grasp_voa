import yaml


class Alg:
    def __init__(self, config_file):
        with open(config_file, 'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
        self.possible_grasps = list(yaml_data.keys())
        self.possible_poses = list(list(yaml_data.values())[0].keys())


if __name__ == '__main__':
    gs = Alg('../config/grasp_score/mug.yaml')
    print('ok')
