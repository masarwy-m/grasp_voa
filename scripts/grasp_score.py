import yaml


class GraspScore:
    def __init__(self, config_file):
        with open(config_file, 'r') as yaml_file:
            self.grasp_score = yaml.safe_load(yaml_file)

    def __call__(self, graspID, objectPose):
        return self.grasp_score[graspID][objectPose]


if __name__ == '__main__':
    gs = GraspScore('../config/grasp_score/mug.yaml')
    print(gs('grasp_1', 'up_straight'))
