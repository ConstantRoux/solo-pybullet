import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from time import sleep

URDF = '/opt/openrobots/share/example-robot-data/robots/solo_description/robots/solo12.urdf'
robot = RobotWrapper.BuildFromURDF(URDF, ['/opt/openrobots/share'])


def pin_viewer(configs, robot):
    # os.system("gepetto-gui")  # A executer avant pour lancer le client gepetto viewer
    robot.initViewer(loadModel=True)
    robot.display(robot.q0)
    for c in configs:
        robot.display(c)
        sleep(5e-1)


if __name__ == "__main__":
    configs = []
    for i in range(20):
        configs.append(pin.randomConfiguration(robot.model))
    pin_viewer(configs, robot)
