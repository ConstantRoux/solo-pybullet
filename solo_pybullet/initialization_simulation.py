import numpy as np
import pybullet_data
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import pybullet as p


def configure_simulation(dt):
    # load solo12 model for pinocchio
    urdf_filename = '/opt/openrobots/share/example-robot-data/robots/solo_description/robots/solo12.urdf'
    meshes_dir = '/opt/openrobots/share'
    robot_wrapper = RobotWrapper.BuildFromURDF(urdf_filename, meshes_dir, pin.JointModelFreeFlyer())

    # start pybullet client
    client = p.connect(p.GUI)

    # setup pybullet environment
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(dt)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    # load solo12 model for pybullet
    robot_start_pos = [0, 0, 0.33]
    robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    p.setAdditionalSearchPath("/opt/openrobots/share/example-robot-data/robots/solo_description/robots")
    robot_id = p.loadURDF("solo12.urdf", robot_start_pos, robot_start_orientation)

    # disable default motor control for revolute joints
    rev_joint_idx = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    p.setJointMotorControlArray(robot_id, jointIndices=rev_joint_idx, controlMode=p.VELOCITY_CONTROL,
                                targetVelocities=[0.0 for m in rev_joint_idx],
                                forces=[0.0 for m in rev_joint_idx])

    # enable torque control for revolute joints
    joint_torques = [0.0 for m in rev_joint_idx]
    p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.TORQUE_CONTROL, forces=joint_torques)

    # compute one step of simulation for init
    p.stepSimulation()

    return robot_id, robot_wrapper, rev_joint_idx


def get_pos_vel_joints(robot_id, rev_joint_idx):
    # state of all joints
    joint_states = p.getJointStates(robot_id, rev_joint_idx)

    # position and orientation of the free flying base
    base_state = p.getBasePositionAndOrientation(robot_id)

    # velocity of the free flying base
    base_vel = p.getBaseVelocity(robot_id)

    # Reshaping data into q and qdot
    q = np.vstack((np.array([base_state[0]]).transpose(), np.array([base_state[1]]).transpose(),
                   np.array([[joint_states[i_joint][0] for i_joint in range(len(joint_states))]]).transpose()))
    qdot = np.vstack((np.array([base_vel[0]]).transpose(), np.array([base_vel[1]]).transpose(),
                      np.array([[joint_states[i_joint][1] for i_joint in range(len(joint_states))]]).transpose()))

    return q, qdot
