import numpy as np
import pinocchio as pin
from solo_pybullet.controller.PD import PD
from numpy.linalg import pinv

########################
## WALKING_CONTROLLER ##
########################

# Method : Inverse Kinematics

# Initialization of the controller's parameters
q_ref = np.zeros((19, 1))
flag_q_ref = True


def c_walking_IK(q, qdot, dt, solo, t_simu):
    qa = q[7:]  # actuated, [q1, q2, ..., q12] angular position of the 12 motors
    qa_dot = qdot[6:]  # angular velocity of the 12 motors

    qa_ref = np.zeros((12, 1))  # target angular positions for the motors
    qa_dot_ref = np.zeros((12, 1))  # target angular velocities for the motors

    ###############################################
    # Insert code here to set qa_ref and qa_dot_ref
    global q_ref, flag_q_ref, T, dx, dz

    if flag_q_ref:
        q_ref = solo.q0.copy()
        flag_q_ref = False

    # Initialization of the variables
    K = 100.  # Convergence gain
    T = 1  # period of the foot trajectory
    xF0 = 0.207  # initial position of the front feet
    xH0 = -0.207  # initial position of the hind feet
    z0 = 0  # initial altitude of each foot
    dx = 0.03  # displacement amplitude by x
    dz = 0.06  # displacement amplitude by z

    # Get the frame index of each foot
    ID_FL = solo.model.getFrameId("FL_FOOT")
    ID_FR = solo.model.getFrameId("FR_FOOT")
    ID_HL = solo.model.getFrameId("HL_FOOT")
    ID_HR = solo.model.getFrameId("HR_FOOT")

    # function defining the feet's trajectory
    def _ftraj(t, x0, y0, z0):  # arguments : time, initial position x and z
        global T, dx, dz
        x = []
        y = []
        z = []

        if t >= T:
            t %= T

        x.append(x0 - dx * np.cos(2 * np.pi * t / T))

        if t <= T / 2.:
            z.append(z0 + dz * np.sin(2 * np.pi * t / T))
        else:
            z.append(z0)
        y.append(y0)

        return np.matrix([x, y, z])

    def ftraj(t, x0, y0, z0):  # arguments : time, initial position x and z
        global T, dx, dz
        x = []
        y = []
        z = []

        if t >= T:
            t %= T

        z.append(z0 + dz * np.sin(2 * np.pi * t / T) + dz)
        y.append(y0)
        x.append(x0)

        return np.matrix([x, y, z])

    # Compute/update all the joints and frames
    pin.forwardKinematics(solo.model, solo.data, q_ref)
    pin.updateFramePlacements(solo.model, solo.data)

    # Get the current height (on axis z) and the x-coordinate of the front left foot
    xyz_FL = solo.data.oMf[ID_FL].translation
    xyz_FR = solo.data.oMf[ID_FR].translation
    xyz_HL = solo.data.oMf[ID_HL].translation
    xyz_HR = solo.data.oMf[ID_HR].translation
    print('xyz_FL ' + str(xyz_FL))
    print('xyz_FR ' + str(xyz_FR))
    print('xyz_HL ' + str(xyz_HL))
    print('xyz_HR ' + str(xyz_HR))

    # Desired foot trajectory
    xyzdes_FL = ftraj(t_simu, xF0, 0.14695, z0)
    xyzdes_FR = ftraj(t_simu + T / 2, xF0, -0.14695, z0)
    xyzdes_HL = ftraj(t_simu + T / 2, xH0, 0.14695, z0)
    xyzdes_HR = ftraj(t_simu, xH0, -0.14695, z0)
    # print('xyzdes_FL ' + str(xyzdes_FL))
    # print('xyzdes_FR ' + str(xyzdes_FR))
    # print('xyzdes_HL ' + str(xyzdes_HL))
    # print('xyzdes_HR ' + str(xyzdes_HR))

    # Calculating the exception
    err_FL = xyz_FL - np.asarray(xyzdes_FL).reshape(-1)
    err_FR = xyz_FR - np.asarray(xyzdes_FR).reshape(-1)
    err_HL = xyz_HL - np.asarray(xyzdes_HL).reshape(-1)
    err_HR = xyz_HR - np.asarray(xyzdes_HR).reshape(-1)
    # print('err_FL ' + str(err_FL))
    # print('err_FR ' + str(err_FR))
    # print('err_HL ' + str(err_HL))
    # print('err_HR ' + str(err_HR))

    # Computing the local Jacobian into the global frame
    oR_FL = solo.data.oMf[ID_FL].rotation
    oR_FR = solo.data.oMf[ID_FR].rotation
    oR_HL = solo.data.oMf[ID_HL].rotation
    oR_HR = solo.data.oMf[ID_HR].rotation

    # Getting the different Jacobians
    fJ_FL3 = pin.computeFrameJacobian(solo.model, solo.data, q_ref, ID_FL)[:3, -12:]  # Take only the translation terms
    oJ_FL3 = oR_FL @ fJ_FL3  # Transformation from local frame to world frame
    oJ_FLxyz = oJ_FL3[:, -12:]  # Take the x, y and z components

    fJ_FR3 = pin.computeFrameJacobian(solo.model, solo.data, q_ref, ID_FR)[:3, -12:]
    oJ_FR3 = oR_FR @ fJ_FR3
    oJ_FRxyz = oJ_FR3[:, -12:]

    fJ_HL3 = pin.computeFrameJacobian(solo.model, solo.data, q_ref, ID_HL)[:3, -12:]
    oJ_HL3 = oR_HL @ fJ_HL3
    oJ_HLxyz = oJ_HL3[:, -12:]

    fJ_HR3 = pin.computeFrameJacobian(solo.model, solo.data, q_ref, ID_HR)[:3, -12:]
    oJ_HR3 = oR_HR @ fJ_HR3
    oJ_HRxyz = oJ_HR3[:, -12:]

    # Displacement exception
    nu = np.reshape([err_FL, err_FR, err_HL, err_HR], (12, 1))

    # Making a single x&z-rows Jacobian vector
    J = np.vstack([oJ_FLxyz, oJ_FRxyz, oJ_HLxyz, oJ_HRxyz])

    # Computing the velocity
    qa_dot_ref = -K * pinv(J) @ nu
    q_dot_ref = np.concatenate((np.zeros([6, 1]), qa_dot_ref))

    # Computing the updated configuration
    q_ref = pin.integrate(solo.model, q_ref, q_dot_ref * dt)
    qa_ref = q_ref[7:]

    # DONT FORGET TO RUN GEPETTO-GUI BEFORE RUNNING THIS PROGRAMM #
    # solo.display(q)  # display the robot in the viewer Gepetto-GUI given its configuration q

    # End of the control code
    ###############################################

    # Parameters for the PD controller
    Kp = 8.
    Kd = 0.2
    torque_sat = 3  # torque saturation in N.m
    torques_ref = np.zeros((12, 1))  # feedforward torques

    # Call the PD controller
    torques = PD(qa_ref, np.asarray(qa_dot_ref).reshape(-1), qa[:, 0], qa_dot[:, 0], dt, Kp, Kd, torque_sat)

    # torques must be a numpy array of shape (12, 1) containing the torques applied to the 8 motors
    return torques
