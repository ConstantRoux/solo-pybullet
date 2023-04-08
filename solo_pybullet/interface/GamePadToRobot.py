import numpy as np
from solo_pybullet.interface.Gamepad import inputs, mutex

# TODO static class


def wait_awakening_input():
    ret = False
    with mutex:
        if inputs['Start'] > 0:
            inputs['Start'] = 0
            ret = True
    return ret


def staticInput(previousValues, scale=np.array([16, 10, 100, 2, 2, 1.5]),
                rawValuesWeight=np.array([0.8, 0.8, 1, 0.8, 0.8, 0.8]),
                previousValuesWeight=np.array([0.2, 0.2, 0, 0.2, 0.2, 0.2]),
                diviserFactor=np.array([2, 2, 1, 2, 2, 2])):
    # TODO : clamp Tz value (copyInputs['Hat_V'])
    with mutex:
        copyInputs = inputs.copy()

    Tx = copyInputs['RightJoy_H'] / scale[0]
    Ty = copyInputs['RightJoy_V'] / scale[1]
    Tz = copyInputs['Hat_V'] / scale[2] + 0.16
    Tz = max(0, min(Tz, 0.32))

    Rx = copyInputs['LeftJoy_H'] / scale[3]
    Ry = copyInputs['LeftJoy_V'] / scale[4]
    Rz = copyInputs['Trigger'] / scale[5]

    rawValues = np.array([Tx, Ty, Tz, Rx, Ry, Rz])
    # TODO use np.average with weight
    return (rawValues * rawValuesWeight + previousValues * previousValuesWeight) / diviserFactor


def walkInput(Vmax):
    with mutex:
        copyInputs = inputs.copy()

    Vx = -Vmax * copyInputs['LeftJoy_V']
    Vy = -Vmax * copyInputs['LeftJoy_H']
    omega = 0
    return np.array([Vx, Vy, omega])


if __name__ == '__main__':
    pass
