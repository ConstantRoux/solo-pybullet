import numpy as np

CentreCercle = (2, 1, -0.2)

controllerMax = 255
centerStick = 127
minCenterStick = centerStick-7
maxCenterStick = centerStick+7

valueR2 = 0

valueLeftStickX = centerStick
valueLeftStickY = centerStick

valueRightStickX = centerStick
valueRightStickY = centerStick


def coordLegs(v, theta, omega, O, rMax, legType=None):
    """
    :param v: normalised [0, 1]
    :param theta: angle [-pi, pi]
    :param omega: angle [-pi, pi]
    :param O: center of Circle (x, y, z) in the leg frame
    :param rMax: radius maximum of the circle
    :param legType: type of leg ("front" or "back")
    :return:
    """

    v = v * rMax
    vx = v * np.cos(theta)
    vy = v * np.sin(theta)

    patteX = vx + O[0]
    patteY = vy + O[1]

    # TODO OMEGA
    if legType == "front":
        patteX = patteX
        patteY = patteY
    elif legType == "back":
        patteX = patteX
        patteY = patteY

    return patteX, patteY


def walkController(rMax):
    """
    :param rMax: radius maximum of the circle
    :return:
    """
    Controller = DualShock3Thread(name="DualShock3Thread")
    Controller.start()

    L = ["FLBR", "FRBL"]
    i = 0
    while 1:
        if not (maxCenterStick > valueRightStickX > minCenterStick and maxCenterStick > valueLeftStickX > minCenterStick
                and maxCenterStick > valueRightStickY > minCenterStick and maxCenterStick > valueLeftStickY > minCenterStick):
            theta = np.arctan2(-(valueLeftStickY - 127), valueLeftStickX - 127)
            omega = np.arctan2(-(valueRightStickY - 127), valueRightStickX - 127)
            v = valueR2/controllerMax

            xF, yF = coordLegs(v, theta, omega, CentreCercle, rMax, L[i%2][:2])
            xB, yB = coordLegs(v, theta, omega, CentreCercle, rMax, L[i%2][2:])

        else:
            v = 0
            theta = 0
            omega = 0
            xF, yF = coordLegs(v, theta, omega, CentreCercle, rMax, L[i%2][:2])
            xB, yB = coordLegs(v, theta, omega, CentreCercle, rMax, L[i%2][2:])
        i += 1














