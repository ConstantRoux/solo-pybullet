import threading
import time
from pyjoystick.sdl2 import Key, Joystick, run_event_loop

mutex = threading.Lock()
inputs = {'AXIS_RX': 0, 'AXIS_RY': 0, 'AXIS_RZ': 0, 'AXIS_X': 0, 'AXIS_Y': 0, 'AXIS_Z': 0.16}


def print_add(joy):
    print('Added', joy)


def print_remove(joy):
    print('Removed', joy)


def key_received(key):
    with mutex:
        if key.keyname == 'Axis 0' or key.keyname == '-Axis 0':
            inputs['AXIS_RY'] = -key.value
        elif key.keyname == 'Axis 1' or key.keyname == '-Axis 1':
            inputs['AXIS_RX'] = key.value
        elif key.keyname == 'Axis 2':
            inputs['AXIS_RZ'] = key.value
        elif key.keyname == 'Axis 5':
            inputs['AXIS_RZ'] = -key.value
        elif key.keyname == 'Axis 3' or key.keyname == '-Axis 3':
            inputs['AXIS_X'] = -key.value
        elif key.keyname == 'Axis 4' or key.keyname == '-Axis 4':
            inputs['AXIS_Y'] = key.value
        elif key.keyname == 'Hat 0 [Up]':
            inputs['AXIS_Z'] += 0.01
        elif key.keyname == 'Hat 0 [Down]':
            inputs['AXIS_Z'] -= 0.01



def gamepad_thread():
    run_event_loop(print_add, print_remove, key_received)


def main_thread():
    # launch gamepad thread
    threading.Thread(target=gamepad_thread, args=()).start()

    # play main function
    while True:
        with mutex:
            print(inputs)

        time.sleep(0.01)


if __name__ == '__main__':
    main_thread()
