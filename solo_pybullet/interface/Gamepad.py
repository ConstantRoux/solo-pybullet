import threading
import time
from pyjoystick.sdl2 import run_event_loop

mutex = threading.Lock()
# H : Horizontal, V : Vertical
inputs = {'LeftJoy_H': 0, 'LeftJoy_V': 0, 'Trigger': 0, 'RightJoy_H': 0, 'RightJoy_V': 0, 'Hat_V': 0, 'Start': 0, 'Select': 0}


def print_add(joy):
    """
    Bind the gamepad when connected to the application
    :param joy:
    :return:
    """
    print('Added', joy)


def print_remove(joy):
    """
    Remove the gamepad when disconnected for the computer
    :param joy:
    :return:
    """
    print('Removed', joy)


def key_received(key):
    """
    Get the input and map them to variables
    :param key:
    :return:
    """
    with mutex:
        if key.keyname == 'Axis 0' or key.keyname == '-Axis 0':
            inputs['LeftJoy_V'] = -key.value
        elif key.keyname == 'Axis 1' or key.keyname == '-Axis 1':
            inputs['LeftJoy_H'] = key.value
        elif key.keyname == 'Axis 2':
            inputs['Trigger'] = key.value
        elif key.keyname == 'Axis 5':
            inputs['Trigger'] = -key.value
        elif key.keyname == 'Axis 3' or key.keyname == '-Axis 3':
            inputs['RightJoy_H'] = -key.value
        elif key.keyname == 'Axis 4' or key.keyname == '-Axis 4':
            inputs['RightJoy_V'] = key.value
        elif key.keyname == 'Hat 0 [Up]' and key.value == 1:
            inputs['Hat_V'] += 1
        elif key.keyname == 'Hat 0 [Down]' and key.value == 4:
            inputs['Hat_V'] -= 1
        elif key.keyname == 'Button 7' and key.value == 1:
            inputs['Start'] += 1
        elif key.keyname == 'Button 6' and key.value == 1:
            inputs['Select'] += 1


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
