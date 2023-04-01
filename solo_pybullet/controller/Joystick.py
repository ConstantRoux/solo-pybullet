import time
import pygame


class Joystick:
    """
    Get both joystick value (-1 to 1) from an available controller
    If not, return (-999, -999), (-999, -999)
    """

    @staticmethod
    def get_value():
        pygame.init()

        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()

        if joystick_count == 0:
            print("No joystick available!")
            return (-999, -999), (-999, -999)
        else:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()

            # Mise à jour des événements
            pygame.event.pump()

            # 0 => Joy left, axis right
            # 1 => Joy left, axis bot
            # 2 => Joy right, axis right
            # 3 => Joy right, axis bot
            # 4 => trigger left
            # 5 => trigger right
            return (round(joystick.get_axis(0), 3), round(joystick.get_axis(1), 3)), \
                   (round(joystick.get_axis(2), 3), round(joystick.get_axis(3), 3))


if __name__ == '__main__':
    while True:
        joy1, joy2 = Joystick.get_value()

        # check exception
        if joy1 == (-999, -999):
            break
            
        print(joy1, joy2)
