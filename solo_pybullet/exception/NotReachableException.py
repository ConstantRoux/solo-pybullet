class NotReachableException(Exception):
    def __init__(self, pos):
        super().__init__('Position {} is not reachable.'.format(pos))
