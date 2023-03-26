class NoSolutionException(Exception):
    def __init__(self, pos, constraints):
        super().__init__('No solution was found for {} position with {} constraints.'.format(pos, constraints))
