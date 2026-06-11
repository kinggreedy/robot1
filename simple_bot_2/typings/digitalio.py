class Direction:
    INPUT = 0
    OUTPUT = 1


class Pull:
    UP = 1
    DOWN = 2


class DigitalInOut:
    def __init__(self, pin):
        self.pin = pin
        self.direction = None
        self.pull = None
        self.value = False

    def deinit(self):
        pass
