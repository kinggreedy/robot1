class Direction:
    OUTPUT = 1


class DigitalInOut:
    def __init__(self, pin):
        self.pin = pin
        self.direction = None
        self.value = False
