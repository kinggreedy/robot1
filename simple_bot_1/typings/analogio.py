class AnalogIn:
    def __init__(self, pin):
        self.pin = pin
        self._value = 0

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, val):
        self._value = val

