import asyncio
import time
import digitalio


class StepperMotor:
    FULLSTEP = (
        (1, 1, 0, 0),
        (0, 1, 1, 0),
        (0, 0, 1, 1),
        (1, 0, 0, 1),
    )

    def __init__(
        self,
        pin1,
        pin2,
        pin3,
        pin4,
        rpm=10,
        steps_per_rev=2048,
    ):
        self.pins = []

        for pin in (pin1, pin2, pin3, pin4):
            p = digitalio.DigitalInOut(pin)
            p.direction = digitalio.Direction.OUTPUT
            self.pins.append(p)

        self.steps_per_rev = steps_per_rev
        self.set_rpm(rpm)

        self.position = 0
        self.target = 0
        self.seq_index = 0
        self.run_forever = False

    def set_rpm(self, rpm):
        self.rpm = rpm
        self.step_interval = 60 / (
            rpm * self.steps_per_rev
        )

    def move(self, steps):
        self.target += steps

    def move_to(self, position):
        self.target = position

    def move_forever(self,dir):
        self.target = self.position+dir
        self.run_forever = True
    
    def stop(self):
        self.run_forever=False
        self.target = self.position

    @property
    def busy(self):
        return self.position != self.target

    def release(self):
        for pin in self.pins:
            pin.value = False

    def _apply_step(self, direction):
        self.seq_index = (self.seq_index + direction) % 4
        pattern = self.FULLSTEP[self.seq_index]

        for pin, value in zip(self.pins, pattern):
            pin.value = value

        self.position += direction

    async def run(self):
        next_step = time.monotonic()

        while True:

            if self.position != self.target:

                now = time.monotonic()

                if now >= next_step:

                    direction = (
                        1 if self.target > self.position
                        else -1
                    )

                    self._apply_step(direction)
                    if self.run_forever:
                        self.target = self.position+direction
                    next_step += self.step_interval

            await asyncio.sleep(0)