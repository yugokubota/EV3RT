import argparse

from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor

class LineTracer(object):
    def __init__(self, target: int, power: int, pid_p: float) -> None:
        self.running = False
        self.target = target
        self.power = power
        self.pid_p = pid_p

    def __call__(
        self,
        hub: Hub,
        right_motor: Motor,
        left_motor: Motor,
        touch_sensor: TouchSensor,
        color_sensor: ColorSensor,
    ) -> None:
        if not self.running and touch_sensor.is_pressed():
            self.running = True

        if not self.running:
            return

        brightness = color_sensor.get_brightness()
        delta = brightness - self.target
        power_ratio = self.pid_p * delta
        print(f'brightness = {brightness}, delta = {delta}, power_ratio = {power_ratio}')

        if power_ratio > 0:
            right_motor.set_power(self.power)
            left_motor.set_power(int(self.power / (1 + power_ratio)))
        else:
            right_motor.set_power(int(self.power / (1 - power_ratio)))
            left_motor.set_power(self.power)


def run(backend: str, target: int, power: int, pid_p: float, **kwargs) -> None:
    (ETRobo(backend=backend)
     .add_hub('hub')
     .add_device('right_motor', device_type=Motor, port='A')
     .add_device('left_motor', device_type=Motor, port='B')
     .add_device('touch_sensor', device_type=TouchSensor, port='D')
     .add_device('color_sensor', device_type=ColorSensor, port='E')
     .add_handler(LineTracer(target, power, pid_p))
     .dispatch(**kwargs))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--logfile', type=str, default=None, help='Path to log file')
    args = parser.parse_args()
    run(backend='raspike_art', interval=0.04, target=45, power=70, pid_p=0.5, logfile=args.logfile)
