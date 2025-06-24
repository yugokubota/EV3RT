import argparse
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor
from simple_pid import PID

class LineTracer(object):
    def __init__(self, interval: float, target: int, power: int, pid_p: float, pid_i: float, pid_d: float) -> None:
        self.running = False
        self.power = power
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=target, sample_time=interval, output_limits=(-power, power))

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
        turn = int(self.pid(brightness))
        print(f'brightness = {brightness}, turn = {turn}')

        right_motor.set_power(self.power - turn)
        left_motor.set_power(self.power + turn)


def run(backend: str, interval: float, target: int, power: int, pid_p: float, pid_i: float, pid_d: float, **kwargs) -> None:
    (ETRobo(backend=backend)
     .add_hub('hub')
     .add_device('right_motor', device_type=Motor, port='A')
     .add_device('left_motor', device_type=Motor, port='B')
     .add_device('touch_sensor', device_type=TouchSensor, port='D')
     .add_device('color_sensor', device_type=ColorSensor, port='E')
     .add_handler(LineTracer(interval, target, power, pid_p, pid_i, pid_d))
     .dispatch(interval, **kwargs))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--logfile', type=str, default=None, help='Path to log file')
    args = parser.parse_args()
    run(backend='raspike_art', interval=0.04, target=45, power=45, pid_p=0.85, pid_i=0.1, pid_d=0.05, logfile=args.logfile)
