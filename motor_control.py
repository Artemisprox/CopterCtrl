"""PID-based motor speed/position closed-loop controller at 100Hz."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Optional

from motor_comm import MotorCommDriver


@dataclass
class PIDConfig:
    kp: float
    ki: float
    kd: float
    out_min: float = -32768
    out_max: float = 32767
    integral_min: float = -10000
    integral_max: float = 10000


class PID:
    def __init__(self, cfg: PIDConfig):
        self.cfg = cfg
        self.integral = 0.0
        self.last_error: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.last_error = None

    def step(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement
        self.integral += error * dt
        self.integral = max(self.cfg.integral_min, min(self.integral, self.cfg.integral_max))

        derivative = 0.0
        if self.last_error is not None and dt > 1e-9:
            derivative = (error - self.last_error) / dt
        self.last_error = error

        out = self.cfg.kp * error + self.cfg.ki * self.integral + self.cfg.kd * derivative
        return max(self.cfg.out_min, min(out, self.cfg.out_max))


class MotorController100Hz:
    """Cascaded position-speed PID loop running at 100 Hz in a dedicated thread.

    - Outer loop: position PID -> speed reference
    - Inner loop: speed PID -> drive command (written to target_speed register)
    """

    def __init__(
        self,
        driver: MotorCommDriver,
        speed_pid: PID,
        pos_pid: PID,
        hz: float = 100.0,
    ) -> None:
        self.driver = driver
        self.speed_pid = speed_pid
        self.pos_pid = pos_pid
        self.dt = 1.0 / hz

        self.target_speed = 0.0
        self.target_position: Optional[float] = None

        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def set_speed_target(self, speed: float) -> None:
        with self._lock:
            self.target_speed = speed
            self.target_position = None

    def set_position_target(self, position: float) -> None:
        with self._lock:
            self.target_position = position

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run_loop, name="motor-ctrl-100hz", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run_loop(self) -> None:
        next_ts = time.perf_counter()
        while not self._stop_evt.is_set():
            t0 = time.perf_counter()
            try:
                speed_fb = float(self.driver.get_actual_speed())
                pos_fb = float(self.driver.get_actual_position())

                with self._lock:
                    target_pos = self.target_position
                    speed_sp = self.target_speed

                if target_pos is not None:
                    speed_sp = self.pos_pid.step(target_pos, pos_fb, self.dt)

                cmd = self.speed_pid.step(speed_sp, speed_fb, self.dt)
                self.driver.set_target_speed(int(cmd))
            except Exception as exc:
                # Real project: log to your logger/telemetry here.
                print(f"[motor-loop] warning: {exc}")

            next_ts += self.dt
            sleep_s = next_ts - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                # Overrun guard: re-sync the schedule to avoid drift accumulation
                next_ts = time.perf_counter()


if __name__ == "__main__":
    # Example usage
    drv = MotorCommDriver(port="/dev/ttyUSB0", slave_id=1, baudrate=19200)
    speed = PID(PIDConfig(kp=0.8, ki=0.1, kd=0.00, out_min=-3000, out_max=3000))
    pos = PID(PIDConfig(kp=2.0, ki=0.0, kd=0.01, out_min=-1500, out_max=1500))

    ctrl = MotorController100Hz(driver=drv, speed_pid=speed, pos_pid=pos, hz=100.0)

    drv.set_enable(True)
    ctrl.start()

    # 位置闭环示例：转到位置 10000
    ctrl.set_position_target(10000)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.stop()
        drv.set_enable(False)
        drv.close()
