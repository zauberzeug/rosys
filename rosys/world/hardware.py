from __future__ import annotations
from pydantic import BaseModel
import abc
from .velocity import Velocity

from typing import TYPE_CHECKING
if TYPE_CHECKING:  # NOTE: avoid cyclic import
    from .world import World


class HardwareGroup(BaseModel, abc.ABC):
    name: str
    output: bool = False

    @abc.abstractproperty
    def commands(self) -> list[str]:
        pass

    def with_output(self):
        self.output = True
        return self

    def parse(self, words: list[str], world: World):
        pass


class Bluetooth(HardwareGroup):
    device_name: str

    def __init__(self, **data):
        super().__init__(name='bluetooth', **data)

    @property
    def commands(self) -> list[str]:
        return [
            f'new bluetooth {self.name} ESP_{self.device_name}',
        ]


class Can(HardwareGroup):
    rxPin: str
    txPin: str

    def __init__(self, **data):
        super().__init__(name='can', **data)

    @property
    def commands(self) -> list[str]:
        return [
            f'new can {self.name} {self.rxPin},{self.txPin}',
        ]


class Led(HardwareGroup):
    pin: str
    interval: float = 0.1
    duty: float = 0.5
    repeat: bool = True

    @property
    def commands(self) -> list[str]:
        return [
            f'new led {self.name} {self.pin}',
            f'set {self.name}.interval={self.interval}',
            f'set {self.name}.duty={self.duty}',
            f'set {self.name}.repeat={"1" if self.repeat else "0"}',
        ]


class Button(HardwareGroup):
    pin: str

    @property
    def commands(self) -> list[str]:
        return [
            f'new button {self.name} {self.pin}',
        ]


class RoboClawWheels(HardwareGroup):
    address: int = 128
    baud: int = 38400
    m_per_tick: float
    width: float

    @property
    def commands(self) -> list[str]:
        return [
            f'new roboclawwheels {self.name} {self.address},{self.baud}',
            f'set {self.name}.mPerTick={self.m_per_tick}',
            f'set {self.name}.width={self.width}',
        ]

    def parse(self, words: list[str], world: World):
        world.robot.odometry.append(Velocity(
            linear=float(words.pop(0)),
            angular=float(words.pop(0)),
            time=world.robot.hardware_time,
        ))
        world.robot.temperature = float(words.pop(0))
        world.robot.battery = float(words.pop(0))


class ODriveMotor(HardwareGroup):
    can: Can
    device_id: int
    m_per_tick: float

    @property
    def commands(self) -> list[str]:
        return [
            f'new odrivemotor {self.name} 0,{self.can.name},{hex(self.device_id)[2:]}',
            f'set {self.name}.mPerTick={self.m_per_tick}',
        ]


class ODriveWheels(HardwareGroup):
    left: ODriveMotor
    right: ODriveMotor
    width: float
    left_power_factor: float = 1.0
    right_power_factor: float = 1.0

    @property
    def commands(self) -> list[str]:
        return self.left.commands + self.right.commands + [
            f'new odrivewheels {self.name} {self.left.name},{self.right.name}',
            f'set {self.name}.width={self.width}',
            f'set {self.name}.leftPowerFactor={self.left_power_factor}',
            f'set {self.name}.rightPowerFactor={self.right_power_factor}',
        ]

    def parse(self, words: list[str], world: World):
        world.robot.odometry.append(Velocity(
            linear=float(words.pop(0)),
            angular=float(words.pop(0)),
            time=world.robot.hardware_time,
        ))
