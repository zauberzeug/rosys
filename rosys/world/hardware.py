from __future__ import annotations
from pydantic import BaseModel
import abc


class HardwareGroup(BaseModel, abc.ABC):

    name: str
    output: bool = False

    @abc.abstractproperty
    def commands(self) -> list[str]:

        pass

    def with_output(self):

        self.output = True
        return self


class Bluetooth(HardwareGroup):

    device_name: str

    def __init__(self, **data):

        super().__init__(name='bluetooth', **data)

    @property
    def commands(self) -> list[str]:

        return [
            f'new bluetooth {self.name} ESP_{self.device_name}'
        ]


class Can(HardwareGroup):

    rxPin: int
    txPin: int

    def __init__(self, **data):

        super().__init__(name='can', **data)

    @property
    def commands(self) -> list[str]:

        return [
            f'new can {self.name} {self.rxPin},{self.txPin}',
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
    left_torque_factor: float = 1.0
    right_torque_factor: float = 1.0

    @property
    def commands(self) -> list[str]:

        return self.left.commands + self.right.commands + [
            f'new odrivewheels {self.name} {self.left.name},{self.right.name}',
            f'set {self.name}.width={self.width}',
            f'set {self.name}.leftTorqueFactor={self.left_torque_factor}',
            f'set {self.name}.rightTorqueFactor={self.right_torque_factor}',
        ]
