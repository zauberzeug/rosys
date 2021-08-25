from __future__ import annotations
from pydantic import BaseModel
import abc


class HardwareGroup(BaseModel, abc.ABC):

    @abc.abstractproperty
    def commands(self) -> list[str]:

        pass


class Bluetooth(HardwareGroup):

    name: str = 'bluetooth'
    device_name: str

    @property
    def commands(self) -> list[str]:

        return [
            f'new bluetooth {self.name} ESP_{self.device_name}'
        ]


class Can(HardwareGroup):

    name: str = 'can'
    rxPin: int
    txPin: int

    @property
    def commands(self) -> list[str]:

        return [
            f'new can {self.name} {self.rxPin},{self.txPin}',
        ]


class ODriveAxis(HardwareGroup):

    name: str
    can: Can
    device_id: int
    m_per_tick: float

    @property
    def commands(self) -> list[str]:

        return [
            f'new odriveaxis {self.name} 0,{self.can.name},{hex(self.device_id)[2:]}',
            f'set {self.name}.mPerTick={self.m_per_tick}',
        ]


class WheelPair(HardwareGroup):

    name: str
    left: ODriveAxis
    right: ODriveAxis
    width: float
    left_torque_factor: float = 1.0
    right_torque_factor: float = 1.0

    @property
    def commands(self) -> list[str]:

        return self.left.commands + self.right.commands + [
            f'new wheelpair {self.name} {self.left.name},{self.right.name}',
            f'set {self.name}.width={self.width}',
            f'set {self.name}.leftTorqueFactor={self.left_torque_factor}',
            f'set {self.name}.rightTorqueFactor={self.right_torque_factor}',
        ]
