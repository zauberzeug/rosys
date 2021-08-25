from rosys.world.hardware import Bluetooth, Can, ODriveAxis, WheelPair

bluetooth = Bluetooth(device_name='hello_bot')

can = Can(rxPin=32, txPin=33)

wheel_pair = WheelPair(
    name='drive',
    left=ODriveAxis(name='drive0', can=can, device_id=0x100, m_per_tick=0.01571),
    right=ODriveAxis(name='drive1', can=can, device_id=0x0, m_per_tick=-0.01571),
    width=0.207,
    left_torque_factor=1.0,
    right_torque_factor=-1.0,
)

hardware = [
    bluetooth,
    can,
    wheel_pair,
]
