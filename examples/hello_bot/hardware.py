from rosys.world.hardware import Bluetooth, Can, ODriveAxis, WheelPair

bluetooth = Bluetooth(device_name='hello_bot')

can = Can(rxPin=32, txPin=33)

odrive0 = ODriveAxis(name='drive0', can=can, device_id=0x100, m_per_tick=0.01571).with_output()
odrive1 = ODriveAxis(name='drive1', can=can, device_id=0x0, m_per_tick=-0.01571).with_output()

wheel_pair = WheelPair(
    name='drive',
    left=odrive0,
    right=odrive1,
    width=0.207,
    left_torque_factor=1.0,
    right_torque_factor=-1.0,
)

hardware = [
    bluetooth,
    can,
    odrive0,
    odrive1,
    wheel_pair,
]
