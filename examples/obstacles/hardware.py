from rosys.world.hardware import Bluetooth, Can, ODriveMotor, ODriveWheels

bluetooth = Bluetooth(device_name='hello_bot')

can = Can(rxPin='32', txPin='33')

wheels = ODriveWheels(
    name='wheels',
    left=ODriveMotor(name='drive0', can=can, device_id=0x100, m_per_tick=0.01571),
    right=ODriveMotor(name='drive1', can=can, device_id=0x0, m_per_tick=-0.01571),
    width=0.207,
    left_power_factor=1.0,
    right_power_factor=-1.0,
).with_output()

hardware = [
    bluetooth,
    can,
    wheels,
]
