# Safety

Python is fast enough for most highlevel logic. But of course it has no realtime guarantees.
Safety relevant behaviour should therefore be written in [Lizard](https://lizard.dev/) and executed on a suitable microprocessor.
The microprocessor governs the hardware of the robot and must be able to perform safety actions like triggering emergency hold etc.
We suggest you use an industrial PC like the [Zauberzeug Robot Brain](https://www.zauberzeug.com/product-robot-brain.html).
It provides an Linux system with AI acceleration to run RoSys, an integrated [ESP32](https://www.espressif.com/en/products/socs/esp32) to run Lizard and six I/O sockets for CAN, RS484, SPI, I2C ... with a software controllable ENABLE switch.

## Operation States

### on

This is the default state after launching RoSys. To enter the next state "manual.drive" you should write an [Automation](basic_concepts.md#automations) which ensures the robot is ready (eg. no active emergency stops).

### manual.drive

Normally this is the default target after "on".
All axis should not be moved.
Except the drive units.
The purpose of this state is to steer the robot by an operator.

### homing

To transition to higher level operation states like "manual.operate" or "auto" the robot requires to know the zero positions of each motor axis.
Normally homing is done sequencially on a per axis basis.

### manual.operate

Mostly used during development but also sometimes for maintainance.
All axis can be controlled by the operator as long as the movement does not violate safety requirements.
It can only be activated if homing was sucessful.

### auto

This state indicates the fully autonomous operation of the robot.
Activation must happen through a users interaction and requires an successful homing.

### hold

In this state the robot must completely stop as quick as possible.
For example by applying the breaks.
This state is entered if any safety requirement is violated.

### stop

Cuts all power connections leading to a total emidate shutdown which must be reactivated manually.
Most robots do not require this state.
