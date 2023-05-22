# Microstrain Software Library

## Introduction

The software in this folder works with the LORD Microstrain [3DM-GX4-45](https://www.microstrain.com/inertial/3dm-gx4-45)
device. Microstrain provides a data acquisition tool for windows, along with drivers. The product has been discontinued in
favor of a newer version, the GX5. This software is tested against the GX4, however, much of it will likely work for the
current version since it utilizes the standard Microstrain Packet (MIP) command structure.

## Hardware Description

This is a miniaturized high performance sensor that includes a gyro, magnetometer, Global Position System (GPS) receiver,
accelerometer, and pressure sensor. Both raw and processed data outputs are available. There is a complementary filter or
extended kalman filter as well. It supports either USB or RS232 communication. There are additional specifications on hardware
performance available on their site.

<p align="center">
  <img src="https://www.microstrain.com/sites/default/files/styles/larger__550x550_/public/gx4-45_0.jpg?itok=-DNyOySM)" />
</p>

## Software Description

The software library uses pyserial to implement the commands from their data communications protocol manual, which utilziles their
MIP format. Classes are implemented to handle the packet structure, field data, and interacting with the unit
itself. Additional features are available beyond what is implemented in the software library.

### How to run

If you download the library `microstrain.py` and the binary `run_microstrain.py` to a computer with a device connected you can just run from
a terminal in the directory of the code:

```
python3 ./run_microstrain.py
```

Running without arguments will print the license and help information. This would be a good file to modify to run specific actions for your
workflow.

### Testing

The other file in this folder, `microstrain_test.py` define the pytest unittests that were used to help with the development of this library.
The manual offers a pretty comprehensive set of examples to use to verify the correct serial communications are sent. During development, the
code was tested with a unit on both windows and linux (Raspberry Pi) devices.

### Contributions or Updates

Feel free to contact the owners for either feature requests or contributions you'd like to see. If you decide to contribute to the code please
follow the [GNU GPLv3 license](https://github.com/Utilis-Machina/software/blob/main/LICENSE).
