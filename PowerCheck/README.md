# PowerCheck Software Library

## Introduction

The software in this folder works with the West Mountain Radio [PowerCheck+](http://www.westmountainradio.com/product_info.php?products_id=pwrcheck)
device. WMR provide a windows tool for configuring and working with the device. This library implements a python library to allow running on
other operating systems, and provides similar functionality.

## Hardware Description

The PowerCheck+ unit has a touch screen for basic interactions, a micro-USB port, and up and downstream connectors to attach to a power supply
and feeds. The device can independently collect data and store it on EEPROM. The screen displays realtime and plots of data history
on its screen. For more information the link above includes a tab where the operator's manual can be found.

<p align="center">
  <img src="http://www.westmountainradio.com/images/catalog/pwrcheck_plus-bigZoom.jpg" />
</p>

## Software Description

The software library uses pyserial to implement the binary interface defined in their Design Doc. It allows for both pulling data from the
device as well as changing the configuration settings on it. There are more features defined in the design doc that could be implemented in
the future if there is interest.

### How to run

If you download the library `power_check.py` and the binary `run_power_check.py` to a computer with a device connected you can just run from
a terminal in the directory of the code:

```
python3 ./run_power_check.py
```

Running without arguments will print the license and help information. This would be a good file to modify to run specific actions for your
workflow.

### Testing

The other file in this folder, `power_check_test.py` define the pytest unittests that were used to help with the development of this library.
During development, the code was tested with a unit on both windows and linux (Raspberry Pi) devices.

### Contributions or Updates

Feel free to contact the owners for either feature requests or contributions you'd like to see. If you decide to contribute to the code please
follow the [GNU GPLv3 license](https://github.com/Utilis-Machina/software/blob/main/LICENSE).
