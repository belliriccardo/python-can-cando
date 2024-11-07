# python-can-cando

[![PyPI - Implementation](https://img.shields.io/pypi/implementation/python-can-cando)](https://pypi.org/project/python-can-cando/)
[![PyPI - Version](https://img.shields.io/pypi/v/python-can-cando)](https://pypi.org/project/python-can-cando/)
[![PyPI Downloads](https://static.pepy.tech/badge/python-can-cando)](https://www.pepy.tech/projects/python-can-cando)
[![PyPI - Python Version](https://img.shields.io/pypi/pyversions/python-can-cando)](https://pypi.org/project/python-can-cando/)

-----

- [Description](#description)
- [Installation](#installation)
- [Usage](#usage)
- [Extra information](#extra-information)
        - [A note on bus load](#a-note-on-bus-load)
- [Compatibility](#compatibility)
- [License](#license)

## Description
This module is a plugin that lets you use [netronics](http://www.netronics.ltd.uk/)' can adapters (namely the [CANdo](https://www.cananalyser.co.uk/cando.html) and [CANdoISO](https://www.cananalyser.co.uk/candoiso.html) devices) in [python-can](https://python-can.readthedocs.io/en/stable/)'s [plugin interface](https://python-can.readthedocs.io/en/stable/plugin-interface.html).

## Installation

You should have installed the [netronics' CANdo drivers](https://www.cananalyser.co.uk/download.html) before using this library; otherwise you will get a DLL error (or, under linux, a shared library error).

```console
pip install python-can-cando
```

## Usage

Just like any other `python-can` plugin, you can access the class like so:

```python
import can
bus = can.Bus(interface="cando", channel=0)
# All of your other python-can code...
```

Or you could just instantiate it directly:

```python
from can_cando import CANdoBus
bus = CANdoBus(channel=0)
# All of your other python-can code...
```

## Extra information

There are some inherent limitations to this device; you should really check the programmer's guide and the actual implementation of some methods, such as the `cando_transmit`'s documentation on how to use the device's internal repeat buffers; the number of such available buffers and message timing periodicity is fixed and limited. The implementation of `BusABC.send_periodic` is intentionally dummy-overridden for this reason.

Anyways, the programmer's guide and all the relative datasheets for the devices can be downloaded from [netronics' download page](https://www.cananalyser.co.uk/download.html).

###### A note on bus load

I found that the device tends to get the `CANDO_CAN_RX_OVERRUN` status if there is either too much bus load or sudden spikes of bus load (and therefore it loses some can messages); if you have other nodes in the network that send a lot of periodic messages with a short period, you might consider using the appropriate filters, rather that just reading all the messages and filtering them in software.

## Compatibility
I developed this library with a CANdoISO device; I don't have a CANdo device to test it, but it should work just as well.
There are some minor differences between the two (for example a different clock frequency), but from a software point of view the interfaces are very similar.

This library was developed and tested on Windows 10 and python 3.12, but it should be good to go with python up till 3.6 (checked with [vermin](https://github.com/netromdk/vermin)) and non-Windows OSs.

Anyways, any feedback is appreciated and welcome; just open an issue or a pull request and I'll gladly take a look at it.

## License

`python-can-cando` is distributed under the terms of the [LGPL-3.0-or-later](https://spdx.org/licenses/LGPL-3.0-or-later.html) license.

[def]: #python-can-cando
