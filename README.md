### IMPORTANT : This software is currently being finalised. You should not use it on your machine unless you are a developer and know what you are doing, otherwise it has great chances to damage something or even hurt you !

# PlasMarlin plasma CNC Firmware

PlasMarlin is a fork of Marlin for plasma CNC machines.

## Objectives
In the last few years, 3D printing community became something huge and had allowed the development of amazing tools. On the other side, CNC plasma world did not moved as fast. The obvious reason for that is there are less people building plasma machines. This project intends to make plasma community benefits from the tools developed for 3D printers.


PlasMarlin aims to provide a **simple**, **low-cost** and **reliable** machine controller, which can run on a single board, without the need for an OS.
Marlin and Grbl have already paved the way for controlling CNC machines with simple hardware.

### Why not to just contribute to Marlin ?
Because plasma is a dangerous thing, we did not wanted to be a specific configuration among plenty of others intended to printers. This could lead to untested cases where configuration parameters would interfer.
Therefore, plasma can vary a lot on certain code sections and keeping both ways would have been messy.
That's why, with safety in mind, we decided to cut the cord with the Marlin mother repository.

## Features

  - PlasMarlin natively integrate THC (Torch Height Control). This include software, but also schematics to build an isolated probing circuit that communicate with Plasmarlin through I2C. (Documentation to be done)

  - As for the THC, a ohmic probing circuit is provided to detect height of the material before cut. By probing the material electrically, we avoid pushing on it which is more precise on thin materials. (Documentation to be done)

and more to be completed...


## Documentation

to be completed

## License

PlasMarlin inherits from the Marlin [GPL license](/LICENSE).

PlasMarlin is published under the GPL license because we believe in open development. The GPL comes with both rights and obligations. Whether you use PlasMarlin firmware as the driver for your open or closed-source product, you must keep PlasMarlin open, and you must provide your compatible PlasMarlin source code to end users upon request. The most straightforward way to comply with the PlasMarlin license is to make a fork of PlasMarlin on Github, perform your modifications, and direct users to your modified fork.

While we can't prevent the use of this code in plasma CNC that are closed source or crippled by a patent, we would prefer that you choose another firmware or, better yet, make your own.
