# solarcontroller_master
This is the Arduino code that has been successfully collecting hot water (in some version or another) since 2006.


Note: this code only works on pre-1.0 versions of the Arduino IDE. I have yet to migrate it.

The solar controller has two interfaces. One, via serial, accepts commands such as 'sm 1200' to set the minimum summer collection temperature to 120 (1200 decidegrees).   The other is via a menu-based LCD on the controller itself (in the basement).  The menu system is complicated enough that it is handled by a separate Arduino (one runnng code from the solarcontroller_slave repository), which is connected to the master via I2C.  The communication protocol between the two is a makeshift one I created only for this project.
