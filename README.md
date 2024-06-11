# DGNSS_Ranger
Shoulder-Mounted Collapsable LIDAR-DGNSS Rangefinder/Locator Development Platform

![](https://github.com/sparkfunX/DGNSS_Ranger/blob/main/Ranger_B%20v25.jpg?raw=true)

## Theory of Operation 
By using two GNSS receivers operating in RTK with corrections data from a fixed base (or comparable corrections source) the relative position of two antennas can be determined. This relative position can be used to derive a heading without the usue of a magnetic compass. Furthermore, by placing a ranging sensor (in this case, a LIDAR) in line with these antennas, it's possible to derive the Geodetic coordinates of a subject at distance. This is acheived by virtually extending a line that connects the two antennas by the distance reported by the LIDAR. This may be useful in applications such as surveying, geotagging, structural measurments, etc. where it's neccessary to retrieve the coordinates of an inaccessible subject. The shoulder-mounted form factor of this platform makes it easier to use and transport than many traditional surveying tools, at the expense of some positional accuracy. 

## Sources of Error
We've determined the theoretical maximum 3D positional error of the platform at full distance (40 meters) to be 0.85 meters. This error is the result of compounding error from various sources.

#### GNSS Error
The maximum 3D accuracy of the ZED-F9P with a clear view of the sky and adequate corrections data is 0.02 meters. This error is inherent to the method and can't be technically addressed by this project.

#### Differential Error at Distance 
Assuming the maximum 3D positional error of the forward antenna is 0.02m, and that its distance from the rear antenna is 0.95m, we derive an angular error of 1.206° (The α angle of the right triangle with b side 0.95 and a side 0.02) such that at 40 meters, the maximum positional error becomes 0.85 meters (The a side of the right triangle with α angle 1.206° and b side 40). This error can be reduced by increasing the length of the platform and, therefore, the distance between the two GNSS antennas. This is because the angular error is reduced with an increased distance given the same positional error (Observe the difference in angle between a right triangle with b side 0.95 and a side 0.02 and another right triangle with b side 2 and a side 0.02). The distance between the two antenna does not need to be calibrated, as it is measured using DGNSS, and so it is possible to extend the platform's langth as much as is practical in a given application without modifying the firmware. This error also shrinks with the distance to the subject and so more accurate coordinates are derived from nearer subjects. 

#### LIDAR Ranging Error
The accuracy of any LIDAR rangefinder will depend on a number of environmental factors, including the cross-sectional area and reflectivity of the subject to be measured. For this reason, ranging errors are not included in our maximum error figure. 

## Electrical Assembly
Almost all of the electronics on the platform are SparkFun Qwiic development boards and can be connected using Qwiic cables in whatever order is most mechanically convenient. The only exceptions are the trigger button and the LIDAR-lite module. The trigger button is simple wired between Pin 13 and GND on the Thing Plus development board. The LIDAR-lite is wired to the Qwiic Boost board. Be careful to connect the forward and rear ZED-F9P breakouts to their respective antennas. The platform can be powered from any 5V power source using the USB-C connector on the Thing Plus development board. 

## Bill of Materials
The full bill of materials for this project can be found [here](https://github.com/sparkfunX/DGNSS_Ranger/blob/main/DGNSS%20Ranger%20BOM.pdf). Most, if not all, of the mechanical parts can be substituted with a suitable equivalent. The grip, for instance, can be substituted with any accessory-rail-mounted foregrip. The platform is built using two pieces of aluminum extrusion which are connected end-to-end so that it can be dismantled for travel, but a single piece of extrusion will also work. The length of the extrusion is not critical, and the platform will become more accurate the longer it is (see "Differential Error at Distance"). All of the 3D printed parts for the platform can be found in this repository and are linked from their respective rows in the BOM. The parts used in our prototype were all printed on a Creality CR-10 FDM printer using PLA filament. 

## Firmware
The firmware as provided here is INCOMPLETE. It can currently initialize the platform and generate example measurements using a fixed distance measurement, some small work needs to be done to retrieve real distance information from the LIDAR unit. The user interface is extremely minimal, with debug information being provided over a serial connection. 

## Provided As-Is
This platform is a work in progress and should not be considered a complete, or even functioning, measurement platform. 

Repository Contents
-------------------

* **/Firmware** - Arduino Code
* **/Mechanical** - Fusion 360 design archive, STL models, and drawings

License Information
-------------------

This project is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact technical support on our [SparkFun forums](https://forum.sparkfun.com/viewforum.php?f=152).

Distributed as-is; no warranty is given.

- Your friends at SparkFun.
