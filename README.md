# Common Ros2_control package for differential drive, It supports the following motor drives 
1. Roboteq
2. Moons
3. ZlTech zlac8015

### Udev rules
In order to automatically give read/write permission to connected drives at the start of the system you need to setup some udev rules.
An example of them can be found in [udev.rules](/irctc_udev.rules) 