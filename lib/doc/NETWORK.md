# Network

RoboRIO

10.1.0.2

* VH-109 Robot Radio: 10.1.0.1 
* RoboRIO: 10.1.0.2 mask 255.255.255.0
* VH-109 Field Radio: 10.1.0.4 
* Driver Station: 10.1.0.5 mask 255.255.255.0 default gateway: 10.1.0.4
* Other devices: 10.1.0.6 to 10.1.0.19 mask 255.255.255.0

https://docs.wpilib.org/en/stable/docs/networking/networking-introduction/ip-configurations.html#static-configuration

If this doesn't work, the roborio could be using the link-local fallback
address, which is something in the 169.254.0.0 network.

169.254.x.y

https://docs.wpilib.org/en/stable/docs/networking/networking-introduction/networking-basics.html

If this happens, the roborio is probably set up wrong.

try connecting to it at

roboRIO-100-FRC.local

http://roborio-100-frc.local/#!/SystemConfig

Make sure the IPv4 configuration is not DHCP.  This is the bad configuration:
<img src="network-bad.png"/>

Change the configuration to "static" and fill
in the correct IP address (10.1.0.2) and
mask (255.255.255.0).

The Windows network also needs to be set up
statically.

<img src="windows.png"/>

The cameras use addresses like 10.1.0.30, 10.1.0.31, etc.

