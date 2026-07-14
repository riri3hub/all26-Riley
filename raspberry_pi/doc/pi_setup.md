# Raspberry Pi Setup

This is the new setup procedure for 2027, which does not
use the unmaintained "WPILibPi" distribution.

We use only the Raspberry Pi 5.  We have a bunch of 4's,
but we don't use them, they're too slow.

## Burn the SD Card

* Make sure your laptop is connected to the internet.

* Use the [Raspberry Pi Imager](https://www.raspberrypi.com/software/) app.

* Use the [Raspberry Pi OS Lite (64-bit) "Trixie" image](https://www.raspberrypi.com/software/operating-systems/)
    * The "lite" option is behind the "other" option.  Don't use the "full" image.
    * Choose a unique name like "camera31," "camera32" etc.
    * add a user "pi" with password "raspberry"
    * enable SSH with password authentication

## Configure the network

Set up the static ethernet addressing scheme.

* In the `bootfs` volume, edit file called `network-config`, using Notepad.
  * This is a file containing configuration in
    the [format](https://docs.cloud-init.io/en/latest/reference/network-config.html) 
    used by `cloud-init`.
* Use an IP between 10.1.0.30 and 10.1.0.35.
* Make the ethernet `optional` to avoid holding up the boot.
* The robot hosts the pi needs to talk to are all on the same
  segment, so no gateway is required for that.
* For the part of the setup involving the internet (below),
  you'll be bridging with your laptop, so there
  needs to be a default gateway.  What should it be?  When you're
  working with cameras, you're usually impersonating the RoboRIO
  using 10.1.0.2, so we'll use that.
* The "internet" part also needs name resolution, so we'll use Google,
  which is `8.8.8.8`
* The file `network-config` should look like this:

```yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses: 
        - 10.1.0.31/24
      nameservers:
        addresses:
          - 8.8.8.8
      routes:
        - to: default
          via: 10.1.0.2
      optional: true
```
* Save the file with ctrl-S and close Notepad.

## Turn off the radios

Turn off the wifi and bluetooth radios (because FRC prohibits them)

* In the `bootfs` volume, find the file called `config.txt`, using Notepad.
  * This file uses a simple format, `name=value`
* Add these lines at the top of `config.txt`:

```ini
dtoverlay=disable-wifi-pi5
dtoverlay=disable-bt-pi5
```
* Save the file with ctrl-S and close Notepad.

## Turn on the Pi

* Install the SD card, reassemble the Pi in its case, and and turn it on.
* IMPORTANT: WRITE THE IP ADDRESS ON THE PI CASE, or you won't know what it is, later.

## Set up your laptop network

Give the Pi access to the internet:

* Connect your laptop to wifi, and "share" the connection
  * On windows, sharing wifi changes the Ethernet address, so you need to change it back:
* Change your ethernet address to `10.1.0.2`.

## Connect

* Connect the Pi to your laptop with an ethernet cable,
  or through a switch if you're working on many Pi's at the same time.
* Get a command window ("cmd" on Win11)
* type `ssh pi@10.1.0.31` (or whatever the address is)
* use password `raspberry`
* verify the pi can see the internet (`ping 8.8.8.8`)

## Update the Pi

* Make sure the OS is up to date.

```
sudo apt update 
sudo apt upgrade 
```

## Install Pi Camera Support

* Install camera packages.

```
sudo apt install libcamera-dev 
sudo apt install python3-picamera2 
```

## Set up Python

To avoid dependency between our app and the Raspberry Pi
python installation, we use the python "virtual environment"
feature.

* Create a virtual environment.
```
python -m venv ./env
```
* Install required libraries in the virtual environment.
```
source ./env/bin/activate
python -m ensurepip --default-pip
python -m pip install --upgrade pip
python -m pip install numpy
python -m pip install opencv-python
python -m pip install robotpy
python -m pip install robotpy-apriltag
python -m pip install robotpy-cscore
```

## Install Supervisord

For now, the supervisor installation is described in `studies/piborg`.

## Troubleshooting

What if the steps above don't work?

* LED flashing 4 slow, 3 fast
  * This indicates hardware damage.

* you can try flashing the EEPROM, using a different image from the 
  raspberry pi imager.  if it succeeds, the green LED flashes forever.

## References

* [Raspberry Pi doc on cloud-init](https://www.raspberrypi.com/news/cloud-init-on-raspberry-pi-os/)
* [Raspberry Pi doc on config.txt](https://www.raspberrypi.com/documentation/computers/config_txt.html)
* [Netplan format](https://netplan.readthedocs.io/en/latest/netplan-yaml/)
* [Netplan examples](https://netplan.readthedocs.io/en/0.106/examples/)
* [network-config format](https://docs.cloud-init.io/en/latest/reference/network-config-format-v2.html#network-config-v2)
* [post about network-config](https://forums.raspberrypi.com/viewtopic.php?t=396296)
* [another post about network-config](https://forums.raspberrypi.com/viewtopic.php?t=396987)
* [post about config.txt](https://help.pisignage.com/hc/en-us/articles/52406458481817-Disabling-Wi-Fi-and-Bluetooth-on-Raspberry-Pi)
* [Raspberry Pi LED flashing codes](https://www.raspberrypi.com/documentation/computers/configuration.html#led-warning-flash-codes)

## Appendix

For developent, we need this, but I think not for the pi:
```
sudo apt install python3-aiohttp 
```