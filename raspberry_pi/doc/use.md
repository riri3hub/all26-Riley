# How to use this thing

If you've followed the directions to ...

* [Setup a Raspberry Pi](pi_setup.md)
* [Deploy to the Raspberry Pi](deploy.md)

... then the next step is to see what this thing does.

## Setup your laptop to impersonate the RoboRIO

* Connect to the Pi using ethernet
* Set your laptop IP to `10.1.0.2`.
* Find the folder on your desktop called "2026 WPILib Tools"
* In that folder, find the `glass` app, and start it.
* Find the "NetworkTables Settings" window and apply "server" mode.

## Look at glass

* Find the "NetworkTables" window.
* There should be a section starting with "pi", with the ID of the camera
* There should also be a section starting with "vision"
* So NetworkTables would include stuff like this:
```
> pi
  > 12345asdf
    > calibrate
    > undistort_view
> sync
  > 12345asdf
    > request
> vision
  > 12345asdf
    > blips
```

## Look at supervisord

* Find the IP address of the camera (you wrote it down earlier).  Say it's `10.1.0.30`.
* Use a browser to connect to the camera on port 9001, e.g. `10.1.0.30:9001`
* This should show the `supervisor` web interface.
  * It should show one process, called "app", which should be in the green "running" state.
  * You can use this to look at logs, or restart the app.

## Look at the dashboard stream

The main camera stream is small (to avoid overloading the network) and annotated.

* Use a browser to connect to the camera on port 1181, e.g. `10.1.0.30:1181`


## Look at the undistorted view

The undistorted stream is large, unannotated, and optional.

* In glass, toggle the "undistort_view" value
* Use a browser to connect to the camera on port 1182, e.g. `10.1.0.30:1182`
  