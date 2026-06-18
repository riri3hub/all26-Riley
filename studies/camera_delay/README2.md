# README 2

Getting the camera tester working again.

* Hooked up a Neo Vortex, updated its firmware.
* Updated the code in studies/camera_delay, adjusted the can id.
* Made sure the camera delay tests pass
* Deployed the code
* Connect an xbox controller
* Start the Glass tool
* Look at the output of the sensor (AS5048)
* Mount the "knob" to the sensor itself, and turn it
* Make sure the graph in Glass looks good.
* Mount the sensor to the rig and repeat the test.
* Placement is sensitive, make sure it works well.
* In glass, there should be a measurement from the sensor, and a
  measurement from the camera.
* Adjust Robot.STATIC_OFFSET so that the sensor and camera measurements
  are the same.
* Enable the RIO
* Pull the controller "left trigger" to adjust the motor speed.
* Run the motor at a moderate speed and look at the difference between
  camera and sensor.  If the timing is handled correctly, the difference
  will always be zero.
* The delay is managed in raspberry_pi/app/camera/real_request.py, in
  EXTRA_DELAY_MS.  Adjust it until the delay doesn't change with
  speed.

In addition to calibrating the delay, it would be good to measure
what the delay actually is.  This is logged in Glass by RawTags,
under "lag".

The number varies a lot, mostly because of the "beating" of the
camera clock against the RIO clock.  We see between 40 and 100 ms.