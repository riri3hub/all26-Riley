# Camera Delay

This project supports direct measurement of end-to-end camera delay, using a test rig.

## Test Rig

The test rig comprises a rotating disc, with a small Apriltag on it,
driven at a low speed by a motor.

We make two measurements of the disc angle:

* One of the absolute encoders we commonly use, connected to the Rio.
* A camera running the usual Apriltag detection code.

## Position Sensor Delay

The delay of the position sensor itself is quoted as 0.1 ms:

https://look.ams-osram.com/m/287d7ad97d1ca22e/original/AS5048-DS000298.pdf

The delay due to the protocol is uniformly distributed between zero and one ms,
because the sensor output is repeats every ms, and the Rio provides the
most-recently-read value at reading time, with very little delay.

So the total position sensor delay is uniformly distributed between 0.1 and 1.1 ms.

I'm not sure how to *measure* this delay. I think we should just assume a
fixed 0.6 ms delay for the position sensor.

## Camera Delay

The camera pipeline works as follows:

1. The sensor is made active, usually for a very short time, currently 1.5 ms.
2. The sensor records a blurred average of the real world during that time.
3. The sensor is made inactive, keeping the captured pixels.
4. The pixels are streamed out of the sensor, row by row.
5. When the first pixel arrives at the Raspberry Pi's image signal processor, it records the system time.
6. Quite a bit of time passes, maybe 30 ms.
7. Finally the last pixel arrives, making a complete frame.
8. The frame is used by our analysis code, which takes awhile, maybe another 30 ms.
9. The total delay is computed (see below).
10. The NetworkTables timestamp is computed (see below).
11. The results are sent over NetworkTables.
12. The NetworkTables receiver adjusts the timestamp for clock offset.
13. Our code polls for NetworkTables input.
14. A small amount of work happens to decode the input.

The description above has no adjustable fudge factors, and yet we have felt the need
to add them: there is some source of delay not accounted for here.

### Total Delay

The delay internal to the camera/pi system is implemented in raspberry_pi/app/camera/delay.py,
computed as follows:

1. The image timestamp is rewound by half the shutter open time.
2. The timestamp is rewound slightly more, using an arbitrary constant (calibrated here)
3. The result is subtracted from the current system time, producing the *delay*.

### Network Tables Timestamp

The delay calculated above is subtracted from the Network Tables time (a different timebase
than the system time), to produce the Network Tables timestamp.

## Calibration Principle

The general idea is to measure the tag rotation two ways, and to make sure those two ways
are consistent, no matter the speed of rotation.

* The duty-cycle sensor is very fast, so it produces a measurement that reflects roughly
  the "current position" of the tag.
* The camera is very slow, so it produces a measurement reflecting the "past position", 
  with a timestamp.

Because these two measurements reflect different times, we'll keep a timestamped
history of all the measurements received, and insert the measurements at the correct times.
We then sample the history at a point further in the past than any of the delays: one second ago.

If the static offset is set correctly, and the delay is set correctly, then these samples
should always be the same.  You'll see a log item for the difference, which should be zero.

If the delay is set incorrectly, then the measurement difference will vary with the rotational
velocity of the tag: essentially the "past position" is being inserted into the "wrong" place
in the history.

Our goal is to find a delay constant that makes the measurement difference invariant to rotational speed.

## Running the Calibration

Step by step:

* Initial setup
    * Hook up the Neo Vortex using the REV Client 2.0.
    * Make sure the motor firmware is up to date.
    * Note the CAN id of the motor, make sure the `CanId` in Robot.java matches.
    * Hook up the magnetic sensor to the RoboRIO, make sure the `RoboRioChannel` in Robot.java matches.
    * Made sure the camera delay tests pass.
    * Connect a battery and turn on the test rig.
    * Deploy the code.
    * Start up the Driver Station app and connect an xbox controller.
* Validating the rotary position sensor
    * Start the Glass tool, and make a graph of the output of the AS5048 sensor
    * Remove the sensor from the test rig and mount the "test knob"
    * Turn it
    * The graph should show smooth variation as you turn the knob
    * The graph should look like a full-range "sawtooth".
    * If the graph is not smooth, or not full-range, get help.
    * Mount the sensor to the rig and repeat the test, turning the motor by hand.
    * Placement is sensitive.  Look carefully, make sure it works well.
* Validating the camera
    * Open the camera management page and look at the image feed
    * The tag should be near the center of the image, with the outline consistently applied.
    * In glass, there should be a measurement from the camera.
    * Move the motor by hand and watch the measurement change.
    * Like the sensor, it should be a smooth, full-range sawtooth.  If not, get help.
* Setting the static offset
    * In glass, there should be a measurement from the sensor, and a measurement from the camera.
    * Adjust Robot.STATIC_OFFSET so that the sensor and camera measurements are the same.
    * Move the motor around a few times to make sure the "sameness" doesn't depend on the position.
    * A little bit of noise and variation is ok.
* Setting the delay
    * Enable the RIO
    * Pull the controller "left trigger" to adjust the motor speed.
    * Run the motor at a moderate speed and look at the difference between
      camera and sensor.  If the timing is handled correctly, the difference
      will always be zero.
    * The delay is managed in by the camera code: each camera configuration (`raspberry_pi/app/camera/config`)
      has a different implementation of `Config.extra_delay_ms()`.  Find the correct configuration for the
      camera you're using, and adjust it until the delay doesn't change with speed.


## Total Lag

In addition to calibrating the delay, it would be good to measure
what the delay actually is.  This is logged in Glass by `RawTags`,
under "lag".

The number varies a lot.  I think there are three main reasons for the variation:

* Variance in the frame analysis itself.  The analyzer takes a long time looking at candidate
  "corners" in the image.  A blank white image is handled very fast, an image with lots of
  detail takes a long time.   There seems to be some variation in end-to-end time in the
  python code, even for the same frame twice, and I'm not sure what the cause is.  In any case,
  sometimes the analysis can "fit" between frames (so substantially less than 16 ms), but sometimes
  the camera only picks up every third frame (so extra delay of between 32 and 48 ms),
  so there's between least 16 and 32 ms of variation.
* "Beating" between the frame analysis time and the 60 Hz camera clock
  * Sometimes the python code picks up a frame right after it is available, so the frame is
    only about 20 ms old.
  * If the analysis of one frame takes longer than the collection time for the next one, then
    the analysis loop will gradually fall behind: the next one will be, say, 25 ms old, and
    the one after that will be 30 ms old.  This pattern will continue, but it will be reset
    once the analysis lag is greater than the frame period.
  * So even for a fixed analysis latency, the frame age will vary from "shortest possible" of
    around 20 ms to about twice that, 40 ms.
* "Beating" between the 60 Hz camera clock and the 50 Hz RIO clock.
  * Sometimes the RIO will be ready to read the output exactly when it arrives.
  * Sometimes the RIO is in the middle of its cycle when the data arrives, so it won't read the
    new data until the next cycle, up to 20 ms later.

The total lag variance is the sum: around 20 ms from analysis, around 20 ms from analysis/frame
"beating", and 20 ms from RIO "beating", so around 60 ms total.  This total matches the
observed variation in the total lag, which is between 40 and 100 ms.


## Sources of Lag

Sources of camera delay are discussed extensively elsewhere:

* https://docs.google.com/spreadsheets/d/1eJXAVCPSDcnFvezaWv0vLu5IyZi7KkjZglR7nizP6fc
* https://docs.google.com/document/d/1JPJC3cn6eorBrsogOWRN7W_bMo1DgAfIKKjA5wP1av4
* https://docs.google.com/document/d/1Wmumdv1L7AmdqdHs8FIjNInB5be5Tu6RLEa4f_on6Pk