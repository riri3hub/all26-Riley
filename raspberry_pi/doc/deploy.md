# Deploy

How to deploy to the Raspberry Pi.

## Overview

Runtime:

* The code is packed into a "zip" file.
* There's a very short script, `runapp.py` that runs
  the app directly out of the zip file.

Deployment:

* Gradle [distribution](https://docs.gradle.org/current/userguide/distribution_plugin.html)
  plugin to make a zip file.
* Gradle [gradle-ssh](https://gradle-ssh-plugin.github.io/docs)
  plugin to copy the zip to the pi, and to run commands on the pi.

Management:

* To start and stop the app, restart it when it fails,
  and to look at its logs, we use
  [supervisor](https://supervisord.org/index.html).
  * See [pi setup](pi_setup.md) to set up supervisor.


## How to do it

Deploying is a simple matter of finding
the gradle task and running it.

* Install the vscode Gradle extension (from Microsoft).
* In the gradle "activity" pane, find the deploy task:
    * `raspberry_pi > Tasks > deploy > Deploy Camera Code`.
* Click the little play button next to the task.
* In the "terminal" window, look at the output; there
  should be at least one "success".
* Use your laptop browser to connect to `10.1.0.31:9001`
  (or whatever IP).
  * you should see the supervisor web UI,
    with the app in the green "running" state.
* You can click on the "stdout" or "stderr" links to see
  the normal and error logs, respectively.


## What it does

You can see what the deploy does by reading
`build.gradle`.

* Whe `distribution` plugin makes a "zip" file.
* The `ssh` plugin connects to each Pi, one at a time, and:
  * copies the `runapp.py` file to `/home/pi`
    (even though it rarely changes).
  * copies the zip file. to `/home/pi`
  * restarts the app, using `supervisorctl`.

It tries to deploy to all the cameras that could possibly
be connected, so it will fail for cameras that don't exist.
This is fine.  :-)

You should be able to see from `build.gradle` how to adapt this
system to a larger number of cameras, or cameras with different
IP addresses.

## Appendix

It's also possible to run the deploy on the command-line, if you
want.

* If you haven't before, choose `Tasks > build setup > wrapper`
  from the vscode Gradle pane.
  * This puts `gradlew` at the project root.
  * Only do this one time.
* Type `./gradlew deploy`