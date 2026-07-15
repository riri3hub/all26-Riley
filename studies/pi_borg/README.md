# Pi Borg

This is a study of management and deployment of Python to our Raspbery Pi coprocessors.

* [supervisor](https://supervisord.org/index.html) for management.
* gradle for test, packaging, and deployment.
    * [distribution](https://docs.gradle.org/current/userguide/distribution_plugin.html) to make a zip file
    * [gradle-ssh](https://gradle-ssh-plugin.github.io/docs) to copy the zip to the pi, and to run commands on the pi

There's a [design doc](https://docs.google.com/document/d/15mUhT1vEGO8y0hBersCINBw1t07pDdcYMedGS_gy2Vk/edit?tab=t.0).

The name "borg" references the thing Google uses for deployment and management,
which works completely differently, but it's a pithy name.

## Set up Supervisord

Once the Raspberry Pi is
[set up](../../raspberry_pi/doc/pi_setup.md),
we need to add supervisord and configure it.

For these steps, the ssh password is "raspberry".

* Copy the supervisord conf file to the pi.
* Install supervisord.
    * Because we want it to run at startup, we use the
      system package.
* Move the conf file to the privileged location.
* Get supervisor to read it.
```
scp supervisord.conf pi@10.1.0.31:/tmp
ssh pi@10.1.0.30
sudo apt install supervisor
sudo mv /tmp/supervisord.conf /etc/supervisor
sudo systemctl restart supervisor
supervisorctl status
```

If you haven't yet installed the app code, you should see supervisor
complain that it can't find it:

```
app                              FATAL     can't find command '/home/pi/venv/bin/python'
```

At this point, your laptop browser should show the supervisor
web page at `http://10.1.0.31:9001/`, with the red "fatal" status.

## Make the zip file

The first step is to use the `distribution` plugin to
make the zip file.

The code here is structured in the same way as our actual pi camera code,
with a script ("runapp.py") at the project root that loads the "zip" file
and runs "main".  There's a directory full of modules ("app") and a
directory full of tests ("tests").

* Setup vscode
    * Choose __Tasks >> build setup >> wrapper__ from the vscode Gradle pane.  This puts `gradlew` at the project root.
* Run the distribution job.
    * There are two ways:
        * Run distribution on the command line
            * You can say `./gradlew distZip`
        * Run distribution from vscode
            * You can choose __Tasks >> distribution >> distZip__ from the Gradle pane.
* Find the zip file
    * One way or the other, the result apears as `build/distributions/app.zip`

## Copy the files

Using the gradle pane, find the task `pi_borg > Tasks > other > deployAll`

Run the `deployAll` task.

* Gradle copies `runapp.py` to `/home/pi`
* Gradle copies `app.zip` to `/home/pi`

## Restart the app

* Gradle uses `supervisorctl` to control the app.

## Confirm the app is running

* Connect to `10.1.0.30:9001` (or whatever IP), and you
  should see the supervisor web UI.

