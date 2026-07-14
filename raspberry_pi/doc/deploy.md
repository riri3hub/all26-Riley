# Deploy

How to deploy to the Raspberry Pi.

## New way

See `studies/pi_borg`

## Old way

All our deployments use the same artifact,
built out of this directory, using the board identity to
select behavior.

We use python's ability to load modules out of zip files,
so you zip up the source directory and upload it.
There's also a very short python script, runapp.py,
that runs the zip file -- you just upload this one time.

* Install the vscode Gradle extension (from Microsoft).
    * It will make available a Gradle "activity" which includes the task "distribution/distZip".
* Click the little play button
    * It will produce a "zip" file containing the "app" code.
    * To make it easier to find, you can "pin" the task.
* Find the zip file in build/distributions/app.zip.
* Upload it to the RPi, next to runapp.py.