# Run Locally

How to run the code and the tests.

# Run on your laptop

Everything except the camera itself works on your laptop.
There's a "fake" camera instead, which loads a file from
the `images` directory.

From the command line in the raspberry_pi directory:

```
python3 runapp.py
```

Alternatively, use vscode to open runapp.py, and
press the litle "run" button.

Then look at http://localhost:1181/

And you should see the test image being processed over and over.

# Run the tests

There are two ways to run the tests:

Install the vscode python extension, and a little Erlenmeyer flask will appear in the toolbar on the left; this scans the source tree for test methods, and makes a little list of them.  You can run all of them, or individual ones.

The same extension also puts a little marker in the file editor, on the left side of each test method, and provides test output there too, in case of failures.

You can also run all the tests from the command line from the raspberry_pi directory:

```
python3 runtests.py
```
