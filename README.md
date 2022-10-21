# Pico W ov2640
Captures an image from a OV2640 and sends over a wifi connection to a machine
running the `image_read.py` script.

Currently captures an image in SVGA resolution (800 by 600) when you press a button.

To build, create a secrets.h with format:
```
char wifi_ssid[] = "MySSID";
char wifi_pass[] = "MyPassword";
```

You'll also need to edit the IP address near the top of main.c.

Then run the `image_read.py` script locally and load the firmware on to the Pico.  

The Pico will light an LED after booting and connecting and then you can take images by pressing a button.
