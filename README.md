# Pico W ov2640
Captures an image from a OV2640 and sends over a wifi connection to a machine
running the `image_read.py` script.

Currently captures a single image in CIF resolution (352 by 288).

To build, create a secrets.h with format:
```
char wifi_ssid[] = "MySSID";
char wifi_pass[] = "MyPassword";
```

Then run the `image_read.py` script locally and load the firmware on to the Pico.
