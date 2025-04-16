# Goodix GTM802 touchscreen driver
This is a driver for the GTM802 touchscreen controller found in old tablets. I've used (Priit Laes)[https://patchwork.kernel.org/project/linux-input/patch/1449473161-3535-3-git-send-email-plaes@plaes.org/#16453331] code as the source for this driver, as the other one seems to be similar. I've also used the original (ViewPad10e kernel source)[https://web.archive.org/web/20130617230114if_/http://www1.viewsonic.com/support/downloads/drivers/_download/tablet/viewpad10e/ViewPad10e_Kernal_Source_code.zip] to extract some initialization configs.

# Usage on RPi
Because I've used the majority of the GPIOs on my RPi (using the parallel DPI in 18 bit mode), I had to use the `i2c-gpio` (which conveniently uses GPIOs not in use by the DPI).
There are two other lines that require to be connected, the RESET and the INT pins. I've used GPIO25 for the INT and GPIO26 for RESET.
See `gtm802.dts` for the actual overlay fragment used.

## Build the DTS file
NOTE: If you're not going to use the `i2c-gpio` and rather use other I2C bus, please modify the gtm802.dts, and replace `&i2c_gpio` for the appropriate phandle (like `i2c_arm`, `i2c` on the RPi)
1. Build: `dtc -@ -Hepapr -I dts -O dtb -o gtm802.dtbo gtm802.dts`
2. Copy it as an overlay: `sudo cp gtm802.dtbo /boot/firmware/overlays/`

## Build and install the .ko module
NOTE: You should need to have the environment setup (i.e. gcc, kernel headers, etc).
1. `make`
2. `sudo make install`
3. Not sure if it's required, but `modprobe `

## Enable overlays and module
1. (If you plan to use `i2c-gpio`): If you don't have the `i2c-gpio` overlay already loaded, you can enable it by doing this:
`echo "dtoverlay=i2c-gpio" | sudo tee -a /boot/firmware/config.txt > /dev/null`
Just run this once if you don't have `i2c-gpio` overlay.
2. Enable the gtm802 overlay:
`echo "dtoverlay=gtm802" | sudo tee -a /boot/firmware/config.txt > /dev/null`

## Final steps
I had an issue where the touch screen was 180º rotated from the actual display. My solution was to disable Wayland (couldn't make it work with wayfire), and add the following line to `/usr/share/X11/xorg.conf.d/40-libinput.conf`:
`Option "CalibrationMatrix" "-1 0 1 0 -1 1 0 0 1"`
(It should be on the `libinput touchscreen catchall` section)


# Notes
I've left out the proper reset procedure, which makes the host drive the INT pin as well as setting the RST pin.
