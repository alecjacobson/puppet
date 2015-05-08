# Tangible and Modular Input Device for Character Articulation

Prototype application and firmware/driver for device described in "Tangible and
Modular Input Device for Character Articulation" [Jacobson et al. 2014].

## Dependencies
Building on Linux is significantly easier than on Mac OS X. Building on Windows
is probably possible, but never tested. The major issues arise from the use of
FTDI which is poorly supported on Mac OS X.

 - c++11
 - ftd2xx
 - libigl
   - eigen3
   - openGL
   - tetgen
   - mosek
   - glut (preferrably [patched](http://www.alecjacobson.com/weblog/?p=3659) to
     support mouse wheel and command key)
 - boost
 - alut (optional for user study apps)
 - openAL (optional for user study apps)

## Installation

On Mac OS X, one must first install an old version of the ftd2xx drivers. See
http://www.alecjacobson.com/weblog/?p=2934

If you have previously installed the ftdi usb serial drivers then you will need
to uninstall them or "unload" them *every time* you restart your computer:

    sudo kextunload /System/Library/Extensions/FTDIUSBSerialDriver.kext/

or 

    sudo kextunload -b com.apple.driver.AppleUSBFTDI

Install the above dependencies and then (crossing fingers) issue:

    make
