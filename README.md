# Multiband Phasing Receiver (MBPR)
This is an HF SSB direct conversion receiver that tunes from 3.5MHz to 30MHz. Quick access to the ham bands is provided. The architecture is based on a quadrature sampling detector and image rejection is implemented with a polyphase RC network. The polyphase RC network provides about 40dB opposite sideband suppression using 1% components. This makes it also suitable for listening to AM broadcast stations. Emphasis was on simplicity and easy to get components and easy to build (even though it uses surface mount components).
# Circuit Description
# AGC
# Component Notes
# VK7IAN
If you would like to customise the splash (start-up) page, just uncomment the YOUR_CALL macro definition (in MBPR.ino), set it to your callsign and recompile the project. 
# Build (Arduino IDE Settings)
Arduino IDE build settings are as follows:

 * Optimize: -O2
 * USB Stack: No USB

Other settings are as default.
