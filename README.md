# Multiband Phasing Receiver (MBPR)
This is an HF SSB direct conversion receiver that tunes from 3.5MHz to 30MHz. Quick access to the ham bands is provided. The architecture is based on a quadrature sampling detector and image rejection is implemented with a polyphase RC network. The polyphase RC network provides about 40dB opposite sideband suppression using 1% components. This makes it also suitable for listening to AM broadcast stations. Emphasis was on simplicity and easy to get components and easy to build (even though it uses surface mount components).
# Circuit Description
For best results, the receiver should be connected to a resonant antenna with a low impedance at the desired frequencies. Signals from the antenna are AC coupled to a pair of back-to-back diodes to protect the input from very large signals. C7 couples the signal to U2 which is a 4-way switch that determines which of the 4 bandpass filters is active. U2, a CMOS switch, requires a DC bias of 2.5 volts which is provided by R5 and R8 and is coupled via L7 a 10uH inductor. The inductor allows DC to pass but blocks RF thus keeping the RF energy out of the power supply. Another 4-way switch selects the output of the filter to pass to the RF amplifier. The quadrature sampling detector (QSD) responds to the 3rd harmonic with a loss of only 9dB. The filters have good attenuation of the 3rd harmonic as well broadcast band interference on the low side.

The RF amplifier provides a gain of about 16dB, a 50 ohm termination for the bandpass filters, and a bias voltage at the input and output for U1 and the QSD U3. The MS5351 is clocked by a TCXO, so no calibration is required, at a frequency of 26 MHz. A quadrature signal is used to clock U3, another 4-way multiplexer acting as a QSD. The 100 ohm resistors with capacitors to ground terminate the RF signal with a cut-off frequency of about 16KHz. A polyphase RC network then phase-shifts the in-phase and quadrature signals from the QSD so that when subtracted at their output produce an upper sideband signal with a suppressed lower sideband. With 1% components in the network, better than 40dB opposite sideband suppression is achieved in practice. The SSB filter consists of a total of 5 poles of lowpass filtering in conjunction with a notch filter that delivers a bandwidth of about 2400Hz. To listen to lower sideband the phasing of the quadrature signal from the MS5351 is reversed.

A headphone amplifier is implemented with an LM4875 which has a maximum gain of 20db, operating in bridge mode (so no DC blocking capacitor is required) and a voltage controlled attenuation function. AGC is controlled by the microcontroller by monitoring the audio signal and generating a PWM signal which is filtered by R38 and C61 to provide a DC attenuation voltage to the VOL input of the amplifier. A 7805 regulator provides 5V DC to all the devices except the 3.3V devices which are powered from the 3.3V regulator on the microcontroller board.
# AGC
# Component Notes
# VK7IAN
If you would like to customise the splash (start-up) page, just uncomment the YOUR_CALL macro definition (in MBPR.ino), set it to your callsign and recompile the project. 
# Build (Arduino IDE Settings)
Arduino IDE build settings are as follows:

 * Optimize: -O2
 * USB Stack: No USB

Other settings are as default.
