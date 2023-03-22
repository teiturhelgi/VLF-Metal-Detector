# VLF-Metal-Detector
Very Low Frequency Metal Detector designed and built as a project for three week course in Electromagnetics at DTU.


System description:

The system is powered by a 9V battery, connected through a switch that can toggle the power 
to the system on and off. Power is supplied to the micro-controller and the power amplifier, 
which takes a square-wave signal from the micro-controller and turns it into an oscillating 
signal in the transmitter coil. The oscillating current in the transmitter coil creates a 
magnetic field, inducing eddy currents in a nearby metal object. This current in the object
in turn generates an opposing magnetic field, which induces an oscillating current in the 
receiver coil. The reflected signal is filtered with an analog low-pass filter and amplified 
with an operational amplifier, before being sampled by the micro-controller’s ADC.
