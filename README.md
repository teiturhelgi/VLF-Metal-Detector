# VLF-Metal-Detector
Metal Detector designed and built as a project for a three-week course in Electromagnetics at DTU.

I mainly worked on the signal processing and programming of the microcontroller unit (ESP32). 
The signal processing that was used was a simple DFT algorithm followed by a IIR filter 
for the OLED display. By sampling with a frequency 4x higher than the sampled signal, the
sine and cosine calculations in the DFT become very simple and can be easily realised with 
a few lines of code (see make_dft function below).

The program is written in C and uses a FreeRTOS Task notification system along with interrupts 
to manage the various tasks. Sound toggle and calibration buttons guarded by GPIO interrupts. 
A timer interrupt controls the sampling and output square wave signal. The sound feature is 
realised using beeps at an increasing frequency with increasing amplitude of the received signal. 



System description:

The system is powered by a 9V battery, connected through a switch that can toggle the power 
to the system on and off. Power is supplied to the micro-controller and the power amplifier, 
which takes a square-wave signal from the micro-controller and turns it into an oscillating 
signal in the transmitter coil. The oscillating current in the transmitter coil creates a 
magnetic field, inducing eddy currents in a nearby metal object. This current in the object
in turn generates an opposing magnetic field, which induces an oscillating current in the 
receiver coil. The reflected signal is filtered with an analog low-pass filter and amplified 
with an operational amplifier, before being sampled by the micro-controller’s ADC.



Project report: 
https://www.dropbox.com/s/6226cem7v92r3bm/Metal_detector_project_62739_Group_10-3.pdf?dl=0



System overview:
![Metal%20detector](https://user-images.githubusercontent.com/58830507/226971663-24cc26eb-e046-4368-864e-edbc3a7d76d8.jpg)

ESP32 Processor diagram:
![ESP32%20Processor%20Design%20](https://user-images.githubusercontent.com/58830507/226971114-2d54ec5a-f927-4fe9-98da-83a64ce6b4ae.jpeg)

DFT Function diagram: 
![ESP32_make_dft_v2](https://user-images.githubusercontent.com/58830507/226967423-6c472080-fa5d-4320-b816-a61e58c2d486.jpeg)

Task Diagrams:
![ESP32_Tasks_DFT_Cal](https://user-images.githubusercontent.com/58830507/226967551-fec11e18-da59-4512-9d8b-877dc3c6266f.jpeg)
![ESP32_Tasks_Display_Sound_v2](https://user-images.githubusercontent.com/58830507/226967618-d53e1cb9-a898-429e-bb75-b768ac668295.jpeg)

Interrupt Service Routine Diagrams: 
![ESP32%20ISR's%20v3](https://user-images.githubusercontent.com/58830507/226967108-0ca28fc5-7a06-4c09-a877-eb90b869d7f6.jpeg)

