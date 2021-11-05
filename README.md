# Foolproof Temperature Checker R5
This is a project for hackster.io contest "Reinventing Healthy Spaces with AWS IoT Edukit". The project consists of:
* M5Stack Core2 for AWS IoT Edukit (ESP32)
* OpenMV H7 plus camera
* IR sensor MX90614
* PIR sensor

Some countries have implemented compulsory order following COVID 19 pandemic to wear facemask and check for body temperature when enter a premise. This checker is designed to check if the person is wearing facemask before checking body temperature. Besides, the checker will also connect with AWS IoT Core to confirm if to enable/disable checking, depends on the number of people occupying the premise if exceed the threshold.

P/S: This repo simply to document the work and hopefully serve as reference material for similar project (if any).

- Code for Core2 in PlatformIO
- Code for OpenMV H7 Plus
- Link for post in Hackster
