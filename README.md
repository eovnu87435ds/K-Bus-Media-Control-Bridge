# K-Bus-Media-Control-Bridge
This is an Arduino project to connect to a BMW/Mini K-Bus/I-Bus. This is a low speed "body control" bus that consists of lighting, wipers, park sensors, steering wheel controls, stock stereo, navigation, CD changer, as well as other modules, depending on the year and model. 

This project is designed to read the K-Bus line as a serial input, and connect to my android phone (Pixel XL) as a HID device. When signals from the steering wheel controls are detected, the Arduino will translate them into controls to pass to the phone (Play/Pause, Next/Prev, and Launch Google Assistant).
