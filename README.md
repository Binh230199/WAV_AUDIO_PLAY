# WAV_AUDIO_PLAY
Play .wav Audio File using STM32 DAC and SD card 

This project was done when I was a student, so it could not be optimized.

In this project, a .wav audio file is stored on an SD card. An audio file is 8 bits at 16 kHz; I tested with 48 kHz, but it did not run.
STM32 reads the file part by part and transfers data to the DAC, so your board must have a DAC peripheral. You can use PWM instead of a DAC.
