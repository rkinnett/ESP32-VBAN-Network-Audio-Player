# ESP32 VBAN Network Audio Player
 ESP32 receives audio stream over Wifi, via VBAN protocol, and outputs analog audio.

This was developed as part of a larger project which aims to use the ESP32 as an audio relay between a PC and a VHF/UHF radio to facilitate wireless voice and data modes:

https://hackaday.io/project/170710-esp32-tnc-and-audio-relay-for-hfvhf-packet-radio



Requires:  VBAN Voicemeeter
https://www.vb-audio.com/Voicemeeter/

Supports 16-bit PCM Mono only (but outputs only 8-bit analog).

VBAN SampleRate values 16000 and 24000 Hz do not work well due to odd packet lengths which do not work reliably with the ESP32 DMA buffer.  Other sample rates up to and including 64000 Hz work reliably on my system.  Higher data rates may be supported with further code optimization.  The ESP32 dynamically adapts to VBAN sample rate.



PC Configuration:

Windows sound output device:  select "Voicemeeter Input"
  (Alternatively, for use with soundmodem or similar software, leave the Windows sound output device as your normal sound device, and instead select "Voicemeeter Input" as the output device within soundmodem settings.)

VBAN Voicemeeter:
  Virtual Input:
    Select ">A" to route sound to VB-audio bus A
    
  Menu -->
    VBAN Options (VB-Audio network) -->
      Outgoing Streams #1:
        Source:  BUS A
        Stream Name:  anything
        IP Address To:  IP address of the ESP32
        Port:  6981 (must match value of udpInPort in arduino sketch)
        Sample Rate:  (anything but 16000 and 24000 which do not work well)
        Ch: 1
        Format:  PCM 16 bits
        Net Quality:  Optimal
        
        
        
Hardware Configuration:

The arduino sketch by default outputs analog audio to GPIO26.  GPIO25 may be used instead by chaning i2s dac mode and channel format.  I use GPIO26 to drive a simple open-collector amplifier and 8-ohm speaker.
