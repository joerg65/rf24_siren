RF24 Siren

Runs on ATtiny84. This is part of my Alarm Central written on Odroid C1 / Android.
It sends every 4s the state of sabotage contact and voltage via nRF24. After sending the payload it activates receiving mode and goes to sleep for 4s. A receiving message or a pin change of sabotage contact does also wake up the cpu. The receiving message includes the node and a command of two bits: bit0 - flash light, bit1 - siren. The siren output is deactivated after timeout of approx. 180s due to local law.
An unconfigured sensor sends its node FFh to the Central in plain data then changes to receiving mode. The Central does a autonumbering an sends the node and the AES128key to the sensor. The message contains a Xmodem CRC16 field at byte17:18. It is calculated over the first 17 bytes byte0 node and byte1:16 AES128key. The sensor stores this data to its Flash and does a reboot. From now on it sends every 4 seconds, or at pin change of sabotage contact, node, battery value, state (sabotage/ok) and received command AES128 encrypted to the Central.

The basic of the mirf library I found here: https://github.com/MattKunze/avr-playground/tree/master/mirf  
And the AESLib I found here: https://github.com/DavyLandman/AESLib  

The project was made with Eclipse, so if someone want to make it with Eclipse, it need to be added the 'AVR Eclipse Plugin' to Eclipse.  

To import the source code to Eclipse I found the easiest way as this:  

Select File/New/Other and then select C Project and Next. Give Project name and select AVR Cross Target Application and Next. Deselect Debug and the select Finish.
Next, select your new Project with right mouse click and select Import.../Filesystem and browse for the source code folder. Mark in the left window to select everything and select Finish.

It should build now without errors. To build, select Project/Build Project. The output should be:
```
Device: attiny84

Program:    3488 bytes (42.6% Full)
(.text + .data + .bootloader)

Data:        116 bytes (22.7% Full)
(.data + .bss + .noinit)


Finished building: sizedummy
 

16:41:36 Build Finished (took 1s.283ms)
```

To flash with avrdude:
```
avrdude -cavrisp2 -P/dev/ttyACM0 -pt84 -Uflash:w:rf24_window.hex:a -Ulfuse:w:0xe2:m -Uhfuse:w:0xdf:m -Uefuse:w:0xfe:m
```
Of course one need to adjust the '-cavrisp2' to his flashing tool and depending also the port.

