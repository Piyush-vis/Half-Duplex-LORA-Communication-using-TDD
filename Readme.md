# Half Duplex LORA Communication using TDD

A Transciever using CSS( chirp spread spectrum) modulation.

## The components:

* 2x ESP32
* 2x LoRa modules(SX1278)
* 433 Mhz Antenna
* Lora module to antenna connector
* Jumper wires, Breadboard and other stuff

# Working:

For the current version, the transciever works by taking inputs for transmitting messages and showing recieved messages through the Serial monitor only.

# Features Implemented:

CRC16 error correction on the application layer.
A fixed sync word for all the messages.
Uses message-acknowledgment pairs for automatic repeat request protocol.