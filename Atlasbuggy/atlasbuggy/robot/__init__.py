"""
This module handles all interfacing with serial ports and passing data from them.
The idea is every microcontroller gets one corresponding robot object and port (which runs
on its own process).
What data you send over serial is up to you. There is no strict packet protocol.
"""
