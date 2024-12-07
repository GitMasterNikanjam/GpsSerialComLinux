# GpsSerialComLinux Library 

- This library is used for GPS UART communication with Raspberry Pi or general Linux-based OS.   
- It uses the TinyGPSPlus library for GPS NMEA data parsing, written by Mohammad Nikanjam.  

## Some Features of this library

    1- UART communication and configuration for read/write GPS serial data.    
    2- GPS NMEA data parsing by TinyGPSPlus library.    
    3- Manage time by chrono library. eg for get micros() functions.    
    4- For raspberry pi base systems: Used pigpio library for set PPS GPIO pin and PPS signal interrupt. PPS (Pulse Per Second) pin of GPS trigged exact every 1 second.    
    6- For raspberry pi base systems: It can set any GPIO pin for PPS signal and interrupt.    
    7- UART configuration is based on non blocking mode. So read/write communication work with no stop code.   
    8- UART baude rate can set to custom value. 9600 or 115200. default value is 9600.    
    9- PPS signal configuration is optionalal and it can work without PPS signal. Although with PPS signal, GPS time data will has more accuracy and reliability.      
    10- Multi thread option provided for GPS data update in real time applications if needed.        
    11- GPS data update method can be use optionally by seperate thread or just use it manually without seperated thread.   
 