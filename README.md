# 9DoF Gyroscope interfacing library with FRDMk64  

Build Status: [![Build Status](https://travis-ci.com/arunkumar-mourougappane/Gyro-mag-FRDM.svg?branch=master)](https://travis-ci.com/arunkumar-mourougappane/Gyro-mag-FRDM)


    
    README
    Explains contents of Gyro-mag-FRDM
    Author : Arunkumar Mourougappane
    Language Used : C++0x
    Date: 10/11/15

--0. Notes--

This repository includes the fully capable driver for invense MPU9250 9DOF
gyroscope with acceleromete and magnetormeter.
This repo is more of a derivative and is to be impreoved over time. 
This repo includes code to enable interfacing of MPU9250 with any
mbed board and is capable of delivering data at USB with a baud rate of 
38400bits/sec and is further extensible.
Data including pitch and yaw are also calculated and can be made use of.
The parent project includes most of above. This project will be adding further with more accurate
angle predctions and use of MPU with interrupts.
