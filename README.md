robokite
========

A fork of the robokite project aiming to provide an open source platform to control kites

This project started in 2013. It aims to provide tools or interface between existing tools to control big kites in a few years. Applications vary from ship propulsion to Airborne Wind Energy, through arstistic control.

The first step is to design a simple kite control system:

    inexpensive
    adaptable to any small kite
    based on a community of users 

The solutions which are tested:

    
    Mobile phone (webcam, accelerometer, magnetometer) for kite position sensing
    Wifi streaming to a computer and processing whith python (simplecv) in a first step
    Processing with OpenCV for Android them streaming to reduce latency.
    A python tornado server
    A browser with HTML5 as interface (websocket for real time, slider for controls, webgl for visualization)
    Serial NMEA communication
    Arduino as a DAC (Digital-Analog Converter)
    Sabertooth as motor control card
    Recycled cordless drill as motor


The project will host the documentation and source code. Please use the following command to get the documentation offline:

git clone https://github.com/baptistelabat/robokite.wiki.git

Please use the following command to get the code:

git clone https://github.com/baptistelabat/robokite.git

This project was initiated by Baptiste LABAT from Nautilabs (french maritime consultancy).

Please mail nautilabs@gmail.com to be involved. 
