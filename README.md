robokite
========

The robokite project aims to provide an open source platform to control kites.

This project started in 2013. It aims to provide tools or interface between existing tools to control big kites in a few years. Applications vary from ship propulsion to Airborne Wind Energy, through arstistic control.

Watch the video below to understand the current state of the project or have a look to the project blog http://robokite.blogspot.fr/

[![Joystick kite control](http://4.bp.blogspot.com/-ituEgWwpdzA/VHY0rxaLOUI/AAAAAAAAAmY/w92dK0EN41g/s1600/Screenshot%2Bfrom%2B2014-11-26%2B21%3A13%3A43.png)](https://www.youtube.com/watch?v=0MbRyefYqPs&index=1&list=UUa67hFWRqXyehBhyk3YfBnA)


The first step is to design a simple kite control system:

* inexpensive
* adaptable to any small kite
* based on a community of users 

The solutions which are tested:
    
* Mobile phone (webcam, accelerometer, magnetometer, and gyro if available) for kite position sensing
* Wifi streaming to a computer (might be raspberry) and processing whith python (simplecv) in a first step
* Processing with OpenCV for Android then streaming to reduce latency, in a second step
* A webserver (python tornado)
* A browser with HTML5 as interface (websocket for real time, slider for controls, webgl for visualization, graphs, etc)
* A common gamepad joystick to send orders
* Serial NMEA or mavlink communication
* Arduino as a DAC (Digital-Analog Converter)
* Sabertooth as motor control card (to be replaced by open hardware)
* Recycled cordless drill as motor, but scalable.

Installation
------------

Please use the following command to get the code (you need to have git installed):

    git clone https://github.com/baptistelabat/robokite.git

This will create a repertory robokite containing the source code

The following command will download the depencies (tested on ubuntu 14.04)

    cd robokite
    chmod 777 install.sh
    ./install.sh

Feature status
--------------

Currently, the project enables to control the kite with some motors controlled by a joystick.

Documentation
--------------

The project hosts the documentation on the [wiki] (https://github.com/baptistelabat/robokite/wiki). Please use the following command to get the documentation offline:

    git clone https://github.com/baptistelabat/robokite.wiki.git

To get more up to date news, you can follow the [robokite blog](http://robokite.blogspot.fr/) 

Mailing list
------------

There is a public [mailing list](https://groups.google.com/group/robokite)
for Robokite users and developers.


Development
-----------

We are currently looking for some beta testers and developers.

Please mail nautilabs@gmail.com to be involved. 

Acknowledgments
------------

The robokite project is based on many open source or open hardware parts and was possible thanks to the support of several individuals and structures.

See [acknowlegement page in wiki] (https://github.com/baptistelabat/robokite/wiki/Acknowledgment)

License
-------

Robokite is covered by the MIT license, see the accompanying file LICENSE.md for details.

This repository contains additional code that may be covered by other licenses.
