robokite
========

The robokite project aims to provide an open source platform to control kites.

This project started in 2013. It aims to provide tools or interface between existing tools to control big kites in a few years. Applications vary from ship propulsion to Airborne Wind Energy, through arstistic control.

Watch the video below to understand the current state of the project or have a look to the project blog http://robokite.blogspot.fr/

[![Joystick kite control](http://4.bp.blogspot.com/-ituEgWwpdzA/VHY0rxaLOUI/AAAAAAAAAmY/w92dK0EN41g/s1600/Screenshot%2Bfrom%2B2014-11-26%2B21%3A13%3A43.png)](https://www.youtube.com/watch?v=0MbRyefYqPsjA)


The first step is to design a simple kite control system:

* inexpensive
* adaptable to any small kite
* based on a community of users 

The solutions which are tested:

    
* Mobile phone (webcam, accelerometer, magnetometer, and gyro if available) for kite position sensing
* Wifi streaming to a computer (might be raspberry) and processing whith python (simplecv) in a first step
* Processing with OpenCV for Android then streaming to reduce latency, in a second step
* A python tornado server
* A browser with HTML5 as interface (websocket for real time, slider for controls, webgl for visualization, graphs, etc)
* Serial NMEA communication
* Arduino as a DAC (Digital-Analog Converter)
* Sabertooth as motor control card (to be replaced by open hardware)
* Recycled cordless drill as motor, but scalable.


The project host the documentation (on the wiki https://github.com/baptistelabat/robokite/wiki) and source code. Please use the following command to get the documentation offline:

    git clone https://github.com/baptistelabat/robokite.wiki.git

To get more update news, you can follow the blog http://robokite.blogspot.fr/

Please use the following command to get the code:

    git clone https://github.com/baptistelabat/robokite.git

This project was initiated by Baptiste LABAT from Nautilabs (french maritime consultancy).
We are currently looking for some beta testers and developers.

Please mail nautilabs@gmail.com to be involved. 
