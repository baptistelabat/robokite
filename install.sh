# Download python dependencies for ground station

sudo pip install cython
# Library to insure serial communication with arduino
sudo pip install pyserial

# Gamepad/joystick user interface library
sudo apt-get install python-pygame

# Optional library needed to make some recording for post-analysis
sudo apt-get install libhdf5-dev
sudo pip install h5py

# Tornado server is needed to get a graphical user interface in a modern web browser
sudo pip install tornado

	# Mavlink
git clone git://github.com/mavlink/mavlink.git
cd mavlink/pymavlink
sudo python setup.py install
cd ..
# Select repertory message_definitions/v1.0/ardupilotmega.xml
sudo python  mavgenerate.py -o mavlinkv10.py message_definitions/v1.0/common.xml
# At this point, you will have to close the gui
cd ..

# Download arduino dependencies for ground station
	# Sabertooth library
wget www.dimensionengineering.com/software/SabertoothArduinoLibraries.zip
unzip SabertoothArduinoLibraries.zip
mv Sabertooth ~/sketchbook/libraries
mv SyRenSimplified ~/sketchbook/libraries
mv USBSabertooth ~/sketchbook/libraries
mv SabertoothSimplified ~/sketchbook/libraries
rm SabertoothArduinoLibraries.zip
rm install.txt
	# Interrupt library
git clone https://github.com/GreyGnome/PinChangeInt.git ~/sketchbook/libraries/PinChangeInt
	# NMEA library
git clone https://github.com/mikalhart/TinyGPSPlus.git ~/sketchbook/libraries/TinyGPSPlus
	# PID library
git clone https://github.com/br3ttb/Arduino-PID-Library.git
mv Arduino-PID-Library/PID_v1 ~/sketchbook/libraries
sudo rm -R Arduino-PID-Library
	# Modified software serial library
mv Arduino/libraries/mySoftwareSerial ~/sketchbook/libraries


# Download arduino dependencies for embedded board
	# MPU9150 and I2C libraries
git clone https://github.com/sparkfun/MPU-9150_Breakout.git
mv MPU-9150_Breakout/firmware/I2Cdev ~/sketchbook/libraries
mv MPU-9150_Breakout/firmware/MPU6050 ~/sketchbook/libraries
sudo rm  -R MPU-9150_Breakout/

	# Mavlink library
git clone https://github.com/mavlink/c_library.git
mv c_library/ardupilotmega ~/sketchbook/libraries
sudo rm -R c_library

# Download dependencies needed to visualize, modify 3D models
	# Openscad
sudo apt-get install openscad
	# Kokopelli
sudo apt-get install python python-dev python-pip gcc g++ libpng12-dev make bash cmake
sudo pip install numpy PyOpenGL PyOpenGL_accelerate
git clone https://github.com/mkeeter/kokopelli.git
cd kokopelli
make
sudo make install
cd util
./install_wxpython3.0.sh  
cd ..
cd .. 

# Download dependencies needed for software tests, post-treatment analysis, or new/old features development

sudo apt-get install youtube-dl
# Download samples video
chmod 777 Recording/Videos/downloadTestVideos.sh
.robokite/Recording/Videos/downloadTestVideos.sh

# SimpleCV
sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose
sudo apt-get install ipython python-opencv python-scipy python-numpy python-pygame python-setuptools python-pip
sudo pip install python-dev
sudo pip install svgwrite
sudo pip install https://github.com/sightmachine/SimpleCV/zipball/develop


# Basemap
sudo apt-get install python-mpltoolkits.basemap

git clone https://github.com/PaulStoffregen/RadioHead.git ~/sketchbook/libraries/RadioHead

