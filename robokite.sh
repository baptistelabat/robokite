gnome-terminal -e "python /home/baptiste/kites/robokite/Motors/joystick.py"
gnome-terminal -e "python /home/baptiste/kites/robokite/Motors/motorJoy.py"
gnome-terminal -e "mavproxy.py --master=localhost:14550 --aircraft=robokite"
