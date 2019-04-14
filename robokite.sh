x-terminal-emulator -e "python Motors/joystick.py"
x-terminal-emulator -e "python Motors/motorJoy.py"
x-terminal-emulator -e "mavproxy.py --master=localhost:14550 --aircraft=robokite"
