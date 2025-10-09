# Instructions

## 1 Python

create a venv environment

pip install bluepy package inside a venv virtual environement 

source ./bluepy-env(your venv name)/bin/activate

sudo /home/Akiraw(Your user name)/bluepy-env(Your venv name)/bin/python ble_cccd_demo.py

## 2 C
cd ./Documents

Build the following repo in rpi
https://github.com/labapart/gattlib

nano ble_notify_test.c
copy the code inside
and then X and then Y and then Enter

gcc -o ble_notify_test ble_notify_test.c $(pkg-config --cflags --libs gattlib) -lpthread

open another terminal
sudo bluetoothctl
[bluetooth]# agent on
[bluetooth]# default-agent

in original terminal
sudo ./ble_notify_test

in another terminal
look for a passkey and enter in on your phone

change the charasteristic value to test

Note: If you connected once, you should before next run of sudo ./ble_notify_test
[bluetooth]# devices
[bluetooth]# remove (the device address you just connected)
sudo systemctl restart bluetooth


