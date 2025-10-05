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

nano ble_cccd_writer.c
copy the code inside
and then X and then Y and then Enter

gcc ble_cccd_writer.c -o ble_cccd_writer -lgattlib -lpthread

sudo ./ble_cccd_writer
