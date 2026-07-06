#!/bin/sh
python3 pub5.py "$(cat atag_pm45.txt)"
python pub5.py "30 45 0 0" # camera-claw y-err is 45mm
python pub5.py "40 0 0 0" # camera-claw x-err is 10mm
# python pub5.py "20 0 0 -40" # z -40 mm
python pub5.py "12 0 0 0" # claw get cube
python pub5.py "20 0 0 40" # z +40 mm
python pub5.py "0 0 0 0" # servo 2,3,4 home
python pub5.py "5 0 0 0" # servo#5 home
python pub5.py "1 -90 0 0"
python pub5.py "11 0 0 0" # claw open
python pub5.py "1 0 0 0" # servo#1 home

