#!/bin/bash
if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit 1
fi

sudo apt install -y python-pip
pip install svgpathtools

sudo dpkg -i *.deb
sudo apt --fix-broken install -y
exit 0
