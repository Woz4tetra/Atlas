#!/bin/sh

#  dependencies.sh
#  
#
#  Created by Benjamin Warwick on 8/27/16.
#

sudo apt-get install build-essential
sudo apt-get install linux-headers-`uname -r`

sudo apt-get install python3.5 python3.5-dev
cd /usr/bin
sudo rm python3
sudo ln -s python3 python3.5
sudo mv python3.5 python3

sudo apt-get python3-numpy
sudo apt-get install python3-numpy
sudo apt-get install python3-matplotlib
sudo apt-get install python3-scipy
sudo apt-get install python3-pip
sudo pip3 install pykalman

sudo apt-get install mercurial python3-opengl     libav-tools libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsmpeg-dev     libsdl1.2-dev libportmidi-dev libswscale-dev libavformat-dev libavcodec-dev     libtiff5-dev libx11-6 libx11-dev fluid-soundfont-gm timgm6mb-soundfont     xfonts-base xfonts-100dpi xfonts-75dpi xfonts-cyrillic fontconfig fonts-freefont-ttf
cd ~/Downloads/
hg clone https://bitbucket.org/pygame/pygame
cd pygame
python3 setup.py build
sudo python3 setup.py install
python3 -m pygame.tests

cd ~/Documents/
git clone https://github.com/Woz4tetra/Atlas
echo "export PYTHONPATH=\$PYTHONPATH:~/Documents/Atlas/AutoBuggy" >> ~/.bashrc

python3 Atlas/rccar/plotter.py