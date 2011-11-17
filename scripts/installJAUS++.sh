#!/bin/bash
BASE_DIR=$PWD
VERSION_MINOR='110519'
if [ -f '/usr/local/lib/active/libjauscore.so' ]
then
  echo 'JAUS++ already installed, not reinstalling'
else
  sudo apt-get install libpng-dev libX11-dev libxtst-dev
  wget 'http://downloads.sourceforge.net/project/active-ist/JAUS%2B%2B/2.'$VERSION_MINOR'/JAUS%2B%2B-2.'$VERSION_MINOR'-src.tar.gz'
  tar -xf 'JAUS++-2.'$VERSION_MINOR'-src.tar.gz'
  rm 'JAUS++-2.'$VERSION_MINOR'-src.tar.gz'
  cd 'JAUS++-2.'$VERSION_MINOR'-src/libraries/jaus++/2.0/build/cmake'
  cmake .
  make
  sudo make install
  cd $BASE_DIR
  rm -Rf 'JAUS++-2.'$VERSION_MINOR'-src'
  if [ -e /etc/ld.so.conf.d/jaus++.conf ]
  then
    echo "Deleting old ld config file"
    sudo rm /etc/ld.so.conf.d/jaus++.conf
  fi
  echo "Creating ld config file /etc/ld.so.conf.d/jaus++.conf"
  sudo touch /etc/ld.so.conf.d/jaus++.conf
  sudo sh -c "echo '/usr/local/lib/active' >> /etc/ld.so.conf.d/jaus++.conf"
  sudo ldconfig
fi
