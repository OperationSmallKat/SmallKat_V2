#!/bin/sh
# update & upgrade #
sudo -n true
test $? -eq 0 || exit 1 "you should have sudo priveledge to run this script"
passwd
SmallKat
sudo apt-get update
sudo apt-get upgrade
##
##
sudo apt install wget
sudo apt install libopencv2.4-java
wget https://github.com/CommonWealthRobotics/BowlerStudio/releases/download/0.25.2/BowlerScriptingKernel-0.31.1.jar
git clone https://github.com/keionbis/SmallKat.git
sudo apt-get install dnsmasq hostapd
denyinterfaces wlan0
sudo cp interfaces /etc/network
sudo cp hostapd.conf /etc/hostapd/hostapd.conf
sudo /usr/sbin/hostapd /etc/hostapd/hostapd.conf
sudo cp hostapd /etc/default/hostapd
sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig  
sudo cp dnsmasq.conf /etc/dnsmasq.conf  
sudo service hostapd start  
sudo service dnsmasq start  
sudo reboot
