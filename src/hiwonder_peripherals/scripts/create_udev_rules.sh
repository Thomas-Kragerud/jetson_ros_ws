#!/bin/bash

sudo cp `rospack find hiwonder_peripherals`/scripts/*.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
