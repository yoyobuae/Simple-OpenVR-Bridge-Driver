#!/bin/bash

driver_name=apriltagtrackers
driver_path=$(pwd)/$driver_name

#get the openvr directory using jq
vrpath=$(jq '.runtime' ~/.config/openvr/openvrpaths.vrpath) 

#jq returns brackets, quotations and whitespaces around the string, so we remove them
vrpath=$(echo $vrpath | sed 's|[]"[]||g' | sed 's|^[[:space:]]*||' | sed 's|[[:space:]]*$||')

#path to the vrpathreg.sh script. Would rather use /bin/linux64/vrpathreg executable, but it is bugged right now
vrpathreg_exe="$vrpath"/bin/vrpathreg.sh

#check if all the needed paths exist
if ! test -d "$driver_path"; then
    echo "Driver not found: $driver_path"
	echo
	echo Check that you downloaded the right files and make sure to unzip the folder before running this script!
	echo
    exit
fi

if ! test -f "$vrpathreg_exe"; then
    echo "vrpathreg.exe not found: $vrpathreg_exe"
	echo
	echo This usualy means an error with your SteamVR installation.
	echo
    exit
fi

#check if installations of older versions exists, and remove it
if test -d "$vrpath"/drivers/apriltagtrackers; then
    echo "Found old driver install at $vrpath/drivers/apriltagtrackers, removing"
    rm -r $vrpath/drivers/apriltagtrackers
fi

#cleanup previous drivers, then installs the driver
$vrpathreg_exe removedriver "$driver_path"
$vrpathreg_exe removedriverswithname $driver_name
$vrpathreg_exe adddriver "$driver_path"

#shows currently installed drivers
$vrpathreg_exe show

echo
echo The driver has been installed successfully!
echo 


