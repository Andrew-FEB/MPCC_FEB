#!/bin/bash
if [ $# -ne 1 ]; then
	echo "Usage: ./update_homedir OLD_USERNAME"
else
	echo "egrep -lRIZ '/home/$1'|xargs -0 -l sed -i 's/\/home\/$1/\/home\/$USERNAME/g'"
	egrep -lRIZ "/home/$1"|xargs -0 -l sed -i "s/\/home\/$1/\/home\/$USERNAME/g"
fi
