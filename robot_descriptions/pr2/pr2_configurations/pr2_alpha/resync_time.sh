#!/bin/bash

/etc/init.d/ntp stop
ntpdate fw1
/etc/init.d/ntp start
/etc/init.d/chrony restart