#!/bin/bash

ssh -t c2 "sudo /etc/init.d/chrony stop && sudo ntpdate fw1 && sudo /etc/init.d/chrony start"
ssh -t c1 "sudo /etc/init.d/chrony stop && sudo ntpdate fw1 && sudo /etc/init.d/chrony start"
ssh -t c3 "sudo /etc/init.d/chrony stop && sudo ntpdate fw1 && sudo /etc/init.d/chrony start"
ssh -t c4 "sudo /etc/init.d/chrony stop && sudo ntpdate fw1 && sudo /etc/init.d/chrony start"