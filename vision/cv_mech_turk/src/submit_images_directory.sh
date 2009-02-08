#!/bin/bash  
echo " "
echo "./submit_images_directory.sh <session-name>"
echo " "
echo "<session-name> might for example be 'Ikea-1'"
echo " "
echo "Tis is a script to submit the whole images directory to mech turk"
echo "Assumes that you are working above a directory called images with .jpg images in it"
echo "Assumes you have imagemagick installed..."
echo "                        -- Gary Bradski"
echo "start For loop" 
for pic in images/*.jpg
do
        echo "Submitting $pic ..."
        ./submit_img.py --session=$1 $pic
 #       echo "convert $pic $(basename $pic .png).jpg"
done
cd ..
echo "finished"

