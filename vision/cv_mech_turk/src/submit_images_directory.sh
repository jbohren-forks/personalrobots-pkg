#!/bin/bash  
echo " "
echo "./submit_images_directory.sh <session-name> <directory>"
echo " "
echo "<session-name> might for example be 'Ikea-1'"
echo " "
echo "Tis is a script to submit the whole images <directory> to mech turk"
echo "Assumes that you are working with a directory with .jpg images in it"
echo "                        -- Gary Bradski"
echo "start For loop" 
for pic in $2/*.jpg
do
        echo "Submitting $pic ..."
        ./submit_img.py --session=$1 $pic
 #       echo "convert $pic $(basename $pic .png).jpg"
done
echo "finished"

