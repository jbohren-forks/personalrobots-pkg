#!/bin/bash  
echo " "
echo "./convertPNGtoJPG.sh <directory>"
echo " "
echo "Little script to convert .png to .jpg in <directory> that has .png images in it"
echo "Assumes you have imagemagick installed...   Gary Bradski"
pushd $1
echo "start For loop"
for pic in *.png
do
        echo "converting $pic"
        convert $pic $(basename $pic .png).jpg
done
popd
echo "finished"

