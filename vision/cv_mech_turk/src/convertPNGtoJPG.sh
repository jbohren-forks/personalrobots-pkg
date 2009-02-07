echo "Little script to convert .png to .jpg"
echo "Assumes that you are working above a directory called images"
echo "Assumes you have imagemagick installed...   Gary Bradski"
cd /images
echo "start For loop"
for pic in *.png
do
        echo "converting $pic"
        echo "convert $pic $(basename $pic .png).jpg"
done
cd ..
echo "finished"

