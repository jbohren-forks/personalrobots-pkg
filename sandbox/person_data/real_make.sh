#Run this to actually make the realtime dependencies
function rosmake-deps { \
        #argument is name of launch file \
        rosmake `roslaunch-deps $1`; \
}


rosmake-deps *.launch */*.launch
rosmake-deps config/controllers.xml
rosmake-deps config/perception.xml
