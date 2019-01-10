pushd `dirname $0` > /dev/null
SCRIPTDIR=`pwd`
popd > /dev/null

java -Xms128M -Xmx2G -cp ".:*:$SCRIPTDIR/*" -jar repeater.jar "$@"