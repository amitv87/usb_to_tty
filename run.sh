set -e
# set -x

SRC_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $SRC_ROOT_DIR

PROJECT_PATH="$SRC_ROOT_DIR"

BUILD_PATH="$SRC_ROOT_DIR/.build"
[[ ! -z "${getDevBuildPath}" ]] && eval "$getDevBuildPath" && BUILD_PATH=$(getDevBuildPath);

export DEPS_DIR_NAME=deps
export DEPS_SRC_DIR=$SRC_ROOT_DIR/$DEPS_DIR_NAME

export DEPS_BUILD_PATH=$BUILD_PATH/$DEPS_DIR_NAME

export TARGET="usb_to_tty"

export NUM_CPU=$(getconf _NPROCESSORS_ONLN)

SILENT_MAKE="-s"
# VERBOSE_ARGS="-DCMAKE_VERBOSE_MAKEFILE=TRUE"

run(){
  $BUILD_PATH/$TARGET
}

printSize(){
  echo BUILD_PATH: $BUILD_PATH
  cd $BUILD_PATH
  md5 $TARGET
  size $TARGET
  ls -la $TARGET*
  cd ~-
}

build(){
  mkdir -p $BUILD_PATH
  cd $BUILD_PATH
  [[ ! -f "Makefile" ]] && cmake $VERBOSE_ARGS $PROJECT_PATH
  make $SILENT_MAKE -j $NUM_CPU $@
  cd ~-
}

compile(){
  build all;
  printSize;
}

shell(){
  miniterm.py --raw /dev/tty.usbserial-FT9MUI4A 115200
}

ACTION="$1"

if [[ $ACTION == "r" ]] ; then
  build clean;
  compile;
elif [[ $ACTION == "c" ]]; then
  build clean;
elif [[ $ACTION == "b" ]]; then
  compile;
elif [[ $ACTION == "br" ]]; then
  compile;
  run;
elif [[ $ACTION == "i" ]] ; then
  deps/init.sh
elif [[ $ACTION == "bd" ]]; then
  deps/build.sh
else
  run
fi
