set -e
# set -x

SRC_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $SRC_ROOT_DIR

PROJECT_PATH="$SRC_ROOT_DIR"

BUILD_PATH="$SRC_ROOT_DIR/.build"
[[ ! -z "${getDevBuildPath}" ]] && eval "$getDevBuildPath" && BUILD_PATH=$(getDevBuildPath);

export TARGET="usb_to_tty"

NUM_CPU=$(getconf _NPROCESSORS_ONLN)

run(){
  $BUILD_PATH/$TARGET
  # | TZ=UTC ts '[%F %.T]'
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
  [[ ! -f "Makefile" ]] && cmake $PROJECT_PATH
  # V=1 
  make -s -j $NUM_CPU $@
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
else
  run
fi
