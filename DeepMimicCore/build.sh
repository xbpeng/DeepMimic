#!/bin/bash

echo "DeepMimic Linux script to download, build the dependencies and the core library"

debian() {
  echo "Reminder to install these OS packages: "
  echo "sudo apt install libgl1-mesa-dev libx11-dev libxrandr-dev libxi-dev mesa-utils clang cmake libopenmpi-dev freeglut3"
}

# Download the given url if not yet downloaded
download() {
  url=$1
  fn=${url##*/}
  if [ ! -f $fn ]; then
    wget $url
    if [ ! $? == 0 ]; then
      echo "Failed to download $url"
      exit 1
    fi
  fi
}

bullet() {
  download https://github.com/bulletphysics/bullet3/archive/refs/tags/2.88.tar.gz
  if [ ! -d bullet3-2.88 ]; then
    tar -xvzf 2.88.tar.gz || exit 1
  fi
  cd bullet3-2.88
  # Note: build_cmake_pybullet_double.sh imposes double but DeepMimic prefers float
  mkdir -p build_cmake
  cd build_cmake
  if [ ! -f Makefile ]; then    
    cmake -DCMAKE_INSTALL_PREFIX=install -DBUILD_PYBULLET=OFF -DBUILD_PYBULLET_NUMPY=OFF -DUSE_DOUBLE_PRECISION=OFF -DBT_USE_EGL=ON \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON .. || exit 1
  fi
  if [ ! -d install ]; then
    make install -j $(command nproc 2>/dev/null || echo 12) || exit 1
  fi
  export BULLET_INSTALL_DIR=$PWD/install
  export BULLET_INC_DIR=$PWD/install/include/bullet
  export BULLET_LIB_DIR=$PWD/install/lib
  echo "Bullet built and installed in $PWD/install"
  cd $THIRD
}

eigen() {
  download https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz
  if [ ! -d eigen-3.3.7 ]; then 
    tar -xvzf eigen-3.3.7.tar.gz || exit 1
  fi
  cd eigen-3.3.7
  mkdir -p build && cd build
  if [ ! -f Makefile ]; then 
    cmake -DCMAKE_INSTALL_PREFIX=install .. || exit 1
  fi
  if [ ! -d install ]; then 
    make install || exit 1
  fi
  export EIGEN_DIR=$PWD/install/include/eigen3
  echo "Eigen installed in $PWD/install"
  cd $THIRD
}

GL() {
  echo "Checking GL..."
  glxinfo | grep "version str"
  if [ ! -f /usr/include/GL/gl.h ]; then 
    echo "No gl.h ?"; exit 1; 
  fi
}

freeglut() {
  download https://downloads.sourceforge.net/project/freeglut/freeglut/3.0.0/freeglut-3.0.0.tar.gz
  if [ ! -d freeglut-3.0.0 ]; then
    tar -xvzf freeglut-3.0.0.tar.gz || exit 1
  fi
  cd freeglut-3.0.0
  if [ ! -f Makefile ]; then
    cmake . -DCMAKE_INSTALL_PREFIX=install || exit 1
  fi
  if [ ! -d install ]; then
    make install || exit 1
  fi
  if [ ! -f install/lib/libglut.so ]; then
    echo "Cannot find libglut.so"; exit 1
  fi
  export FREEGLUT_INSTALL_DIR=$PWD/install/
  export FREEGLUT_INC_DIR=$PWD/install/include
  export FREEGLUT_LIB_DIR=$PWD/install/lib
  echo "Freeglut built and installed in $PWD/install"
  cd $THIRD
}

glew() {
  download https://downloads.sourceforge.net/project/glew/glew/2.1.0/glew-2.1.0.tgz
  if [ ! -d glew-2.1.0 ]; then
    tar -xzf glew-2.1.0.tgz || exit 1
  fi
  cd glew-2.1.0
  if [ ! -d install ]; then
    make DESTDIR=install install || exit 1
  fi
  [ ! -f $PWD/install/usr/lib64/libglew.so ] || exit 1
  export GLEW_INSTALL_DIR=$PWD/install/usr/
  export GLEW_LIB_DIR=$PWD/install/usr/lib64
  export GLEW_INC_DIR=$PWD/install/usr/include
  echo "glew built and installed in $PWD/install"
  cd $THIRD
}

swig4() {
  download https://downloads.sourceforge.net/project/swig/swig/swig-4.0.0/swig-4.0.0.tar.gz
  if [ ! -d swig-4.0.0 ]; then
    tar -xzf swig-4.0.0.tar.gz || exit 1
  fi
  cd swig-4.0.0
  if [ ! -f Makefile ]; then
    ./configure --without-pcre --prefix $PWD/install || exit 1
  fi
  if [ ! -f swig ]; then
    make swig || exit 1
    ./swig -version || exit 1
    make install || exit 1
  fi
  export PATH=$PWD/install/bin:$PATH
  echo "swig built and installed in $PWD/install/bin"
  cd $THIRD
}

checkCmake() {
  command -v cmake || exit 1
}

if [ -f /etc/debian_version ]; then debian; fi

export SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
cd $SCRIPT_DIR
export DMC=$PWD
mkdir -p third
cd third
export THIRD=$PWD

bullet
eigen
freeglut
glew
swig4

cd $DMC

if [ ! -f _DeepMimicCore.so ] || [ ! -f DeepMimicCore.py ]; then
  make python || exit 1
  [ -f _DeepMimicCore.so ] || exit 1
  [ -f DeepMimicCore.py ] || exit 1
fi

command -v patchelf >/dev/null
if [ $? == 0 ]; then
  patchelf --set-rpath $GLEW_LIB_DIR:$FREEGLUT_LIB_DIR _DeepMimicCore.so
else
  echo "Warning: cannot find the patchelf tool so cannot add rpath to _DeepMimicCore.so. Advice: set LD_LIBRARY_PATH accordingly:"
  export LD_LIBRARY_PATH=$GLEW_LIB_DIR:$FREEGLUT_LIB_DIR:$LD_LIBRARY_PATH
  echo $LD_LIBRARY_PATH
fi

echo "Checking _DeepMimicCore.so dymamic deps ..."
ldd _DeepMimicCore.so | grep "not found"
if [ $? != 1 ]; then
  echo "Some deps have not been found"
  exit 1
fi

echo "Checking python wrapper ..."
python3 DeepMimicCore.py || exit 1
echo "Done"

