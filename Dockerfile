FROM ubuntu:18.04 as glvnd

# Set up libglvnd for OpenGL GUI support
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        ca-certificates \
        make \
        automake \
        autoconf \
        libtool \
        pkg-config \
        python \
        libxext-dev \
        libx11-dev \
        x11proto-gl-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/libglvnd
RUN git clone --branch=v1.0.0 https://github.com/NVIDIA/libglvnd.git . && \
    ./autogen.sh && \
    ./configure --prefix=/usr/local --libdir=/usr/local/lib/x86_64-linux-gnu && \
    make -j"$(nproc)" install-strip && \
    find /usr/local/lib/x86_64-linux-gnu -type f -name 'lib*.la' -delete

RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
        gcc-multilib \
        libxext-dev:i386 \
        libx11-dev:i386 && \
    rm -rf /var/lib/apt/lists/*

# 32-bit libraries
RUN make distclean && \
    ./autogen.sh && \
    ./configure --prefix=/usr/local --libdir=/usr/local/lib/i386-linux-gnu --host=i386-linux-gnu "CFLAGS=-m32" "CXXFLAGS=-m32" "LDFLAGS=-m32" && \
    make -j"$(nproc)" install-strip && \
    find /usr/local/lib/i386-linux-gnu -type f -name 'lib*.la' -delete


FROM tensorflow/tensorflow:1.13.1-gpu-py3

COPY --from=glvnd /usr/local/lib/x86_64-linux-gnu /usr/local/lib/x86_64-linux-gnu
COPY --from=glvnd /usr/local/lib/i386-linux-gnu /usr/local/lib/i386-linux-gnu

COPY internal/10_nvidia.json /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json

RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    echo '/usr/local/lib/i386-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    ldconfig

ENV LD_LIBRARY_PATH /usr/local/lib/x86_64-linux-gnu:/usr/local/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},display

ARG USER
ARG HOME

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 USER=$USER HOME=$HOME

RUN echo "The working directory is: $HOME"
RUN echo "The user is: $USER"

RUN mkdir -p $HOME
WORKDIR $HOME

RUN apt-get update && apt-get install -y \
        sudo \
        git \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# install dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential \
    apt-utils \
    curl \
    nano \
    vim \
    libfreetype6-dev \
    libpng12-dev \
    libzmq3-dev \
    git \
    python-numpy \
    python-dev \
    python-opengl \
    cmake \
    zlib1g-dev \
    libjpeg-dev \
    xvfb \
    libav-tools \
    xorg-dev \
    libboost-all-dev \
    libsdl2-dev \
    swig \
    libgtk2.0-dev \
    wget \
    ca-certificates \
    unzip \
    aptitude \
    pkg-config \
    qtbase5-dev \
    libqt5opengl5-dev \
    libassimp-dev \
    libpython3.5-dev \
    libboost-python-dev \
    libtinyxml-dev \
    golang \
    python-opencv \
    terminator \
    tmux \
    libcanberra-gtk-module \
    libfuse2 \
    libnss3 \
    fuse \
    python3-tk \
    libglfw3-dev \
    libgl1-mesa-dev \
    libgl1-mesa-glx \
    libglew-dev \
    libosmesa6-dev \
    net-tools \
    xpra \
    xserver-xorg-dev \
    libffi-dev \
    libxslt1.1 \
    feedgnuplot \
    libglew-dev \
    parallel \
    htop \
    apt-transport-https

RUN apt-get install -y mesa-utils \
    && apt-get install -y clang \
    && apt-get install -y cmake \
    && apt-get install wget

RUN wget https://github.com/bulletphysics/bullet3/archive/refs/tags/2.88.tar.gz \
    && tar -xvf 2.88.tar.gz \
    && cd ./bullet3-2.88 \
    && ./build_cmake_pybullet_double.sh \
    && cd ./build_cmake && make install


RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz \
    && tar -xvf eigen-3.3.7.tar.gz\
    && cd ./eigen-3.3.7 \
    && mkdir build && cd build \
    && cmake .. \
    && make install

#RUN wget -O freeglut-3.0.0.tar.gz "https://sourceforge.net/projects/freeglut/files/freeglut/3.0.0/freeglut-3.0.0.tar.gz/download?use_mirror=gigenet&use_mirror=gigenet&r=http%3A%2F%2Ffreeglut.sourceforge.net%2F" \
 #   && tar -xvf freeglut-3.0.0.tar.gz \
  #  && cd ./freeglut-3.0.0 \
   # && cmake .\
    #&& make && make install

RUN wget -O glew-2.1.0.tar.gz "https://sourceforge.net/projects/glew/files/glew/2.1.0/glew-2.1.0.tgz/download" \
    && tar -xvf glew-2.1.0.tar.gz \
    && cd ./glew-2.1.0 \
    && make && make install && make clean

RUN wget -O swig-4.0.0.tar.gz "https://downloads.sourceforge.net/swig/swig-4.0.0.tar.gz" \
    && tar -xvf swig-4.0.0.tar.gz \
    && cd swig-4.0.0 \
    && ./configure --without-pcre \
    && make && make install

RUN apt-get install -y libopenmpi-dev
RUN pip install PyOpenGL PyOpenGL_accelerate
RUN pip install mpi4py

COPY . /src/