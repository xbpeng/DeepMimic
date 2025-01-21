#!/bin/bash

# CONFIGS IN MAKEFILE.AUTO ARE FOR A LINUX MACHINE WITH MINICONDA3 CONDA ENV

# PLEASE ENSURE THAT YOUR MINICONDA3 DISTRIBUTION IS AT THE PATH /home/ubuntu/miniconda3,
# OR OTHERWISE UPDATE THE PATHS IN MAKEFILE.AUTO FOR THIS TO WORK

# setup_deepmimic.sh
set -e  # Exit on any error

echo "Setting up DeepMimic environment..."

# First check if Miniconda3 is installed at the expected location
if [ ! -d "/home/ubuntu/miniconda3" ]; then
    echo "Miniconda3 not found at /home/ubuntu/miniconda3. Installing Miniconda3..."
    
    # Download the latest Miniconda3 installer for Linux
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh
    
    # Install Miniconda3 silently to the specified location
    bash /tmp/miniconda.sh -b -p /home/ubuntu/miniconda3
    
    # Remove the installer
    rm /tmp/miniconda.sh
    
    # Add Miniconda3 to PATH in .bashrc if not already present
    if ! grep -q "miniconda3/bin" /home/ubuntu/.bashrc; then
        echo 'export PATH="/home/ubuntu/miniconda3/bin:$PATH"' >> /home/ubuntu/.bashrc
    fi
    
    # Initialize conda for bash
    /home/ubuntu/miniconda3/bin/conda init bash
    
    echo "Miniconda3 has been installed. Please restart your terminal or run:"
    echo "source /home/ubuntu/.bashrc"
    
    # Source the bashrc to get immediate access to conda
    source /home/ubuntu/.bashrc

    export PATH="/home/ubuntu/miniconda3/bin:$PATH"

fi

# Ensure conda command is available
if ! command -v conda &> /dev/null; then
    echo "Error: conda command not found even after installation."
    echo "Please restart your terminal or run: source /home/ubuntu/.bashrc"
    exit 1
fi

# Check if environment already exists
if conda info --envs | grep -q "deep_mimic_env"; then
    echo "Conda environment 'deep_mimic_env' already exists."
    echo "Please choose an option:"
    echo "1 - Use existing environment"
    echo "2 - Delete existing and create new environment"
    echo "3 - Exit"
    read -p "Enter your choice (1-3): " choice
    case $choice in
        1)
            echo "Using existing environment..."
            # Initialize conda
            source ~/miniconda3/etc/profile.d/conda.sh
            conda activate deep_mimic_env
            ;;
        2)
            echo "Deleting existing environment..."
            # Initialize conda
            source ~/miniconda3/etc/profile.d/conda.sh
            # Ensure we're in base environment
            conda activate base
            # Now try to remove the environment
            conda env remove -n deep_mimic_env -y
            if [ $? -eq 0 ]; then
                echo "Successfully deleted environment"
                echo "Creating new environment..."
                conda create -n deep_mimic_env python=3.11 -y
                conda activate deep_mimic_env
            else
                echo "Failed to delete environment. Error code: $?"
                exit 1
            fi
            ;;
        3)
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo "Invalid choice. Exiting..."
            exit 1
            ;;
    esac
else
    # If environment doesn't exist, create it
    echo "Creating new environment..."
    # Initialize conda
    source ~/miniconda3/etc/profile.d/conda.sh
    conda create -n deep_mimic_env python=3.11 -y
    conda activate deep_mimic_env
fi

# Install system dependencies
sudo apt install -y libgl1-mesa-dev libx11-dev libxrandr-dev libxi-dev
sudo apt install -y mesa-utils
sudo apt install -y clang
sudo apt install -y cmake

# Install PyBullet
wget https://github.com/bulletphysics/bullet3/archive/refs/tags/2.88.zip
unzip 2.88.zip
cd bullet3-2.88
./build_cmake_pybullet_double.sh
cd build_cmake
sudo make install
cd ../../

# Install Eigen
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip
unzip eigen-3.3.7.zip
cd eigen-3.3.7
mkdir build && cd build
cmake ..

# Install headless OpenGL
sudo apt-get install -y xvfb
cd ../../

# Install FreeGLUT
sudo apt-get update
sudo apt-get install libglu1-mesa-dev

wget https://github.com/freeglut/freeglut/releases/download/v3.0.0/freeglut-3.0.0.tar.gz
tar -xzf freeglut-3.0.0.tar.gz
cd freeglut-3.0.0
cmake . -DFREEGLUT_BUILD_SHARED_LIBS=ON -DFREEGLUT_BUILD_STATIC_LIBS=OFF -DCMAKE_C_FLAGS="-fcommon"
make
sudo make install
cd ../

# Install GLEW
wget https://sourceforge.net/projects/glew/files/glew/2.1.0/glew-2.1.0.zip/download
unzip download
cd glew-2.1.0
make
sudo make install
make clean
cd ../

# Install SWIG
wget https://sourceforge.net/projects/swig/files/swig/swig-4.0.0/swig-4.0.0.tar.gz/download
mv download.1 swig-4.0.0.tar.gz
tar -xzf swig-4.0.0.tar.gz
cd swig-4.0.0
./configure --without-pcre
make
sudo make install
cd ../

# Install MPI and Python packages
sudo apt install -y libopenmpi-dev
pip install PyOpenGL PyOpenGL_accelerate
pip install tensorflow

# Update apt and install mpi4py
sudo apt update
conda install mpi4py -y

# Install C build tools
sudo apt-get update
sudo apt-get install -y build-essential libc++-dev libc++abi-dev

# Build DeepMimicCore
cd DeepMimicCore
make -f Makefile.auto python

echo "Setup completed successfully!"
