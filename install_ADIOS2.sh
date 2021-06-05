#! /bin/bash

source script/discover_os

discover_os

# check if the directory $1/HDF5 exist

if [ -d "$1/ADIOS2" -a -f "$1/ADIOS2/include/adios2.h" ]; then
  echo "ADIOS2 is already installed"
  exit 0
fi


wget http://ppmcore.mpi-cbg.de/upload/ADIOS2-2.7.1.tar.gz
tar -xf ADIOS2-2.7.1.tar.gz
cd ADIOS2-2.7.1

mkdir build
cd build
cmake ../. -DADIOS2_USE_MPI=ON -DADIOS2_USE_Fortran=OFF  -DCMAKE_INSTALL_PREFIX=$1/ADIOS2
make -j $2

make install
rm ADIOS2-2.7.1.tar.gz
rm -rf ADIOS2-2.7.1
if [ $? -ne 0 ]; then
    echo "ADIOS2 error installing"
    exit 0
fi
echo 1 > $1/ADIOS2/version
