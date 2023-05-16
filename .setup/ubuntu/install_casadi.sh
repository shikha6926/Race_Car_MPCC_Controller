#!/usr/bin/env bash

printf "downloading casadi\n"
cd /
git clone --branch 3.5.5 https://github.com/casadi/casadi.git
cd casadi
printf "building casadi\n"
sudo mkdir -p build
cd build
sudo cmake -D WITH_IPOPT=ON ..
sudo make install -j4