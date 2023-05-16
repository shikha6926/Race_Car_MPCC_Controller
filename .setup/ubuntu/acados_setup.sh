#!/usr/bin/env bash

printf "downloading acados\n"
cd /
git clone --branch 0.1.7 https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init

printf "building acados\n"
sudo mkdir -p build
cd build
sudo cmake -DACADOS_WITH_QPOASES=ON ..
sudo make install -j4

printf "install acados_template python package\n"
cd /
sudo pip3 install acados/interfaces/acados_template