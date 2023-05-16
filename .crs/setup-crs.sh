#!/usr/bin/env bash

set -e
printf "starting CRS installer...\n"
printf "installing CRS environment...\n"
sudo install .crs/crs /usr/bin/
sudo mkdir -p /etc/bash_completion.d/
sudo cp .crs/crs-completion.bash /etc/bash_completion.d/
sudo chmod 644 /etc/bash_completion.d/crs-completion.bash
printf "done!\n"
