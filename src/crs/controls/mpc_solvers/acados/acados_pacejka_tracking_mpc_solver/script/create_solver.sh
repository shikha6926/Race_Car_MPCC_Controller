#!/bin/bash
cd ${BASH_SOURCE%/*}  # make sure we run in the folder with the python code
rm -r ../src/c_generated_code # remove old code

# generate c code
python3 generate_acados_solver_tracking_mpc.py --config solver_tracking_mpc.yaml <<< "y"
cd ../src/c_generated_code

# build the code
make shared_lib
