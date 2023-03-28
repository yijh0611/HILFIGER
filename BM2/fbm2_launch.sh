#!/bin/bash
if [ "$#" -ne 2 ]; then
    echo "Illegal number of parameters. Usage:"
    echo "./fbm2_launch.sh </path/to/images> <result_filename>"
    echo "./fbm2_launch.sh dataset/crack_detection_dataset/evaluation example_fbm2_results.txt" 
    exit -1
fi
echo "Evaluation bag path: $1"
echo "Result file name: $2"

python3 ./carrot_ws/fbm2_solution.py $1 $2

echo "Done!"