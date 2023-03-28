The solution is a python3 file.

Just like the example solution, it takes two arguments: 
    --> Path to evaluation images (e.g. /home/metrics/FBM2/dataset/crack_detection_dataset/evaluation)
    --> Filename of resulting txt (e.g. example_fbm2_results.txt)

Dependencies (See dependencies.txt) --> pip3, pytorch, ultralytics

When starting from the original image:
-->
sudo apt update
sudo apt upgrade
sudo apt install python3-pip -y
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip3 install ultralytics

Execution --> python3 ./fbm2_solution.py /home/metrics/FBM2/dataset/crack_detection_dataset/evaluation results.txt

Output -->  /home/metrics/FBM2/results/results.txt