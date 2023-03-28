import os
import sys

import numpy as np
import torch
from ultralytics import YOLO

def main(args):
  
    WEIGHTS = '/home/metrics/FBM2/carrot_ws/crack_model/x.pt'
    
    if torch.cuda.is_available():
        
        DEVICE = "0"
        
    else:
        
        DEVICE = "cpu"

    if os.path.isfile(WEIGHTS):

        try:

            EVALUATION_DIR = args[1] + "/"
            SAVE_DIR = '/home/metrics/FBM2/results/' + args[2]
            conf_cut = 0.1
            model = YOLO(WEIGHTS)

            # change according to the image format
            img_format = '.jpg'
            
            file_list = os.listdir(EVALUATION_DIR)
            img_list  = [file for file in file_list if file.endswith(img_format)]
            img_list = img_list
            print("Evaluation start")

            with open(SAVE_DIR, 'w') as f :

                for img in img_list :

                    results = model.predict(source = EVALUATION_DIR + img, device = DEVICE, save = False, show = False, verbose = False)

                    for result in results:
                        
                        conf_list = result.boxes.conf
                        xyxy_list = result.boxes.xyxy

                    conf_list = conf_list.cpu().detach().numpy()
                    xyxy_list = xyxy_list.cpu().detach().numpy()

                    if len(conf_list) != 0:
                        
                        idx = np.where(conf_list > conf_cut)

                        for xyxy in xyxy_list[idx]:
                            
                            f.write(img + " " + str(int(xyxy[0])) + " " + str(int(xyxy[1])) + " " + str(int(xyxy[2])) + " " + str(int(xyxy[3])) + "\n")

            print("Evaluation finished : {} saved to {}".format(args[2], SAVE_DIR))

        except IndexError as e:
            
            print("Error : Need 2 inputs to execute as --> python3 ./fbm2_solution.py /directory/to/evaluation result_file_name.txt")

        except FileNotFoundError as e:
            
            print(e, "\nDirectory Error : execute as --> python3 ./fbm2_solution.py /directory/to/evaluation result_file_name.txt")
    else:
        
        print("Error : directory to the detector model is incorrect. Check if .pt file is in the /carrot_ws directory")

if __name__ == '__main__':
    main(sys.argv)
