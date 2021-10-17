# auto_calibration
Automatic calibration algorithm that calibrates four cameras of an around view 
monitor (AVM) system simultaneously in a natural driving situation

- Features and Process:
  - Converts fisheye images of the surrounding views of a vehicle into bird-eye images 
  - Extracts lanemarks on a street in a driving environment
  - Remove falsely detected lanemarks based on orientations and sorting vanishing points
    using a voting box algorithm
  - Calibrate the cameras using Levenberg–Marquardt (LM) algorithm

- Libraries and APIs used: C++, Ceres Solver, Eigen, OpenCV

- This project is an attempt to realized the paper 
  *Automatic Calibration of an Around View Monitor System Exploiting Lane Markings*
  by Kyoungtaek Choi, Ho Gi Jung, and Jae Kyu Suhr

- This repository only includes header files of the project due to privacy reason
