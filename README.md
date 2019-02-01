To use online learning for radar-based human model (radar_detector_ol.cpp), make sure:

1. You have libsvm (`sudo apt-get install libsvm-dev libsvm-tools`)
2. You have `grid.py` (must be executable `chmod +x`) in `.ros`

Then you can launch the prgoram with:
`roslaunch radar.launch bag:=/home/z/Downloads/3L4AV/processed/corridor-behind-normal.bag 2> /dev/null`
