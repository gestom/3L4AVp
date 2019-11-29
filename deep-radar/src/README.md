Deep Radar
===

## 1. Collect data

Run `python getTrainingData.py`
You will need to adjust some parameters, such as the search area. These are all contained at the top of the file
The code will publish on the following topics:

```
    /deep_radar/input/estimated_position  
    /deep_radar/input/input_bounds  
    /deep_radar/input/radar_repeat  
    /deep_radar/input/search_area_publisher  
```
You can then use these to adjust your search area in the lidar. 

This creates the files in the folder `dataset/myDataset/`
This is the training data to be used by the neural network

## 2. Augment dataset (Optional)

Run `python datasetAugmenter.py` to augment the dataset.
This will duplicate each frame in the dataset many times, with distortions, rotations added randomly. This improves the ability of the neural network to generalise.
This augmented set will be saved alongside the original set.

## 3. Combine datasets (Optional)

If you wish to train against multiple datasets, you can combine them into a single set to feed into the neural network to train against.
You can run `combineDatasets.py` and then select the two to be combined. They will not be overwritten.
Be aware that to maintain a constant tensor shape, if one dataset uses more points, it will be truncated to match the size of the smaller. This may affect the outcome.

## 4. Train
Run `python train.py`
This will begin the process of training the neural network.
To check that it is working properly, the loss should decrease over time, and the accuracy increase. If this is not the case check your dataset.
Either you have a problem in there, or there is not enough training data.
This will save the trained weights into the model folder

## 5. Run model

Run `python inference.py` to begin real time inference of the radar data.
This will publish the results as a ros message. 
The easiest way to view the output is using rviz.
