# Sensor Fusion and Multi Object Tracking

This project was completed as part of Udacity Nanodegree Program. 
LIDAR and Camera detections from the Waymo Open Dataset (highway sequence) are fused for Tracking Objects in the Driving scene using an Extended Kalman filter. A Track Management and Data Association module has been implemented for handling multiple objects in the scene. It takes care of Track Initialisation and maintains a Track State (confirmed, tentitive, initialised)and Track score. False positive objects with low score or those that exit the driving scene are deleted.

The following diagram contains an outline of the data flow and of the individual steps that make up the algorithm.

![flow](https://user-images.githubusercontent.com/56697957/168473301-c18c7b7b-ccdc-471d-a11d-921a83295daa.JPG)

## Installation Instructions. 

#### Package Requirements
All dependencies required for the project have been listed in the file `requirements.txt`. 

#### Waymo Open Dataset Reader
The Waymo Open Dataset Reader is a very convenient toolbox that allows you to access sequences from the Waymo Open Dataset.The installation instructions can be found in `tools/waymo_reader/README.md`. 

#### Waymo Open Dataset Files
This project makes use of different sequences for Testing Algorithm: These are: 
- Sequence 1 : `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`
- Sequence 2 : `training_segment-10072231702153043603_5725_000_5745_000_with_camera_labels.tfrecord`
- Sequence 3 : `training_segment-10963653239323173269_1924_000_1944_000_with_camera_labels.tfrecord`

To download these files, you will have to register with Waymo Open Dataset first: [Open Dataset – Waymo](https://waymo.com/open/terms)

Once you have done so, please [click here](https://console.cloud.google.com/storage/browser/waymo_open_dataset_v_1_2_0_individual_files) to access the Google Cloud Container that holds all the sequences. Once you have been cleared for access by Waymo (which might take up to 48 hours), you can download the individual sequences. 
The sequences listed above can be found in the folder "training". Please download them and put the `tfrecord`-files into the `dataset` folder of this project.

## Project File Structure

📦project<br>
 ┣ 📂dataset --> should contain the downloaded Waymo Open Dataset sequences <br>
 ┃<br>
 ┣ 📂misc<br>
 ┃ ┣ evaluation.py --> plot functions for tracking visualization and RMSE calculation<br>
 ┃ ┣ helpers.py --> misc. helper functions, e.g. for loading / saving binary files<br>
   ┗ params.py --> parameter file for the tracking <br>
 ┣ 📂results --> binary files with pre-computed Lidar Detections <br>
 ┃ <br>
 ┣ 📂student <br>
 ┃ ┣ association.py --> data association logic for assigning measurements to tracks <br>
 ┃ ┣ filter.py --> extended Kalman filter implementation  <br>
 ┃ ┣ measurements.py --> sensor model and measurement processing for camera and lidar  <br>
 ┃ ┗ trackmanagement.py --> track and track management classes <br>
 ┃ <br>
 ┣ 📂tools --> external tools<br>
 ┃ ┗ 📂waymo_reader --> functions for loading Waymo data sequences<br>
 ┃<br>
 ┣ loop_over_dataset.py --> Main script for Object Tracking <br>

### Using Sensor Data and Precomputed measurements

This project uses pre-computed lidar detections of Waymo Sequences so everyone can use same input data for implementing Tracking Algorithm [download the pre-computed lidar detections](https://drive.google.com/drive/folders/1IkqFGYTF6Fh_d8J3UjQOSNJ2V42UDZpO?usp=sharing) (~1 GB), unzip them and put them in the folder `results`.
For Camera measurements, noise is added the Waymo Image Labels and resulting noisy data is used as input for Sensor Fusion and Tracking.

### Running the Project
Please execute loop_over_dataset.py to run the project

![Screenshot (2)](https://user-images.githubusercontent.com/56697957/168471904-5ecda818-35b7-41cb-adaf-c8500cf6ab6b.png)

    
### Results and Performance:

#### 1.Ghost Object/ Clutter Rejection : 
The Algorithm sucessfully deletes false positive Lidar or Camera detections which have a low Track Score and High estimation Covariance.

![ezgif com-gif-maker](https://user-images.githubusercontent.com/56697957/168469775-4debdce8-e0d2-4c94-a4ba-1bc387967387.gif)

#### 2.Initialisation of New Tracks:
New Tracks are sucessfully initialised for handling new Cars that enter the driving scene

![ezgif com-gif-maker (1)](https://user-images.githubusercontent.com/56697957/168470125-713c15fa-92ca-41cd-be40-8303aa9fadc9.gif)

#### 3.Root Mean Squared Error:
The state estimations were compared to Ground truth data and accumulated trackwise RMSE values for the entire sequence have been plotted below. 

![rmse](https://user-images.githubusercontent.com/56697957/168470304-2703adfe-9800-46a3-8721-691eedfa6678.JPG)

## External Dependencies
Parts of this project are based on the following repositories: 
- [Simple Waymo Open Dataset Reader](https://github.com/gdlg/simple-waymo-open-dataset-reader)




