# crazyKhoreia
## Brief description.
crazyKhoreia is a robotic perception system intended for UAVs (aka drones), which takes a digital image and converts it into a waypoint matrix using X, Y and Z coordinates in meters for UAVs choreography design.

A preprint paper of this project is available [here](https://www.techrxiv.org/articles/preprint/CrazyKhoreia_a_robotic_perception_system_for_UAV_path_planning_from_digital_images/21836868/1).

If this work is useful to you, please cite us:
```
@article{Restrepo2023, author = "Santiago Restrepo and Victor Romero Cano", title = "{CrazyKhoreia, a robotic perception system for UAV path planning from digital images}", year = "2023", month = "1", url = "https://www.techrxiv.org/articles/preprint/CrazyKhoreia_a_robotic_perception_system_for_UAV_path_planning_from_digital_images/21836868", doi = "10.36227/techrxiv.21836868.v1" } 
```
## How does it work?
crazyKhoreia takes a digital image and uses a vectorizing technic based on openCV [findContours](https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a) algorithm to obtain contours, then, it gets waypoints from those contours, after that, depending on the usage mode (either lightPainting or multi-UAV formation), it optimizes the waypoints and finds a feasible 2D path for travelling through **all** of them, or it computes safe trajectories for multi-UAV formation.


### Light painting mode (class lightPainting)
Light painting is a photography technic consisting on capturing a long exposure photograph using a moving light source, the light source's movement creates an effect on the camera sensor which captures its trail, therefore, it "paints" all over the camera's frame, creating a painting optical effect.

![lightPainting.jpg](images/lightPainting.jpg) Source: [Rafael Rodrigo Perurena](https://www.flickr.com/photos/rafoto/2653254686)
### Multi UAV formation mode (class multiDroneFormation)
A multi UAV formation is made by several UAVs grouped together in patterns to construct a figure visible with a naked eye. It's been used by companies such as [Intel](https://www.intel.com/content/www/us/en/technology-innovation/intel-drone-light-shows.html) or [DSS](https://droneshowsoftware.com) to create terrific light shows performed by UAVs like the Super Bowl's half-time shows.
![Intel.jpeg](images/Intel.jpeg) Source: [Intel](https://twitter.com/intel/status/828430024411713536)

## Installation instructions.
### Prerequisites.
Before using crazyKhoreia, please install the following dependencies.
```console
$ pip3 install opencv-python
$ pip3 install numpy
$ pip3 install matplotlib
$ pip3 install scipy
$ pip3 install scikit-learn
```

### Installing from Pypi with Python 3.
To install crazyKhoreia you'll need Python 3 and pip:
```console
pip3 install crazyKhoreia
```
### Installing from source.
If you'd like to develop, edit or have full access into the crazyKhoreia system, then you may install it from source.
```console
git clone https://github.com/santiagorg2401/crazyKhoreia.git
```
## Usage.
crazyKhoreia's usage it's pretty straightforward, you'll only need a digital image and a few parameters:

| Parameter | Usage mode | Class | Description | Data type |
| --- | --- | --- | --- | --- |
| dims | all | [lightPainting](https://github.com/santiagorg2401/crazyKhoreia/blob/master/src/crazyKhoreia/lightPainting.py) [multiDroneFormation](https://github.com/santiagorg2401/crazyKhoreia/blob/master/src/crazyKhoreia/multiDroneFormation.py) | 2x3 float array containing flight space constraints in the x, y, and z axis [[MIN_X, MIN_Y, MIN_Z],[MAX_X, MAX_Y, MAX_Z]] | array
| in_path | all | [lightPainting](https://github.com/santiagorg2401/crazyKhoreia/blob/master/src/crazyKhoreia/lightPainting.py) [multiDroneFormation](https://github.com/santiagorg2401/crazyKhoreia/blob/master/src/crazyKhoreia/multiDroneFormation.py)| Image file path. | str
|out_path| all| [lightPainting](https://github.com/santiagorg2401/crazyKhoreia/blob/master/src/crazyKhoreia/lightPainting.py) [multiDroneFormation](https://github.com/santiagorg2401/crazyKhoreia/blob/master/src/crazyKhoreia/multiDroneFormation.py)| Files output path. | str
| led | light painting | [lightPainting](https://github.com/santiagorg2401/crazyKhoreia/blob/master/src/crazyKhoreia/lightPainting.py) | Set led to ```True``` if you want to add led control within the waypoints output file, else, set ```False```. | bool
| detail | light painting | [lightPainting](https://github.com/santiagorg2401/crazyKhoreia/blob/master/src/crazyKhoreia/lightPainting.py) | Used in [clean_waypoints](https://github.com/santiagorg2401/crazyKhoreia/blob/9bada2480789167e003016494ea361c302cc203b/src/crazyKhoreia/lightPainting.py#L31) method to delete the points that their euclidian distance is minor than **detail**. | float
| speed | light painting | [lightPainting](https://github.com/santiagorg2401/crazyKhoreia/blob/master/src/crazyKhoreia/lightPainting.py) | Used in [calculate_stats](https://github.com/santiagorg2401/crazyKhoreia/blob/9bada2480789167e003016494ea361c302cc203b/src/crazyKhoreia/lightPainting.py#L48) to estimate flight duration, assuming constant speed. **Side note:** It doesn't affect the waypoints dataset. | float
|sleepTime | light painting | [lightPainting](https://github.com/santiagorg2401/crazyKhoreia/blob/master/src/crazyKhoreia/lightPainting.py) | Used in [calculate_stats](https://github.com/santiagorg2401/crazyKhoreia/blob/9bada2480789167e003016494ea361c302cc203b/src/crazyKhoreia/lightPainting.py#L48) to estimate flight duration, assuming that the UAV stops at each reached waypoint for the flew time duration plus a **sleepTime** percentage from it. **Side note:** It doesn't affect the waypoints dataset. | float
|video | light painting | [lightPainting](https://github.com/santiagorg2401/crazyKhoreia/blob/master/src/crazyKhoreia/lightPainting.py) | Set video to ```True``` if you want to render an animation of the light painting generation, else set ```False```. | bool
| boxShape | multiDroneFormation | [multiDroneFormation](https://github.com/santiagorg2401/crazyKhoreia/blob/master/src/crazyKhoreia/multiDroneFormation.py) | Refers to the bounding box for each UAV, contains an 1x3 array, containing the box's: (length (X axis), wide (Y axis), height (Z axis)) in meters. | array

Take into account that lightPainting and multiDroneFormation classes creates an instance of the crazyKhoreia class in its constructor method.

To run the program, create an instance of the lightPainting class.
```console
from crazyKhoreia.lightPainting import lightPainting

lp = lightPainting(dims, in_path, out_path, detail=0.05, speed=1.0, sleepTime=1.5, video=False, led=False)
```

Or if you want to execute multiDroneFormation.
```console
from crazyKhoreia.multiDroneFormation import multiDroneFormation

mdf = multiDroneFormation(dims, boxShape, in_path, out_path, nmbr_drones)
```
After its execution you'll notice the output files within the set output path.

## Trouble?
Start a new [discussion](https://github.com/santiagorg2401/crazyKhoreia/discussions) if you have any question related to the project, but, if you have a technical issue or a bug to report, then please create an [issue](https://github.com/santiagorg2401/crazyKhoreia/issues).

## Authors.
- [Santiago Restrepo García.](https://github.com/santiagorg2401)

## Tutor.
- [Víctor Adolfo Romero Cano.](https://github.com/vromerocano)
