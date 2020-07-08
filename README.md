# Raspicam Stereo ROS
ROS node based on the Arducam Stereo Hat ([link](https://www.arducam.com/product/b0195-synchronized-stereo-camera-hat-raspberry-pi/)) 

##	 Original Implementation

Based on ArduCAM/MIPI_Camera ([link](https://github.com/ArduCAM/MIPI_Camera) )

## Testing
Current developement is based on the Raspberry Pi4 - 4Gb
and Sony IMX219 sensor: Raspberry Pi NoIR Camera Module V2 - 8MP ([link](https://www.amazon.com/Raspberry-Pi-Camera-Module-1080P30/dp/B071WP53K7/ref=sr_1_3?dchild=1&keywords=noir+raspi&qid=1594230648&sr=8-3))

## Available Formats on IMX219

```
mode: 7, 1600x600
mode: 8, 2560x720
mode: 9, 3840x1080
mode: 10, 5184x1944
mode: 11, 6528x1848
mode: 12, 6528x2464
```

##	 ROS Parameters

Setup inside config/params.yaml:

- node_rate_loop: 30
- width: 1600
- height: 600
- quality: 50
- auto_exposure: false
- exposure_value: 1
- auto_wb: true
- auto_wb_compensation: false
- red_gain: 10
- blue_gain: 50
- binning: 2 # 2: no-binning, 1: x2-binning 0: x4-binning 3: x2 analog (special) - not working
- binning_type: 1 # 0 :average, 1: sum


##	 Output Example

<img src="https://github.com/tiralonghipol/raspicam_stereo/blob/master/imgs/example_output.png" 
alt="node_graph" height="300"/>


## Contact
* [Paolo De Petris](mailto:pdepetris@nevada.unr.edu)
