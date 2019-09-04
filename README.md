# JAM-ME: Leveraging jamming to help drones complete their mission

<p align="center">
     <img alt="License" src="https://img.shields.io/static/v1.svg?label=license&message=GPL3&color=brightgreen">
     <img alt="Release" src="https://img.shields.io/static/v1.svg?label=release&message=1.0&color=blue">
     <img alt="License" src="https://img.shields.io/static/v1.svg?label=build&message=passing&color=brightgreen">
     <img alt="Organization" src="https://img.shields.io/static/v1.svg?label=org&message=CRI-LAB&color=blue">
</p>

<p align="center">
     <img alt="Setup Phase" src="./img/power_distance.png" width="500">
</p>

<em>JAM-ME</em>: a solution that allows the drone to exploit an adversarial jamming signal to implement an emergency but yet effective navigation system that does not require any other type of on-board navigational sensor/instrument---the drone still being able to accomplish its mission.

## How it works

Description

* *For further details, please refer to the paper.*

## How to run the code
In order to replicate a scenario with no wind and the target position not equal to the jammer position, you can execute the following `code` on the MATLAB Command Window:

`clc; clear all; d = drone(); d = d.setwinddir(-pi/2); d = d.set_wind_strength(0); d.target_pos = [8,11]; d.mov = [0.1 0]; d = d.fly(); d.show()
`

## Requirements
* MATLAB release R2018b or newer
* Statistics and Machine Learning Toolbox

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
`JAM-ME` is released under the GNU General Public License v3.0 <a href="LICENSE">license</a>.
