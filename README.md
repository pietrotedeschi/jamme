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

We consider the following entities:
* **Drone**:An UAV flying from a source position to a target destination. We assume the drone not being remote controlled but pre-programmed according to a mission plan. The mission plan involves a set of way-points to reach a pre-determined target.
* **Adversary**: We assume the adversary is able to reprogram the drone and being able to change the mission plan parameters and all the flight control systems.
* **Target**: The destination point that the drone has to reach.
* **Jammer**: The radio device used to protect the target neighbourhood. We assume a very powerful, omnidirectional jammer, being able to jam all the radio frequencies in the radio spectrum over a circle of radius *d*. As following, the mathematical model adopted: 

<img src="http://latex.codecogs.com/gif.latex?P_r&space;=&space;P_t&space;&plus;&space;G_t&space;&plus;&space;G_r&space;&plus;&space;20\log_{10}(\frac{c}{4\pi&space;df})" title="P_r = P_t + G_t + G_r + 20\log_{10}(\frac{c}{4\pi df})" />

<img src="http://latex.codecogs.com/gif.latex?d&space;=&space;\frac{1}{\frac{4\pi&space;f}{c}&space;10^{(\frac{P_r-P_t}{20})" title="d = \frac{1}{\frac{4\pi f}{c} 10^{(\frac{P_r-P_t}{20})" />

we obtained $d = \mathcal{D}_{thr}$, by assuming the drone power receiving threshold equals to $P_r = -30$dBm, the constant jammer transmission power is $P_t = 20$dBm, the antennas' gains set to $G_t=G_r=0$ ( we are assuming that the jammer adopts an isotropic antenna to radiate the energy uniformly in all the directions) \cite{fahlstrom2012introduction}, and the GPS frequency $f = 1575.42$MHz.}
\end{itemize}

In order to prove the feasibility of drone navigation under jamming conditions, we consider the following challenging scenario configuration.

\Figure[ht](topskip=0pt, botskip=0pt, midskip=0pt)[angle=0, width=0.90\columnwidth]{figures/scenario.pdf}
{\textcolor{blue}{\textbf{Scenario Configuration.}}\label{fig:scenario}}

\textcolor{blue}{\textbf{Scenario Configuration.} In this scenario, a jammer is protecting an area against drones and \acp{UAV}. Basically, the drone is programmed to fly over the area in order to reach the target. Since in the jammed area, the \ac{GPS} will be not available to the drone, the adversary re-programs the firmware of the drone to leverage the jamming signal source as radio-beacon, and finally reach the target position.}

{\bf Target position awareness.} The adversary (and therefore the drone) is aware of the target position, i.e., its \ac{GPS} coordinates. We assume the target being a static object such as a critical infrastructure, i.e., airport, hospital, oil, and gas refinery, or a static person (e.g. a VIP attending a public event) \cite{maduro}. 

{\bf Unknown jammer position.} We assume the jammer position to be unknown; though it should be in the close neighbourhood of the target. We observe that a jammer standing at the same position of the target guarantees maximum range protection while a jammer placed to a different position might expose one side of the target.
Regardless of these considerations, we will show that our solution is agnostic with respect to the relative position of the jammer.

{\bf Unknown drift forces inside the jamming area.} We consider a scenario where the drone might be affected by an unknown wind drift. We consider a drift force constituted by both a random direction and a random strength. Nevertheless, no drift is considered outside the jamming area, since the drone, being able to receive the \ac{GPS} signal, can autonomously compensate and correct its position accordingly.
That is, even if the drift force is there, it does not affect the drones navigation capabilities since this last one can compensate the drift generated effects.

{\bf No \acp{INS}.} We assume the worst case scenario according to which the drone does not resort to any navigation system based on sensors that might allow to compute its current position and planning the future trajectory. 

{\bf Simulator parameters.} Table \ref{tab:notation} wraps up the notation used throughout this paper and it introduces some of the parameters adopted in the simulator. 

\begin{table}[ht]
\caption{Notation summary\label{tab:notation}}
\begin{tabular}{l | l}
 \ac{RSS}                           & Received Signal Strength \\
 $\mathcal{P}_{thr} = -30 dBm$      & Received Signal Strength at the jamming\\ 
                                    & area boundary  \\
 $\mathcal{D}_{thr} = 479 m$        & Distance between the jammer and the \\
                                    & jamming area boundary \\
 $P(t)$                             & Received Signal Strength by the drone \\
 $V_D = 1 m/s$                      & Reference speed of the drone \\ 
 $\alpha_w$                         & Wind angle \\
 $(T_x, T_y)$                       & Target position \\
 $(ep_x, ep_y)$                     & Entry point in the jamming area \\
 $P_T(t)$                           & Expected Received Signal Strength at the \\
                                    & target position \\
\end{tabular}
\end{table}

\section{Leveraging jamming for drone navigation}
\label{sec:drone_navigation_jamming}

In this section, we provide the architectural model of a \textit{flight controller} being able to fly a drone close to a target in the presence of a jammer. As previously introduced in Section \ref{sec:scenario}, our idea mainly resorts to leverage the \ac{RSS} estimated by the drone with respect to the jammer, so as to infer on the direction to take to reach the target.

Figure \ref{fig:controller} depicts the block diagram of a standard \textit{closed loop control system} constituted by a \ac{PID} controller and the drone.

* *For further details, please refer to the paper.*

## Requirements
* MATLAB release R2018b or newer
* Statistics and Machine Learning Toolbox

## How to run the code
In order to replicate a scenario with no wind and the target position not equal to the jammer position, you can execute the following `code` on the MATLAB Command Window:

`clc; clear all; d = drone(); d = d.setwinddir(-pi/2); d = d.set_wind_strength(0); d.target_pos = [8,11]; d.mov = [0.1 0]; d = d.fly(); d.show()
`

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
`JAM-ME` is released under the GNU General Public License v3.0 <a href="LICENSE">license</a>.
