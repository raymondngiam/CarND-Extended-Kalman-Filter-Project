# Udacity Self Driving Car Nanodegree
## Extended Kalman Filter Project 

### Overview

This is a project for Udacity's Self Driving Car Nanodegree. The objective is to utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

### Video Demo

![Demo](/images/small.gif)

---

### Implementation Summary

**State Transition Model**

Using constant velocity (CV) model, the process model is as described below:

![](https://latex.codecogs.com/gif.latex?\begin{pmatrix}p_{x}'%5C%5Cp_{y}'%5C%5Cv_{x}'%5C%5Cv_{y}'\end{pmatrix}&space;=&space;\begin{pmatrix}1&0&\Delta&space;t&0%5C%5C0&1&0&\Delta&space;t%5C%5C0&0&1&0%5C%5C0&0&0&1\end{pmatrix}&space;\begin{pmatrix}p_{x}%5C%5Cp_{y}%5C%5Cv_{x}%5C%5Cv_{y}\end{pmatrix}&space;&plus;&space;\begin{pmatrix}\upsilon&space;_{px}%5C%5C\upsilon&space;_{py}%5C%5C\upsilon&space;_{vx}%5C%5C\upsilon_{vy}\end{pmatrix})

![](https://latex.codecogs.com/gif.latex?\mathbf{x'}=\mathbf{F}\mathbf{x}&plus;\boldsymbol{\upsilon})

![](https://latex.codecogs.com/gif.latex?\boldsymbol{\upsilon}\sim&space;\mathcal{N}(0,\mathbf{Q}))

Process noise covariance matrix, ![](https://latex.codecogs.com/gif.latex?\mathbf{Q}), is defined as follows:

![](https://latex.codecogs.com/gif.latex?\mathbf{Q}=\begin{pmatrix}\frac{\Delta&space;t^{4}}{4}\sigma_{ax}^{2}&0&\frac{\Delta&space;t^{3}}{2}\sigma_{ax}^{2}&0%5C%5C0&\frac{\Delta&space;t^{4}}{4}\sigma_{ay}^{2}&0&\frac{\Delta&space;t^{3}}{2}\sigma_{ay}^{2}%5C%5C\frac{\Delta&space;t^{3}}{2}\sigma_{ax}^{2}&0&\Delta&space;t^{2}\sigma_{ax}^{2}&0%5C%5C0&\frac{\Delta&space;t^{3}}{2}\sigma_{ax}^{2}&0&&space;\Delta&space;t^{2}\sigma_{ay}^{2}\end{pmatrix})

where, ![](https://latex.codecogs.com/gif.latex?\sigma_{ax}^{2}) and ![](https://latex.codecogs.com/gif.latex?\sigma_{ay}^{2}) are variance of the acceleration in x and y direction.


**Measurement Model (Lidar)**

![](https://latex.codecogs.com/gif.latex?\begin{pmatrix}p_{x}%5C%5Cp_{y}\end{pmatrix}=\begin{pmatrix}1&0&0&0%5C%5C0&1&0&0\end{pmatrix}\begin{pmatrix}p_{x}'%5C%5Cp_{y}'%5C%5Cv_{y}'%5C%5Cv_{y}'\end{pmatrix}&plus;\begin{pmatrix}\omega_{px}%5C%5C\omega_{py}\end{pmatrix})

![](https://latex.codecogs.com/gif.latex?\mathbf{x}=\mathbf{H}\mathbf{x'}&plus;\boldsymbol{\omega_{lidar}})

![](https://latex.codecogs.com/gif.latex?\boldsymbol{\omega_{lidar}}\sim&space;\mathcal{N}(0,\mathbf{R_{lidar}}))

![](https://latex.codecogs.com/gif.latex?\mathbf{R_{lidar}}=\begin{pmatrix}\sigma_{px}^{2}&0%5C%5C0&\sigma_{px}^{2}\end{pmatrix})

where, ![](https://latex.codecogs.com/gif.latex?\sigma_{px}^{2}) and ![](https://latex.codecogs.com/gif.latex?\sigma_{py}^{2}) are variance of the x and y measurements of the lidar sensor.


**Measurement Model (Radar)**

![](https://latex.codecogs.com/gif.latex?\begin{pmatrix}\rho%5C%5C\phi%5C%5C\dot{\rho}\end{pmatrix}=\begin{pmatrix}\sqrt{p_{x}'^{2}&plus;p_{y}'^{2}}%5C%5C\arctan(\frac{p_{y}'}{p_{x}'})%5C%5C\frac{p_{x}'v_{x}'&plus;p_{y}'v_{y}'}{\sqrt{p_{x}'^{2}&plus;p_{y}'^{2}}}\end{pmatrix}&plus;\begin{pmatrix}\omega_{\rho}%5C%5C\omega_{\phi}%5C%5C\omega_{\dot{\rho}}\end{pmatrix})

![](https://latex.codecogs.com/gif.latex?\mathbf{x}=h(\mathbf{x}')&plus;\boldsymbol{\omega_{radar}})

![](https://latex.codecogs.com/gif.latex?\boldsymbol{\omega_{radar}}\sim&space;\mathcal{N}(0,\mathbf{R_{radar}}))

![](https://latex.codecogs.com/gif.latex?\mathbf{R_{radar}}=\begin{pmatrix}\sigma_{\rho}^{2}&0&0%5C%5C0&\sigma_{\phi}^{2}&0%5C%5C0&0&\sigma_{\dot{\rho}}^{2}\end{pmatrix})

where, ![](https://latex.codecogs.com/gif.latex?\sigma_{\rho}^{2}), ![](https://latex.codecogs.com/gif.latex?\\sigma_{\phi}^{2}) and ![](https://latex.codecogs.com/gif.latex?\sigma_{\dot{\rho}}^{2}) are variance of the ![](https://latex.codecogs.com/gif.latex?\rho), ![](https://latex.codecogs.com/gif.latex?\phi), and ![](https://latex.codecogs.com/gif.latex?\dot{\rho}) measurements of the radar sensor.


**Kalman Filter Process Flow**

![Kalman Filter Process Flow](/images/screenshot-from-2017-02-27-19-56-58.png)


**Kalman Filter for Lidar**

Prediction:

![](https://latex.codecogs.com/gif.latex?\mathbf{x'}=\mathbf{F}\mathbf{x})

![](https://latex.codecogs.com/gif.latex?\mathbf{P'}=\mathbf{F}\mathbf{P}\mathbf{F^{T}}&plus;\mathbf{Q})

Measurement Update:

![](https://latex.codecogs.com/gif.latex?\mathbf{y}=\mathbf{z}-\mathbf{H}\mathbf{x'})

![](https://latex.codecogs.com/gif.latex?\mathbf{S}=\mathbf{H}\mathbf{P'}\mathbf{H^{T}&plus;\mathbf{R}})

![](https://latex.codecogs.com/gif.latex?\mathbf{K}=\mathbf{P'}\mathbf{H^{T}}\mathbf{S^{-1}})

![](https://latex.codecogs.com/gif.latex?\mathbf{x}=\mathbf{x'}&plus;\mathbf{K}\mathbf{y})

![](https://latex.codecogs.com/gif.latex?\mathbf{P}=(\mathbf{I}&plus;\mathbf{K}\mathbf{H})\mathbf{P'})


**Extended Kalman Filter for Radar**

Prediction:

![](https://latex.codecogs.com/gif.latex?\mathbf{x'}=\mathbf{F}\mathbf{x})

![](https://latex.codecogs.com/gif.latex?\mathbf{P'}=\mathbf{F}\mathbf{P}\mathbf{F^{T}}&plus;\mathbf{Q})

Measurement Update:

![](https://latex.codecogs.com/gif.latex?\mathbf{y}=\mathbf{z}-h(\mathbf{x'}))

![](https://latex.codecogs.com/gif.latex?\mathbf{S}=\mathbf{H_{j}}\mathbf{P'}\mathbf{H_{j}^{T}&plus;\mathbf{R}})

where ![](https://latex.codecogs.com/gif.latex?\mathbf{H_{j}}) is the Jacobian matrix of the nonlinear measurement function, ![](https://latex.codecogs.com/gif.latex?h(\mathbf{x'})) w.r.t the state vector ![](https://latex.codecogs.com/gif.latex?\mathbf{x}).

![](https://latex.codecogs.com/gif.latex?\mathbf{K}=\mathbf{P'}\mathbf{H_{j}^{T}}\mathbf{S^{-1}})

![](https://latex.codecogs.com/gif.latex?\mathbf{x}=\mathbf{x'}&plus;\mathbf{K}\mathbf{y})

![](https://latex.codecogs.com/gif.latex?\mathbf{P}=(\mathbf{I}&plus;\mathbf{K}\mathbf{H_{j}})\mathbf{P'})

---

### Installation

1. Download the Udacity Self Driving Car Nanodegree simulator from [here](https://github.com/udacity/self-driving-car-sim/releases).

2. Set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) by running the shell script in the project top directory:

```
$ install-ubuntu.sh
```

**Other Important Dependencies**

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4

---

### How to run

The main program can be built and run by doing the following from the project top directory.

1. mkdir build

2. cd build

3. cmake ..

4. make

5. ./ExtendedKF

Open the simulator, select `Project 1/2: EFK and UKF` and press `Start`.
