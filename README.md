[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

# A Simple Python Implementation of Information-Theoretic Model Predictive Path Integral Controller

A simple python implementation of Information-Theoretic Model Predictive Path-Integral (MPPI) controller introduced in [[G. Williams et al., 2018]](#references) in their 2018 paper. This project is based on the work by [Mizuho Aoki](https://mizuhoaoki.github.io/) but contains the following modifications

* Notebook to generate and visualize custom trajectories
* JIT acceleration of cost function evaluation using Numba.
* Ideal-Differential Drive Kinematic model
* Both ```stage``` and ```terminal``` costs implemented as simple quadratic const function.

This work was part of my final project for EE 7500: Model Predictive Control class taught by [Dr. Xiangyu Meng](https://sites.google.com/view/xmeng/home?authuser=0) at Louisiana State University.

## Supported Dynamics models

* Ideal Differential Drive: See my project report in ```pdfs_notes``` for more details
* Ideal Bicycle model --> Head over to to Mizuho's repository.

## Video demonstrations

* Oval path with circular obstacles

https://github.com/user-attachments/assets/94782162-5836-463f-b8b2-d517084519e6

* Figure-8 path.

https://github.com/user-attachments/assets/59da69be-e605-4256-b4b6-df89c486909e

## Setup

* Clone this repo, create a mamba (or conda enviornment) and install the required dependencies

```bash
mamba create --name ee7500_proj --clone base
mamba activate ee7500_proj
mamba install ipykernel numpy pandas matplotlib seaborn numba
```

## Optional

* [ffmpeg](https://ffmpeg.org/)
  * mp4 movie writer
  * <details>
    <summary>installation details</summary>

    * For Ubuntu Users
      * `sudo apt-get update`
      * `sudo apt-get -y install ffmpeg`
    * For Windows Users
      * Install [scoop](https://scoop.sh/)
      * `scoop install ffmpeg`
    * For macOS Users
      * Install [homebrew](https://brew.sh/)
      * `brew install ffmpeg`
    * Check the official website if necessary
      * https://ffmpeg.org/
    </details>

## Usage

* To see the differential drive model in action run either ```mppi_ee7500_proj_oval.ipynb``` or ```mppi_ee7500_proj_figure_eight.ipynb``` notebooks.
* To visualize or create custom paths, use the ```traj_gen.ipynb```.
* To add new dynamics models see the ```./scripts/diffdrive.py```.
* To add/modify cost functions/distribution sampling see the ```./scripts/mppi.py```.
* Various plotting and computational utility functions are in ```./scripts/utils.py```.


## References
1. G. Williams et al. "Information-Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving" 
    * URL : https://ieeexplore.ieee.org/document/8558663
    * PDF : https://arxiv.org/pdf/1707.02342.pdf

## Other notable projects
* mpc_python: https://github.com/mcarfagno/mpc_python
* nav2's MPPI impelemtation: https://vimeo.com/879001391
