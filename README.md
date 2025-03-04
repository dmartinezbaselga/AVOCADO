# AVOCADO: Adaptive Optimal Collision Avoidance driven by Opinion (T-RO, 2025)

This repo provides the code of the work for out paper AVOCADO: Adaptive Optimal Collision Avoidance driven by Opinion, published in IEEE Transactions on Robotics in 2025.

## [PAPER](https://arxiv.org/pdf/2407.00507) || [VIDEO](https://youtu.be/zOq-3si8K5Q) || [HARDWARE EXPERIMENTS VIDEO](https://youtu.be/hib4Tbsfc30)

## Abstract

We present AVOCADO (AdaptiVe Optimal Collision Avoidance Driven by Opinion), a novel navigation approach to address holonomic robot collision avoidance when the robot does not know how cooperative the other agents in the environment are. AVOCADO departs from a Velocity Obstacle's (VO) formulation akin to the Optimal Reciprocal Collision Avoidance method. However, instead of assuming reciprocity, it poses an adaptive control problem to adapt to the cooperation level of other robots and agents in real time. This is achieved through a novel nonlinear opinion dynamics design that relies solely on sensor observations. As a by-product, we leverage tools from the opinion dynamics formulation to naturally avoid the deadlocks in geometrically symmetric scenarios that typically suffer VO-based planners.
Extensive numerical simulations show that AVOCADO surpasses existing motion planners in mixed cooperative/non-cooperative navigation environments in terms of success rate, time to goal and computational time. In addition, we conduct multiple real experiments that verify that AVOCADO is able to avoid collisions in environments crowded with other robots and humans.

## Native Setup
Tested with Python 3.8 (but should work with others):
<pre>
git clone https://github.com/dmartinezbaselga/AVOCADO.git
cd AVOCADO
pip install Cython matplotlib
pip install -e . # You might need to run the flag --no-build-isolation
</pre>

## Container Setup
<pre>
git clone https://github.com/dmartinezbaselga/AVOCADO.git
cd AVOCADO
docker compose up -d # builds and starts the container
xhost + # this will enable visualization
docker exec -it avocado-dev /bin/bash # open a terminal inside the container
pip install -r requirements.txt
cd AVOCADO
pip install -e .
</pre>
To shut down the container:
<pre>
exit # first get out of the container
docker compose down # in the same dir you did docker compose up
</pre>
## Getting started

Once you have installed AVOCADO library, you may just execute the Python scripts. You may either install the other motion planners, named actors in **actors.py**. They are currently comented. Most of them may be downloaded [here](https://github.com/dmartinezbaselga/intrinsic-rewards-navigation) (branch AVOCADO-exp) and RL-RVO [here](https://github.com/hanruihua/rl_rvo_nav). Follow these repos installation instructions.

The Python scripts provided to run experiments are the following:
- **simple_example.py**: Probably the best file to start with. It launches a simple simulation.
- **simple_example_v2.py** and **simple_example_v3.py**: They run simulations in a "lower level" than **simple_example.py**. These three files run AVOCADO using different code. See them select the best way to run AVOCADO in your own code.
- **alpha_experiment.py**: See the Opinion evolution vs. a non-cooperative agent.
- **bayesian_optimization.py** and **circle_tests.py**: Run experiments to get the best parameters for some scenarios based on metrics.
- **example_visualization.py**: Run a "raw" experiment, without our simulator.
- **experiments.py**: Run quantitative experiments of the paper (Check the motion planners that are commented or not in this file too).
- **plot_data_results.py**: Plot the results of the quantitative experiments. You will have to comment or not some lines of the script to obtain all the plots.
- **qualitative_experiments.py** and **render_sim_videos.py**: Examples of how to run the qualitative experiments of the paper.
- **times_experiment.py**: Experiments regarding the computation time of the methods.

You should open and change this files to run your own experiments. Note that inside all of them, some experiments are commented or not.

To run the baselines, install [RL-RVO](https://github.com/hanruihua/rl_rvo_nav) and [this repo in the AVOCADO-exp branch](https://github.com/dmartinezbaselga/intrinsic-rewards-navigation/tree/AVOCADO-exp).

## Simulations (Circle scenarios)

<img src="doc/avocado-25-1-circle.gif" width=25% heigth=25%\><img src="doc/avocado-25-13-circle.gif" width=25% heigth=25%\><img src="doc/avocado-25-19-circle.gif" width=25% heigth=25%\><img src="doc/avocado-25-25-circle.gif" width=25% heigth=25%\>


## Citation
If you use this work in your own research or wish to refer to the paper's results, please use the following BibTeX entries.

```bibtex
@misc{martinez2024avocado,
      title={AVOCADO: Adaptive Optimal Collision Avoidance driven by Opinion}, 
      author={Martinez-Baselga, Diego and Sebasti{\'a}n, Eduardo and Montijano, Eduardo and Riazuelo, Luis and Sag{\"u}{\'e}s, Carlos and Montano, Luis},
      year={2024},
      eprint={2407.00507},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2407.00507}, 
}
```

### Attribution
AVOCADO builds on the RVO2 Library:
- Copyright (c) 2008 University of North Carolina at Chapel Hill
- Licensed under the Apache License, Version 2.0
- Original authors: Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, and Dinesh Manocha
- For more information, visit the original project at <https://gamma.cs.unc.edu/RVO2/>.

### Acknowledge
This work uses [ORCA Python bindings](https://github.com/sybrenstuvel/Python-RVO2). The authors are thankful for their work and for making it available.

### Licensing
This project uses a dual-licensing structure:
1. **Apache License 2.0**:
   - Applies to the original code from the RVO2 Library.
   - See [LICENSE](https://www.apache.org/licenses/LICENSE-2.0) for the full text.
2. **GNU Affero General Public License, version 3 (AGPL-3.0)**:
   - Applies to all modifications and new code developed as part of AVOCADO.
   - See [LICENSE](https://www.gnu.org/licenses/agpl-3.0.html#license-text) for the full text.

### Modifications
The following changes have been made to the original RVO2 Library:
1. Implementation of the methods and functionalities described in AVOCADO: Adative Optimal Collision Avoidance Driven by Opinion.
2. Implementation of a Python simulator and visualizer.

### Bug Reports and Support
For issues related to the work, please contact:
- Diego Martinez-Baselga: `diegomartinez@unizar.es`
- Eduardo Sebastián: `esebastian@unizar.es`
