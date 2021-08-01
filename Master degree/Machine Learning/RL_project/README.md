# Reinforcement Learning project
This project is an implementation of the Soft Actor Critic (SAC) algorithm applied to the MuJoCo environment "[Ant-v2](https://gym.openai.com/envs/Ant-v2/)". It is based on the paper "[Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor](https://arxiv.org/abs/1801.01290)".

Please follow the instructions to compile and run our project.

### Install Anaconda

We have used the [Anaconda framework](https://docs.anaconda.com/anaconda/install/)

As you may see, it is available for several OSs but since the supported distributions of MuJoCo are either for Linux or for OS X, we suggest to choose one of these two OSs.

### MuJoCo license

1. Ask for a [MuJoCo license](https://www.roboti.us/license.html) following 'Step 1' instructions
2. Register your PC by compiling the 'Step 3' form.

### Install MuJoCo

Please follow the [instructions](https://github.com/openai/mujoco-py/) to correctly install MuJoCo

We provide you a [working tutorial](https://www.chenshiyu.top/blog/2019/06/19/Tutorial-Installation-and-Configuration-of-MuJoCo-Gym-Baselines/)

### Run the project

You may need additional requirements that are listed below. These can be added by installing it in the Anaconda environment that you have chosen to run the project as follows:
```bash
pip install gym
sudo apt-get install python-opengl -y
sudo apt install xvfb -y
pip install pyvirtualdisplay
pip install piglet
```

The following commands are required to install the package with the algorithm:
```bash
cd rl_project
pip install -e sac/.
```

To finally run the project, we provide you two Python Notebooks that differ in the type of training and in the adopted environment:
- "project_ant.ipynb", Soft Actor-Critic tested on 'Ant-v2' environment (with a stronger training inspired by the SpinningUp Documentation);
- "project_car.ipynb", Soft Actor-Critic tested on 'MountainCarContinuous-v0' environment (with a fast and simple training).

#### Results

We provide in the 'videos' folder some videos that represent our work and in the 'results' folder some related plots. We distinguish them with the 'ant_' or 'car_' prefixes, that denote the outcomes of "project_ant.ipynb" or "project_car.ipynb" respectively. 

The improvements that the algorithm has introduced are visible by comparing the provided videos of a random agent ("ant/car_init.avi") against a sac-trained agent ("ant/car_sac.avi").


## Authors
Leandro Maglianella - 1792507

Lorenzo Nicoletti - 1797464



