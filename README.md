# Pyite

This module is a python implementation of  [IT&E](https://github.com/resibots/ite_v2).

IT&E code for developing hexapod experiments similar to [Cully et al. (2015), Nature](https://github.com/resibots/cully_2015_nature). This code is not replicating the exact experiments of the Nature paper, but can be seen as a more modern implementation of the underlying algorithm.

#### Citing this code

If you use our code for a scientific paper, please cite:

Antoine Cully, Jeff Clune, Danesh Tarapore, and Jean-Baptiste Mouret. **"Robots that can adapt like animals."** *Nature 521, no. 7553 (2015): 503-507*.

In BibTex:

    @article{cully_robots_2015,
        title = {Robots that can adapt like animals},
        volume = {521},
        pages = {503--507},
        number = {7553},
        journal = {Nature},
        author = {Cully, Antoine and Clune, Jeff and Tarapore, Danesh and Mouret, Jean-Baptiste},
        year = {2015}
    }



Here are some IT&E use cases. You can click on the GIFs to watch the full videos.

[![](miscs/minitaur.gif)](https://www.youtube.com/watch?v=v90CWJ_HsnM)

[![](miscs/hexapod.gif)](https://www.youtube.com/watch?v=T-c17RKh3uE&list=PLc7kzd2NKtSdd4CjMjOJH1rmmVyf0EmBW&index=2&t=0s)


## How to install ?

Dependencies (use pip or your favorite package manager)
- python3
- matplotlib
- pybullet
- numpy
- tensorflow
- gym


Clone this repository and run :

``git submodule update --init --recursive``


## Use with simulation

IT&E will be used with the maps contained in maps folter. The damages used here are defined in the pybullet_minitaur_sim and pyhexapod repositories.

To create new maps or change the damages please refer to the pybullet_minitaur_sim and pyhexapod repository.

You can disable the gui by modifying the params at the end of ite.py

####  For the minitaur


``python ite.py maps/minitaur/centroids_40000_16.dat maps/minitaur/archive_20000.dat 0 minitaur``



#### For the hexapod :

``python ite.py maps/hexapod/centroids_40000_6.dat maps/hexapod/archive_20000.dat 0 hexapod``



## Use with the real robot

#### For the minitaur

In order to run it with the real minitaur robot please refer to the [minitaur_framework](https://github.com/resibots/minitaur_framework.git)

Note that ite_maps.py has been configured to test 10 different maps from 10 different experiments
#### For the hexapod :

In order to run it with the real pexod robot, you will need to use the C++ API : [IT&E](https://github.com/resibots/ite_v2)
