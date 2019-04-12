#! /usr/bin/env python
#| This file is a part of the pyite framework.
#| Copyright 2019, INRIA
#| Main contributor(s):
#| Jean-Baptiste Mouret, jean-baptiste.mouret@inria.fr
#| Eloïse Dalin , eloise.dalin@inria.fr
#| Pierre Desreumaux , pierre.desreumaux@inria.fr
#|
#| Antoine Cully, Jeff Clune, Danesh Tarapore, and Jean-Baptiste Mouret.
#|"Robots that can adapt like animals." Nature 521, no. 7553 (2015): 503-507.
#|
#| This software is governed by the CeCILL license under French law
#| and abiding by the rules of distribution of free software.  You
#| can use, modify and/ or redistribute the software under the terms
#| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
#| following URL "http://www.cecill.info".
#|
#| As a counterpart to the access to the source code and rights to
#| copy, modify and redistribute granted by the license, users are
#| provided only with a limited warranty and the software's author,
#| the holder of the economic rights, and the successive licensors
#| have only limited liability.
#|
#| In this respect, the user's attention is drawn to the risks
#| associated with loading, using, modifying and/or developing or
#| reproducing the software by the user in light of its specific
#| status of free software, that may mean that it is complicated to
#| manipulate, and that also therefore means that it is reserved for
#| developers and experienced professionals having in-depth computer
#| knowledge. Users are therefore encouraged to load and test the
#| software's suitability as regards their requirements in conditions
#| enabling the security of their systems and/or data to be ensured
#| and, more generally, to use and operate it in the same conditions
#| as regards security.
#|
#| The fact that you are presently reading this means that you have
#| had knowledge of the CeCILL license and that you accept its terms.
from copy import *
from math import *
import numpy as np
import GPy as GPy
import sys
import matplotlib.pyplot as plt
#Load the CVT voronoi centroids from input archive
def load_centroids(filename):
	points = np.loadtxt(filename)
	return points

#Load map data from archive
def load_data(filename, dim,dim_ctrl):
	print("Loading ",filename)
	data = np.loadtxt(filename)
	fit = data[:, 0:1]
	desc = data[:,1: dim+1]
	x = data[:,dim+1:dim+1+dim_ctrl]
	return fit, desc, x

#Aquisition function for the bayesian optimization
def UCB(mu_map,kappa,sigma_map):
	GP = []
	for i in range(0,len(mu_map)):
		GP.append(mu_map[i] + kappa*sigma_map[i])
	return np.argmax(GP)

def plot_mu_sigma(mu_map, sigma_map):
	# Visualize the result
	le = len(mu_map)
	tmp = mu_map.reshape((le,))
	tmp_sampled = []
	sigma_sampled = []
	for i in range(0,le):
		if(i%50==0):
			tmp_sampled.append(tmp[i])
			sigma_sampled.append(sigma_map[i][0])
	tmp_sampled = np.array(tmp_sampled)
	sigma_sampled = np.array(sigma_sampled)
	plt.plot(range(0,len(tmp_sampled)), tmp_sampled, 'or')
	plt.fill_between(range(0,len(tmp_sampled)), tmp_sampled-sigma_sampled, tmp_sampled+sigma_sampled, color='gray', alpha=0.2)
	plt.show()

def plot_mean(n_descs,means,variances):
	x = range(0,len(n_descs))
	y = copy(means)
	e = copy(variances)
	plt.plot(x, y,'b+')
	plt.show()
