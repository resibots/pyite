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
