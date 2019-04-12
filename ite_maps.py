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
from ite_utils import *
import sys
sys.path.append('..')

def ite(params):
	#define arrays that will contain data to save
	num_its = []
	percent_affected_in_map, in_number = [], []
	mean_final, var_final = [], []
	tested_descs_maps, tested_ctrls_maps, tested_indexes_maps, tested_perfs_maps = [], [], [], []
	best_ctrls, best_descs, best_index, best_dist = [], [], [], []

	############## NUM OF MAPS TO TEST #################################################
	maps_to_test = range(1,11) #Here it is also the names of the experiements folders

	############### ITE CONSTANTS ######################################################
	alpha = 0.8
	kappa = 0.03
	rho = 0.4
	variance_noise_square = 0.001
	episode_duration = 10
	random = 0 #Put this to 1 to use random strategy instead of ITE
	max_iteration_number = 20 #max iteration number to allow

	################# PARSE INFO TO GET THE MAP ########################################
	if len(sys.argv) < 4:
		sys.exit('Usage:\n\n%s path_to_exp_files mode robot  \n\nmode is 0 for simulation or 1 for ros \nrobot is minitaur or hexapod\n\nEx : %s ~/centroids_40_5.dat ~/archive_100.dat 1 minitaur' %  (sys.argv[0],sys.argv[0]))
	robot = sys.argv[3]
	if(robot=="minitaur"):
		import ite_minitaur_utils as robot_utils
		from ite_minitaur_utils import init_env_ros,init_env_sim,eval_ros,eval_sim,safety_check
		dim_ctrl = 24
		dim_x = 16
		sys.path.append(params["pybullet_minitaur_sim_path"])
	elif(robot=="hexapod"):
		import ite_hexapod_utils as robot_utils
		from ite_hexapod_utils import init_env_ros,init_env_sim,eval_ros,eval_sim,safety_check
		dim_ctrl = 36
		dim_x = 6
		sys.path.append(params["pyhexapod_path"])
	else:
		sys.exit("robot arg is either minitaur or hexapod")
	ros = int(sys.argv[2])
	if(ros==1):
		init_env_ros()


	for map in maps_to_test:
		#utils to fill in array to be saved
		percents = []
		in_count = 0
		means_prev = []

		path_to_centroid = sys.argv[1] +str(map)+"/"+params["centroid_file_name"]
		path_to_archive = sys.argv[1] +str(map)+"/"+params["archive_file_name"]

		centroids = load_centroids(path_to_centroid)
		fits, descs, ctrls = load_data(path_to_archive, centroids.shape[1],dim_ctrl)

		################# FILTER OR NOT THE MAP BASED ON TRAVELLED DISTANCE #################
		n_fits, n_descs, n_ctrls = [], [], []
		for i in range(0,len(fits)):
			#if(fits[i]>1): #filter the fitness , discard the ones < 1
			if(1):
				n_fits.append(fits[i])
				n_descs.append(descs[i])
				n_ctrls.append(ctrls[i])
		n_fits = np.array(n_fits)
		n_descs = np.array(n_descs)
		n_ctrls = np.array(n_ctrls)

		n_fits_real = copy(np.array(n_fits))
		fits_saved = copy(n_fits)
		next_index_to_test = np.argmax(n_fits)

		print ("Max in the map : ", max(n_fits))
		print("Associated ctrl index : ", next_index_to_test)
		# ############### FOR ROS ONLY TO SYNC WITH PYMINITAUR/HEXAPOD PACKAGE AND SEND CMDS #########
		if(ros==1):
			import rospy
			#Define node and rate
			rospy.init_node('ite', anonymous=True)
			rate = rospy.Rate(1000) # 10hz

		################ ELSE USE SIMULATION #############################################
		else:
			init_env_sim()

		################ INIT ITE  ###########################################################
		robot_utils.end_episode_distance = -1000
		prev_ep_distance = -1000

		init, run = True, True
		num_it = 0
		real_perfs, tested_indexes, tested_ctrls,tested_descs =[-1000], [],[],[]
		X, Y = [], []
		############## BEGIN ITE ###################################################
		while(run):
			stop_cond =  alpha*max(n_fits_real)
			if(ros==1):
				raw_input("Press Enter to continue...")
			print("")
			print("Iteration i : ", num_it)

			if(not init):
				#define GP kernerl
				ker = GPy.kern.Matern52(dim_x,lengthscale = rho, ARD=False) + GPy.kern.White(dim_x,np.sqrt(variance_noise_square))
				#define Gp which is here the difference btwn map perf and real perf
				m = GPy.models.GPRegression(X,Y,ker)
				#predict means and variances for the difference btwn map perf and real perf
				means, variances = m.predict(n_descs)
				#Add the predicted difference to the map found in simulation
				for j in range(0,len(n_fits_real)):
					n_fits_real[j] = means[j] + fits_saved[j]
				#Compute how many percent of the behaviors have been significantly changed by this iteration
				changed = 0
				if(len(means_prev)>0):
					for y in range(0,len(means)):
						if(abs(means[y]-means_prev[y])>0.1):
							changed = changed + 1
					percent = changed*100/len(means)
					percents.append(percent)
				means_prev = means
				#apply acquisition function to get next index to test
				next_index_to_test = UCB(n_fits_real,kappa,variances)
			else:
				real_perfs = []
				init = False
			#if the behavior to test has already been tested, don't test it again
			if(next_index_to_test in tested_indexes):
				for h in range(0,len(tested_indexes)):
					if(next_index_to_test==tested_indexes[h]):
						tmp_id = h
				real_perf = real_perfs[tmp_id]
				ctrl_to_test = n_ctrls[next_index_to_test]
				tested_indexes.append(next_index_to_test)
				in_count = in_count + 1
			else:
				ctrl_to_test = n_ctrls[next_index_to_test]
				tested_indexes.append(next_index_to_test)
				# eval the real performance
				if(ros==1):
					eval_ros(episode_duration,ctrl_to_test,rate,ros)
					if(robot_utils.end_episode_distance == prev_ep_distance):
						sys.exit("WARNING : ite hasn't been able to recover the latest distance, exiting ite")
					else:
						prev_ep_distance = robot_utils.end_episode_distance
						real_perf = robot_utils.end_episode_distance
				else:
					real_perf, real_desc = eval_sim(ctrl_to_test)
			if(len(X)==0):
				X.append(n_descs[next_index_to_test])
				Y.append(np.array(real_perf)-fits_saved[next_index_to_test])
				X = np.array(X)
				Y = np.array(Y)
			else:
				X = np.vstack((X,n_descs[next_index_to_test]))
				Y = np.vstack((Y,np.array(real_perf)-fits_saved[next_index_to_test]))
			#store it
			real_perfs.append(real_perf)
			tested_ctrls.append(ctrl_to_test)
			#store the tested descs
			tested_descs.append(n_descs[next_index_to_test])
			o = np.argmax(real_perfs)
			max_index = tested_indexes[o]
			max_perf = real_perfs[o]
			print("Max performance found with ",num_it," iterations : ", max(real_perfs))
			print("Associated max index ", max_index)
			print(real_perfs)
			print(tested_indexes)
			num_it = num_it + 1
			cond = max(real_perfs)
			if(cond > stop_cond):
				run = False
			if(num_it == max_iteration_number):
				run = False
			#Restart the minitaur if it has tried an impossible motion
			if(ros==1):
				safety_check()

		print(" ")
		print("Max performance found with ",num_it," iterations : ", max(real_perfs))
		print("Associated max index ", max_index)
		print("Associated controller ",tested_ctrls[np.argmax(real_perfs)])
		print("Associated descriptor ", tested_descs[np.argmax(real_perfs)])
		best_ctrls.append(tested_ctrls[np.argmax(real_perfs)])
		best_descs.append(tested_descs[np.argmax(real_perfs)])
		num_its.append(num_it)
		percent_affected_in_map.append(np.mean(np.array(percents)))
		in_number.append(in_count)
		mean_final.append(means)
		var_final.append(variances)
		best_dist.append(max(real_perfs))
		best_index.append(max_index)
		tested_descs_maps.append(tested_descs)
		tested_ctrls_maps.append(tested_ctrls)
		tested_indexes_maps.append(tested_indexes)
		tested_perfs_maps.append(real_perfs)
		np.save('num_its',np.array(num_its))
		np.save('percent_affected_in_map',np.array(percent_affected_in_map))
		np.save('in_number',np.array(in_number))
		np.save('mean_final',np.array(mean_final))
		np.save('var_final',np.array(var_final))
		np.save('best_dist',np.array(best_dist))
		np.save('best_index',np.array(best_index))
		np.save('best_ctrls',np.array(best_ctrls))
		np.save('best_descs',np.array(best_descs))
		np.save('tested_descs_maps',np.array(tested_descs_maps))
		np.save('tested_ctrls_maps',np.array(tested_ctrls_maps))
		np.save('tested_perfs_maps',np.array(tested_perfs_maps))
		np.save('tested_indexes_maps',np.array(tested_indexes_maps))
		# plot_GP(mu_map,sigma_map,fits,descs)

params = \
	    {
	        "pybullet_minitaur_sim_path": "/home/eloise/minitaur_framework/pybullet_minitaur_sim",
			"pyhexapod_path": "/home/eloise/hexa_ite_test/pyhexapod",
			"centroid_file_name" : "centroids_40000_16.dat",
			"archive_file_name" : "archive_20000.dat"
	    }

if __name__ == "__main__":
	ite(params)
