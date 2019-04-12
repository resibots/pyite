#! /usr/bin/env python
#| This file is a part of the pyite framework.
#| Copyright 2019, INRIA
#| Main contributor(s):
#| Jean-Baptiste Mouret, jean-baptiste.mouret@inria.fr
#| Eloise Dalin , eloise.dalin@inria.fr
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
def init_env_ros():
		import rospy
		import sys
		import std_msgs
		from std_srvs.srv import Empty, EmptyResponse
		from std_msgs.msg import Bool
		from pyminitaur.srv import Distance, GoToNextEpisode,Duration, Mode, Ctrl
		global end_episode_distance
		global impossible_motion
		#Check robot namespace
		if rospy.has_param('robotname'):
			robotname = rospy.get_param('robotname')
		else:
			robotname = "robot0"
		def go_to_next_episode(req):
			global next_ep
			next_ep = True
			print("Next ep")
			return True
		def set_end_episode_distance(req):
			global end_episode_distance
			end_episode_distance = req.end_episode_distance
			return True
		rospy.Service('go_to_next_episode', GoToNextEpisode, go_to_next_episode)
		rospy.Service('set_end_episode_distance', Distance, set_end_episode_distance)
		def impossible_callback(data):
			global impossible_motion
			impossible_motion = data.data
		rospy.Subscriber(robotname + '/state/impossible_motion', Bool, impossible_callback)
		global impossible_motion
		impossible_motion = False

def init_env_sim():
	return 0
	#from simulator_map_elites_damaged import eval_minitaur

def eval_ros(episode_duration,ctrl_to_test,rate,ros):
	import rospy
	import std_msgs
	import sys
	from std_srvs.srv import Empty, EmptyResponse
	from std_msgs.msg import Bool
	from pyminitaur.srv import Distance, GoToNextEpisode,Duration, Mode, Ctrl
	global next_ep
	global end_episode_distance
	# eval the real performance
	if(ros==1):
		#set episode duration
		setduration = rospy.ServiceProxy('set_episode_duration', Duration)
		try:
			resp1 = setduration(episode_duration)
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
		setctrl = rospy.ServiceProxy('set_ctrl', Ctrl)
		try:
			resp1 = setctrl(ctrl_to_test)
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
		setmode = rospy.ServiceProxy('set_mode', Mode)
		try:
			resp1 = setmode(1)
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
		next_ep =  False #will be true when an episode running
		while(next_ep == False):
			if(rospy.is_shutdown()):
				try:
					resp1 = setmode(0)
				except rospy.ServiceException as exc:
					print("Service did not process request: " + str(exc))
				sys.exit("ROS IS DOWN")
			else:
				rate.sleep()

def eval_sim(ctrl_to_test, gui = False):
	from simulator_map_elites_damaged import eval_minitaur
	return eval_minitaur(ctrl_to_test,gui_eval = gui)

def safety_check():
	import rospy
	import sys
	import std_msgs
	from std_srvs.srv import Empty, EmptyResponse
	from std_msgs.msg import Bool
	from pyminitaur.srv import Distance, GoToNextEpisode,Duration, Mode, Ctrl
	global impossible_motion
	if(impossible_motion == True):
		restart = rospy.ServiceProxy('restart', Empty)
		try:
			resp = restart()
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
