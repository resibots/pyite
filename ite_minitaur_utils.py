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
