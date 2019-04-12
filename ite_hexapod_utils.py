def init_env_ros():
		print("WARNING TO USE ITE ON  THE REAL HEAXAPOD PLEASE USE C++ VERSION")
		return 0

def init_env_sim():
    return 0
	#from simulator_map_elites_damaged import eval_minitaur

def eval_ros(rospy,episode_duration,ctrl_to_test,next_ep,rate):
	print("WARNING TO USE ITE ON  THE REAL HEAXAPOD PLEASE USE C++ VERSION")
	return 0


def eval_sim(ctrl_to_test,gui = False):
    from simulator import eval_hexapod
    return eval_hexapod(ctrl_to_test,gui_eval = gui,damage=True)

def safety_check():
	return 0
