px4_add_module(
	MODULE modules__antenna_tracker
	MAIN antenna_tracker
	SRCS
		antenna_tracker_main.cpp
		antenna_tracker.hpp
	DEPENDS
		px4_work_queue
	)
