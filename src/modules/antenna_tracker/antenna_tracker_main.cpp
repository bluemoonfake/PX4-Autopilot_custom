#include "antenna_tracker.hpp"

AntennaTracker::AntennaTracker() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

bool AntennaTracker::init()
{
	return true;
}

void AntennaTracker::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}
}

int AntennaTracker::task_spawn(int argc, char *argv[])
{
	AntennaTracker *instance = new AntennaTracker();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int AntennaTracker::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "start")) {
		return start(argc, argv);
	}
	if (!strcmp(argv[0], "stop")) {
		return stop();
	}
	if (!strcmp(argv[0], "status")) {
		return status();
	}
	return print_usage("unknown command");
}

int AntennaTracker::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Antenna tracker controller module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("antenna_tracker", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int antenna_tracker_main(int argc, char *argv[])
{
	return AntennaTracker::main(argc, argv);
}
