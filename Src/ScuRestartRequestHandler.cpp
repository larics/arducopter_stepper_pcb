#include <ScuRestartRequestHandler.h>

bool ScuRestartRequestHandler::handleRestartRequest(uavcan::NodeID request_source)
{
	system_reset_flag = true;
	return true;	
}