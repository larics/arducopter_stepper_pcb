#ifndef SCU_RESTART_REQUEST_HANDLER_H
#define SCU_RESTART_REQUEST_HANDLER_H

#include <uavcan/uavcan.hpp>

class ScuRestartRequestHandler: public uavcan::IRestartRequestHandler
{
	bool handleRestartRequest(uavcan::NodeID request_source);
	
	public:
		bool system_reset_flag;
};

#endif
