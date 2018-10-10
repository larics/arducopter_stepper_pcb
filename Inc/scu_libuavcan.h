#ifndef SCU_LIBUAVCAN
#define SCU_LIBUAVCAN

#include <scu_libuavcanSTM32.h>
#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>

#include <uavcan/protocol/HardwareVersion.hpp>
#include <uavcan/protocol/SoftwareVersion.hpp>
#include <uavcan/protocol/RestartNode.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>

#include <ScuParamManager.h>
#include <ScuRestartRequestHandler.h>

#define UAVCAN_NODE_NAME		"stepper.control.unit"

static const unsigned NodeMemoryPoolSize = 25600;
typedef uavcan::Node<NodeMemoryPoolSize> Node;

static Node& getNode();
		
class scu_libuavcan{
	private:
		//prosiruje poruku s source_node_id timestampom ... 
		/* typedef uavcan::MethodBinder<Node*,
        void (Node::*)(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue>&) const>
            KeyValueCallbackBinder;*/
	
		typedef uavcan::MethodBinder<scu_libuavcan*, void (scu_libuavcan::*)(const uavcan::equipment::actuator::ArrayCommand&)>
			motorRefCallbackBinder;
		
		uavcan::Subscriber<uavcan::equipment::actuator::ArrayCommand, motorRefCallbackBinder> *motor_ref_sub;
		
		void motor_ref_callback(const uavcan::equipment::actuator::ArrayCommand& msg);

		uavcan::ParamServer *parameter_server_;
		uavcan::GlobalTimeSyncSlave *time_sync_slave_can_;
	
		Node *node_;
		
		ScuParamManager param_manager;
		ScuRestartRequestHandler restart_request_handler;
		
		uavcan::protocol::SoftwareVersion sw_version;
		uavcan::protocol::HardwareVersion hw_version;
		
	public:
		scu_libuavcan();
		void can_init(void);
		void scu_libuavcan_Init(int uavcanNodeID);
		void start(void);
		void spinOnce(void);
		void spin(uint32_t msec);
		bool get_system_reset_flag(void);
};

#endif
