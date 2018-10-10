#include <scu_libuavcan.h>
#include <gpio.h>

extern xQueueHandle xQueueMotorSetpoint[4];

uint32_t usedBlocks = 0;
uint32_t freeBlocks = 0;
uint32_t peakUsedBlocks = 0;

scu_libuavcan::scu_libuavcan(void)
{
	restart_request_handler.system_reset_flag = false;
	
	sw_version.major = SW_VERSION_MAJOR;
	sw_version.minor = SW_VERSION_MINOR;
	hw_version.major = HW_VERSION_MAJOR;
	hw_version.minor = HW_VERSION_MINOR;
}

void scu_libuavcan::can_init(void)
{
		GPIO_InitTypeDef GPIO_InitStruct;
	
		__GPIOB_CLK_ENABLE();
	
		GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void scu_libuavcan::scu_libuavcan_Init(int uavcanNodeID)
{
		node_ = &getNode();
    node_->setNodeID(uavcanNodeID);
		node_->setName(UAVCAN_NODE_NAME);
		node_->setSoftwareVersion(sw_version);
		node_->setHardwareVersion(hw_version);
		node_->start();
	
		static uavcan::Subscriber<uavcan::equipment::actuator::ArrayCommand, motorRefCallbackBinder> motor_ref_sub_(*node_);
		motor_ref_sub = &motor_ref_sub_;
		motor_ref_sub->start(motorRefCallbackBinder(this, &scu_libuavcan::motor_ref_callback));
	
		static uavcan::ParamServer parameter_server(*node_);
		parameter_server_ = &parameter_server;
		parameter_server_->start(&param_manager);
		
		static uavcan::GlobalTimeSyncSlave time_sync_slave_can(*node_);
		time_sync_slave_can_=&time_sync_slave_can;
		time_sync_slave_can_->start();
		
		uavcan_stm32::clock::adjustUtc(uavcan::UtcDuration::fromUSec(xTaskGetTickCount()));
		
		node_->setRestartRequestHandler(&restart_request_handler);
}

void scu_libuavcan::start(void)
{
	node_->setModeOperational();
}

void scu_libuavcan::spinOnce(void)
{
	node_->spinOnce();
}

bool scu_libuavcan::get_system_reset_flag(void)
{
	return restart_request_handler.system_reset_flag;
}

void scu_libuavcan::motor_ref_callback(const uavcan::equipment::actuator::ArrayCommand& msg)
{
	int32_t ref = msg.commands[0].command_value;
	xQueueOverwrite(xQueueMotorSetpoint[0],(void *)&ref);
	
	ref = msg.commands[1].command_value;
	xQueueOverwrite(xQueueMotorSetpoint[1],(void *)&ref);
				
	ref = msg.commands[2].command_value;
	xQueueOverwrite(xQueueMotorSetpoint[2],(void *)&ref);
				
	ref = msg.commands[3].command_value;
	xQueueOverwrite(xQueueMotorSetpoint[3],(void *)&ref);
}

void scu_libuavcan::spin(uint32_t msec)
{
	usedBlocks = node_->getAllocator().getNumUsedBlocks();
  freeBlocks = node_->getAllocator().getNumFreeBlocks();
  peakUsedBlocks = node_->getAllocator().getPeakNumUsedBlocks();

	node_->spin(uavcan::MonotonicDuration::fromMSec(msec));
}

static Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}
