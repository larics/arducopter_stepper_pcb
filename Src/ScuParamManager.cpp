#include <ScuParamManager.h>
#include <flash.h>

extern mmControl_TypeDef scu_parameters;

void ScuParamManager::getParamNameByIndex(Index index, Name& out_name) const
{
	if (index == 0) { out_name = "MM_CONTROL_P"; }
	else if (index == 1 ) { out_name = "MM_CONTROL_OMEGA_MAX"; }
	else if (index == 2 ) { out_name = "MM_CONTROL_ACC_MAX"; }
	else if (index == 3 ) { out_name = "MM_CONTROL_DEAD_ZONE"; }
	else if (index == 3 ) { out_name = "MM_CONTROL_SAMPLING_FREQUENCY"; }
}

void ScuParamManager::assignParamValue(const Name& name, const Value& value)
{
	if (name == "MM_CONTROL_P")
	{
		if (value.is(uavcan::protocol::param::Value::Tag::real_value))
		{
			scu_parameters.P = 
				*value.as<uavcan::protocol::param::Value::Tag::real_value>();
		}
	}
	else if (name == "MM_CONTROL_OMEGA_MAX")
	{
		if (value.is(uavcan::protocol::param::Value::Tag::integer_value))
		{
			scu_parameters.omega_max = 
				(uint32_t)*value.as<uavcan::protocol::param::Value::Tag::integer_value>();
		}
	}
	else if (name == "MM_CONTROL_ACC_MAX")
	{
		if (value.is(uavcan::protocol::param::Value::Tag::integer_value))
		{
			scu_parameters.acc_max = 
				(uint32_t)*value.as<uavcan::protocol::param::Value::Tag::integer_value>();
		}
	}
	else if (name == "MM_CONTROL_DEAD_ZONE")
	{
		if (value.is(uavcan::protocol::param::Value::Tag::integer_value))
		{
			scu_parameters.dead_zone = 
				(uint32_t)*value.as<uavcan::protocol::param::Value::Tag::integer_value>();
		}
	}
	else if (name == "MM_CONTROL_SAMPLING_FREQUENCY")
	{
		if (value.is(uavcan::protocol::param::Value::Tag::integer_value))
		{
			scu_parameters.SamplingFrequency = 
				(uint32_t)*value.as<uavcan::protocol::param::Value::Tag::integer_value>();
		}
	}
}

void ScuParamManager::readParamValue(const Name& name, Value& out_value) const
{
	if (name == "MM_CONTROL_P")
	{
		out_value.to<uavcan::protocol::param::Value::Tag::real_value>() =
			scu_parameters.P;
	}
	else if (name == "MM_CONTROL_OMEGA_MAX")
	{
		out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = 
			scu_parameters.omega_max; 
	}
	else if (name == "MM_CONTROL_ACC_MAX")
	{
		out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() =
			scu_parameters.acc_max;
	}
	else if (name == "MM_CONTROL_DEAD_ZONE")
	{
		out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = 
			scu_parameters.dead_zone;
	}
	else if (name == "MM_CONTROL_SAMPLING_FREQUENCY")
	{
		out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = 
			scu_parameters.SamplingFrequency;
	}
}

void ScuParamManager::readParamDefaultMaxMin(const Name& name, Value& out_def,
		NumericValue& out_max, NumericValue& out_min) const
{
	if (name == "MM_CONTROL_P")
	{
		out_def.to<uavcan::protocol::param::Value::Tag::real_value>() =
			MM_CONTROL_P;
    //out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 253;
    //out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 0;
	}
	else if (name == "MM_CONTROL_OMEGA_MAX")
	{
		out_def.to<uavcan::protocol::param::Value::Tag::integer_value>() =
			MM_CONTROL_OMEGA_MAX;
    out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 10000;
    out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 0;
	}
	else if (name == "MM_CONTROL_ACC_MAX")
	{
		out_def.to<uavcan::protocol::param::Value::Tag::integer_value>() =
			MM_CONTROL_ACC_MAX;
    out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 20000;
    out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 0;
	}
	else if (name == "MM_CONTROL_DEAD_ZONE")
	{
		out_def.to<uavcan::protocol::param::Value::Tag::integer_value>() =
			MM_CONTROL_DEAD_ZONE;
    out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 100;
    out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 0;
	}
	else if (name == "MM_CONTROL_SAMPLING_FREQUENCY")
	{
		out_def.to<uavcan::protocol::param::Value::Tag::integer_value>() =
			MM_CONTROL_SAMPLING_FREQUENCY;
    out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 1000;
    out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 1;
	}
}

int ScuParamManager::eraseAllParams()
{
	readDefaultParameters(&scu_parameters);
	return 0;
}

int ScuParamManager::saveAllParams()
{
	writeParams2Flash(&scu_parameters);
	return 0;
}

	