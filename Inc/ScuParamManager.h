#ifndef SCUPARAMMANAGER_H
#define SCUPARAMMANAGER_H

#include <uavcan/uavcan.hpp>

#include <uavcan/protocol/param_server.hpp>

class ScuParamManager: public uavcan::IParamManager
{
	void getParamNameByIndex(Index index, Name& out_name) const;
	void assignParamValue(const Name& name, const Value& value);
	void readParamValue(const Name& name, Value& out_value) const;
	void readParamDefaultMaxMin(const Name& name, Value& out_def,
		NumericValue& out_max, NumericValue& out_min) const;
	int eraseAllParams();
	int saveAllParams();
};

#endif
