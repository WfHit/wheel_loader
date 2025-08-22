px4_add_module(
	MODULE modules__vla_proxy
	MAIN vla_proxy
	SRCS
		vla_proxy.cpp
	DEPENDS
		px4_work_queue
	MODULE_CONFIG
		module.yaml
)
