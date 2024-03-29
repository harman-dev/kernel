#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE lnp_ldisc_trace

#define TRACE_SYSTEM lbf

#if !defined(_TRACE_LBF_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_LBF_H

#include <linux/tracepoint.h>

#define device_name(state) { state, #state }
#define show_device_name(val)		\
	__print_symbolic(val,		\
		device_name(D0),	\
		device_name(D2),	\
		device_name(D3),	\
		device_name(D0_TO_D2),	\
		device_name(D2_TO_D0))

#define host_wake(state) { state, #state }
#define show_hostwake(val)		\
	__print_symbolic(val,		\
		host_wake(LOW),	\
		host_wake(HIGH))

#define bt_fmr(state) { state, #state }
#define show_bt_fmr(val)			\
	__print_symbolic(val,			\
		bt_fmr(IDLE),	\
		bt_fmr(ACTIVE))

DECLARE_EVENT_CLASS(lbf_ldisc,

	TP_PROTO(const char *func),

	TP_ARGS(func),

	TP_STRUCT__entry(
		__string(name, func)
	),

	TP_fast_assign(
		__assign_str(name, func);
	),

	TP_printk("->%s",
			__get_str(name))
);

DEFINE_EVENT(lbf_ldisc, lbf_func_start,
	TP_PROTO(const char *func),
	TP_ARGS(func)
);

DEFINE_EVENT(lbf_ldisc, lbf_func_end,
	TP_PROTO(const char *func),
	TP_ARGS(func)
);
TRACE_EVENT(lbf_bt_fmr_st,

	TP_PROTO(const char *func, char cmd),

	TP_ARGS(func, cmd),

	TP_STRUCT__entry(
		__string(name, func)
		__field(char, cmd)
	),

	TP_fast_assign(
		__assign_str(name, func);
		__entry->cmd = cmd;
	),

	TP_printk(":%s bt_fmr_st:%s", __get_str(name),
		show_bt_fmr(__entry->cmd))
);

TRACE_EVENT(lbf_bt_rx_buf,

	TP_PROTO(const char *func, const u8 *buff, int len),

	TP_ARGS(func, buff, len),

	TP_STRUCT__entry(
		__string(name, func)
		__dynamic_array(u8, data, len)
		__field(int, len)
	),

	TP_fast_assign(
		__assign_str(name, func);
		memcpy(__get_dynamic_array(data), buff, len);
		__entry->len = len;
	),

	TP_printk(":%s bt_rx:%s", __get_str(name),
		 __print_hex(__get_dynamic_array(data), __entry->len))
);

TRACE_EVENT(lbf_bt_tx_buf,

	TP_PROTO(const char *func, const u8 *buff, int len),

	TP_ARGS(func, buff, len),

	TP_STRUCT__entry(
		__string(name, func)
		__dynamic_array(u8, data, len)
		__field(int, len)
	),

	TP_fast_assign(
		__assign_str(name, func);
		memcpy(__get_dynamic_array(data), buff, len);
		__entry->len = len;
	),

	TP_printk(":%s bt_tx:%s", __get_str(name),
		 __print_hex(__get_dynamic_array(data), __entry->len))
);

TRACE_EVENT(lbf_hostwake,

	TP_PROTO(const char *func, char cmd),

	TP_ARGS(func, cmd),

	TP_STRUCT__entry(
		__string(name, func)
		__field(char, cmd)
	),

	TP_fast_assign(
		__assign_str(name, func);
		__entry->cmd = cmd;
	),

	TP_printk(":%s hostwake_st:%s", __get_str(name),
		show_hostwake(__entry->cmd))
);

TRACE_EVENT(lbf_device_st,

	TP_PROTO(const char *func, char cmd),

	TP_ARGS(func, cmd),

	TP_STRUCT__entry(
		__string(name, func)
		__field(char, cmd)
	),

	TP_fast_assign(
		__assign_str(name, func);
		__entry->cmd = cmd;
	),

	TP_printk(":%s device_st:%s", __get_str(name),
		show_device_name(__entry->cmd))
);

DECLARE_EVENT_CLASS(lbf_ldisc_end,

	TP_PROTO(const char *func, unsigned int err),

	TP_ARGS(func, err),

	TP_STRUCT__entry(
		__string(name, func)
		__field(unsigned int, err)
	),

	TP_fast_assign(
		__assign_str(name, func);
		__entry->err = err;
	),

	TP_printk("<-%s ret=%d",
		__get_str(name), __entry->err)
);

DEFINE_EVENT(lbf_ldisc_end, lbf_ldisc_call,
	TP_PROTO(const char *func, unsigned err),
	TP_ARGS(func, err)
);

DEFINE_EVENT(lbf_ldisc_end, lbf_ldisc_poll,
	TP_PROTO(const char *func, unsigned err),
	TP_ARGS(func, err)
);

DEFINE_EVENT(lbf_ldisc_end, lbf_ldisc_read,
	TP_PROTO(const char *func, unsigned err),
	TP_ARGS(func, err)
);

DEFINE_EVENT(lbf_ldisc_end, lbf_ldisc_receive,
	TP_PROTO(const char *func, unsigned err),
	TP_ARGS(func, err)
);

DEFINE_EVENT(lbf_ldisc_end, lbf_ldisc_room,
	TP_PROTO(const char *func, unsigned err),
	TP_ARGS(func, err)
);

DEFINE_EVENT(lbf_ldisc_end, lbf_ldisc_write,
	TP_PROTO(const char *func, unsigned err),
	TP_ARGS(func, err)
);

DEFINE_EVENT(lbf_ldisc_end, lbf_ldisc_pm,
	TP_PROTO(const char *func, unsigned err),
	TP_ARGS(func, err)
);

DEFINE_EVENT(lbf_ldisc_end, lbf_ldisc_sendpm,
	TP_PROTO(const char *func, unsigned err),
	TP_ARGS(func, err)
);

DEFINE_EVENT(lbf_ldisc_end, lbf_usif_dev,
	TP_PROTO(const char *func, unsigned err),
	TP_ARGS(func, err)
);

DEFINE_EVENT(lbf_ldisc_end, lbf_irq_state,
	TP_PROTO(const char *func, unsigned err),
	TP_ARGS(func, err)
);


DECLARE_EVENT_CLASS(lbf_func_info,

	TP_PROTO(const char *func, char *err),

	TP_ARGS(func, err),

	TP_STRUCT__entry(
		__string(name, func)
		__string(ret, err)
	),

	TP_fast_assign(
		__assign_str(name, func);
		__assign_str(ret, err);
	),

	TP_printk("<-%s: %s",
		__get_str(name), __get_str(ret))
);

DEFINE_EVENT(lbf_func_info, lbf_ldisc_kpi,
	TP_PROTO(const char *func, char *err),
	TP_ARGS(func, err)
);

DEFINE_EVENT(lbf_func_info, lbf_ldisc_ioctl,
	TP_PROTO(const char *func, char *err),
	TP_ARGS(func, err)
);

DEFINE_EVENT(lbf_func_info, lbf_ldisc_pmprocessing,
	TP_PROTO(const char *func, char *err),
	TP_ARGS(func, err)
);

DEFINE_EVENT(lbf_func_info, lbf_ldisc_reset,
	TP_PROTO(const char *func, char *err),
	TP_ARGS(func, err)
);
DEFINE_EVENT(lbf_func_info, lbf_ldisc_info,
	TP_PROTO(const char *func, char *err),
	TP_ARGS(func, err)
);



#endif /* if !defined(_TRACE_LBF_H) || defined(TRACE_HEADER_MULTI_READ) */

/* This part must be outside protection */
#include <trace/define_trace.h>
