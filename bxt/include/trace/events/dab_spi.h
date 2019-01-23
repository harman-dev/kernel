/*
 * dab_spi.h
 *
 *  Created on: 11-Apr-2016
 *      Author: vikram
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM dab_spi

#if !defined(INCLUDE_TRACE_EVENTS_DAB_SPI_H_) || defined(TRACE_HEADER_MULTI_READ)
#define INCLUDE_TRACE_EVENTS_DAB_SPI_H_

#include <linux/ktime.h>
#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(dab,

	TP_PROTO(unsigned long time),

	TP_ARGS(time),

	TP_STRUCT__entry(
		__field(	unsigned long,           time	)
	),

	TP_fast_assign(
		__entry->time = jiffies;
	),

	TP_printk("dab_spi: time=%lu", __entry->time)

);

DEFINE_EVENT(dab, dab_spi_irq,

	TP_PROTO(unsigned long time),

	TP_ARGS(time)

);

DEFINE_EVENT(dab, dab_spi_work_start,

	TP_PROTO(unsigned long time),

	TP_ARGS(time)

);

DEFINE_EVENT(dab, dab_spi_work_end,

	TP_PROTO(unsigned long time),

	TP_ARGS(time)

);

DEFINE_EVENT(dab, dab_spi_xchange,

	TP_PROTO(unsigned long time),

	TP_ARGS(time)

);


#endif /* INCLUDE_TRACE_EVENTS_DAB_SPI_H_ */

/* This part must be outside protection */
#include <trace/define_trace.h>
