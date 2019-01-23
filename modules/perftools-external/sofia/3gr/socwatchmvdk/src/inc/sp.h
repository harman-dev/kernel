#ifndef _SP_H_
#define _SP_H_ 1

#include "sp_defines.h"
#include "pw_version.h"
#include "sofia/mv_svc_hypercalls.h"

#define STM_base_3G 0xE4300000
#define STM_size 0X40
#define STM_TIM0_offset 0x20
#define STM_CAP_offset 0x32

// define this flag to have IDT entry programmed for SoCWatch IRQ handler
#define SOCWATCH_IDT_IRQ 1

extern void SYS_Perfvec_Handler (void);
extern short SYS_Get_cs (void);

#if defined(SWDRV_IA32) && (SOCWATCH_IDT_IRQ)
extern void *SYS_Get_IDT_Base_HWR(void);   /// IDT base from hardware IDTR

#define SYS_Get_IDT_Base SYS_Get_IDT_Base_HWR
#endif // defined(SWDRV_IA32) && (SOCWATCH_IDT_IRQ)

#if defined(SWDRV_EM64T) && (SOCWATCH_IDT_IRQ)
extern void SYS_Get_IDT_Base (void **);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
typedef struct gate_struct gate_struct_t;
#else
typedef struct gate_struct64 gate_struct_t;
#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
#endif // defined(SWDRV_EM64T) && (SOCWATCH_IDT_IRQ)

// miscellaneous defines
#define CPU() (raw_smp_processor_id())
#define GET_BOOL_STRING(b) ( (b) ? "TRUE" : "FALSE" )

#define _STRINGIFY(x)     #x
#define STRINGIFY(x)      _STRINGIFY(x)
#define _STRINGIFY_W(x)   L#x
#define STRINGIFY_W(x)    _STRINGIFY_W(x)

/*
 * 64bit Compare-and-swap.
 */
#define CAS64(p, o, n) cmpxchg64((p), (o), (n)) == (o)

typedef struct PWCollector_msg PWCollector_msg_t;

#define SOCWATCH_MSG_PREFIX "socwatch"

#if defined(DO_DEBUG)
#define PW_PRINT_DEBUG(fmt,args...) { printk(KERN_INFO SOCWATCH_MSG_PREFIX" [DEBUG] " fmt,##args); }
#else
#define PW_PRINT_DEBUG(fmt,args...) {;}
#endif

#define PW_PRINT(fmt,args...) { printk(KERN_INFO SOCWATCH_MSG_PREFIX": " fmt,##args); }

#define PW_PRINT_WARNING(fmt,args...) { printk(KERN_ALERT SOCWATCH_MSG_PREFIX" [Warning] " fmt,##args); }

#define PW_PRINT_ERROR(fmt,args...) { printk(KERN_CRIT SOCWATCH_MSG_PREFIX" [ERROR] " fmt,##args); }

#endif // _SP_H_
