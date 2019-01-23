/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2014 - 2015 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  SoC Watch Developer Team <socwatchdevelopers@intel.com>
  Intel Corporation,
  1906 Fox Drive,
  Champaign, IL 61820

  BSD LICENSE

  Copyright(c) 2014 - 2015 Intel Corporation.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#define MOD_AUTHOR "Robert Knight"
#define MOD_DESC "prototype delivery of sofia VMM buffers"

#include "sp_defines.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <asm/io.h>
#if defined(SWDRV_EM64T)
#include <asm/desc.h>
#endif
#include "sp.h"
#include "sp_vmm_buffer.h"
#include "spioctl.h"
#include "control.h"
/*
 * There is a build error because a typedef is being used in vmm_platform_service.h
 * SoCWatch doesn't use the typedef.
 * But adding this header as a work-around to prevent a build error of the socwatch driver
 * because of the typedef mentioned above.
 */
#include "sofia/socwatch/mv_svc_socwatch.h"
#include "sofia/vtimer/mv_svc_vtimer.h"
#include "sofia/sysprof/mv_svc_sysprof.h"
#include "sofia/mv_hypercalls.h"

/* *******************************************
 * Compile-time constants
 * *******************************************
 */
#define SOCWATCH_IRQ 120

/* *******************************************
 * Local data structures.
 * *******************************************
 */
#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
#include <linux/compat.h>
/*
 * Helper struct used to translate IOCTLs
 * from 32b user programs in 64b kernels.
 */
struct spdrv_ioctl_arg32 {
    pw_s32_t in_len;
    pw_s32_t out_len;
    compat_caddr_t in_arg;
    compat_caddr_t out_arg;
};
#endif // COMPAT && x64

/* *******************************************
 * Variables.
 * *******************************************
 */

/* Per-CPU variable containing the currently running vcpu. */
//static DEFINE_PER_CPU(int, curr_vcpu) = 0;

static int sp_dev_major_num = -1;
static dev_t sp_dev;
static struct cdev *sp_cdev;
static struct class *sp_class = NULL;

int num_CPUs = 0;
bool flush_now;
unsigned long STM_base = 0;
void *pSTM = NULL;
unsigned int socwatch_irq = 0;

extern struct socwatch_buffer_info* pPhysBufferInfo;
extern struct socwatch_buffer_info* pBufferInfo;
wait_queue_head_t read_queue;

// static int enable_tracepoints = 0;

#if SOCWATCH_IDT_IRQ

#if defined(SWDRV_IA32)
unsigned long long idt_entry_saved_value[4];

typedef union {
    unsigned long long  u64[1];
    unsigned short int  u16[4];
} local_handler_t;
#endif //SWDRV_IA32

#if defined(SWDRV_EM64T)
pvoid idt_entry_saved_value[4];

#pragma pack(push,1)
typedef struct _idtgdtDesc {
    u16    idtgdt_limit;
    pvoid  idtgdt_base;
} idt_desc_t;
#pragma pack(pop)
#endif //SWDRV_EM64T
#endif //SOCWATCH_IDT_IRQ

#if defined(DO_DEBUG)
static atomic_t num_interrupts = ATOMIC_INIT(0);
static atomic_t vmm_buff_count = ATOMIC_INIT(0);
#endif

/* *******************************************
 * Function definitions.
 * *******************************************
 */
// mapSTM returns 0 if it worked, non-zero if it failed
int mapSTM(void)
{
    int ret = 0;
    pSTM = ioremap(STM_base, STM_size);
    if (pSTM == NULL)
        return 1;
    return ret;
};

void unmapSTM(void)
{
    iounmap((void*)pSTM);
};

// STM clock is read only in the case of 3G or 3GR devices
static unsigned long long readSTM(void)
{
    unsigned int lower32, lower32_2, upper24;
    void *addressl = pSTM + STM_TIM0_offset;
    void *addressh = pSTM + STM_CAP_offset;

    lower32 = ioread32(addressl);
    upper24 = ioread32(addressh);
    lower32_2 = ioread32(addressl);
    upper24 &= 0x0000ffff;

    if (lower32 <= lower32_2)
        return (((unsigned long long) upper24 << 32) | lower32);
    else {
        // handle overflow
        upper24 = ioread32(addressh);
        upper24 &= 0x0000ffff;
        return (((unsigned long long) upper24 << 32) | lower32_2);
    }
};

static unsigned long long readTSC(void)
{
    unsigned long long tsc = 0;
    rdtscll(tsc);
    return tsc;
}

static u32 timestampCounterFreqHz(void)
{
    return mv_svc_timestamp_counter_frequency();
};

// The device_read condition is checked upon executing the wait_event_interruptable function
// 3 situations exist: a) only  vmm buffer is ready, b) only a linux buffer is ready, c) both a
//  vmm buffer and linux buffer are ready
//  In a), only the vmm_buf_rdy variable is set, that branch is taken, and a single buffer is returned
//  In b), only the linux_buf_rdy variable is set, that branch is taken, and a single buffer is returned
//  In c), which can occur during the collection and probably will occur at the end of a collection as soon
//   as the flush_now variable is set, the first buffer found is returned to user mode following the same
//   process as a) or b). Usermode will immediatly call read again since read returned a non-zero result,
//   and the wait_event_interruptable condition will be checked again and find another buffer read.
//   This loop will continue until no buffers are found to have data which will cause this function
//   to return 0. Usermode code should end the user mode loop when 0 is returned.
//   To force the end of collection return of partially filled buffers, call wake_up_interruptible(&read_queue)
//   on collection stop.
// Deallocation of ring0 buffers during the end of collection flushing of data to user mode is avoided
//  by allocating and deallocating all buffers during driver load and unload
ssize_t device_read(struct file *file,	/* see include/linux/fs.h   */
        char __user * buffer,	/* buffer to be
                                 * filled with data */
        size_t length,	/* length of the buffer     */
        loff_t * offset)
{
    unsigned int to_copy;
    unsigned long uncopied;
    uint64_t cur_buf;
    int cpu;
    bool vmm_buf_rdy=false;

    if (flush_now) {
        vmm_buf_rdy = vmm_is_buffer_ready(&cpu);
        PW_PRINT_DEBUG("device_read()- flush_now=%d, vmm_buf_rdy=%d\n", flush_now, vmm_buf_rdy);
    }
    else {
        if (wait_event_interruptible(read_queue, (vmm_buf_rdy=vmm_is_buffer_ready(&cpu)))) {
            PW_PRINT_ERROR("device_read() wait_event_interruptible failed\n");
            return -ERESTARTSYS;
        }
    }

    if (vmm_buf_rdy) {
#if defined(DO_DEBUG)
        atomic_inc(&vmm_buff_count);
#endif
        cur_buf = (uint64_t)(unsigned long)phys_to_virt((unsigned long)pBufferInfo->buffer_delivered[cpu]);
        to_copy = pBufferInfo->buffer_data_size[cpu];

        if (flush_now) {
            PW_PRINT_DEBUG("device_read-received vmm buffer for core %d at %p, with %d bytes\n", cpu, (void *)(unsigned long)cur_buf, to_copy);
        }

        if (to_copy > length) {
            PW_PRINT_ERROR("user buffer is too small\n");
            return -1;
        }

        uncopied = copy_to_user(buffer, (void *)(unsigned long)cur_buf, to_copy);
        *offset += to_copy-uncopied;

        if (uncopied) {
            PW_PRINT_ERROR("device_read() failed to copy %ld bytes\n", uncopied);
            // copy_to_user returns an unsigned
            return -1;
        }
        else {
            // Mark the buffer empty
            // TODO Investigate whether we need to add a memory barrier here
            pBufferInfo->buffer_data_size[cpu] = 0;
            pBufferInfo->buffer_status[cpu] = SOCWATCH_BUFFER_CONSUMED;
        }

        // PW_PRINT_DEBUG("device_read() copied %d bytes to usermode\n", to_copy);
        return to_copy;
    }

    // nothing  to copy, return 0
    return 0;
};

#if SOCWATCH_IDT_IRQ
asmlinkage void socwatch_irq_handler( struct pt_regs *regs )
{
#if defined(DO_DEBUG)
    atomic_inc(&num_interrupts);
#endif

    wake_up_interruptible(&read_queue);
}
#else // SOCWATCH_IDT_IRQ
static irqreturn_t socwatch_irq_handler(int irq, void *dev_id)
{
#if defined(DO_DEBUG)
    atomic_inc(&num_interrupts);
#endif

    wake_up_interruptible(&read_queue);

    return IRQ_HANDLED;
}
#endif // SOCWATCH_IDT_IRQ


static long spdrv_Configure(u32 __user* remote_config_bitmap)
{
    u32 local_config_bitmap = 0;
    if (get_user(local_config_bitmap, remote_config_bitmap)) {
        PW_PRINT_ERROR("ERROR: couldn't copy in remote args!\n");
        return -1;
    }
    printk(KERN_INFO "OK, local bitmap = %u\n", local_config_bitmap);
    flush_now = false;
    vmm_reset_buffers();

    mv_svc_socwatch_config(local_config_bitmap, ((struct socwatch_buffer_info *)pPhysBufferInfo));

    return 0;
}

static long spdrv_Stop(void)
{
    u32 control = 0;

    PW_PRINT_DEBUG("'mv_svc_socwatch...' stop call on (virtual?) core %d\n", raw_smp_processor_id());
    CONTROL_Invoke_Parallel((pvoid)mv_svc_socwatch_run_control, (pvoid)(size_t)control);

    // set the flag that indictes partially filled linux buffers should be
    //  returned to user mode via the device_read function

    flush_now = true;

    // force the device_read function to check if any buffers are partially filled with data
    wake_up_interruptible(&read_queue);
    // print_tracepoint_activity();

#if defined(DO_DEBUG)
    PW_PRINT_DEBUG("total injected interrupts %d\n", atomic_read(&num_interrupts));
    PW_PRINT_DEBUG("total buffers read by socwatch %d\n", atomic_read(&vmm_buff_count));

    atomic_set(&num_interrupts, 0);
    atomic_set(&vmm_buff_count, 0);
#endif

#if defined(SOCWATCH_LOGGING_DEBUG)
    {
        int i;

        for (i = 0; i < num_CPUs; i++) {
            PW_PRINT_DEBUG("Core %d: Logged = %d, Dropped = %d, Full Buffers generated = %d, First TSC = %lld, Last TSC= %lld\n",
                    i, pBufferInfo->samples_logged[i], pBufferInfo->samples_dropped[i], pBufferInfo->buffers_generated[i],
                    pBufferInfo->first_tsc[i], pBufferInfo->last_tsc[i]);
        }
    }
#endif
    return 0;
}

static long spdrv_Start(void)
{
    uint32_t control = 1;

    PW_PRINT_DEBUG("'mv_svc_socwatch...' start call on (virtual) core %d\n", raw_smp_processor_id());
    CONTROL_Invoke_Parallel((pvoid)mv_svc_socwatch_run_control, (pvoid)(size_t)control);

    return 0;
}

static long spdrv_handle_cmd(u32 __user* remote_cmd)
{
    u32 local_cmd = 0;
    long status = 0;

    if (get_user(local_cmd, remote_cmd)) {
        PW_PRINT_ERROR("ERROR: couldn't copy in remote command!\n");
        return -1;
    }
    switch (local_cmd) {
        case SPDRV_CMD_START:
            PW_PRINT_DEBUG("RECEIVED CMD START!\n");
            status = spdrv_Start();
            break;
        case SPDRV_CMD_STOP:
            PW_PRINT_DEBUG("RECEIVED CMD STOP!\n");
            status = spdrv_Stop();
            break;
        default:
            PW_PRINT_ERROR("ERROR: invalid command %d passed to the SoFIA driver!\n", local_cmd);
            status = -1;
            break;
    }
    return status;
};

long spdrv_GetVersion(u64 __user* remote_args)
{
    u64 local_version = (u64)SPDRV_VERSION_MAJOR << 32 |
                        (u64)SPDRV_VERSION_MINOR << 16 | 
                        (u64)SPDRV_VERSION_OTHER;

    return put_user(local_version, remote_args);
};

long spdrv_GetClock(u32 __user* remote_in_args, u64 __user* remote_args)
{
    u32 isLTE = 0;
    u64 clocks[] = {0, 0};
    if (get_user(isLTE, remote_in_args)) {
        PW_PRINT_ERROR("ERROR: couldn't copy in remote args for SPRDV_OPERATION_CLOCK!\n");
        return -1;
    }

    if (isLTE) {
        // Read TSC if the device is LTE
        clocks[0] = readTSC();
        clocks[1] = timestampCounterFreqHz();
    } else {
        // Read STM if the device is 3G/3GR
        clocks[0] = readSTM();
        clocks[1] = timestampCounterFreqHz();
    }
    return copy_to_user(remote_args, clocks, sizeof(clocks));
};

long spdrv_GetTopology(struct sysprof_vcore_info* remote_args)
{
    int i=0;
    size_t info_size = sizeof(struct sysprof_vcore_info) * SYSPROF_MAX_VCORES;
    int ret = 0;
    struct sysprof_vcore_info *infos = kmalloc(info_size, GFP_KERNEL);
    if (!infos) {
        printk(KERN_ERR "ERROR: couldn't allocate space for VCPU mapping!\n");
        return -1;
    }
#if DO_VCPU_PCPU_MAPPING
    mv_svc_sysprof_get_vcore_map((struct sysprof_vcore_info *)virt_to_phys(infos));
#else // DO_VCPU_PCPU_MAPPING
    memset(infos, (u8)-1, info_size);
#endif // DO_VCPU_PCPU_MAPPING
    for (i = 0; i < 16; i++) {
        PW_PRINT("Element %d: PCPU=%d Context=%d\n", i, infos[i].pcore, infos[i].context);
    }
    if (copy_to_user(remote_args, infos, info_size)) {
        printk(KERN_ERR "ERROR: unable to copy VCPU mapping to userspace!\n");
        ret = -1;
    }
    kfree(infos);
    return ret;
};

static long handle_ioctl(unsigned int num, struct spdrv_ioctl_arg __user* remote_args)
{
    long status = 0;
    struct spdrv_ioctl_arg local_args;

    if (copy_from_user(&local_args, remote_args, sizeof(local_args))) {
        PW_PRINT_ERROR("ERROR: couldn't copy in remote args!\n");
        return -1;
    }

    switch (num) {
        case SPDRV_OPERATION_CMD:
            status = spdrv_handle_cmd((u32 __user *)local_args.in_arg);
            break;

        case SPDRV_OPERATION_CONFIGURE:
            status = spdrv_Configure((u32 __user *)local_args.in_arg);
            break;

        case SPDRV_OPERATION_VERSION:
            status = spdrv_GetVersion((u64 __user*)local_args.out_arg);
            break;

        case SPDRV_OPERATION_CLOCK:
            status = spdrv_GetClock((u32 __user *)local_args.in_arg, (u64 __user*)local_args.out_arg);
            break;

        case SPDRV_OPERATION_TOPOLOGY:
            status = spdrv_GetTopology((struct sysprof_vcore_info __user*)local_args.out_arg);
            break;
    }
    return status;
}

static long device_unlocked_ioctl(struct file *filep, unsigned int ioctl_num, unsigned long ioctl_param)
{
    return handle_ioctl(_IOC_NR(ioctl_num), (struct spdrv_ioctl_arg __user* )ioctl_param);
};


#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
long device_compat_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
    struct spdrv_ioctl_arg32 __user* remote_args32 = compat_ptr(ioctl_param);
    struct spdrv_ioctl_arg __user* remote_args = compat_alloc_user_space(sizeof(*remote_args));
    int tmp;
    u32 data;

    if (!remote_args) {
        return -1;
    }
    if (get_user(tmp, &remote_args32->in_len) || put_user(tmp, &remote_args->in_len)) {
        return -1;
    }
    if (get_user(tmp, &remote_args32->out_len) || put_user(tmp, &remote_args->out_len)) {
        return -1;
    }
    if (get_user(data, &remote_args32->in_arg) || put_user(compat_ptr(data), &remote_args->in_arg)) {
        return -1;
    }
    if (get_user(data, &remote_args32->out_arg) || put_user(compat_ptr(data), &remote_args->out_arg)) {
        return -1;
    }
    return handle_ioctl(_IOC_NR(ioctl_num), remote_args);
};
#endif // COMPAT && x64


static int device_open(struct inode *inode, struct file *file)
{
    // PW_PRINT_DEBUG(" driver has been opened\n");

    flush_now = false;
    vmm_reset_buffers();

    return 0;
}

struct file_operations Fops = {
    .open = &device_open,
    .read = &device_read,
    .unlocked_ioctl = &device_unlocked_ioctl,
#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
    .compat_ioctl = &device_compat_ioctl,
#endif // COMPAT && x64
};

#if SOCWATCH_IDT_IRQ

#if defined(SWDRV_IA32)
static void enable_socwatch_idt_interrupt(pvoid param)
{
    unsigned long         eflags;
    unsigned long long    *idt_base;
    local_handler_t lhandler;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)) || \
    (defined (DRV_CHROMEOS) && LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
    unsigned long         cr0_value;
#endif
    int cpu;

    preempt_disable();
    cpu = CONTROL_THIS_CPU();
    preempt_enable();

    local_irq_save(eflags);

    idt_base = SYS_Get_IDT_Base();

    /*
     * save the original value in the IDT entry
     */
    //TODO spin lock here, as this is a shared data structure
    idt_entry_saved_value[cpu] = idt_base[socwatch_irq];

    /*
     * install socwatch handler
     * These are the necessary steps to have an ISR entry
     * Note the changes in the data written
     */
    lhandler.u64[0] = (unsigned long)SYS_Perfvec_Handler;
    lhandler.u16[3] = lhandler.u16[1];
    lhandler.u16[1] = SYS_Get_cs();
    lhandler.u16[2] = 0xee00;

    /*
     * From 3.10 kernel, the IDT memory has been moved to a read-only location
     * which is controlled by the bit 16 in the CR0 register.
     * The write protection should be temporarily released to update the IDT.
     */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)) || \
    (defined (DRV_CHROMEOS) && LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
    cr0_value = read_cr0();
    write_cr0(cr0_value & ~X86_CR0_WP);
#endif

    idt_base[socwatch_irq] = lhandler.u64[0];

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)) || \
    (defined (DRV_CHROMEOS) && LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
    write_cr0(cr0_value);
#endif

    // Request the socwatch VIRQ from the VMM
    mv_virq_request(socwatch_irq, 1);

    // Enable(?) the socwatch VIRQ in the VMM
    mv_virq_unmask(socwatch_irq);

    local_irq_restore(eflags);
    return;
}

static void disable_socwatch_idt_interrupt(pvoid param)
{
    unsigned long         eflags;
    unsigned long long    *idt_base;
    int cpu;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)) || \
    (defined (DRV_CHROMEOS) && LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
    unsigned long         cr0_value;
#endif

    preempt_disable();
    cpu = CONTROL_THIS_CPU();
    preempt_enable();

    local_irq_save(eflags);
    idt_base = SYS_Get_IDT_Base();

    /*
     * From 3.10 kernel, the IDT memory has been moved to a read-only location
     * which is controlled by the bit 16 in the CR0 register.
     * The write protection should be temporarily released to update the IDT.
     */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)) || \
    (defined (DRV_CHROMEOS) && LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
    cr0_value = read_cr0();
    write_cr0(cr0_value & ~X86_CR0_WP);
#endif

    //TODO spinlock
    idt_base[socwatch_irq] = idt_entry_saved_value[cpu];

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)) || \
    (defined (DRV_CHROMEOS) && LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
    write_cr0(cr0_value);
#endif

    local_irq_restore(eflags);
}
#endif // SWDRV_IA32

#if defined(SWDRV_EM64T)
static void set_idt_function(gate_struct_t *idt_base, pvoid func)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)

    _set_gate(&idt_base[socwatch_irq], GATE_INTERRUPT, (unsigned long) func, 3, 0);
#else

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)) || \
    (defined (DRV_CHROMEOS) && LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
    unsigned long         cr0_value;
#endif
    gate_struct_t  local;
    // _set_gate() cannot be used because the IDT table is not exported.

    pack_gate(&local, GATE_INTERRUPT, (unsigned long)func, 3, 0, __KERNEL_CS);

    /*
     * From 3.10 kernel, the IDT memory has been moved to a read-only location
     * which is controlled by the bit 16 in the CR0 register.
     * The write protection should be temporarily released to update the IDT.
     */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)) || \
    (defined (DRV_CHROMEOS) && LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
    cr0_value = read_cr0();
    write_cr0(cr0_value & ~X86_CR0_WP);
#endif

    write_idt_entry((idt_base), socwatch_irq, &local);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)) || \
    (defined (DRV_CHROMEOS) && LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
    write_cr0(cr0_value);
#endif
#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
}

static void enable_socwatch_idt_interrupt(pvoid param)
{
    unsigned long         eflags;
    idt_desc_t idt_desc;
    gate_struct_t old_gate;
    gate_struct_t *idt_base;

    int cpu;

    preempt_disable();
    cpu = CONTROL_THIS_CPU();
    preempt_enable();

    local_irq_save(eflags);

    SYS_Get_IDT_Base((pvoid*)&idt_desc);
    idt_base = idt_desc.idtgdt_base;

    memcpy (&old_gate, &idt_base[socwatch_irq], 16);
    /*
     * save the original value in the IDT entry
     */
    //TODO spin lock here, as this is a shared data structure
    idt_entry_saved_value[cpu] = (pvoid) ((((u64) old_gate.offset_high) << 32)   |
                                         (((u64) old_gate.offset_middle) << 16) |
                                          ((u64) old_gate.offset_low));

    PW_PRINT_DEBUG("saved_ih is 0x%llx\n", idt_entry_saved_value[cpu]);

    /*
     * install socwatch handler
     */
    set_idt_function(idt_base, SYS_Perfvec_Handler);

    // Request the socwatch VIRQ from the VMM
    mv_virq_request(socwatch_irq, 1);

    // Enable(?) the socwatch VIRQ in the VMM
    mv_virq_unmask(socwatch_irq);

    local_irq_restore(eflags);
    return;
}

static void disable_socwatch_idt_interrupt(pvoid param)
{
    unsigned long         eflags;
    gate_struct_t *idt_base;
    idt_desc_t idt_desc;
    int cpu;

    preempt_disable();
    cpu = CONTROL_THIS_CPU();
    preempt_enable();

    local_irq_save(eflags);

    SYS_Get_IDT_Base((pvoid*)&idt_desc);
    idt_base = idt_desc.idtgdt_base;

    set_idt_function(idt_base, idt_entry_saved_value[cpu]);

    local_irq_restore(eflags);
}
#endif // SWDRV_EM64T

#endif // SOCWATCH_IDT_IRQ

static int __init initSP( void )
{
    int initSTM, error;
    struct device *dev;

    socwatch_irq = SOCWATCH_IRQ;
    STM_base = STM_base_3G;

    initSTM = mapSTM();
    if (initSTM) {
        PW_PRINT_ERROR("unable to ioremap the STM clock\n");
        error = -EIO;
        goto cleanup_return_error;
    }

    num_CPUs = num_possible_cpus();
    PW_PRINT_DEBUG("Number of possible CPUs: %d\n", num_CPUs);

    // init all of the data buffers
    error = vmm_init_buffers(num_CPUs);
    if (error) {
        PW_PRINT_ERROR("Error failed to allocate vmm buffers\n");
        goto cleanup_return_error;
    }

    // create the char device "sp"
    alloc_chrdev_region(&sp_dev, 0, 1, SP_DEVICE_NAME);
    sp_dev_major_num = MAJOR(sp_dev);
    sp_class = class_create(THIS_MODULE, SP_DEVICE_NAME);
    if(IS_ERR(sp_class)) {
        error = PTR_ERR(sp_class);
        PW_PRINT_ERROR("Error registering sp class\n");
        goto cleanup_return_error;
    }

    dev = device_create(sp_class, NULL, sp_dev, NULL, SP_DEVICE_NAME);
    if (dev == NULL) {
        error = PTR_ERR(dev);
        PW_PRINT_ERROR("Error during call to device_create\n");
        goto cleanup_return_error;
    }

    sp_cdev = cdev_alloc();
    if (sp_cdev == NULL) {
        error = -ENOMEM;
        PW_PRINT_ERROR("Error allocating character device\n");
        goto cleanup_return_error;
    }
    sp_cdev->owner = THIS_MODULE;
    sp_cdev->ops = &Fops;
    if( cdev_add(sp_cdev, sp_dev, 1) < 0 )  {
        error = -1;
        PW_PRINT_ERROR("Error registering device driver\n");
        goto cleanup_return_error;
    }

    // initialize a work queue to be used for signalling when data is ready to copy to usermode
    init_waitqueue_head(&read_queue);

#if SOCWATCH_IDT_IRQ
    PW_PRINT_DEBUG("request socwatch IDT irq on (virtual) core %d\n", raw_smp_processor_id());

    // enable_socwatch_idt_interrupt() is scheduled to run on all cores
    CONTROL_Invoke_Parallel(enable_socwatch_idt_interrupt, NULL);
#else // SOCWATCH_IDT_IRQ

    // register IRQ handler
    PW_PRINT_DEBUG("request socwatch irq on (virtual) core %d\n", raw_smp_processor_id());
    error = request_irq(socwatch_irq,
            socwatch_irq_handler,
            0,
            // IRQF_PERCPU|IRQF_NOBALANCING,
            "socwatch",
            NULL);

    if (error) {
        error = -EBUSY;
        PW_PRINT_ERROR("Error request_irq() failed during driver init\n");
        goto cleanup_return_error;
    }
#endif // SOCWATCH_IDT_IRQ

    return 0;

cleanup_return_error:
#if !defined(SOCWATCH_IDT_IRQ)
    // deregister irq
    free_irq(socwatch_irq, NULL);
#endif
    // release char device
    unregister_chrdev(sp_dev_major_num, SP_DEVICE_NAME);
    device_destroy(sp_class, sp_dev);
    class_destroy(sp_class);
    unregister_chrdev_region(sp_dev, 1);
    cdev_del(sp_cdev);

    vmm_destroy_buffers();
    unmapSTM();
    return error;
}

static void __exit exitSP( void )
{
    // deregister irq
#if SOCWATCH_IDT_IRQ
    // disable_socwatch_idt_interrupt() is scheduled to run on all cores
    CONTROL_Invoke_Parallel(disable_socwatch_idt_interrupt, NULL);
#else
    free_irq(socwatch_irq, NULL);
#endif // SOCWATCH_IDT_IRQ

    unmapSTM();

    vmm_destroy_buffers();
    // release char device
    unregister_chrdev(sp_dev_major_num, SP_DEVICE_NAME);
    device_destroy(sp_class, sp_dev);
    class_destroy(sp_class);
    unregister_chrdev_region(sp_dev, 1);
    cdev_del(sp_cdev);

    return;
}

module_init(initSP);
module_exit(exitSP);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(MOD_AUTHOR);
MODULE_DESCRIPTION(MOD_DESC);
