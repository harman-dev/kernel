#ifndef _ASM_X86_INTEL_PUNIT_IPC_H_
#define  _ASM_X86_INTEL_PUNIT_IPC_H_

#include <linux/notifier.h>
#include <asm/intel-mid.h>

/*Commands supported on GLM core,
* are handled by different bars, unify
* them together here
*/
#define IPC_BIOS_PUNIT_CMD_BASE				0x00
#define IPC_GTD_PUNIT_CMD_BASE				0x20
#define IPC_ISPD_PUNIT_CMD_BASE				0x40

/*BIOS => Pcode commands*/
#define IPC_BIOS_PUNIT_CMD_ZERO			(IPC_BIOS_PUNIT_CMD_BASE + 0x00)
#define IPC_BIOS_PUNIT_CMD_VR_INTERFACE		(IPC_BIOS_PUNIT_CMD_BASE + 0x01)
#define IPC_BIOS_PUNIT_CMD_READ_PCS		(IPC_BIOS_PUNIT_CMD_BASE + 0x02)
#define IPC_BIOS_PUNIT_CMD_WRITE_PCS		(IPC_BIOS_PUNIT_CMD_BASE + 0x03)
#define IPC_BIOS_PUNIT_CMD_READ_PCU_CONFIG	(IPC_BIOS_PUNIT_CMD_BASE + 0x04)
#define IPC_BIOS_PUNIT_CMD_WRITE_PCU_CONFIG	(IPC_BIOS_PUNIT_CMD_BASE + 0x05)
#define IPC_BIOS_PUNIT_CMD_READ_PL1_SETTING	(IPC_BIOS_PUNIT_CMD_BASE + 0x06)
#define IPC_BIOS_PUNIT_CMD_WRITE_PL1_SETTING	(IPC_BIOS_PUNIT_CMD_BASE + 0x07)
#define IPC_BIOS_PUNIT_CMD_TRIGGER_VDD_RAM	(IPC_BIOS_PUNIT_CMD_BASE + 0x08)
#define IPC_BIOS_PUNIT_CMD_READ_TELE_INFO	(IPC_BIOS_PUNIT_CMD_BASE + 0x09)
#define IPC_BIOS_PUNIT_CMD_READ_TELE_TRACE_CTRL	(IPC_BIOS_PUNIT_CMD_BASE + 0x0a)
#define IPC_BIOS_PUNIT_CMD_WRITE_TELE_TRACE_CTRL \
		(IPC_BIOS_PUNIT_CMD_BASE + 0x0b)
#define IPC_BIOS_PUNIT_CMD_READ_TELE_EVENT_CTRL	(IPC_BIOS_PUNIT_CMD_BASE + 0x0c)
#define IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT_CTRL \
		(IPC_BIOS_PUNIT_CMD_BASE + 0x0d)
#define IPC_BIOS_PUNIT_CMD_READ_TELE_TRACE	(IPC_BIOS_PUNIT_CMD_BASE + 0x0e)
#define IPC_BIOS_PUNIT_CMD_WRITE_TELE_TRACE	(IPC_BIOS_PUNIT_CMD_BASE + 0x0f)
#define IPC_BIOS_PUNIT_CMD_READ_TELE_EVENT	(IPC_BIOS_PUNIT_CMD_BASE + 0x10)
#define IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT	(IPC_BIOS_PUNIT_CMD_BASE + 0x11)
#define IPC_BIOS_PUNIT_CMD_READ_MODULE_TEMP	(IPC_BIOS_PUNIT_CMD_BASE + 0x12)
#define IPC_BIOS_PUNIT_CMD_RESERVED		(IPC_BIOS_PUNIT_CMD_BASE + 0x13)
#define IPC_BIOS_PUNIT_CMD_READ_VOLTAGE_OVER	(IPC_BIOS_PUNIT_CMD_BASE + 0x14)
#define IPC_BIOS_PUNIT_CMD_WRITE_VOLTAGE_OVER	(IPC_BIOS_PUNIT_CMD_BASE + 0x15)
#define IPC_BIOS_PUNIT_CMD_READ_RATIO_OVER	(IPC_BIOS_PUNIT_CMD_BASE + 0x16)
#define IPC_BIOS_PUNIT_CMD_WRITE_RATIO_OVER	(IPC_BIOS_PUNIT_CMD_BASE + 0x17)
#define IPC_BIOS_PUNIT_CMD_READ_VF_GL_CTRL	(IPC_BIOS_PUNIT_CMD_BASE + 0x18)
#define IPC_BIOS_PUNIT_CMD_WRITE_VF_GL_CTRL	(IPC_BIOS_PUNIT_CMD_BASE + 0x19)
#define IPC_BIOS_PUNIT_CMD_READ_FM_SOC_TEMP_THRESH \
		(IPC_BIOS_PUNIT_CMD_BASE + 0x1a)
#define IPC_BIOS_PUNIT_CMD_WRITE_FM_SOC_TEMP_THRESH \
		(IPC_BIOS_PUNIT_CMD_BASE + 0x1b)

/*GT Driver => Pcode commands*/
#define IPC_GTD_PUNIT_CMD_ZERO			(IPC_GTD_PUNIT_CMD_BASE + 0x00)
#define IPC_GTD_PUNIT_CMD_CONFIG		(IPC_GTD_PUNIT_CMD_BASE + 0x01)
#define IPC_GTD_PUNIT_CMD_READ_ICCP_LIC_CDYN_SCAL \
		(IPC_GTD_PUNIT_CMD_BASE + 0x02)
#define IPC_GTD_PUNIT_CMD_WRITE_ICCP_LIC_CDYN_SCAL \
		(IPC_GTD_PUNIT_CMD_BASE + 0x03)
#define IPC_GTD_PUNIT_CMD_GET_WM_VAL		(IPC_GTD_PUNIT_CMD_BASE + 0x06)
#define IPC_GTD_PUNIT_CMD_WRITE_CONFIG_WISHREQ	(IPC_GTD_PUNIT_CMD_BASE + 0x07)
#define IPC_GTD_PUNIT_CMD_READ_REQ_DUTY_CYCLE	(IPC_GTD_PUNIT_CMD_BASE + 0x16)
#define IPC_GTD_PUNIT_CMD_DIS_VOL_FREQ_CHANGE_REQUEST \
		(IPC_GTD_PUNIT_CMD_BASE + 0x17)
#define IPC_GTD_PUNIT_CMD_DYNA_DUTY_CYCLE_CTRL	(IPC_GTD_PUNIT_CMD_BASE + 0x1a)
#define IPC_GTD_PUNIT_CMD_DYNA_DUTY_CYCLE_TUNING \
		(IPC_GTD_PUNIT_CMD_BASE + 0x1c)

/*ISP Driver => Pcode commands*/
#define IPC_ISPD_PUNIT_CMD_ZERO			(IPC_ISPD_PUNIT_CMD_BASE + 0x00)
#define IPC_ISPD_PUNIT_CMD_CONFIG		(IPC_ISPD_PUNIT_CMD_BASE + 0x01)
#define IPC_ISPD_PUNIT_CMD_GET_ISP_LTR_VAL	(IPC_ISPD_PUNIT_CMD_BASE + 0x02)
#define IPC_ISPD_PUNIT_CMD_ACCESS_IU_FREQ_BOUNDS \
		(IPC_ISPD_PUNIT_CMD_BASE + 0x03)
#define IPC_ISPD_PUNIT_CMD_READ_CDYN_LEVEL	(IPC_ISPD_PUNIT_CMD_BASE + 0x04)
#define IPC_ISPD_PUNIT_CMD_WRITE_CDYN_LEVEL	(IPC_ISPD_PUNIT_CMD_BASE + 0x05)

/*Error codes*/
#define IPC_ERR_SUCCESS				0
#define IPC_ERR_INVALID_CMD			1
#define IPC_ERR_INVALID_PARAMETER		2
#define IPC_ERR_CMD_TIMEOUT			3
#define IPC_ERR_CMD_LOCKED			4
#define IPC_ERR_INVALID_VR_ID			5
#define IPC_ERR_VR_ERR				6

/*See pcode sw spec for detail
* @cmd: 8bit, see above unified macros
* @para1, para2: 8bit, optional parameters for this cmd
* @in: optional input data, 32bit for BIOS cmd, 2 32bit for GTD and ISPD
* @out: optional output data, similar to in
*/
int intel_punit_ipc_simple_command(int cmd, int para1, int para2);
int intel_punit_ipc_command(u32 cmd, u32 para1, u32 para2, u32 *in, u32 *out);

#endif