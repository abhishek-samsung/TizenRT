/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2013 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */
#include "ameba_soc.h"

typedef struct {
	u32 CTRL;
	u32 MAIR0;
	u32 MAIR1;
	u32 RBAR[MPU_MAX_REGION];
	u32 RLAR[MPU_MAX_REGION];
} MPU_BackUp_TypeDef;

typedef struct {
	u32 CPURegbackup_HP[25];
	u32 CPUPSP_HP;
	u32 PortSVC_Backup_HP;
	u32 NVICbackup_HP[9];
	u32 SYSTICKbackup_HP[4];
	u32 SCBbackup_HP[4];
	u32 IPCbackup_HP;
	u32 BASEPRI_backup_HP;
	u32 PRIMASK_backup_HP;
	u8 NVICIPbackup_HP[MAX_PERIPHERAL_IRQ_NUM];
	MPU_BackUp_TypeDef MPU_BK;
} CPU_BackUp_TypeDef;

CPU_BackUp_TypeDef PMC_BK;
u32 WakeEventFlag_KM4 = _FALSE;
extern SLEEP_ParamDef sleep_param;
VOID SOCPS_WakeFromPG_KM4(VOID);

VOID SOCPS_NVICBackup_HP(void)
{
	int i = 0;

	PMC_BK.SYSTICKbackup_HP[0] = SysTick->CTRL;
	PMC_BK.SYSTICKbackup_HP[1] = SysTick->LOAD;
	PMC_BK.SYSTICKbackup_HP[2] = SysTick->VAL;

	PMC_BK.NVICbackup_HP[0] = NVIC->ISER[0];
	PMC_BK.NVICbackup_HP[1] = NVIC->ISER[1];
	PMC_BK.NVICbackup_HP[2] = NVIC->ISER[2];
	PMC_BK.NVICbackup_HP[3] = NVIC->ICER[0];
	PMC_BK.NVICbackup_HP[4] = NVIC->ICER[1];
	PMC_BK.NVICbackup_HP[5] = NVIC->ICER[2];

	for (i = 0; i < MAX_PERIPHERAL_IRQ_NUM; i++) {
		PMC_BK.NVICIPbackup_HP[i] = NVIC->IPR[i];
	}

	PMC_BK.NVICbackup_HP[6] = NVIC->ISPR[0];
	PMC_BK.NVICbackup_HP[7] = NVIC->ISPR[1];
	PMC_BK.NVICbackup_HP[8] = NVIC->ISPR[2];

#if defined(__VTOR_PRESENT) && (__VTOR_PRESENT == 1)
	PMC_BK.SCBbackup_HP[0] = SCB->VTOR;
#endif
}

VOID SOCPS_NVICReFill_HP(VOID)
{
	int i = 0;

	/* Configure SysTick to interrupt at the requested rate. */
	SysTick->CTRL = PMC_BK.SYSTICKbackup_HP[0];
	SysTick->LOAD = PMC_BK.SYSTICKbackup_HP[1];
	SysTick->VAL = PMC_BK.SYSTICKbackup_HP[2];

	NVIC->ISPR[0] = PMC_BK.NVICbackup_HP[6];
	NVIC->ISPR[1] = PMC_BK.NVICbackup_HP[7];
	NVIC->ISPR[2] = PMC_BK.NVICbackup_HP[8];

	NVIC->ICER[0] = PMC_BK.NVICbackup_HP[3];
	NVIC->ICER[1] = PMC_BK.NVICbackup_HP[4];
	NVIC->ICER[2] = PMC_BK.NVICbackup_HP[5];

	for (i = 0; i < MAX_PERIPHERAL_IRQ_NUM; i++) {
		NVIC->IPR[i] = PMC_BK.NVICIPbackup_HP[i];
	}

	NVIC->ISER[0] = PMC_BK.NVICbackup_HP[0];
	NVIC->ISER[1] = PMC_BK.NVICbackup_HP[1];
	NVIC->ISER[2] = PMC_BK.NVICbackup_HP[2];

#if defined(__VTOR_PRESENT) && (__VTOR_PRESENT == 1)
	SCB->VTOR = PMC_BK.SCBbackup_HP[0];
#endif
}


VOID SOCPS_MPUBackup_HP(void)
{
	int i = 0;
	PMC_BK.MPU_BK.CTRL = MPU->CTRL;

	PMC_BK.MPU_BK.MAIR0 = MPU->MAIR0;
	PMC_BK.MPU_BK.MAIR1 = MPU->MAIR1;
	for (i = 0; i < MPU_MAX_REGION; i++) {
		MPU->RNR = i;
		PMC_BK.MPU_BK.RBAR[i] = MPU->RBAR;
		PMC_BK.MPU_BK.RLAR[i] = MPU->RLAR;
	}
}

VOID SOCPS_MPUReFill_HP(VOID)
{
	int i = 0;
	MPU->CTRL = 0;
	MPU->MAIR0 = PMC_BK.MPU_BK.MAIR0;
	MPU->MAIR1 = PMC_BK.MPU_BK.MAIR1;
	for (i = 0; i < MPU_MAX_REGION; i++) {
		MPU->RNR = i;
		MPU->RBAR = PMC_BK.MPU_BK.RBAR[i];
		MPU->RLAR = PMC_BK.MPU_BK.RLAR[i];
	}
	MPU->CTRL = PMC_BK.MPU_BK.CTRL;
}

_OPTIMIZE_NONE_
IMAGE2_RAM_TEXT_SECTION
VOID SOCPS_SleepPG(VOID)
{
	u32 nDeviceIdOffset = 0;
	u32 Rtemp = 0;
	u32 KR4_is_NP;

	//InterruptDis(UART_LOG_IRQ);
	Img2EntryFun0.RamWakeupFun = SOCPS_WakeFromPG_KM4;

	/* exec sleep hook functions */
	nDeviceIdOffset = pmu_exec_sleep_hook_funs();
	if (nDeviceIdOffset != PMU_MAX) {
		pmu_exec_wakeup_hook_funs(nDeviceIdOffset);
		DBG_8195A("DBG: Sleep blocked because Dev %x  busy\n", nDeviceIdOffset);
		return;
	}

	KR4_is_NP = LSYS_GET_KR4_IS_NP(HAL_READ32(SYSTEM_CTRL_BASE, REG_LSYS_SYSTEM_CFG1));

	/* enable boot from ps */
	Rtemp = HAL_READ32(SYSTEM_CTRL_BASE, REG_LSYS_BOOT_CFG);
	Rtemp |= LSYS_BIT_BOOT_WAKE_FROM_PS_HS;
	HAL_WRITE32(SYSTEM_CTRL_BASE, REG_LSYS_BOOT_CFG, Rtemp);

	/*Backup KM4 IPC configuration*/
	PMC_BK.IPCbackup_HP = IPC_IERGet(IPCKM4_DEV);
	PMC_BK.BASEPRI_backup_HP = __get_BASEPRI();
	PMC_BK.PRIMASK_backup_HP = __get_PRIMASK();

	/* backup registgers */
	SOCPS_NVICBackup_HP();
	SOCPS_MPUBackup_HP();

	//asm volatile("SLPPG_WAKEUP_POINT:\n");
	asm volatile("MOV %0, r0\n"	:"=r"(PMC_BK.CPURegbackup_HP[0])::"memory");
	asm volatile("MOV %0, r1\n"	:"=r"(PMC_BK.CPURegbackup_HP[1])::"memory");
	asm volatile("MOV %0, r2\n"	:"=r"(PMC_BK.CPURegbackup_HP[2])::"memory");
	asm volatile("MOV %0, r3\n"	:"=r"(PMC_BK.CPURegbackup_HP[3])::"memory");
	asm volatile("MOV %0, r4\n"	:"=r"(PMC_BK.CPURegbackup_HP[4])::"memory");
	asm volatile("MOV %0, r5\n"	:"=r"(PMC_BK.CPURegbackup_HP[5])::"memory");
	asm volatile("MOV %0, r6\n"	:"=r"(PMC_BK.CPURegbackup_HP[6])::"memory");
	asm volatile("MOV %0, r7\n"	:"=r"(PMC_BK.CPURegbackup_HP[7])::"memory");
	asm volatile("MOV %0, r8\n"	:"=r"(PMC_BK.CPURegbackup_HP[8])::"memory");
	asm volatile("MOV %0, r9\n"	:"=r"(PMC_BK.CPURegbackup_HP[9])::"memory");
	asm volatile("MOV %0, r10\n":"=r"(PMC_BK.CPURegbackup_HP[10])::"memory");
	asm volatile("MOV %0, r11\n":"=r"(PMC_BK.CPURegbackup_HP[11])::"memory");
	asm volatile("MOV %0, r12\n":"=r"(PMC_BK.CPURegbackup_HP[12])::"memory");
	asm volatile("MOV %0, r13\n":"=r"(PMC_BK.CPURegbackup_HP[13])::"memory");//PSP
	asm volatile("MOV %0, r14\n" :"=r"(PMC_BK.CPURegbackup_HP[14])::"memory");
	asm volatile("MOV %0, pc\n" :"=r"(PMC_BK.CPURegbackup_HP[15])::"memory");
	asm volatile("MRS %0, PSR\n":"=r"(PMC_BK.CPURegbackup_HP[16])	::"memory");
	asm volatile("NOP");
	asm volatile("NOP");

	PMC_BK.CPURegbackup_HP[14] = PMC_BK.CPURegbackup_HP[15];
	PMC_BK.CPUPSP_HP = PMC_BK.CPURegbackup_HP[13];

	/* Enable low power mode:  */
	if (WakeEventFlag_KM4 != _TRUE) {
		WakeEventFlag_KM4 = _TRUE;
		/* mask KM4 irqs */
		Rtemp = HAL_READ32(PMC_BASE, SYSPMC_CTRL);
		Rtemp |= PMC_BIT_KM4_IRQ_MASK;
		HAL_WRITE32(PMC_BASE, SYSPMC_CTRL, Rtemp);

		for (int x = 0; x <= MAX_PERIPHERAL_IRQ_NUM / 32; x++) {
			if (NVIC->ISER[x] & NVIC->ISPR[x]) {
				DBG_PRINTF(MODULE_KM4, LEVEL_ERROR, "x: %d, ISER: 0x%x, ISPR: 0x%x\n", x, NVIC->ISER[x], NVIC->ISPR[x]);
				goto resume;
			}
		}

		if (KR4_is_NP) {
			DBG_PRINTF(MODULE_KM4, LEVEL_INFO, "M4G AP\n");
			//TODO: set rw_prot, open KR4 access authority for reg WAK_MASK0_AP/WAK_MASK1_AP/REG_LSYS_CKE_GRP0/SYSPMC_CTRL
			//currently set 0 to all rw_prot register
			sleep_param.sleep_type = SLEEP_PG;
			DCache_Clean(0xFFFFFFFF, 0xFFFFFFFF);
			ipc_send_message(IPC_KM4_TO_KR4, IPC_M2R_TICKLESS_INDICATION, (PIPC_MSG_STRUCT)&sleep_param);
		} else {
			DBG_PRINTF(MODULE_KM4, LEVEL_INFO, "M4G NP\n");
			//SOCPS_NP_suspend_config(SLEEP_PG);
			SOCPS_PLL_open(DISABLE);

			DCache_Clean(0xFFFFFFFF, 0xFFFFFFFF);
			Cache_Enable(DISABLE);

			if (ps_config.np_config_ddr == APM_PSRAM_SlEEP_Mode || ps_config.np_config_ddr == WB_PSRAM_SlEEP_Mode) {
				set_psram_suspend_and_restore(DISABLE);
			}

			/* close CPU */
			Rtemp = HAL_READ32(PMC_BASE, SYSPMC_OPT);
			Rtemp &= ~PMC_BIT_PST_SLEP_ESOC;
			HAL_WRITE32(PMC_BASE, SYSPMC_OPT, Rtemp);

			/* set power mode */
			Rtemp = HAL_READ32(PMC_BASE, SYSPMC_CTRL);
			Rtemp |= PMC_BIT_PMEN_SLEP;
			Rtemp &= ~PMC_BIT_PMEN_DSLP;
			HAL_WRITE32(PMC_BASE, SYSPMC_CTRL, Rtemp);

		}
		/* it will take 6.5us to enter clock gating mode after register set */
		__WFI();

		if (KR4_is_NP) {
			//currently set 1 to all rw_prot register

		} else {
			if (ps_config.np_config_ddr == APM_PSRAM_SlEEP_Mode || ps_config.np_config_ddr == WB_PSRAM_SlEEP_Mode) {
				set_psram_suspend_and_restore(ENABLE);
			}

			Cache_Enable(ENABLE);

			//SOCPS_NP_resume_config(SLEEP_PG);
			SOCPS_PLL_open(ENABLE);
		}

	} else {
		NewVectorTable[11] = (HAL_VECTOR_FUN)PMC_BK.PortSVC_Backup_HP;

		if (KR4_is_NP) {
			DBG_PRINTF(MODULE_KM4, LEVEL_INFO, "M4GW AP\n");
			//currently set 1 to all rw_prot register

		} else {
			DBG_PRINTF(MODULE_KM4, LEVEL_INFO, "M4GW NP\n");
			//SOCPS_NP_resume_config(SLEEP_PG);
			SOCPS_PLL_open(ENABLE);
		}

		/* ReFill registers */
		__set_PRIMASK(PMC_BK.PRIMASK_backup_HP);
		__set_BASEPRI(PMC_BK.BASEPRI_backup_HP);
		/*Refill KM4 IPC configuration*/
		IPC_IERSet(IPCKM4_DEV, PMC_BK.IPCbackup_HP);
		SOCPS_NVICReFill_HP();
		SOCPS_MPUReFill_HP();
		//rtl_cryptoEngine_init();
	}

	if (HAL_READ32(PMC_BASE, SYSPMC_CTRL) & PMC_BIT_PMEN_SLEP) {
		/* clear power mode */
		Rtemp = HAL_READ32(PMC_BASE, SYSPMC_CTRL);
		Rtemp &= ~PMC_BIT_PMEN_SLEP;
		HAL_WRITE32(PMC_BASE, SYSPMC_CTRL, Rtemp);
	}

resume:
	if (HAL_READ32(PMC_BASE, SYSPMC_CTRL) & PMC_BIT_KM4_IRQ_MASK) {
		/* unmask KM4 irqs */
		Rtemp = HAL_READ32(PMC_BASE, SYSPMC_CTRL);
		Rtemp &= ~PMC_BIT_KM4_IRQ_MASK;
		HAL_WRITE32(PMC_BASE, SYSPMC_CTRL, Rtemp);
	}

	/* clear boot from ps */
	Rtemp = HAL_READ32(SYSTEM_CTRL_BASE, REG_LSYS_BOOT_CFG);
	Rtemp &= ~LSYS_BIT_BOOT_WAKE_FROM_PS_HS;
	HAL_WRITE32(SYSTEM_CTRL_BASE, REG_LSYS_BOOT_CFG, Rtemp);

	pmu_exec_wakeup_hook_funs(PMU_MAX);
	WakeEventFlag_KM4 = _FALSE;
}

VOID SOCPS_DeepSleep(VOID)
{
	DCache_CleanInvalidate(0xFFFFFFFF, 0xFFFFFFFF);

	ipc_send_message(IPC_KM4_TO_KR4, IPC_M2R_TICKLESS_INDICATION, (PIPC_MSG_STRUCT)&sleep_param);

	Systick_Cmd(DISABLE);
	__disable_irq();

	__WFI();

	__enable_irq();
	Systick_Cmd(ENABLE);
}

void SOCPS_vWFSSVCHandler_KM4(void)
{
	asm volatile("mov r0, %0\n"::"r"(PMC_BK.CPUPSP_HP):"r0");
	asm volatile
	(
		"ldmia r0!, {r4-r7}				\n" /* Pop the registers that are not automatically saved on exception entry and the critical nesting count. */
		"ldmia r0!, {r8-r11}				\n"
		"msr psp, r0						\n" /* Restore the task stack pointer. */
		"orr r14, r14, #0xc					      \n"
		"bx r14							\n"
	);
}

IMAGE2_RAM_TEXT_SECTION
VOID SOCPS_WakeFromPG_KM4(VOID)
{
	/* we should Cache_Flush when we wake */
	Cache_Enable(DISABLE);
	Cache_Enable(ENABLE);


	/* Make PendSV, CallSV and SysTick the same priroity as the kernel. */
	/* uint8_t  SHP[12]: Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
	/* SCB->SHP[8~11] */
	HAL_WRITE32(0, 0xE000ED20, 0xF0F00000);
	PMC_BK.PortSVC_Backup_HP = (u32)NewVectorTable[11];
	NewVectorTable[11] = (HAL_VECTOR_FUN)SOCPS_vWFSSVCHandler_KM4;
	/* push cpu register into stack based on SOCPS_vWFSSVCHandler */
	/* push cpu register into stack based on SOCPS_vWFSSVCHandler */
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 4))) = (PMC_BK.CPURegbackup_HP[16] | 0x1000000);     //PSR
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 8))) = PMC_BK.CPURegbackup_HP[15];     //PC
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 12))) = PMC_BK.CPURegbackup_HP[14];     //LR
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 16))) = PMC_BK.CPURegbackup_HP[12];     //R12
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 20))) = PMC_BK.CPURegbackup_HP[3];      //R3
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 24))) = PMC_BK.CPURegbackup_HP[2];      //R2
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 28))) = PMC_BK.CPURegbackup_HP[1];      //R1
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 32))) = PMC_BK.CPURegbackup_HP[0];      //R0
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 36))) = PMC_BK.CPURegbackup_HP[11];     //R11
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 40))) = PMC_BK.CPURegbackup_HP[10];     //R10
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 44))) = PMC_BK.CPURegbackup_HP[9];      //R9
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 48))) = PMC_BK.CPURegbackup_HP[8];      //R8
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 52))) = PMC_BK.CPURegbackup_HP[7];      //R7
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 56))) = PMC_BK.CPURegbackup_HP[6];      //R6
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 60))) = PMC_BK.CPURegbackup_HP[5];      //R5
	(* ((volatile unsigned long *)(PMC_BK.CPUPSP_HP - 64))) = PMC_BK.CPURegbackup_HP[4];      //R4
	PMC_BK.CPUPSP_HP = PMC_BK.CPUPSP_HP - 64; //PSP

	asm volatile(
		" cpsie i				\n" /* Globally enable interrupts. */
		" svc 0					\n" /* System call to start first task. */
		" nop					\n"
	);
}


IMAGE2_RAM_TEXT_SECTION
void SOCPS_SleepCG(void)
{
	u32 nDeviceIdOffset = 0;
	u32 Rtemp;
	u32 KR4_is_NP = 0;

	KR4_is_NP = LSYS_GET_KR4_IS_NP(HAL_READ32(SYSTEM_CTRL_BASE, REG_LSYS_SYSTEM_CFG1));

	/* exec sleep hook functions */
	nDeviceIdOffset = pmu_exec_sleep_hook_funs();
	if (nDeviceIdOffset != PMU_MAX) {
		pmu_exec_wakeup_hook_funs(nDeviceIdOffset);
		DBG_8195A("Oops: Sleep Fail %x !!!!!\n", nDeviceIdOffset);
		return;
	}

	/* mask KM4 irqs */
	Rtemp = HAL_READ32(PMC_BASE, SYSPMC_CTRL);
	Rtemp |= PMC_BIT_KM4_IRQ_MASK;
	HAL_WRITE32(PMC_BASE, SYSPMC_CTRL, Rtemp);

	for (int32_t x = 0; x <= MAX_PERIPHERAL_IRQ_NUM / 32; x++) {
		if (NVIC->ISER[x] & NVIC->ISPR[x]) {
			DBG_PRINTF(MODULE_KM4, LEVEL_ERROR, "x: %d, ISER: 0x%x, ISPR: 0x%x\n", x, NVIC->ISER[x], NVIC->ISPR[x]);
			goto resume;
		}
	}

	if (KR4_is_NP) {
		DBG_PRINTF(MODULE_KM4, LEVEL_INFO, "M4C AP\n");
		//TODO: set token bit, open KR4 access authority for reg WAK_MASK0_AP/WAK_MASK1_AP/REG_LSYS_CKE_GRP0/SYSPMC_CTRL
		//currently set 0 to all rw_prot register
		//HAL_WRITE32(PMC_BASE, SYSPMC_OPT, 0);

		sleep_param.sleep_type = SLEEP_CG;
		ipc_send_message(IPC_KM4_TO_KR4, IPC_M2R_TICKLESS_INDICATION, (PIPC_MSG_STRUCT)&sleep_param);
	} else {
		DBG_PRINTF(MODULE_KM4, LEVEL_INFO, "M4C NP\n");
		//SOCPS_NP_suspend_config(SLEEP_CG);
		SOCPS_PLL_open(DISABLE);

		DCache_Clean(0xFFFFFFFF, 0xFFFFFFFF);
		Cache_Enable(DISABLE);

		if (ps_config.np_config_ddr == APM_PSRAM_SlEEP_Mode || ps_config.np_config_ddr == WB_PSRAM_SlEEP_Mode) {
			set_psram_suspend_and_restore(DISABLE);
		}

		/* don't close CPU*/
		Rtemp = HAL_READ32(PMC_BASE, SYSPMC_OPT);
		Rtemp |= (PMC_BIT_PST_SLEP_ESOC);
		HAL_WRITE32(PMC_BASE, SYSPMC_OPT, Rtemp);

		/* set power mode */
		Rtemp = HAL_READ32(PMC_BASE, SYSPMC_CTRL);
		Rtemp |= PMC_BIT_PMEN_SLEP;
		Rtemp &= ~PMC_BIT_PMEN_DSLP;
		HAL_WRITE32(PMC_BASE, SYSPMC_CTRL, Rtemp);

	}
	/* it will take 6.5us to enter clock gating mode after register set */
	__WFI();

	if (KR4_is_NP) {
		DBG_PRINTF(MODULE_KM4, LEVEL_INFO, "M4CW AP\n");
		//TODO: set token bit, close KR4 access authority

	} else {
		if (ps_config.np_config_ddr == APM_PSRAM_SlEEP_Mode || ps_config.np_config_ddr == WB_PSRAM_SlEEP_Mode) {
			set_psram_suspend_and_restore(ENABLE);
		}

		Cache_Enable(ENABLE);

		//SOCPS_NP_resume_config(SLEEP_CG);
		SOCPS_PLL_open(ENABLE);
	}

	if (HAL_READ32(PMC_BASE, SYSPMC_CTRL) & PMC_BIT_PMEN_SLEP) {
		/* clear power mode */
		Rtemp = HAL_READ32(PMC_BASE, SYSPMC_CTRL);
		Rtemp &= ~PMC_BIT_PMEN_SLEP;
		HAL_WRITE32(PMC_BASE, SYSPMC_CTRL, Rtemp);
	}

resume:
	if (HAL_READ32(PMC_BASE, SYSPMC_CTRL) & PMC_BIT_KM4_IRQ_MASK) {
		/* unmask KM4 irqs */
		Rtemp = HAL_READ32(PMC_BASE, SYSPMC_CTRL);
		Rtemp &= ~PMC_BIT_KM4_IRQ_MASK;
		HAL_WRITE32(PMC_BASE, SYSPMC_CTRL, Rtemp);
	}

	/* TODO: exec sleep hook functions */
	pmu_exec_wakeup_hook_funs(PMU_MAX);

}


/**
  * @brief  KR4 wake KM4 by IPC
  * @param  None
  * @note  ipc_msg_temp.msg represents who wakes up KM4
  		* ipc_msg_temp.msg = 0: FW wakeup KM4
  * @retval None
  */
IMAGE2_RAM_TEXT_SECTION
void SOCPS_KR4WKM4_ipc_int(VOID *Data, u32 IrqStatus, u32 ChanNum)
{
	/* To avoid gcc warnings */
	UNUSED(Data);
	UNUSED(IrqStatus);
	UNUSED(ChanNum);

	PIPC_MSG_STRUCT	ipc_msg_temp = (PIPC_MSG_STRUCT)ipc_get_message(IPC_KR4_TO_KM4, IPC_R2M_WAKE_AP);

	u32 type = ipc_msg_temp->msg;

	if (type == FW_NPWAP_IPC) {
		DBG_8195A("FW wakeup KM4 via IPC \n");
	} else if (type == TIMER_NPWAP_IPC) {
		DBG_8195A("TIMER wakeup KM4 via IPC \n");
	}

}

IPC_TABLE_DATA_SECTION
const IPC_INIT_TABLE   ipc_KR4WKM4_table[] = {
	{IPC_USER_DATA, 	SOCPS_KR4WKM4_ipc_int,	(VOID *) NULL, IPC_KR4_TO_KM4, IPC_R2M_WAKE_AP, IPC_RX_FULL},
};