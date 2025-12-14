/*
 * Copyright (c) 2023 AXERA in AX620E project.
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <asm/armv8/mmu.h>
#include <asm/io.h>
#include <sdhci.h>
#include <malloc.h>
#include <asm/arch/ax620e.h>
#include <asm/arch/boot_mode.h>
#include <asm/arch-axera/dma.h>

#ifdef CONFIG_ARM64
static struct mm_region ax620e_mem_map[] = {
	{
#ifdef CONFIG_AXERA_AX630C_DDR4_RETRAIN
		.virt = 0x40001000UL,
		.phys = 0x40001000UL,
		.size = MEM_REGION_DDR_SIZE - 0x1000,
#else
		.virt = 0x40000000UL,
		.phys = 0x40000000UL,
		.size = MEM_REGION_DDR_SIZE,
#endif
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
                        PTE_BLOCK_INNER_SHARE
	}, {
		.virt = 0x00000000UL,
		.phys = 0x00000000UL,
		.size = 0x10500000UL,//寄存器空间
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	},{
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = ax620e_mem_map;
#endif

int arch_cpu_init(void)
{
	/* We do some SoC one time setting here. */
	axi_dma_hw_init();
	return 0;
}

struct boot_mode_info boot_info_data;

u32 get_boot_voltage(void)
{
	u32 val;
	val = readl(PIN_MUX_G11_VDET_RO0);
	/* 0: 3.3V   1: 1.8V */
	if (((val >> 0) & BIT(0))) { //3.3v
		return 0;
	} else { //1.8v
		return 1;
	}
}

boot_mode_info_t *get_dl_and_boot_info(void)
{
	boot_mode_info_t *boot_mode = (boot_mode_info_t *) BOOT_MODE_INFO_ADDR;
	printf("boot_mode->magic = 0x%x\n", boot_mode->magic);
	printf("boot_mode->dl_channel = %d\n", boot_mode->dl_channel);
	printf("boot_mode->storage_sel = %d\n", boot_mode->storage_sel);
	printf("boot_mode->boot_type = %d\n", boot_mode->boot_type);

	boot_mode->mode = NORMAL_BOOT_MODE;

	if (boot_mode->dl_channel == DL_CHAN_UART1
	    || boot_mode->dl_channel == DL_CHAN_UART0) {
		boot_mode->mode = UART_UPDATE_MODE;
	}
	if (boot_mode->dl_channel == DL_CHAN_USB) {
		boot_mode->mode = USB_UPDATE_MODE;
	}
	if (boot_mode->is_sd_boot == true) {
		boot_mode->mode = SD_BOOT_MODE;
	}

	return boot_mode;
}

// console ,download, coredump to disable wtd0
void wdt0_enable(bool enable)
{
	/* set wdt timeout clk to 30S */
	writel((30 * WDT0_CLK_FREQ) >> 16, (void *)(WDT0_TORR_ADDR));
	writel(0x1, (void *)(WDT0_TORR_START_ADDR));
	writel(0x0, (void *)(WDT0_TORR_START_ADDR));

	/* set wdt0 clk source to 24MHz */
	writel(BIT(19), (void *)(PERI_SYS_GLB_CLK_MUX0_SET));

	if (enable) {
		writel(1, WDT0_BASE);
	} else {
		writel(0, WDT0_BASE);
	}
}

void set_wdt0_timeout(u32 time)
{
	u32 time_temp = time / 2;
	/* set wdt timeout clk to time_temp */
	writel((time_temp * WDT0_CLK_FREQ) >> 16, (void *)(WDT0_TORR_ADDR));
	writel(0x1, (void *)(WDT0_TORR_START_ADDR));
	writel(0x0, (void *)(WDT0_TORR_START_ADDR));

	/* set wdt0 clk source to 24MHz */
	writel(BIT(19), (void *)(PERI_SYS_GLB_CLK_MUX0_SET));

	writel(1, WDT0_BASE);
}

#if 0
void reboot(void)
{
	wdt0_enable(true);
	printf("trigger watchdog, reboot now ...\n");
	while (1) ;
}
#else
static void chip_rst_sw(void)
{
	u32 tmp;

	tmp = readl(TOP_CHIPMODE_GLB_BACKUP0);
	tmp &= ~(BOOT_DOWNLOAD | BOOT_KERNEL_FAIL);
	writel(tmp, (void *)TOP_CHIPMODE_GLB_BACKUP0);

	tmp = readl((void *)COMM_ABORT_CFG);
	tmp |= CHIP_RST_SW;
	printf("Set REG 0x%X, value 0x%X\n", COMM_ABORT_CFG, tmp);
	writel(tmp, (void *)COMM_ABORT_CFG);
}

void reboot(void)
{
	chip_rst_sw();
	printf("trigger chip_rst_sw ...\n");
	while (1) ;
}
#endif

#ifndef CONFIG_SYSRESET
void reset_cpu(unsigned long ignored)
{
	reboot();
}
#endif

u32 dump_reason;
u32 axera_get_boot_reason(void)
{
	u32 abort_cfg;
	u32 abort_status;
	abort_status = readl(COMM_ABORT_STATUS);
	dump_reason = abort_status;
	/*clear abort alarm status */
	abort_cfg =
	    ABORT_WDT0_CLR | ABORT_WDT2_CLR | ABORT_THM_CLR | ABORT_SWRST_CLR;
	writel(abort_cfg, COMM_ABORT_CFG);
	/*enable watchdog, thermal abort function */
	abort_cfg = ABORT_WDT2_EN | ABORT_WDT0_EN | ABORT_THM_EN;
	writel(abort_cfg, COMM_ABORT_CFG);
	return abort_status;
}

static void print_boot_reason(void)
{
	u32 tmp;
	tmp = axera_get_boot_reason();
	printf("boot_reason:0x%x\n",tmp);
	if (tmp & (1 << 4)) {
		printf("wdt2 reset\n");
	}
	if (tmp & (1 << 2)) {
		printf("wdt0 reset\n");
	}
	if (tmp & (1 << 1)) {
		printf("thm reset\n");
	}
	if (tmp & (1 << 0)) {
		printf("swrst reset\n");
	}
}

static void print_board_info(void)
{
	print_chip_type();
	print_board_id();
}
#ifdef CONFIG_SUPPORT_AB
static int set_slot_ab(void)
{
	u32 slottype = 0;

	slottype = readl(TOP_CHIPMODE_GLB_BACKUP0);

	if (slottype & SLOTA) {
		env_set("bootsystem", "A");
		printf("From slota boot\n");
	}
	if (slottype & SLOTB) {
		env_set("bootsystem", "B");
		printf("From slotb boot\n");
	}
	env_save();
	return 0;
}
#endif

#ifdef CONFIG_SUPPORT_RECOVERY
static int set_recovery(void)
{
	u32 bootable = 0;

	bootable = readl(TOP_CHIPMODE_GLB_BACKUP0);
	if ((bootable & BOOT_KERNEL_FAIL) || (bootable & BOOT_RECOVERY)) {
		env_set("bootable", "recovery");
	}
	env_save();
}
#endif

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
#ifdef CONFIG_SUPPORT_AB
	set_slot_ab();
#endif
#ifdef CONFIG_SUPPORT_RECOVERY
	set_recovery();
#endif
	print_board_info();
	wdt0_enable(1);
	print_boot_reason();
#if defined(CONFIG_CMD_AXERA_DOWNLOAD) || defined(CONFIG_CMD_AXERA_BOOT)
	setup_boot_mode();
#endif
	set_ephy_led_pol();

	return 0;
}
#endif
