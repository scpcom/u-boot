/*
 * AXERA AX620E Controller Interface
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/boot_mode.h>
#include <fs.h>
#include <blk.h>
#include <memalign.h>
#include <fat.h>
#include <linux/sizes.h>
#include <asm/io.h>
#include <asm/arch/ax620e.h>
#include <image-sparse.h>
#include "../secureboot/secureboot.h"
#include <dm/uclass.h>
#include <dm/device.h>
#include <mtd.h>
#include "../../legacy-mtd-utils.h"
#include "axera_update.h"
#include <mmc.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <dm/device_compat.h>
#include <dm/lists.h>
#include <linux/compat.h>
#include <asm/io.h>
#include <mapmem.h>
#include <part.h>
#include <fat.h>
#include <fs.h>
#include <rtc.h>
#include <linux/time.h>

#define DUMP_FILE_NAME_LEN 32
#define AXERA_REASON_MASK 0xf98e7c6d

extern struct boot_mode_info boot_info_data;
extern boot_mode_info_t *get_dl_and_boot_info(void);
extern void set_wdt0_timeout(u32 time);
extern u32 dump_reason;

char dump_file_name[DUMP_FILE_NAME_LEN];
char dump_info_name[DUMP_FILE_NAME_LEN];

struct axera_memory_dump_struct {
	u64 axera_dump_info_addr;
	u64 axera_dump_info_size;
	u64 axera_dump_addr;
	u64 axera_dump_size;
};

#if defined CONFIG_AXERA_MEMORY_DUMP_SD || CONFIG_AXERA_MEMORY_DUMP_EMMC || CONFIG_AXERA_MEMORY_DUMP_USB_STORAGE
int last_kernel_dtb_check(struct axera_memory_dump_struct *ax_info)
{
	int offset;
	int len,n,mem;
	char buff[8],*mptr;
	unsigned long addr, size, memory_addr;
	//const char *plat_prop;
	//const u32 *prop;
	const u32 *val;
	char *fdt = (char *)DTB_IMAGE_ADDR;

	if (fdt_check_header(fdt)) {
		printf("Invalid device tree header\n");
		return -1;
	}

	offset = fdt_path_offset(fdt, "/reserved-memory/axera_memory_dump@0");
	if (offset < 0)
		printf("reserved_mem error \n");

	val = fdt_getprop(fdt, offset, "reg", &len);
	if (val == NULL) {
		printf("get prop val failed!\n");
		return -1;
	}
#ifdef CONFIG_ARM64
	addr = fdt32_to_cpu(val[0]);
	addr = addr << 32;
	addr |= fdt32_to_cpu(val[1]);

	size = fdt32_to_cpu(val[2]);
	size = size << 32;
	size |= fdt32_to_cpu(val[3]);
#else
	addr = fdt32_to_cpu(val[1]);
	size = fdt32_to_cpu(val[3]);
#endif
	printf("addr = 0x%llx  size = 0x%llx\n", addr, size);

	offset = fdt_path_offset(fdt, "/memory@40000000");
	if (offset < 0)
		printf("memory node error \n");

	val = fdt_getprop(fdt, offset, "reg", &len);
	if (val == NULL) {
		printf("get prop val failed!\n");
		return -1;
	}

	memory_addr = fdt32_to_cpu(val[0]);
	memory_addr = memory_addr << 32;
	memory_addr |= fdt32_to_cpu(val[1]);
	printf("memory_addr = 0x%llx\n", memory_addr);

	ax_info->axera_dump_info_addr = addr;
	ax_info->axera_dump_info_size = size;
	ax_info->axera_dump_addr = memory_addr;
	if((mptr = strstr(OS_MEM_ARGS,"mem=")) == NULL) {
		ax_info->axera_dump_size = 0x40000000;
		printf("mem= not found\n");
		return 0;
	}
	memset(buff,0,sizeof(buff));
	strcpy(buff,mptr+4);
	mptr = strstr(buff,"M");
	*mptr = 0;
	mem = strtoul(buff,NULL, 10)*0x100000;
	ax_info->axera_dump_size = mem;

	return 0;
}

static int  get_boot_reason_mask(u32 boot_reason,struct axera_memory_dump_struct  *axera_dump_info)
{
	u32 reason_mask;
	reason_mask = readl(axera_dump_info->axera_dump_info_addr+1024);
	pr_err("boot_reason = 0x%x reason_mask = 0x%x\n",boot_reason,reason_mask);
	return reason_mask;
}

#ifdef CONFIG_AXERA_MEMORY_DUMP_EMMC
static void  clear_boot_reason_mask(struct axera_memory_dump_struct  *axera_dump_info)
{
	printf("clean reason_mask\n");
        writel(0x0,(axera_dump_info->axera_dump_info_addr+1024));
}
#endif

void display_kernel_time(struct axera_memory_dump_struct *axera_dump_info,char dump_file_name[],int flag)
{
	struct rtc_time tm;
	memcpy((void *)&tm,(void *)(axera_dump_info->axera_dump_info_addr+1088),sizeof(tm));
	printf("last kernel crash time :%d-%d-%d %d:%d:%d\n",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
	memset(dump_file_name,0,DUMP_FILE_NAME_LEN);
	if(flag == 1) {
		sprintf(dump_file_name,"vmcore.dump.%4d%02d%02d%02d%02d%02d",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
		sprintf(dump_info_name,"vmcore.dump.info.%4d%02d%02d%02d%02d%02d",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
	} else {
		sprintf(dump_file_name,"/vmcore.dump.%4d%02d%02d%02d%02d%02d",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
		sprintf(dump_info_name,"/vmcore.dump.info%4d%02d%02d%02d%02d%02d",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
	}
	printf("saving sysdump to %s\n", dump_file_name);
}
#endif

#ifdef CONFIG_AXERA_MEMORY_DUMP_USB_STORAGE
static int dump_memory_to_usb_storage(u32 boot_reason)
{
	struct blk_desc *usb_stor_desc = NULL;
	int ret;
	loff_t size;
	unsigned long addr;
	unsigned long count;
	long offset;
	u32 reason_mask;

	void *buf;
	struct axera_memory_dump_struct  axera_dump_info;

	ret = last_kernel_dtb_check(&axera_dump_info);
	if(ret < 0)
		return -1;

	printf("usb-storage memory dump...\n");

	ret = run_command("usb start", 0);
	if (ret) {
		printf("memory dump usb start error %d\n", ret);
		return -1;
	}

	/* step1 check usb-storage is present */
	usb_stor_desc = blk_get_dev("usb", 0);
	if (NULL == usb_stor_desc) {
		printf("usb-storage is not present, exit dump\n");
		return -1;
	}

	/* we register usb to fatfs */
	if (fat_register_device(usb_stor_desc, 1)) {
		printf("memory dump usb-storage register part1 fat fail, try part0\n");

		if (fat_register_device(usb_stor_desc, 0)) { /* in normal condition, part0 is MBR */
			printf("usb-storage register part0 fat fail, exit usb-storage memory dump\n");
			return -1;
		}
	}

	reason_mask = get_boot_reason_mask(boot_reason,&axera_dump_info);

	if((boot_reason != 0) && (reason_mask == 0xf98e7c6d)) {

		display_kernel_time(&axera_dump_info,dump_file_name, 1);
		addr =  axera_dump_info.axera_dump_addr;
		count = axera_dump_info.axera_dump_size;
		offset = 0;

		printf("usb addr = 0x%lx  count = 0x%lx dump_file_name = %s\n",addr,count,dump_file_name);
		buf = map_sysmem(addr, count);
   		ret = file_fat_write(dump_file_name, buf, offset, count, &size);
   		unmap_sysmem(buf);
   		if (ret < 0) {
			printf("file_fat_write failed\n");
           		return -1;
   		}

		ret = run_command("fatls usb 0", 0);
		if (ret) {
			printf("fatls usb 0 failed\n");
		}
	}
	return 0;
}
#endif

#ifdef CONFIG_AXERA_MEMORY_DUMP_EMMC
int ext4fs_memory_dump(struct blk_desc *fs_dev_desc, unsigned long int addr,unsigned long int size,
                       unsigned long int info_addr,unsigned long int info_size,char *filename);
static int dump_memory_to_emmc(u32 boot_reason)
{
	int ret;
	u32 reason_mask;
	struct axera_memory_dump_struct  axera_dump_info;
	struct blk_desc *emmc_desc;
	ret = last_kernel_dtb_check(&axera_dump_info);
	if(ret < 0){
		printf("[error] last_kernel_dtb_check: %d\n", ret);
		return -1;
	}
	printf("emmc  memory dumping ...\n");
	reason_mask = get_boot_reason_mask(boot_reason, &axera_dump_info);
	if((boot_reason != 0) && (reason_mask == AXERA_REASON_MASK)) {
		/* step1 check emmc is present */
 		emmc_desc = blk_get_dev("mmc", EMMC_DEV_ID);
 		if (NULL == emmc_desc) {
			printf("memory dump sd is not present, exit emmc dump\n");
			return -1;
 		}
		display_kernel_time(&axera_dump_info,dump_file_name,0);

		ext4fs_memory_dump(emmc_desc,axera_dump_info.axera_dump_addr,axera_dump_info.axera_dump_size,
			axera_dump_info.axera_dump_info_addr,axera_dump_info.axera_dump_info_size,dump_file_name);
		printf("emmc dump Done!!!\n");
		clear_boot_reason_mask(&axera_dump_info);
	}
	return 0;
}
#endif

#ifdef CONFIG_AXERA_MEMORY_DUMP_SD
static int dump_memory_to_sd(u32 boot_reason)
{
	u32 reason_mask;
	struct udevice *dev;
	char *mmc_type = NULL;
	loff_t size;
	unsigned long addr;
	unsigned long count;
	long offset;
	void *buf;
	int ret;

	struct blk_desc *sd_desc = NULL;
	struct axera_memory_dump_struct  axera_dump_info;

	ret = last_kernel_dtb_check(&axera_dump_info);
	if(ret < 0)
		return -1;

	reason_mask = get_boot_reason_mask(boot_reason,&axera_dump_info);

	if((boot_reason != 0) && (reason_mask == AXERA_REASON_MASK)) {
		/* step1 check sd is present */
		sd_desc = blk_get_dev("mmc", SD_DEV_ID);
		if (NULL == sd_desc) {
			printf("memory dump sd is not present, exit sd update\n");
			return -1;
		}

		/* we register fat device */
		if (fat_register_device(sd_desc, 1)) {
			printf("sd: no part1 found, check sd card!\n");
			return -1;
		}

			display_kernel_time(&axera_dump_info, dump_file_name, 1);

			for (uclass_first_device(UCLASS_MMC, &dev); dev; uclass_next_device(&dev)) {
				struct mmc *m = mmc_get_mmc_dev(dev);
				if (m->has_init) {
					mmc_type = IS_SD(m) ? "SD" : "eMMC";
				} else {
					mmc_type = "NONE";
				}

				if(!strcmp(mmc_type,"SD")) {
					break;
				}

			}
			if(strcmp(mmc_type, "SD")) {
				pr_err("SD card no found, memory dump failed\n");
				return -1;
			}

			addr =  axera_dump_info.axera_dump_info_addr;
			count = axera_dump_info.axera_dump_info_size;
			offset = 0;
			printf("dump_info_addr = 0x%lx  dump_info_size = 0x%lx\n",addr,count);
			buf = map_sysmem(addr, count);
			ret = file_fat_write(dump_info_name, buf, offset, count, &size);
			if (ret < 0) {
				printf("Unable to write vmcore.dump\n");
				return -1;
			}
			unmap_sysmem(buf);
			addr =  axera_dump_info.axera_dump_addr;
			count = axera_dump_info.axera_dump_size;
			offset = 0;

			printf("addr = 0x%lx  count = 0x%lx\n",addr,count);
			buf = map_sysmem(addr, count);
			ret = file_fat_write(dump_file_name, buf, offset, count, &size);
			if (ret < 0) {
				printf("Unable to write\n");
				return -1;
			}
			unmap_sysmem(buf);
			printf("%llu bytes written\n", size);
		}
	return 0;
}
#endif

#if defined CONFIG_AXERA_MEMORY_DUMP_SD || CONFIG_AXERA_MEMORY_DUMP_EMMC || CONFIG_AXERA_MEMORY_DUMP_USB_STORAGE
static boot_mode_t sysdump_mode(void)
{
	wdt0_enable(0);

	if (dump_reason & 0x1C) {
		printf("Sysdump started, dump_reason: %d\n", dump_reason);
#ifdef CONFIG_AXERA_MEMORY_DUMP_SD
		if (dump_memory_to_sd(dump_reason) < 0) {
			printf("axera memoey dump sd failed\n");
		} else {
			goto dump_ok;
		}
#endif
#ifdef CONFIG_AXERA_MEMORY_DUMP_EMMC
		if (dump_memory_to_emmc(dump_reason) < 0) {
			printf("axera memoey dump emmc failed\n");
		} else {
			goto dump_ok;
		}
#endif
#ifdef CONFIG_AXERA_MEMORY_DUMP_USB_STORAGE

		if (dump_memory_to_usb_storage(dump_reason) < 0) {
			printf("axera memoey dump usb_storage failed\n");
		} else {
			goto dump_ok;
		}
#endif
	}

#if defined CONFIG_AXERA_MEMORY_DUMP_SD || CONFIG_AXERA_MEMORY_DUMP_EMMC || CONFIG_AXERA_MEMORY_DUMP_USB_STORAGE
dump_ok:
#endif
	wdt0_enable(1);

	return CMD_UNDEFINED_MODE;
}
#endif

#ifdef CONFIG_CMD_AXERA_SDUPDATE
static boot_mode_t sd_update_mode(void)
{
	struct blk_desc *sd_desc = NULL;
	char *update_status = env_get("sdupdate");

	sd_desc = blk_get_dev("mmc", SD_DEV_ID);
	if (NULL == sd_desc) {
		printf("no sd card\n");
		return CMD_UNDEFINED_MODE;
	}

	if ((boot_info_data.mode == NORMAL_BOOT_MODE) || (boot_info_data.mode == SD_BOOT_MODE)) {
		if ((update_status == NULL) || !strcmp(update_status, "retry")) {
			env_set("bootcmd", "sd_update");
			printf("env sdupdate is %s, enter sd update mode\n", update_status);
			boot_info_data.mode = SD_UPDATE_MODE;
			wdt0_enable(0);
			return SD_UPDATE_MODE;
		} else if (!strcmp(update_status, "fail")) {
			printf("sd update failed twice, need check\n");
		}
	}

	return CMD_UNDEFINED_MODE;
}
#endif

static boot_mode_t usb_update_mode(void)
{
	if (boot_info_data.mode == USB_UPDATE_MODE) {
		env_set("bootdelay", "0");
		env_set("bootcmd", "download");
		printf("enter usb download mode\n");
		wdt0_enable(0);
		return USB_UPDATE_MODE;
	}
	return CMD_UNDEFINED_MODE;
}

static boot_mode_t uart_update_mode(void)
{
	if (boot_info_data.mode == UART_UPDATE_MODE) {
		env_set("bootdelay", "0");
		env_set("bootcmd", "download");
		printf("enter uart download mode\n");
		wdt0_enable(0);
		return UART_UPDATE_MODE;
	}
	return CMD_UNDEFINED_MODE;
}

#ifdef CONFIG_CMD_AXERA_TFTP_OTA
static boot_mode_t tftp_update_mode(void)
{
	if ((boot_info_data.mode == NORMAL_BOOT_MODE) && (NULL != env_get("ota_ready"))) {
		printf("env ota_ready=%s\n", env_get("ota_ready"));
		if (!strcmp(env_get("ota_ready"), "true")) {
			env_set("bootcmd", "axera_ota");
			printf("enter tftp ota update\n");
			wdt0_enable(0);
			boot_info_data.mode = TFTP_UPDATE_MODE;
			return TFTP_UPDATE_MODE;
		} else if (!strcmp(env_get("ota_ready"), "retry")) {
			env_set("bootcmd", "axera_ota");
			printf("retry tftp ota update\n");
			wdt0_enable(0);
			boot_info_data.mode = TFTP_UPDATE_MODE;
			return TFTP_UPDATE_MODE;
		}
	}
	return CMD_UNDEFINED_MODE;
}
#endif

#ifdef CONFIG_CMD_AXERA_SDBOOT
static boot_mode_t sd_boot_mode(void)
{
	if (boot_info_data.mode == SD_BOOT_MODE) {
		env_set("bootcmd", "sd_boot");
		printf("enter sd boot mode\n");
		return SD_BOOT_MODE;
	}
	return CMD_UNDEFINED_MODE;
}
#endif

#ifdef CONFIG_CMD_AXERA_USB_STOR_UPDATE
static boot_mode_t usb_stor_mode(void)
{
	char * update_status = env_get("usbupdate");

	if ((boot_info_data.mode == NORMAL_BOOT_MODE) && (NULL != update_status)) {
		if (!strcmp(update_status, "ready") || !strcmp(update_status, "retry")) {
			struct blk_desc *usb_stor_desc = NULL;
			int ret;

			ret = run_command("usb start", 0);
			if (ret) {
				printf("usb start error %d\n", ret);
				return CMD_UNDEFINED_MODE;
			}

			usb_stor_desc = blk_get_dev("usb", 0);
			if (NULL == usb_stor_desc) {
				printf("usb-storage is not present\n");
				return CMD_UNDEFINED_MODE;
			}

			printf("usb-storage is present\n");
			env_set("bootcmd", "usb_storage_update");
			wdt0_enable(0);
			return USB_STOR_MODE;
		}
	}
    return CMD_UNDEFINED_MODE;
}
#endif

static s_boot_func_array boot_func_array[BOOTMODE_FUN_NUM] = {
#if defined CONFIG_AXERA_MEMORY_DUMP_SD || CONFIG_AXERA_MEMORY_DUMP_EMMC || CONFIG_AXERA_MEMORY_DUMP_USB_STORAGE
	sysdump_mode,
#endif
#ifdef CONFIG_CMD_AXERA_SDUPDATE
	sd_update_mode,
#endif
	usb_update_mode,
	uart_update_mode,
#ifdef CONFIG_CMD_AXERA_TFTP_OTA
	tftp_update_mode,
#endif
#ifdef CONFIG_CMD_AXERA_USB_STOR_UPDATE
	usb_stor_mode,
#endif
#ifdef CONFIG_CMD_AXERA_SDBOOT
	sd_boot_mode,
#endif
	0,
};

int setup_boot_mode(void)
{

	int i = 0;
	boot_mode_t boot_mode;
	struct boot_mode_info *const boot_info = get_dl_and_boot_info();
	if (boot_info->magic != BOOT_MODE_ENV_MAGIC) {
		printf("boot_mode magic error\n");
		return -1;
	}
	memcpy(&boot_info_data, boot_info, sizeof(boot_mode_info_t));

	for (i = 0; i < BOOTMODE_FUN_NUM - 1; i++) {
		if (0 == boot_func_array[i]) {
			#if defined CONFIG_SUPPORT_RECOVERY || !defined CONFIG_BOOT_OPTIMIZATION_SUPPORT
				env_set("bootcmd", "axera_boot");
				printf("enter normal boot mode\n");
			#else
				/* #define BOOT_KERNEL_FAIL  BIT(7)
				* #define BOOT_DOWNLOAD     BIT(8)
				* #define BOOT_RECOVERY     BIT(11)
				* Check whether bit7 and bit8 are 1 */
				writel(BOOT_KERNEL_FAIL | BOOT_DOWNLOAD, TOP_CHIPMODE_GLB_BACKUP0_CLR);
				set_wdt0_timeout(180);
				printf("===============================###############=========================================\n");
				printf(">>>>>> Go to the command line and wait for the upgrade (TF card or TFTP) ....... <<<<<<\n");
				printf("===============================###############==========================================\n");
				env_set("bootcmd", "help");
			#endif
			break;
		}
		boot_mode = boot_func_array[i] ();
		if (CMD_UNDEFINED_MODE == boot_mode) {
			continue;
		} else {
			printf("get boot mode in boot func array[%d]\n", i);
			break;
		}
	}
	printf("boot_info_data.mode = %d\n", boot_info_data.mode);

	return 0;
}
