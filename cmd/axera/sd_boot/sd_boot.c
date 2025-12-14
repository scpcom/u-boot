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
#include <cpu_func.h>
#include "../../legacy-mtd-utils.h"
#include "axera_update.h"
#include "../boot/axera_boot.h"
#ifdef CONFIG_CMD_AXERA_GZIPD
#include "../gzipd/ax_gzipd_api.h"

extern int gzip_decompress_image(void *src, void *dest, u32 size);
#endif

extern struct boot_mode_info boot_info_data;

#define READ_IMG_SIZE (5 * 1024 *1024)

#ifndef CONFIG_ARM64
void ax_boot_kernel(char *img_addr,char *dtb_addr);
#endif

#ifdef CONFIG_CMD_AXERA_KERNEL_LZMA
int lzma_decompress_image(void *src, void *dest, u32 size);
#endif


int do_sd_boot(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char boot_cmd[50];
	int cnt, last_size;
	int j;
	struct img_header *boot_img_header = NULL;
#ifdef CONFIG_CMD_AXERA_GZIPD || CONFIG_CMD_AXERA_KERNEL_LZMA
	char *img_addr = (char *)KERNEL_IMAGE_COMPRESSED_ADDR;
	char *dtb_addr = (char *)DTB_IMAGE_COMPRESSED_ADDR;
	u64 kernel_image_size;
	u64 dtb_image_size;
#else
	char *img_addr = (char *)KERNEL_IMAGE_ADDR;
	char *dtb_addr = (char *)DTB_IMAGE_ADDR;
#endif
	int dtb_size;

	if (boot_info_data.mode != SD_BOOT_MODE)
		return 0;

	printf("now enter sd boot\n");

	env_set("bootargs", BOOTARGS_SD);

	sprintf(boot_cmd, "fatload mmc 1:1 0x%x kernel.img 0x%x", SD_BOOT_IMAGE_ADDR, SECBOOT_HEADER_SIZE);
	run_command_list(boot_cmd, -1, 0);

	boot_img_header = (struct img_header *)SD_BOOT_IMAGE_ADDR;
	cnt = boot_img_header->img_size / READ_IMG_SIZE;
	last_size = boot_img_header->img_size % READ_IMG_SIZE;
	j = 0;
#ifdef CONFIG_CMD_AXERA_GZIPD || CONFIG_CMD_AXERA_KERNEL_LZMA
	kernel_image_size = boot_img_header->img_size;
#endif
	printf("kernel size is %d bytes\n", boot_img_header->img_size);

	memset( (void *)img_addr, 0, boot_img_header->img_size);

	while (cnt > 0 && cnt--) {
		memset(boot_cmd, 0, sizeof(boot_cmd));
		memset( (void *)SD_BOOT_IMAGE_ADDR, 0, READ_IMG_SIZE);
		sprintf(boot_cmd, "fatload mmc 1:1 0x%x kernel.img 0x%x 0x%x", SD_BOOT_IMAGE_ADDR, READ_IMG_SIZE, SECBOOT_HEADER_SIZE + READ_IMG_SIZE * j);
		run_command_list(boot_cmd, -1, 0);

		memmove((void *)img_addr + READ_IMG_SIZE * j, (void *)SD_BOOT_IMAGE_ADDR, READ_IMG_SIZE);
		j++;
	}

	if (last_size) {
		memset( (void *)SD_BOOT_IMAGE_ADDR, 0, last_size);
		sprintf(boot_cmd, "fatload mmc 1:1 0x%x kernel.img 0x%x 0x%x", SD_BOOT_IMAGE_ADDR, last_size, SECBOOT_HEADER_SIZE + READ_IMG_SIZE * j);
		run_command_list(boot_cmd, -1, 0);
		memmove((void *)img_addr + READ_IMG_SIZE * j, (void *)SD_BOOT_IMAGE_ADDR, last_size);
	}
	printf("sd boot: kernel img read %d finish\n", j * READ_IMG_SIZE + last_size);

	memset(boot_cmd, 0, sizeof(boot_cmd));
	sprintf(boot_cmd, "fatload mmc 1:1 0x%x dtb.img 0x%x", SD_BOOT_IMAGE_ADDR, SECBOOT_HEADER_SIZE);
	run_command_list(boot_cmd, -1, 0);
	boot_img_header = (struct img_header *)SD_BOOT_IMAGE_ADDR;
	dtb_size = boot_img_header->img_size;
#ifdef CONFIG_CMD_AXERA_GZIPD || CONFIG_CMD_AXERA_KERNEL_LZMA
	dtb_image_size = dtb_size;
#endif

	memset(boot_cmd, 0, sizeof(boot_cmd));
	memset( (void *)dtb_addr, 0, dtb_size);
	memset( (void *)SD_BOOT_IMAGE_ADDR, 0, dtb_size);
	sprintf(boot_cmd, "fatload mmc 1:1 0x%x dtb.img ", SD_BOOT_IMAGE_ADDR);
	run_command_list(boot_cmd, -1, 0);
	memmove((void *)dtb_addr, (void *)(SD_BOOT_IMAGE_ADDR + SECBOOT_HEADER_SIZE), dtb_size);
	printf("sd boot: dtb img read %d finish\n", dtb_size);

#ifdef CONFIG_CMD_AXERA_GZIPD
	flush_dcache_all();
	if (gzip_decompress_image((void *)KERNEL_IMAGE_COMPRESSED_ADDR, (void *)KERNEL_IMAGE_ADDR, kernel_image_size)) {
		pr_err("kernel image decompress failed\n");
		return -1;
	}
	if (gzip_decompress_image((void *)DTB_IMAGE_COMPRESSED_ADDR,  (void *)DTB_IMAGE_ADDR, dtb_image_size)) {
		pr_err("dtb image decompress failed\n");
		return -1;
	}
	invalidate_dcache_all();
#endif

#ifdef CONFIG_CMD_AXERA_KERNEL_LZMA
	printf("unzip kernel...\n");
	flush_dcache_all();
	if (lzma_decompress_image((void *)KERNEL_IMAGE_COMPRESSED_ADDR, (void *)KERNEL_IMAGE_ADDR, kernel_image_size)) {
		pr_err("kernel image decompress failed\n");
		return -1;
	}

	if (gzip_decompress_image((void *)DTB_IMAGE_COMPRESSED_ADDR,  (void *)DTB_IMAGE_ADDR, dtb_image_size)) {
		pr_err("dtb image decompress failed\n");
		return -1;
	}
	invalidate_dcache_all();
#endif

	memset(boot_cmd, 0, sizeof(boot_cmd));
#ifdef CONFIG_ARM64
	printf("boot arm64 Image kernel\n");
	sprintf(boot_cmd, "booti 0x%lx - 0x%lx", (unsigned long)KERNEL_IMAGE_ADDR, (unsigned long)DTB_IMAGE_ADDR);
#else
	printf("boot arm32 Image kernel\n");
	ax_boot_kernel((void *)(unsigned long)KERNEL_IMAGE_ADDR, (void *)(unsigned long)DTB_IMAGE_ADDR);
#endif
	printf("boot cmd is: %s\n", boot_cmd);
	run_command_list(boot_cmd, -1, 0);

	return 0;
}

U_BOOT_CMD(sd_boot, 1, 0, do_sd_boot,
	   "sd boot", "axera enter sd boot mode\n" "it is used for sd boot to kernel\n");
