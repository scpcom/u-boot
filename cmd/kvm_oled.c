// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) Sipeed 2024-2025. All rights reserved.
 */

#include <config.h>
#include <command.h>
#include <common.h>
#include <log.h>
#include <mmio.h>
#include <stdlib.h>
#include <linux/delay.h>
#include <cpu_func.h>

#include <asm/io.h>
#include "part.h"
#include "fs.h"
#include "kvm_oled_ctrl.h"

uint8_t oled_exists = 0;

static int get_value_from_header(const char *filepath, const char *key, char *buffer, size_t buff_len)
{
#if defined(CONFIG_NAND_SUPPORT) || defined(CONFIG_SPI_FLASH)
    const char *storage = "mmc 0:1"; // 假设 FAT 文件在 mmc 0:1
	// const char *set_cmd = "mmc dev 0:1";
#elif defined(CONFIG_EMMC_SUPPORT)
	const char *storage = "mmc 1:1"; // 假设 FAT 文件在 mmc 1:1
	// const char *set_cmd = "mmc dev 1:1";
#else
	const char *storage = "mmc 0:1";
	// const char *set_cmd = "mmc dev 0:1";
#endif
    char *file_content;
    char *line, *found_key, *value;
    uint32_t filesize = 0;
    int ret;

    // // 清空输出 buffer
    // memset(buffer, 0, buff_len);
	// 设置当前 MMC 设备和分区
    // snprintf(buffer, buff_len, set_cmd);
    // ret = run_command(buffer, 0);
    // if (ret) {
    //     printf("Failed to set MMC device\n");
    //     return -1;
    // }

    // 加载文件到固定内存区域 HEADER_ADDR
    snprintf(buffer, buff_len, "fatload %s %p %s", storage, (void *)HEADER_ADDR, filepath);
    ret = run_command(buffer, 0);
    if (ret) {
        printf("Failed to load file: %s\n", filepath);
        return -1;
    }

    // 获取文件大小（存储在环境变量 filesize 中）
    filesize = env_get_ulong("filesize", 16, 0);
    if (filesize == 0 || filesize > 0x10000) { // 限制最大文件大小，防止内存溢出
        printf("Invalid file size for %s\n", filepath);
        return -1;
    }

    // 确保文件内容以 NULL 结尾（便于字符串处理）
    file_content = (char *)HEADER_ADDR;
    file_content[filesize] = '\0';

    // 查找 key=value 的行
    line = strtok(file_content, "\n");
	buffer[0] = 0;
    while (line) {
        // 分割 key 和 value
        char*split = strstr(line, "=");
		if(split)
		{
			char *start = line;
			while(*(start++) == ' '){}
			found_key = start - 1;
			char *end = split;
			while(*(--end) == ' '){}
			*(end+1) = '\0';
			value = "";
			start = split + 1;
			while(1)
			{
				if (*start == ' ')
				{
					++start;
					continue;
				}
				value = start;
				break;
			}
			end = value;
			while(1)
			{
				if(*end == 0 || *end == '\r' || *end == ' ')
				{
					*end = 0;
					break;
				}
				if(end >= file_content + filesize)
					break;
				++end;
			}

			// 检查 key 是否匹配
			if (strcmp(found_key, key) == 0) {
				// 确保 value 非空并拷贝到输出 buffer
				int str_len = strlen(value);
				if(str_len == 0)
				{
					buffer[0] = 0;
					return 0;
				}
				else if (str_len < buff_len) {
					strncpy(buffer, value, buff_len - 1);
					buffer[str_len] = '\0'; // 确保以 NULL 结尾
					return 0; // 找到 key 且 value 非空
				}
				else
				{
					printf("key %s value too long: %s\n", found_key, value);
					return -1;
				}
				break;
        	}
		}
        line = strtok(NULL, "\n");
    }

    return -2; // 未找到 key 或 value 为空
}

static bool fat_file_exists(const char *filename)
{
	struct blk_desc *dev_desc;
#if defined(CONFIG_NAND_SUPPORT) || defined(CONFIG_SPI_FLASH)
	const char *dev_part = "0:1";
	int dev = 0;
#elif defined(CONFIG_EMMC_SUPPORT)
	const char *dev_part = "1:1";
	int dev = 1;
#else
	const char *dev_part = "0:1";
	int dev = 0;
#endif

    dev_desc = blk_get_dev("mmc", dev);
    if (!dev_desc || dev_desc->type == DEV_TYPE_UNKNOWN) {
        printf("Cannot find mmc device %s\n", dev_part);
        return -ENODEV;
    }

    if (fs_set_blk_dev("mmc", dev_part, FS_TYPE_FAT)) {
        printf("Failed to set fs device mmc %s\n", dev_part);
        return -EINVAL;
    }

    if (fs_exists(filename)) {
        printf("File %s exists on mmc %s\n", filename, dev_part);
        return true;
    } else {
        printf("File %s not found on mmc %s\n", filename, dev_part);
        return false;
    }
}

static void oled_show_string(char* str)
{
	OLED_state = oled_probe();
	if(!OLED_state){
		return;
	}

	OLED_Init();
	OLED_ColorTurn(0);              //0正常显示 1 反色显示
	OLED_DisplayTurn(0);    //0正常显示 1 屏幕翻转显示
	OLED_Clear();

	mdelay(OLED_DELAY);

	if(kvm_hw_ver != 2){
		OLED_Clear();
		// OLED_Revolve();
		//OLED_ShowLogo();
		//OLED_ShowSipeedLogo();
	} else {
		OLED_Revolve();
		//OLED_Showline_1();
		//OLED_ShowSipeedLogo();
	}

	OLED_ShowString(0, 1, str, 16);

	mdelay(OLED_DELAY);
}

void kvm_hw_init(void)
{
	uint8_t kvm_alpha = 0;
	uint8_t kvm_beta_pcie = 0;

	char *kvm_hw = NULL;
	char buff[255];
	if(get_value_from_header("kvm", "hw", buff, sizeof(buff)) == 0)
		kvm_hw = buff;
	else
		kvm_hw = env_get("kvm_hw");

	if (!kvm_hw)
		return;

	if (strcmp(kvm_hw,"alpha") == 0) {
		kvm_alpha = 1;
		kvm_hw_ver = 0;
	} else if (strcmp(kvm_hw,"beta") == 0) {
		kvm_beta_pcie = 1;
		kvm_hw_ver = 1;
	} else if (strcmp(kvm_hw,"pcie") == 0) {
		kvm_beta_pcie = 1;
		kvm_hw_ver = 2;
	}

	if (kvm_alpha) {
		mmio_write_32(0x030010D0, 0x2); // I2C1_SCL
		mmio_write_32(0x030010DC, 0x2); // I2C1_SDA
		mmio_write_32(0x030010D4, 0x3); // GPIOE 19 OLED_RST
	}

	if (kvm_beta_pcie) {
		mmio_write_32(0x0300103C, 0x3); // GPIOA 15 I2C5_SCL (bitbang)
		mmio_write_32(0x03001058, 0x3); // GPIOA 27 I2C5_SDA (bitbang)
		mmio_write_32(0x03001050, 0x3); // GPIOA 22 OLED_RST
	}

	if (kvm_alpha || kvm_beta_pcie) {
		mmio_write_32(0x03001070, 0x2); // GPIOA 28 UART2 TX
		mmio_write_32(0x03001074, 0x2); // GPIOA 29 UART2 RX
		mmio_write_32(0x03001068, 0x6); // GPIOA 18 UART1 RX
		mmio_write_32(0x03001064, 0x6); // GPIOA 19 UART1 TX
	}

	if (!kvm_alpha && !kvm_beta_pcie) {
		return;
	}

	char *_bootargs = NULL;
	char new_bootargs[256] = {0};
	_bootargs = env_get("othbootargs");
	memcpy(new_bootargs, _bootargs, strlen(_bootargs));

	char kvm_hw_arg[64] = {0};
	printf("kvm_hw=%s\n", kvm_hw);
	sprintf(kvm_hw_arg, " kvm_hw=%s", kvm_hw);
	memcpy(new_bootargs + strlen(new_bootargs), kvm_hw_arg, strlen(kvm_hw_arg));
	printf("new_othbootargs[%ld]: %s\n", strlen(new_bootargs), new_bootargs);
	env_set("othbootargs", new_bootargs);

	char *kvm_oled = NULL;
	if(get_value_from_header("kvm", "oled", buff, sizeof(buff)) == 0)
		kvm_oled = buff;
	else
		kvm_oled = env_get("kvm_oled");

	if (!kvm_oled)
		return;

	if (strcmp(kvm_oled,"exists") == 0) {
		oled_exists = 1;
	}

	if (!oled_exists) {
		return;
	}

	char kvm_oled_arg[64] = {0};
	printf("kvm_oled=%s\n", kvm_oled);
	sprintf(kvm_oled_arg, " kvm_oled=%s", kvm_oled);
	memcpy(new_bootargs + strlen(new_bootargs), kvm_oled_arg, strlen(kvm_oled_arg));
	printf("new_othbootargs[%ld]: %s\n", strlen(new_bootargs), new_bootargs);
	env_set("othbootargs", new_bootargs);

	oled_show_string("Loading");
}

/***************************************************/
static int do_startoled(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc < 4)
		return CMD_RET_USAGE;

	// check boot key pressed
	// writel(0x00000003, 0x03001078);
	uint32_t val = readl((const volatile void *)0x03020050);
	int boot_key = (val >> 30) & 0x01;  // GPIOA30
	if (boot_key == 0) //  boot key pressed
	{
		char *_bootargs = NULL;
		char new_bootargs[256] = {0};
		printf("boot key pressed\n");
		_bootargs = env_get("othbootargs");
		memcpy(new_bootargs, _bootargs, strlen(_bootargs));
		char *boot_key_arg = " boot_key=1";
		memcpy(new_bootargs + strlen(new_bootargs), boot_key_arg, strlen(boot_key_arg));
		printf("new_othbootargs[%ld]: %s\n", strlen(new_bootargs), new_bootargs);
		env_set("othbootargs", new_bootargs);
		if (fat_file_exists("logo_upgrade.jpeg"))
			env_set("logo", "logo_upgrade.jpeg");
		else
			env_set("logo", "logo.jpeg");
	}
	else
	{
		env_set("logo", "logo.jpeg");
	}

	kvm_hw_init();

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(startoled
	, 4, 0, do_startoled
	, "open oled device with a certain interface."
	, "    - startoled [bus dev reset]"
);

/***************************************************/
static int do_stopoled(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc < 2)
		return CMD_RET_USAGE;
	if (!oled_exists)
		return CMD_RET_SUCCESS;

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(stopoled
	, 2, 0, do_stopoled
	, "close interface of oled device."
	, "    - stopoled [dev]"
);

