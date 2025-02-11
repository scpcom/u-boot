// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Western Digital Corporation or its affiliates
 *
 */

#include <common.h>
#include <fdt_support.h>
#include <mapmem.h>
#include <fdtdec.h>

DECLARE_GLOBAL_DATA_PTR;

/**
 * riscv_fdt_copy_resv_mem_node() - Copy reserve memory node entry
 * @src: Pointer to the source device tree from which reserved memory node
 *	 needs to be copied.
 * @dst: Pointer to the destination device tree to which reserved memory node
 *	 needs to be copied.
 *
 * Return: 0 on success or if source doesn't have reserved memory node.
 *	   Error if copy process failed.
 */
int riscv_fdt_copy_resv_mem_node(const void *src, void *dst)
{
	u32 phandle;
	struct fdt_memory pmp_mem;
	fdt_addr_t addr;
	fdt_size_t size;
	int offset, node, err, rmem_offset;
	bool nomap = true;
	char basename[32] = {0};
	int bname_len;
	int max_len = sizeof(basename);
	const char *name;
	char *temp;

	offset = fdt_path_offset(src, "/reserved-memory");
	if (offset < 0) {
		printf("No reserved memory region found in source FDT\n");
		return 0;
	}

	fdt_for_each_subnode(node, src, offset) {
		name = fdt_get_name(src, node, NULL);

		addr = fdtdec_get_addr_size_auto_noparent(src, node,
							  "reg", 0, &size,
							  false);
		if (addr == FDT_ADDR_T_NONE) {
			debug("failed to read address/size for %s\n", name);
			continue;
		}
		strncpy(basename, name, max_len);
		temp = strchr(basename, '@');
		if (temp) {
			bname_len = strnlen(basename, max_len) - strnlen(temp,
								       max_len);
			*(basename + bname_len) = '\0';
		}
		pmp_mem.start = addr;
		pmp_mem.end = addr + size - 1;
		err = fdtdec_add_reserved_memory(dst, basename, &pmp_mem,
						 &phandle);
		if (err < 0) {
			printf("failed to add reserved memory: %d\n", err);
			return err;
		}
		if (!fdt_getprop(src, node, "no-map", NULL))
			nomap = false;
		if (nomap) {
			rmem_offset = fdt_node_offset_by_phandle(dst, phandle);
			fdt_setprop_empty(dst, rmem_offset, "no-map");
		}
	}

	return 0;
}

/**
 * riscv_board_reserved_mem_fixup() - Fix up reserved memory node for a board
 * @fdt: Pointer to the device tree in which reserved memory node needs to be
 *	 added.
 *
 * In RISC-V, any board compiled with OF_SEPARATE needs to copy the reserved
 * memory node from the device tree provided by the firmware to the device tree
 * used by U-Boot. This is a common function that individual board fixup
 * functions can invoke.
 *
 * Return: 0 on success or error otherwise.
 */
int riscv_board_reserved_mem_fixup(void *fdt)
{
	int err;
	void *src_fdt_addr;

	src_fdt_addr = map_sysmem(gd->arch.firmware_fdt_addr, 0);
	err = riscv_fdt_copy_resv_mem_node(src_fdt_addr, fdt);
	if (err < 0)
		return err;

	return 0;
}

#ifdef CONFIG_OF_BOARD_FIXUP
int board_fix_fdt(void *fdt)
{
	int err;

	err = riscv_board_reserved_mem_fixup(fdt);
	if (err < 0) {
		printf("failed to fixup DT for reserved memory: %d\n", err);
		return err;
	}

	return 0;
}
#endif

int arch_fixup_fdt(void *blob)
{
	int err;
#ifdef CONFIG_EFI_LOADER
	u32 size;
	int chosen_offset;

	size = fdt_totalsize(blob);
	err  = fdt_open_into(blob, blob, size + 32);
	if (err < 0) {
		printf("Device Tree can't be expanded to accommodate new node");
		return err;
	}
	chosen_offset = fdt_path_offset(blob, "/chosen");
	if (chosen_offset < 0) {
		err = fdt_add_subnode(blob, 0, "chosen");
		if (err < 0) {
			printf("chosen node can not be added\n");
			return err;
		}
	}
	/* Overwrite the boot-hartid as U-Boot is the last stage BL */
	fdt_setprop_u32(blob, chosen_offset, "boot-hartid", gd->arch.boot_hart);
#endif


#ifdef CONFIG_OF_LIBFDT
	bd_t *bd = gd->bd;
	int bank;
	u64 dram_start[CONFIG_NR_DRAM_BANKS];
	u64 dram_size[CONFIG_NR_DRAM_BANKS];
	for (bank = 0; bank < CONFIG_NR_DRAM_BANKS; bank++) {
		dram_start[bank] = bd->bi_dram[bank].start;
		dram_size[bank] = bd->bi_dram[bank].size;
	err = fdt_fixup_memory_banks(blob, dram_start, dram_size, CONFIG_NR_DRAM_BANKS);
	if (err)
		return err;
#endif
	}
	/* Copy the reserved-memory node to the DT used by OS */
	err = riscv_fdt_copy_resv_mem_node(gd->fdt_blob, blob);
	if (err < 0)
		return err;

	return 0;
}
