# SPDX-License-Identifier: GPL-2.0+

dtb-$(CONFIG_TARGET_AX25_AE350) += ae350_32.dtb ae350_64.dtb
dtb-$(CONFIG_TARGET_SIFIVE_FU540) += hifive-unleashed-a00.dtb

dtb-$(CONFIG_ARCH_SUNXI) += \
	sun20i-d1-licheerv-86panel.dtb \
	sun20i-d1-licheerv-86panel-720x720.dtb \
	sun20i-d1-licheerv-dock.dtb \
	sun20i-d1-licheerv-dock-160x80.dtb \
	sun20i-d1-licheerv-dock-1.14.dtb \
	sun20i-d1-licheerv-dock-480x272.dtb \
	sun20i-d1-licheerv-dock-800x480.dtb \
	sun20i-d1-licheerv-dock-1024x600.dtb \
	sun20i-d1-licheerv-dock-mipi.dtb \
	sun20i-d1-mangopi-mq-800x480.dtb \
	sun20i-d1-h-mangopi-mq-pro.dtb \
	sun20i-d1-nezha.dtb \
	sun20i-d1-nezha-lcd.dtb \
	sun20i-d1-nezha-min.dtb

targets += $(dtb-y)

DTC_FLAGS += -R 4 -p 0x1000

PHONY += dtbs
dtbs: $(addprefix $(obj)/, $(dtb-y))
	@:

clean-files := *.dtb
