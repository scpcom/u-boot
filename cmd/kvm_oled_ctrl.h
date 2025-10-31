#ifndef OLED_CTRL_H_
#define OLED_CTRL_H_

#if 0
#include "maix_basic.hpp"
#include "maix_time.hpp"
#include "maix_gpio.hpp"
#include "maix_pinmap.hpp"
#include "maix_i2c.hpp"
#include <fcntl.h>
#include <unistd.h>
#endif
#include <string.h>
#include <stdio.h>
//include <pthread.h>

#define OLED_DELAY 					1000

#define OLED_DISABLE 	0
#define OLED_ENABLE		1
#define OLED_ADDR		0x3D
#define OLED_PCIe_ADDR  0x3C
#define OLED_CMD		0x00
#define OLED_DATA		0x40

#define HDMI_STATE      0x01
#define HID_STATE       0x02
#define ETH_STATE       0x04
#define WIFI_STATE      0x08

#define KVM_INIT        0x00
#define KVM_ETH_IP      0x01
#define KVM_WIFI_IP     0x02
#define KVM_HDMI_STATE  0x03
#define KVM_HDMI_RES    0x04
#define KVM_STEAM_TYPE  0x05
#define KVM_STEAM_FPS   0x06
#define KVM_JPG_QLTY    0x07
#define KVM_CPU_IDLE    0x08

#define KVM_RES_none    0x00
#define KVM_RES_480P    0x01
#define KVM_RES_600P    0x02
#define KVM_RES_720P    0x03
#define KVM_RES_1080P   0x04

#define KVM_TYPE_none   0x00
#define KVM_TYPE_MJPG   0x01
#define KVM_TYPE_H264   0x02

#define AlignRightEND   127
#define AlignRightEND_P 63

extern uint8_t OLED_state;
extern uint8_t kvm_hw_ver;

int oled_probe(void);
void OLED_Clear(void);
void OLED_Fill(void);
void OLED_Init(void);
void OLED_Revolve(void);
void OLED_ShowState(uint8_t x,uint8_t y,char chr,uint8_t size);
void OLED_DisplayTurn(uint8_t i);
void OLED_ColorTurn(uint8_t i);
void OLED_ShowError(uint8_t x,uint8_t y,uint8_t _state);
void OLED_ShowCharTurn(uint8_t x,uint8_t y,char chr,uint8_t sizey);
void OLED_ShowNum(uint8_t x, uint8_t y, uint8_t num, uint8_t len, uint8_t sizey);
void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t sizey);
void OLED_ShowStringTurn(uint8_t x, uint8_t y, char *chr, uint8_t sizey);
void OLED_ShowStringtoend(uint8_t x, uint8_t y, char *chr, uint8_t sizey, uint8_t end);
void OLED_ShowString_AlignRight(uint8_t x_end, uint8_t y, char *chr, uint8_t size);
void OLED_ROW(uint8_t _EN);
void OLED_ShowLogo(void);
void OLED_ShowSipeedLogo(void);
void OLED_Showline(void);
void OLED_Showline_1(void);
void OLED_ShowIMG(uint8_t x,uint8_t y,char *chr,uint8_t width,uint8_t height);
void OLED_ShowKVMStreamState(uint8_t kvm_state_s, void* pdata);
void OLED_Show_Res(uint16_t _w, uint16_t _h);
void OLED_ShowKVMState(uint8_t _Interface, int8_t _EN);
void OLED_Show_Network_Error(uint8_t _state);

#endif // OLED_CTRL_H_
