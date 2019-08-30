/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <soc/mstar/miu.h>

#define BIT(nr)			(1 << (nr))
#define PMINFO_BANK_MIU_ANA	0x8
#define PMINFO_BANK_MIU_DIG1	0xc
#define PMINFO_BANK_MIU_DIG	0x10
#define PMINFO_BANK_UART	0x14

.align 3

//r0 should be a pointer to a struct mstar_pm_info
//r1 will point at the pm_uart for debugging
//r4 is used to track the base of the block we're poking
//r5 is used for the value that is being loaded

.macro delay howlong=0x10000
	ldr   r5,=\howlong
1:
	subs  r5, #1
	bne   1b
.endm

.macro setbank infooffset
	ldr	r4, [r0, #\infooffset]
.endm

.macro setbyte offset value
	ldr	r5, =\value
	strb	r5, [r4, #\offset << 1]
.endm

.macro setword offset value
	ldr	r5, =\value
	str	r5, [r4, #\offset << 1]
.endm

.macro setword_defuq offset value
	ldr	r5, =\value
	str	r5, [r4, #\offset]
.endm

.macro setbits offset bits
	ldr	r5, [r4, #\offset << 1]
	orr	r5, r5, #\bits
	str	r5, [r4, #\offset << 1]
.endm

.macro clearbits offset bits
	movw r7, #\bits
	ldr r5, [r4, #\offset << 1]
	bic r5, r5, r7
	str r5, [r4, #\offset << 1]
.endm

.macro sendr6
	str	r6, [r1]
	delay
.endm

.macro putch ch
	ldr	r6, =\ch
	sendr6
.endm

.macro newline
	putch	'\n'
	putch	'\r'
.endm

.macro pollbit offset bit
2:
	ldr	r5, [r4, #\offset << 1]
	and	r5, r5, #\bit
	cmp	r5, #\bit
	bne	2b
.endm

.macro checkdigit what ascii
	ldr	r6, =\ascii
	cmp	r8, #\what
	beq	4f
.endm

.macro printhexdigit
	checkdigit 0 '0'
	checkdigit 1 '1'
	checkdigit 2 '2'
	checkdigit 3 '3'
	checkdigit 4 '4'
	checkdigit 5 '5'
	checkdigit 6 '6'
	checkdigit 7 '7'
	checkdigit 8 '8'
	checkdigit 9 '9'
	checkdigit 10 'a'
	checkdigit 11 'b'
	checkdigit 12 'c'
	checkdigit 13 'd'
	checkdigit 14 'e'
	checkdigit 15 'f'
	ldr        r6, ='?'
4:
	sendr6
.endm

.macro printword offset
	ldr	r9, [r4, #\offset << 1]
	ldr	r7,=16
5:
	subs	r7, #4
	lsr	r8, r9, r7
	and	r8, r8, #0xf
	printhexdigit
	cmp	r7, #0
	bne	5b
	newline
.endm

//This must run in SRAM
ENTRY(msc313_suspend_imi)
	// setup r1 to point to the pm_uart
	ldr r1, [r0, PMINFO_BANK_UART]

	// First block all access except one in group 0, cpu?
	// miu-2
	setbank		PMINFO_BANK_MIU_DIG
	setword_defuq	MIU_DIG_GROUP0_REQ_MASK, 0xfffe
	setword_defuq	MIU_DIG_GROUP1_REQ_MASK, 0xffff
	setword_defuq	MIU_DIG_GROUP2_REQ_MASK, 0xffff
	setword_defuq	MIU_DIG_GROUP3_REQ_MASK, 0xffff
	// miu-1
	setbank		PMINFO_BANK_MIU_DIG1
	setword		0x06, 0xffff
	setword		0x26, 0xffff
	// wait for everything to finish
	delay

	// miu-2
	setbank		PMINFO_BANK_MIU_DIG
	// single command mode register
	setword_defuq	MIU_DIG_MRX, 0x0400
	// turn off auto refresh and ODT
	setbits		MIU_DIG_CNTRL0, MIU_DIG_CNTRL0_REFRESH_OFF
	clearbits	MIU_DIG_CNTRL0, MIU_DIG_CNTRL0_ODT

	// trigger all banks precharge to put the ddr2 back into idle
	// clear command and auto init
	clearbits 	MIU_DIG_CNTRL0, MIU_DIG_CNTRL0_CMD_RAS | \
					MIU_DIG_CNTRL0_CMD_CAS | \
					MIU_DIG_CNTRL0_CMD_WE | \
					MIU_DIG_CNTRL0_CMD_ISSUE | \
					MIU_DIG_CNTRL0_INIT_MIU
	// cas high trigger command
	setbits		MIU_DIG_CNTRL0, MIU_DIG_CNTRL0_CMD_CAS | \
		      			MIU_DIG_CNTRL0_CMD_ISSUE
	// wait for command to complete
	pollbit		MIU_DIG_CNTRL0, MIU_DIG_CNTRL0_CMD_DONE
	// clear command issue and casz
	clearbits	MIU_DIG_CNTRL0, MIU_DIG_CNTRL0_CMD_CAS | \
					MIU_DIG_CNTRL0_CMD_ISSUE
	// we# high
	setbits		MIU_DIG_CNTRL0, MIU_DIG_CNTRL0_CMD_WE | \
					MIU_DIG_CNTRL0_CMD_ISSUE
	// wait for command to complete
	pollbit		MIU_DIG_CNTRL0, MIU_DIG_CNTRL0_CMD_DONE
	// clear command issue and we
	clearbits	MIU_DIG_CNTRL0, MIU_DIG_CNTRL0_CMD_WE | \
					MIU_DIG_CNTRL0_CMD_ISSUE
	// mask all requests
	setword_defuq	MIU_DIG_GROUP0_REQ_MASK, 0xffff
	// set self refresh
	setbits		MIU_DIG_CNTRL0, MIU_DIG_CNTRL0_SELF_REFRESH

	// Now that the DDR is doing it's own thing disable
	// power everything down to save energy

    	// miu-2
    	setbank		0x10
    	setbits		0x04, 0xf000 // mclk delay etc
	// miu-0
	setbank		PMINFO_BANK_MIU_ANA
	setbits		0x00, 0x0018 // set gpio mode, output highz
	setword		0x54, 0xc070 // unknown
	setword		0x08, 0x0000 // clear rx en

	// not sure what this does. causes the cpu to lock up though
	//wriu    0x3c24 0x30    //reg_pwoff_rst0_en, reg_pwoff_rst1_en
	//ldr r4, =0x1f007800
	//setbyte 0x24 0x30

	// Unlock the power down stuff
	setbank		0x0
	setbyte		0x38, 0x0c
	setword		0x24, 0xbabe
	setbyte		0x6e, 0xa5 //commented out in new sdk

	// Finally set pm_gpio4 to turn the lights off
	setbank		0x4
	setbyte		0x08, 0x10

	putch '!'
	newline

ENDPROC(msc313_suspend_imi)
.ltorg

.align 3
ENTRY(msc313_resume_imi)
	// setup r1 to point to the pm_uart
	ldr r1, pm_uart
	putch 'r'

	b cpu_resume

pm_uart: .long 0x1f221000

ENDPROC(msc313_resume_imi)
.ltorg
