// SPDX-License-Identifier: GPL-2.0+
/* USBC
 *
 * otg - musb controller
 * uhc - ehci host controller
 *
 * 0x00 - reset control
 * reset value	msc313	0x228
 * 		ssc8336	0x200
 *                    	                     |    9    |     8
 *                                           |vbusvalid| vbus_sel
 *    6    |    5    |      3      |    2    |    1    |    0
 * otg xiu | uhc xiu | reg suspend | otg_rst | uhc_rst | usb_rst
 *
 * 0x04 - port control
 *       4       |          3         |      1       |       0
 * idpullup_ctrl | interrupt polarity | portctrl_otg | portctrl_uhc
 *
 * 0x08 - interrupt enable
 *    3     |  2   |  1   |  0
 *   id     | bval | aval | vbus
 *
 * 0x0c - interrupt status
 * - order same as above
 *
 * 0x10 - utmi signal
 * 15 - 13 |    12    |    11   |    10    |    9    |    8
 *         | rxactive | rxvalid | rxvalidh | rxerror | txready
 *  7 - 6  |          |         |          |         |    0
 *  dm/dp  |          |         | bvalid   | avalid  | vbusvalid
 *
 * 0x14 - power enable
 * 0x18 - power status
 *
 * MIU select?
 * 0x28 - miu sel 0 - set to 0x00 by the vendor code
 * 0x2c - miu sel 1 - set to 0xff by the vendor code
 * 0x2d - miu sel 2 - set to 0xff by the vendor code
 * 0x30 - miu sel 3 - set to 0xff by the vendor code
 * 0x31
 *        0
 *  miu partition   - set to 1 by the vendor code
 *  
 * 0x101 - vendor sdk says setting bits 0,1,2,3 and 7 here "improve the efficiency of usb access miu when system is busy"
 */

#define MSTAR_USBC_REG_RSTCTRL	0x00
#define MSTAR_USBC_REG_PRTCTRL	0x04
#define MSTAR_USBC_REG_INTEN	0x08
#define MSTAR_USBC_REG_INTSTS	0x0c

#define MSTAR_USBC_REG_MIUCFG0	0x28 // miu sel 0
#define MSTAR_USBC_REG_MIUCFG1	0x2c // miu sel 1 + 2
#define MSTAR_USBC_REG_MIUCFG2	0x30 // miu sel 3 + miu partition

#define MSTAR_RSTCTRL_USB_RST		(1 << 0)
#define MSTAR_RSTCTRL_UHC_RST		(1 << 1)
#define MSTAR_RSTCTRL_OTG_RST		(1 << 2)
#define MSTAR_RSTCTRL_REG_SUSPEND	(1 << 3)
#define MSTAR_RSTCTRL_UHC_XIU		(1 << 5)
#define MSTAR_RSTCTRL_OTG_XIU		(1 << 6)

#define MSTAR_RSTCTRL_VBUSVALID		9

#define MSTAR_PRTCTRL_UHC			(1 << 0)
#define MSTAR_PRTCTRL_OTG			(1 << 1)

#define MSTAR_USBC_INT_VBUS	0
#define MSTAR_USBC_INT_AVAL	1
#define MSTAR_USBC_INT_BVAL	2
#define MSTAR_USBC_INT_ID	3
#define MSTAR_USBC_INT_MASK	(BIT(MSTAR_USBC_INT_VBUS)|BIT(MSTAR_USBC_INT_AVAL)|BIT(MSTAR_USBC_INT_BVAL)|BIT(MSTAR_USBC_INT_ID))
