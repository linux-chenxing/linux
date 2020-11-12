#ifndef __MSC313_H
#define __MSC313_H

/* Bank 0 */
#define BANK0		0x0
/* Bank 1 */
#define BANK1		0x200
#define REG_ANARST	(BANK1 + 0x8)
#define REG_ADCCLKSEL	(BANK1 + 0x74)
#define REG_REF		(BANK1 + 0x174)
#define REG_ADCPL	(BANK1 + 0x198)
#define REG_LDO		(BANK1 + 0x1f8)
/* Bank 2 */
#define BANK2		0x400
#define REG_200GAT	(BANK2 + 0x60)
#define REG_TX1		(BANK2 + 0x74)
#define REG_CLKOADCSEL	(BANK2 + 0x114)
#define REG_SADC	(BANK2 + 0x140)
#define REG_100GAT	(BANK2 + 0x188)
#define REG_TX2		(BANK2 + 0x1e0)
#define REG_INTR	(BANK2 + 0xe8)

static const struct reg_field anarst = REG_FIELD(REG_ANARST, 5, 5);
static const struct reg_field sadcpd = REG_FIELD(REG_SADC, 12, 13);
static const struct reg_field adcplpd = REG_FIELD(REG_ADCPL, 4, 4);
static const struct reg_field refpd = REG_FIELD(REG_REF, 14, 15);
static const struct reg_field txpd1 = REG_FIELD(REG_TX1, 0, 7);
static const struct reg_field txpd2 = REG_FIELD(REG_TX2, 8, 15);
static const struct reg_field clkoadcsel = REG_FIELD(REG_CLKOADCSEL, 0, 7);
static const struct reg_field adcclksel = REG_FIELD(REG_ADCCLKSEL, 8, 8);
static const struct reg_field hundredgat = REG_FIELD(REG_100GAT, 14, 14);
static const struct reg_field twohundredgat = REG_FIELD(REG_200GAT, 4, 4);

struct msc313e_fields {
	struct regmap_field *anarst;
	struct regmap_field *sadcpd;
	struct regmap_field *adcplpd;
	struct regmap_field *refpd;
	struct regmap_field *txpd1;
	struct regmap_field *txpd2;
	struct regmap_field *clkoadcsel;
	struct regmap_field *adcclksel;
	struct regmap_field *hundredgat;
	struct regmap_field *twohundredgat;
};

#endif
