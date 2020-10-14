#define REG_INT			0x00
#define INT_DATA_END		(1 << 0)
#define INT_CMD_END		(1 << 1)
#define INT_ERR			(1 << 2)
#define INT_BUSY_END		(1 << 4)
#define INT_R2N_RDY		(1 << 5)
#define INT_CARDCHANGE		(1 << 6)
#define INT_CARD2CHANGE		(1 << 7)
#define REG_INTMASK		0x04
#define REG_MIUPRIORITY		0x08
#define REG_DMA_ADDR_L		0x0c
#define REG_DMA_ADDR_H		0x10
#define REG_DMA_LEN_L		0x14
#define REG_DMA_LEN_H		0x18
#define REG_FUNC_CTRL		0x1c
#define FUNC_CTRL_SDIO		0x4
#define REG_BLOCK_COUNT		0x20
#define REG_BLOCK_SIZE		0x24
#define REG_CMDRSP_SIZE		0x28
#define REG_SD_MODE		0x2c
#define REG_SD_CTL		0x30
#define REG_SD_STS		0x34
#define SD_STS_DATRDCERR	(1 << 0)
#define SD_STS_DATWRCERR	(1 << 1)
#define SD_STS_DATWRTOUT	(1 << 2)
#define SD_STS_NORSP		(1 << 3)
#define SD_STS_CMDRSPCRCERR	(1 << 4)
#define SD_STS_CARDBUSY		(1 << 6)
#define REG_FIFO		0x80
#define REG_RST			0xfc
#define RST_NRST		0

static struct reg_field sd_mode_clken_field = REG_FIELD(REG_SD_MODE, 0, 0);
static struct reg_field sd_mode_buswidth_field = REG_FIELD(REG_SD_MODE, 1, 2);

static struct reg_field func_ctrl_field = REG_FIELD(REG_FUNC_CTRL, 0, 2);

static struct reg_field st_ctl_rspr2en_field = REG_FIELD(REG_SD_CTL, 0, 0);
static struct reg_field st_ctl_rspen_field = REG_FIELD(REG_SD_CTL, 1, 1);
static struct reg_field st_ctl_cmden_field = REG_FIELD(REG_SD_CTL, 2, 2);
static struct reg_field st_ctl_dtrfen_field = REG_FIELD(REG_SD_CTL, 3, 3);
static struct reg_field st_ctl_jobdir_field = REG_FIELD(REG_SD_CTL, 4, 4);
static struct reg_field st_ctl_admaen_field = REG_FIELD(REG_SD_CTL, 5, 5);
static struct reg_field st_ctl_busydeten_field = REG_FIELD(REG_SD_CTL, 8, 8);
static struct reg_field st_ctl_errdeten_field = REG_FIELD(REG_SD_CTL, 9, 9);
static struct reg_field st_ctl_jobstart_field = REG_FIELD(REG_SD_CTL, 6, 6);

static struct reg_field cmd_rsp_size_cmdsz_field = REG_FIELD(REG_CMDRSP_SIZE, 8, 15);
static struct reg_field cmd_rsp_size_rspsz_field = REG_FIELD(REG_CMDRSP_SIZE, 0, 7);

static struct reg_field sd_sts_status_field = REG_FIELD(REG_SD_STS, 0, 6);
static struct reg_field sd_sts_cardbusy_field = REG_FIELD(REG_SD_STS, 6, 6);
static struct reg_field sd_sts_d0_field = REG_FIELD(REG_SD_STS, 8, 8);

static struct reg_field blocksize_field = REG_FIELD(REG_BLOCK_SIZE, 0, 15);
static struct reg_field blockcount_field = REG_FIELD(REG_BLOCK_COUNT, 0, 15);

static struct reg_field rst_nrst_field = REG_FIELD(REG_RST, RST_NRST, RST_NRST);
static struct reg_field rst_status_field = REG_FIELD(REG_RST, 1, 4);

struct msc313_sdio_adma_desc
{
	u32 ctrl;
	u32 dmaaddr;
	u32 dmalen;
	u32 pad;
};

#define ADMA_DESC_CTRL_END	BIT(0)
#define ADMA_DESC_MIU_SEL	GENMASK(2,1)
#define ADMA_DESC_JOB_CNT	GENMASK(31,16)
