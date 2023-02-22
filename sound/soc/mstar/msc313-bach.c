// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Daniel Palmer <daniel@thingy.jp>
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/dmaengine_pcm.h>

#include <linux/mfd/syscon.h>

#define DRIVER_NAME "msc313-bach"

/*
 * For older versions alignment is 8, for new versions,
 * it's 16, just use 16.
 */
#define MSC313_BACH_ALIGNMENT	16
#define MSC313_BACH_FIFOSZ	8
/* The amount to shift depends on the IP version.. */
#define TO_MIUSIZE(_bach, _x) (_x >> _bach->data->addr_sz_shift)
#define FROM_MIUSIZE(_bach,_x) (_x <<  _bach->data->addr_sz_shift)
#define MAX_PERIODS 128

#define MSC313_BACH_DMA_SUB_CHANNEL_EN			0
#define MSC313_BACH_DMA_SUB_CHANNEL_ADDR		0x4
#define MSC313_BACH_DMA_SUB_CHANNEL_SIZE		0x8
#define MSC313_BACH_DMA_SUB_CHANNEL_TRIGGER		0xc
#define MSC313_BACH_DMA_SUB_CHANNEL_OVERRUNTHRESHOLD	0x10
#define MSC313_BACH_DMA_SUB_CHANNEL_UNDERRUNTHRESHOLD	0x14
#define MSC313_BACK_DMA_SUB_CHANNEL_LEVEL		0x18

struct msc313_bach;
struct msc313_bach_dma_channel;

struct msc313_bach_dma_sub_channel {
	struct msc313_bach_dma_channel *dma_channel;

	struct regmap_field *count;
	struct regmap_field *trigger;
	struct regmap_field *init;
	struct regmap_field *en;
	struct regmap_field *addr_hi, *addr_lo;
	struct regmap_field *size;
	struct regmap_field *trigger_level;
	struct regmap_field *overrunthreshold;
	struct regmap_field *underrunthreshold;
	struct regmap_field *level;

	struct snd_pcm_substream *substream;
};

#define MSC313_BACH_DMA_CHANNEL_CTRL0	0x0
#define MSC313_BACH_DMA_CHANNEL_CTRL8	0x20
#define MSC313_SUB_CHANNEL_READER	0
#define MSC313_SUB_CHANNEL_WRITER	1

struct msc313_bach_dma_channel {
	struct msc313_bach *bach;
	/*
	 * Enabling the channel might cause an interrupt
	 * and bust everything, this lock must be taken
	 * when doing something that might result in an
	 * interrupt and when handling interrupts.
	 */
	spinlock_t lock;

	struct regmap_field *rst;
	struct regmap_field *en;
	struct regmap_field *live_count_en;
	struct regmap_field *rd_int_clear;
	struct regmap_field *rd_empty_int_en;
	struct regmap_field *rd_overrun_int_en;
	struct regmap_field *rd_underrun_int_en;

	struct regmap_field *wr_underrun_flag;
	struct regmap_field *wr_overrun_flag;
	struct regmap_field *rd_underrun_flag;
	struct regmap_field *rd_overrun_flag;
	struct regmap_field *rd_empty_flag;
	struct regmap_field *wr_full_flag;
	struct regmap_field *wr_localbuf_full_flag;
	struct regmap_field *rd_localbuf_empty_flag;

	struct regmap_field *dma_rd_mono;
	struct regmap_field *dma_wr_mono;
	struct regmap_field *dma_rd_mono_copy;

	struct regmap_field *rate_sel;

	struct msc313_bach_dma_sub_channel reader_writer[2];
};

#define MSC313_BACH_SR0_SEL		0x4
#define MSC313_BACH_DMA_TEST_CTRL7	0x1dc

struct msc313_bach_data {
	unsigned int addr_sz_shift;
};

struct msc313_bach {
	struct device *dev;
	const struct msc313_bach_data *data;

	struct clk *clk;

	struct snd_soc_dai_link_component cpu_dai_component;
	struct snd_soc_dai_link_component platform_component;
	struct snd_soc_dai_link_component codec_component;
	struct snd_soc_dai_link dai_link;
	struct snd_soc_card card;

	struct regmap *audiotop;
	struct regmap *bach;

	/* DMA */
	struct regmap_field *dma_int_en;
	struct msc313_bach_dma_channel dma_channels[1];

	/* Analog controls? */
	struct regmap_field *codec_sel;
};

struct msc313_bach_substream_runtime {
	struct msc313_bach_dma_sub_channel *sub_channel;
	bool running;

	snd_pcm_uframes_t last_appl_ptr;

	/* Filled by prepare */
	ssize_t period_bytes;
	ssize_t max_inflight;
	unsigned max_level;
	ssize_t underflow_level;

	/* Hardware queue state */
	/* number of bytes that are in the buffer */
	ssize_t pending_bytes;
	/* number of bytes we have queued into the hardware so far */
	ssize_t total_bytes;
	/*
	 * number of bytes that the hardware has completed, updated
	 * when the irq fires
	 */
	ssize_t processed_bytes;

	/* IRQ stats, updated by irq */
	unsigned irqs;
	unsigned empties;
	unsigned underruns;

	/* Debugging */
	unsigned long start_time;
	unsigned long end_time;
};

/*
 * The amount of bytes the channel is currently munching through is the difference
 * between the bytes queued and the number of bytes that have been processed
 * according to an IRQ coming.
 */
#define BACH_PCM_RUNTIME_INFLIGHT(_brt)	(_brt->total_bytes - _brt->processed_bytes)
#define BACH_PMC_RUNTIME_BYTES_UNTIL_UNDERFLOW(__brt) (BACH_PCM_RUNTIME_INFLIGHT(__brt) - __brt->underflow_level)

/* Bank 1 */
#define REG_MUX0SEL	0xc
#define MSC313_BACH_MMC1_DPGA_CFG2	0x84
#define REG_SINEGEN	0x1d4
/* Bank 2 */
#define REG_DMA_INT	0x21c

/* Audio top */
#define REG_ATOP_OFFSET 0x1000
#define REG_ATOP_ANALOG_CTRL0	(REG_ATOP_OFFSET + 0)
#define REG_ATOP_ANALOG_CTRL1	(REG_ATOP_OFFSET + 0x4)
#define REG_ATOP_ANALOG_CTRL3	(REG_ATOP_OFFSET + 0xc)

/* cpu dai */
static const struct snd_soc_dai_driver msc313_bach_cpu_dai_drv = {
	.name = "msc313-bach-cpu-dai",
	.playback =
	{
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE
	},
	.capture =
	{
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static const struct snd_soc_component_driver msc313_bach_cpu_component = {
	.name = "msc313-bach",
};
/* cpu dai */

/* codec */
/**
* Used to set Playback volume 0~76 (mapping to -64dB~12dB)
*/
#define MAIN_PLAYBACK_VOLUME "Main Playback Volume"

/*
* Used to set Capture volume 0~76 (mapping to -64dB~12dB)
*/
#define MAIN_CAPTURE_VOLUME "Main Capture Volume"

/*
* Used to set microphone gain, total 5 bits ,
* it consists of the upper 2 bits(4 levels) + the lower 3 bits (8 levels)
*/
#define MIC_GAIN_SELECTION "Mic Gain Selection"

/*
* Used to set line-in gain level 0~7
*/
#define LINEIN_GAIN_LEVEL "LineIn Gain Level"

/*
* Used to select ADC input (Line-in, Mic-in)
*/
#define ADC_MUX "ADC Mux"

struct snd_soc_dai_driver msc313_bach_codec_dai_drv = {
	.name	= "Codec",
	.playback =
	{
		.stream_name	= "Main Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture =
	{
		.stream_name	= "Main Capture",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static unsigned int msc313_bach_codec_read(struct snd_soc_component *component, unsigned int reg)
{
	struct msc313_bach *bach = snd_soc_card_get_drvdata(component->card);
	unsigned int val;
	int ret;

	if (reg >= REG_ATOP_OFFSET)
		ret = regmap_read(bach->audiotop, reg - REG_ATOP_OFFSET, &val);
	else
		ret = regmap_read(bach->bach, reg, &val);

	if (ret)
		return ret;

	dev_info(bach->dev, "reg read %04x, %04x\n", reg, val);

	return val;
}

static int msc313_bach_codec_write(struct snd_soc_component *component,
		unsigned int reg, unsigned int value)
{
	struct msc313_bach *bach = snd_soc_card_get_drvdata(component->card);
	int ret;

	dev_info(bach->dev, "reg write %04x, %04x\n", reg, value);

	if (reg >= REG_ATOP_OFFSET)
		ret = regmap_write(bach->audiotop, reg - REG_ATOP_OFFSET, value);
	else
		ret = regmap_write(bach->bach, reg, value);

	return ret;
}

static const DECLARE_TLV_DB_SCALE(infinity_dpga_tlv, -64, 12, 0);

static const struct snd_kcontrol_new msc313_bach_controls[] =
{
	/* playback */
	SOC_DOUBLE_TLV(MAIN_PLAYBACK_VOLUME, MSC313_BACH_MMC1_DPGA_CFG2, 0, 8, 76, 1, infinity_dpga_tlv),

	//SOC_SINGLE_TLV(MAIN_CAPTURE_VOLUME, AUD_CAPTURE_DPGA, 0, 76, 0, infinity_dpga_tlv),
	//SOC_SINGLE_TLV(MIC_GAIN_SELECTION, AUD_MIC_GAIN, 0, 31, 0, NULL),
	//SOC_SINGLE_TLV(LINEIN_GAIN_LEVEL, AUD_LINEIN_GAIN, 0, 7, 0, NULL),

	/* sinegen */
	//SOC_SINGLE("SineGen Enable", REG_SINEGEN, 15, 1, 0),
	//SOC_SINGLE("SineGen Gain Level", REG_SINEGEN, 4, 15, 0),
	//SOC_SINGLE("SineGen Rate Select", REG_SINEGEN, 0, 15, 0),
};

static const char *infinity_adc_select[] = {
	"Line-in",
	"Mic-in",
};

/* Main output mux control */
static const char *msc313_bach_output_select[] = {
	"ADC In",
	"DMA Reader",
};

static const struct soc_enum msc313_bach_outsel_enum =
	SOC_ENUM_SINGLE(REG_MUX0SEL, 5,
		ARRAY_SIZE(msc313_bach_output_select),
		msc313_bach_output_select);

static const struct snd_kcontrol_new msc313_bach_output_mux_controls =
	SOC_DAPM_ENUM("Playback Select", msc313_bach_outsel_enum);

//static const struct soc_enum infinity_adcsel_enum =
//		SOC_ENUM_SINGLE(AUD_ADC_MUX, 0, 2, infinity_adc_select);
//static const struct snd_kcontrol_new infinity_adc_mux_controls =
//		SOC_DAPM_ENUM("ADC Select", infinity_adcsel_enum);

#define OUTPUT_MUX	"OUTPUT_MUX"
#define LINEOUT		"LINEOUT"

static const struct snd_soc_dapm_widget infinity_dapm_widgets[] =
{
	/* top bits */
	//SND_SOC_DAPM_DAC("DAC",   NULL, REG_ATOP_ANALOG_CTRL1, 1, 0),
	SND_SOC_DAPM_ADC("ADC", NULL, REG_ATOP_ANALOG_CTRL1, 0, 0),
	SND_SOC_DAPM_INPUT("DMARD"),
	SND_SOC_DAPM_AIF_IN("DMARD", OUTPUT_MUX, 0, SND_SOC_NOPM, 0, 0),




	//SND_SOC_DAPM_AIF_OUT("DMAWR1", "Sub Capture",  0, SND_SOC_NOPM, 0, 0),
	//SND_SOC_DAPM_AIF_OUT("DMAWR", "Main Capture",   0, SND_SOC_NOPM, 0, 0),
	//SND_SOC_DAPM_AIF_IN("DMARD2",  "Sub Playback",  0, SND_SOC_NOPM, 0, 0),
	//SND_SOC_DAPM_AIF_IN("DIGMIC", NULL,   0, AUD_DIGMIC_PWR, 0, 0),


	//SND_SOC_DAPM_INPUT("LINEIN"),
	//SND_SOC_DAPM_INPUT("MICIN"),


	//SND_SOC_DAPM_ADC("Mic Bias", NULL, AUD_ATOP_PWR, 2, 0),
	//SND_SOC_DAPM_MICBIAS("Mic Bias", AUD_ATOP_PWR, 5, 0),

//	SND_SOC_DAPM_PGA("Main Playback DPGA", AUD_DPGA_PWR, 0, 0, NULL, 0),
	//SND_SOC_DAPM_PGA("Main Capture DPGA",  AUD_DPGA_PWR, 1, 0, NULL, 0),

	//SND_SOC_DAPM_MUX(ADC_MUX, SND_SOC_NOPM, 0, 0, &infinity_adc_mux_controls),

	/* output */
	SND_SOC_DAPM_MUX(OUTPUT_MUX, SND_SOC_NOPM, 0, 0,
			&msc313_bach_output_mux_controls),
	SND_SOC_DAPM_OUTPUT(LINEOUT),
};

static const struct snd_soc_dapm_route infinity_codec_routes[] =
{
	{ .source = "DMARD", .sink = OUTPUT_MUX, },
	{ .source = "ADC", .sink = OUTPUT_MUX, },
	{ .source = OUTPUT_MUX, .sink = LINEOUT, },
};

static const struct snd_soc_component_driver msc313_bach_codec_drv = {
	.write			= msc313_bach_codec_write,
	.read			= msc313_bach_codec_read,
	.controls		= msc313_bach_controls,
	.num_controls		= ARRAY_SIZE(msc313_bach_controls),
	.dapm_widgets		= infinity_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(infinity_dapm_widgets),
	.dapm_routes		= infinity_codec_routes,
	.num_dapm_routes	= ARRAY_SIZE(infinity_codec_routes),
};
/* codec */

/* pcm */
static const struct snd_pcm_hardware msc313_bach_pcm_playback_hardware =
{
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= SNDRV_PCM_RATE_8000_48000,
	.rate_min		= 8000,
	.rate_max		= 48000,
	.channels_min		= 1,
	.channels_max		= 2,
	/* The buffer level only has 16 bits */
	.buffer_bytes_max	= (SZ_64K - 1),
	.period_bytes_min	= 512,
	.period_bytes_max	= 24 * SZ_1K,
	.periods_min		= 2,
	.periods_max		= MAX_PERIODS,
	.fifo_size		= 32,
};

static const struct snd_pcm_hardware msc313_bach_pcm_capture_hardware =
{
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= SNDRV_PCM_RATE_8000_48000,
	.rate_min		= 8000,
	.rate_max		= 48000,
	.channels_min		= 1,
	.channels_max		= 2,
	/* The buffer level only has 16 bits */
	.buffer_bytes_max	= (SZ_64K - 1),
	.period_bytes_min	= 512,
	.period_bytes_max	= 10 * SZ_1K,  MSC313_BACH_ALIGNMENT,
	.periods_min		= 2,
	.periods_max		= 128,
	.fifo_size		= 32,
};

static int msc313_bach_pcm_construct(struct snd_soc_component *component,
		struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;

	snd_pcm_set_managed_buffer_all(rtd->pcm,
				       SNDRV_DMA_TYPE_DEV,
				       card->dev,
				       msc313_bach_pcm_playback_hardware.buffer_bytes_max,
				       msc313_bach_pcm_playback_hardware.buffer_bytes_max);

	return 0;
}

#define PERIOD_BYTES_MIN 0x100

static int msc313_bach_get_level(struct msc313_bach_dma_sub_channel *sub_channel)
{
	unsigned level;

	regmap_field_force_write(sub_channel->count, 1);
	regmap_field_read(sub_channel->level, &level);
	regmap_field_force_write(sub_channel->count, 0);

	return level;
}

static void msc313_bach_dump_dmactrl(struct msc313_bach *bach)
{
	unsigned ctrl0;
	int i;

	for(i = 0; i < 0x9; i++){
		regmap_read(bach->bach, 0x100 + (i * 4), &ctrl0);
		printk("ctrl%d: %04x\n", i, ctrl0);
	}
}

static void msc313_bach_pcm_dumpruntime(struct device *dev,
					struct msc313_bach_substream_runtime *bach_runtime)
{
	struct msc313_bach_dma_sub_channel *sub_channel = bach_runtime->sub_channel;
	int level;

	level = msc313_bach_get_level(sub_channel);

	dev_dbg(dev, "irqs %d, empties %d, underruns %d, pending_bytes: %u, processed_bytes: %u, total_bytes: %u, play time %u ms\n",
		     bach_runtime->irqs,
		     bach_runtime->empties,
		     bach_runtime->underruns,
		     (unsigned) bach_runtime->pending_bytes,
		     (unsigned) bach_runtime->processed_bytes,
		     (unsigned) bach_runtime->total_bytes,
		     jiffies_to_msecs(bach_runtime->end_time - bach_runtime->start_time));
}

static int msc313_bach_pcm_open(struct snd_soc_component *component,
				struct snd_pcm_substream *substream)
{
	struct device *dev = component->dev;
	struct msc313_bach *bach = snd_soc_card_get_drvdata(component->card);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct msc313_bach_substream_runtime *bach_runtime;
	struct msc313_bach_dma_channel *dma_channel = &bach->dma_channels[0];

	dev_info(dev, "%s:%d\n", __func__, __LINE__);

	bach_runtime = kzalloc(sizeof(*bach_runtime), GFP_KERNEL);
	if (!bach_runtime)
		return -ENOMEM;

	runtime->private_data = bach_runtime;

	switch(substream->stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		snd_soc_set_runtime_hwparams(substream, &msc313_bach_pcm_playback_hardware);
		bach_runtime->sub_channel = &bach->dma_channels[0].reader_writer[0];
		bach->dma_channels[0].reader_writer[0].substream = substream;
		break;
	case SNDRV_PCM_STREAM_CAPTURE:
		snd_soc_set_runtime_hwparams(substream, &msc313_bach_pcm_capture_hardware);
		break;
	default:
		return -EINVAL;
	}

	regmap_field_force_write(dma_channel->rst, 1);
	udelay(10);
	regmap_field_force_write(dma_channel->rst, 0);
	udelay(10);

	/* Setup default register config */
	regmap_field_force_write(dma_channel->live_count_en, 1);

	return 0;
}

static int msc313_bach_pcm_close(struct snd_soc_component *component,
		struct snd_pcm_substream *substream)
{
	struct msc313_bach *bach = snd_soc_card_get_drvdata(component->card);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct msc313_bach_dma_channel *dma_channel = &bach->dma_channels[0];
	int ret;

	//printk("%s:%d\n", __func__, __LINE__);

	switch(substream->stream){
	case SNDRV_PCM_STREAM_PLAYBACK:
		dma_channel->reader_writer[MSC313_SUB_CHANNEL_READER].substream = NULL;
		break;
	case SNDRV_PCM_STREAM_CAPTURE:
		break;
	default:
		return -EINVAL;
	}

	regmap_field_write(dma_channel->rst, 1);

	return 0;
}

/*
 * Update when the next underflow interrupt happens and enable the interrupt
 * if needed.
 */
static void msc313_bach_pcm_update_underflow(struct snd_pcm_runtime *runtime)
{
	struct msc313_bach_substream_runtime *bach_runtime = runtime->private_data;
	struct msc313_bach_dma_sub_channel *sub_channel = bach_runtime->sub_channel;
	struct msc313_bach_dma_channel *dma_channel = sub_channel->dma_channel;
	struct msc313_bach *bach = dma_channel->bach;
	unsigned stride = runtime->frame_bits >> 3;
	unsigned miu_underrun_size;
	ssize_t fullperiods;
	ssize_t new_level;

	if (!BACH_PCM_RUNTIME_INFLIGHT(bach_runtime))
		return;

	/*
	 * We want an underrun interrupt before we are totally empty and
	 * at something close to the period.. plus the level seems to have
	 * 8 byte granularity? .. make it 8 samples before the end of the current
	 * period.
	 */
	fullperiods = BACH_PCM_RUNTIME_INFLIGHT(bach_runtime) -
			(BACH_PCM_RUNTIME_INFLIGHT(bach_runtime) % bach_runtime->period_bytes);
	if (fullperiods)
		fullperiods -= bach_runtime->period_bytes;

	new_level = fullperiods ? fullperiods : stride * 8;

	if (new_level != bach_runtime->underflow_level)
		dev_dbg(bach->dev, "underflow level is now %zu\n", new_level);

	bach_runtime->underflow_level = new_level;
	miu_underrun_size = TO_MIUSIZE(bach, bach_runtime->underflow_level);

	regmap_field_write(sub_channel->underrunthreshold, miu_underrun_size);

	/* Enable or reenable the interrupt */
	regmap_field_write(dma_channel->rd_underrun_int_en, 1);

	//
	regmap_field_write(dma_channel->rd_empty_int_en, 1);
}

//#define DEBUG_STUCK_QUEUE

static int msc313_bach_queue_push(struct snd_pcm_substream *substream,
				  ssize_t new_bytes)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct msc313_bach_substream_runtime *bach_runtime = runtime->private_data;
	struct msc313_bach_dma_sub_channel *sub_channel = bach_runtime->sub_channel;
	struct msc313_bach_dma_channel *dma_channel = sub_channel->dma_channel;
	struct msc313_bach *bach = dma_channel->bach;
	unsigned en, trigbit;
	unsigned miu_trigger_level = TO_MIUSIZE(bach, new_bytes);
	int target_level, old_level, new_level;

	old_level = msc313_bach_get_level(sub_channel);

	target_level = old_level + miu_trigger_level;
	if (target_level > bach_runtime->max_level) {
		printk("target level, %d, is max %d\n", target_level, bach_runtime->max_level);
		return -EINVAL;
	}

	regmap_field_write(sub_channel->trigger_level, miu_trigger_level);
	regmap_field_read(sub_channel->trigger, &trigbit);
	regmap_field_force_write(sub_channel->trigger, ~trigbit);

	bach_runtime->total_bytes += new_bytes;

	new_level = msc313_bach_get_level(sub_channel);

#ifdef DEBUG_STUCK_QUEUE
	{
		int delay_level;

		udelay(200);
		delay_level = msc313_bach_get_level(sub_channel);

		printk("old level: %d, new level %d, delay level %d\n",
				old_level, new_level, delay_level);

		if (delay_level == new_level)
			printk("level didn't change after waiting, dma is probably stuck\n");
	}
#endif

	//msc313_bach_pcm_dumpruntime(bach_runtime);

	return 0;
}

static void msc313_bach_queue_update(struct msc313_bach *bach,
				     struct snd_pcm_substream *substream,
				     struct msc313_bach_substream_runtime *bach_runtime) {
	struct snd_pcm_runtime *runtime = substream->runtime;
	ssize_t new_bytes;

	/*
	 * Trying to queue before the channel is running results in either
	 * the data not being queued or the dma locking up, so don't do that.
	 */
	if (!bach_runtime->running)
		return;

	/*
	 * Queue in multiples of the alignment size
	 */
	new_bytes = bach_runtime->pending_bytes -
			(bach_runtime->pending_bytes % MSC313_BACH_ALIGNMENT);

	if (new_bytes) {
		msc313_bach_queue_push(substream, new_bytes);
		bach_runtime->pending_bytes -= new_bytes;
	}

out:
	msc313_bach_pcm_update_underflow(runtime);
}

static int msc313_bach_pcm_trigger(struct snd_soc_component *component,
		struct snd_pcm_substream *substream, int cmd)
{
	struct device *dev = component->dev;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct msc313_bach_substream_runtime *bach_runtime = runtime->private_data;
	struct msc313_bach_dma_sub_channel *sub_channel = bach_runtime->sub_channel;
	struct msc313_bach_dma_channel *dma_channel = sub_channel->dma_channel;
	struct msc313_bach *bach = snd_soc_card_get_drvdata(component->card);
	int ret = 0;
	unsigned long flags;

	//printk("%s:%d %d\n", __func__, __LINE__, cmd);

	/*
	 * Enabling the channel can cause interrupts before we are ready,
	 * take the lock to force an irq to wait until we are finished.
	 */
	spin_lock_irqsave(&dma_channel->lock, flags);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		/* Clear any pending interrupts */
		regmap_field_force_write(dma_channel->rd_int_clear, 1);
		regmap_field_force_write(dma_channel->rd_int_clear, 0);

		/* Unmask interrupts */
		regmap_field_write(dma_channel->rd_overrun_int_en, 0);
		//regmap_field_write(dma_channel->rd_underrun_int_en, 1);
		//regmap_field_write(dma_channel->rd_empty_int_en, 0); //1

		/*
		 * Note: it seems like enabling the DMA channel must happen right
		 * before enabling the reader or the reader locks up.
		 */
		regmap_field_write(dma_channel->en, 1);
		udelay(10);

		/* Start playback */
		regmap_field_write(sub_channel->en, 1);
		udelay(10);
		bach_runtime->start_time = jiffies;
		bach_runtime->running = true;
		msc313_bach_queue_update(bach, substream, bach_runtime);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		regmap_field_write(sub_channel->en, 0);
		udelay(10);
		regmap_field_write(dma_channel->en, 0);
		/* Mask interrupts */
		regmap_field_write(dma_channel->rd_underrun_int_en, 0);
		regmap_field_write(dma_channel->rd_empty_int_en, 0);

		bach_runtime->running = false;
		bach_runtime->end_time = jiffies;

		msc313_bach_pcm_dumpruntime(dev, bach_runtime);
		break;
	default:
		ret = -EINVAL;
	}

	spin_unlock_irqrestore(&dma_channel->lock, flags);

	return ret;
}

static snd_pcm_uframes_t msc313_bach_pcm_pointer(struct snd_soc_component *component,
						 struct snd_pcm_substream *substream)
{
	struct device *dev = component->dev;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct msc313_bach_substream_runtime *bach_runtime = runtime->private_data;
	struct msc313_bach_dma_sub_channel *sub_channel = bach_runtime->sub_channel;
	struct msc313_bach_dma_channel *dma_channel = sub_channel->dma_channel;
	struct msc313_bach *bach = dma_channel->bach;
	ssize_t inflight, done = 0;
	snd_pcm_uframes_t pos;
	int i, level;
	unsigned long flags;

	spin_lock_irqsave(&dma_channel->lock, flags);

	inflight = BACH_PCM_RUNTIME_INFLIGHT(bach_runtime);
	/*
	 * The number of bytes that have been processed before the next IRQ comes will
	 * be roughly the number of bytes that are waiting to be confirmed by an IRQ
	 * minus current number of bytes the hardware says it still hasn't processed.
	 */
	level = msc313_bach_get_level(sub_channel);
	ssize_t infifo = FROM_MIUSIZE(bach, level);
	/* Make sure done doesn't become negative ..*/
	if (inflight) {
		if (infifo < inflight)
			done = inflight - infifo;
	}

	pos = bytes_to_frames(runtime, (bach_runtime->processed_bytes + done) %
			runtime->dma_bytes);

	dev_dbg(dev, "stream position is %lu frames.\n"
		      "total %zu bytes, processed %zu, bytes, pending bytes %zu,\n"
		      "inflight %zu, done %lu bytes, infifo %d bytes\n",
		      pos,
		      bach_runtime->total_bytes,
		      bach_runtime->processed_bytes,
		      bach_runtime->pending_bytes,
		      inflight,
		      done,
		      infifo);
	msc313_bach_pcm_dumpruntime(dev, bach_runtime);

	spin_unlock_irqrestore(&dma_channel->lock, flags);

	return pos;
}

static const int msc313_bach_src_rates[] = {
	8000,
	11025,
	12000, /* unsupported by alsa? */
	16000,
	22050,
	24000, /* unsupported by alsa? */
	32000,
	44100,
	48000,
};

static int msc313_bach_pcm_prepare(struct snd_soc_component *component,
				   struct snd_pcm_substream *substream)
{
	struct device *dev = component->dev;
	struct msc313_bach *bach = snd_soc_card_get_drvdata(component->card);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct msc313_bach_substream_runtime *bach_runtime = runtime->private_data;
	struct msc313_bach_dma_sub_channel *sub_channel = bach_runtime->sub_channel;
	struct msc313_bach_dma_channel *dma_channel = sub_channel->dma_channel;
	unsigned miu_buffer_size, miu_addr;
	unsigned mono = runtime->channels == 1 ? 1 : 0;
	int i, ret;

	if ((runtime->dma_addr % MSC313_BACH_ALIGNMENT) ||
			(runtime->dma_bytes & MSC313_BACH_ALIGNMENT)) {
		dev_err(dev, "dma_addr and/or dma_bytes not aligned\n");
		return -EINVAL;
	}

	bach_runtime->last_appl_ptr = 0;
	bach_runtime->irqs = 0;
	bach_runtime->empties = 0;
	bach_runtime->underruns = 0;
	bach_runtime->pending_bytes = 0;
	bach_runtime->processed_bytes = 0;
	bach_runtime->total_bytes = 0;
	bach_runtime->period_bytes = frames_to_bytes(runtime, runtime->period_size);
	bach_runtime->max_inflight = bach_runtime->period_bytes * 2;

	miu_buffer_size = TO_MIUSIZE(bach, runtime->dma_bytes);
	miu_addr = TO_MIUSIZE(bach, runtime->dma_addr);
	bach_runtime->max_level = TO_MIUSIZE(bach, bach_runtime->period_bytes * MAX_PERIODS);

	/* This is needed to reset the buffer level */
	regmap_field_force_write(sub_channel->trigger, 0);
	regmap_field_force_write(sub_channel->init, 1);
	regmap_field_force_write(sub_channel->init, 0);

	//dev_dbg(dev, "sample rate: %d\n", substream->runtime->rate);
	//dev_dbg(dev, "period %d, (%d bytes)\n", runtime->period_size,
	//		bach_runtime->period_bytes);
	//dev_dbg(dev, "dma addr %08x, size %zu\n",
	//		(unsigned) runtime->dma_addr, runtime->dma_bytes);

	regmap_field_write(sub_channel->addr_hi, miu_addr >> 12);
	regmap_field_write(sub_channel->addr_lo, miu_addr);
	regmap_field_write(sub_channel->size, miu_buffer_size);

	/* Don't care about over run,.. */
	regmap_field_write(sub_channel->overrunthreshold, 0);

	switch(substream->stream) {
		case SNDRV_PCM_STREAM_PLAYBACK:
			regmap_field_write(dma_channel->dma_rd_mono, mono);
			regmap_field_write(dma_channel->dma_rd_mono_copy, mono);
			break;
	}

	ret = -EINVAL;
	for (i = 0; i < ARRAY_SIZE(msc313_bach_src_rates); i++) {
		if (msc313_bach_src_rates[i] == substream->runtime->rate) {
			regmap_field_write(dma_channel->rate_sel, i);
			ret = 0;
			break;
		}
	}

	return ret;
}

static int msc313_bach_pcm_ack(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	struct msc313_bach *bach = snd_soc_card_get_drvdata(component->card);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct msc313_bach_substream_runtime *bach_runtime = runtime->private_data;
	struct msc313_bach_dma_sub_channel *sub_channel = bach_runtime->sub_channel;
	struct msc313_bach_dma_channel *dma_channel = sub_channel->dma_channel;
	unsigned long flags;
	ssize_t new_bytes;

	//printk("%s:%d, %u\n", __func__, __LINE__, (unsigned) runtime->control->appl_ptr);

	spin_lock_irqsave(&dma_channel->lock, flags);

	new_bytes = frames_to_bytes(runtime, runtime->control->appl_ptr - bach_runtime->last_appl_ptr);
	bach_runtime->last_appl_ptr = runtime->control->appl_ptr;
	bach_runtime->pending_bytes += new_bytes;

	msc313_bach_queue_update(bach, substream, bach_runtime);

	spin_unlock_irqrestore(&dma_channel->lock, flags);

	return 0;
}

static const struct snd_soc_component_driver msc313_soc_pcm_drv = {
	.pcm_construct	= msc313_bach_pcm_construct,
	.open		= msc313_bach_pcm_open,
	.prepare	= msc313_bach_pcm_prepare,
	.trigger	= msc313_bach_pcm_trigger,
	.pointer	= msc313_bach_pcm_pointer,
	.ack		= msc313_bach_pcm_ack,
	.close		= msc313_bach_pcm_close,
};
/* pcm */

static const struct regmap_config msc313_bach_regmap_config = {
	.name = "bach",
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static irqreturn_t msc313_bach_irq(int irq, void *data)
{
	struct msc313_bach *bach = data;
	unsigned long flags;
	int i;

	for (i = 0; i < ARRAY_SIZE(bach->dma_channels); i++) {
		struct msc313_bach_dma_channel *dma_channel = &bach->dma_channels[i];
		struct msc313_bach_substream_runtime *bach_runtime;
		struct snd_pcm_substream *substream;
		struct snd_pcm_runtime *runtime;
		unsigned empty, underrun, overrun;
		int level;

		spin_lock_irqsave(&dma_channel->lock, flags);

		level = msc313_bach_get_level(&bach->dma_channels[i].reader_writer[0]);
		/* Handle reader flags */
		substream = dma_channel->reader_writer[0].substream;

		runtime = substream->runtime;
		bach_runtime = runtime->private_data;

		bach_runtime->irqs++;

		regmap_field_read(bach->dma_channels[i].rd_empty_flag, &empty);
		regmap_field_read(bach->dma_channels[i].rd_underrun_flag, &underrun);

		/*
		 * It looks like the interrupt has to be ack'd by setting clear high
		 * it then needs to cleared to get it to trigger again.
		 */
		regmap_field_force_write(bach->dma_channels[i].rd_int_clear, 1);
		regmap_field_force_write(bach->dma_channels[i].rd_int_clear, 0);

		/*
		 * Sometimes interrupts happen at odd times before expected.
		 * Just ignore them.
		 */
		if (!bach_runtime->running) {
			spin_unlock_irqrestore(&dma_channel->lock, flags);
			continue;
		}

		/*
		 * Sometimes an empty interrupt happens just between enabling
		 * and queuing the first period.
		 */
		if (empty && level == 0) {
			/* Disable the interrupt to stop it continually firing */
			regmap_field_write(dma_channel->rd_empty_int_en, 0);

			bach_runtime->empties++;
			bach_runtime->processed_bytes = bach_runtime->total_bytes;
			//msc313_bach_pcm_dumpruntime(bach->dev, bach_runtime);
			spin_unlock_irqrestore(&dma_channel->lock, flags);
			snd_pcm_stop_xrun(substream);
			continue;
		}

		if (underrun && !empty) {
			/*
			 * Disable the underrun interrupt to stop it continually firing,
			 * if something queues before empty happens it will be reenabled.
			 */
			regmap_field_write(dma_channel->rd_underrun_int_en, 0);

			/*
			 * update the stats and work out how far along in the buffer
			 * we now are.
			 */
			bach_runtime->underruns++;
			bach_runtime->processed_bytes +=
					BACH_PMC_RUNTIME_BYTES_UNTIL_UNDERFLOW(bach_runtime);

			msc313_bach_queue_update(bach, substream, bach_runtime);

			spin_unlock_irqrestore(&dma_channel->lock, flags);
			snd_pcm_period_elapsed(substream);
			continue;
		}

		/* If we didn't need to do anything just unlock */
		spin_unlock_irqrestore(&dma_channel->lock, flags);
	}

	return IRQ_HANDLED;
}

static void msc313_bach_the_horror(struct msc313_bach *bach)
{
	regmap_write(bach->audiotop, 0x00, 0x00000A14);
	regmap_write(bach->audiotop, 0x04, 0x00000030);
	regmap_write(bach->audiotop, 0x08, 0x00000080);
	// power downs?
	regmap_write(bach->audiotop, 0x0C, 0x000001A5);
	// dac/adc resets
	regmap_write(bach->audiotop, 0x10, 0x00000000);
	regmap_write(bach->audiotop, 0x14, 0x00000000);
	regmap_write(bach->audiotop, 0x18, 0x00000000);
	regmap_write(bach->audiotop, 0x1C, 0x00000000);
	regmap_write(bach->audiotop, 0x20, 0x00003000);
	regmap_write(bach->audiotop, 0x24, 0x00000000);
	regmap_write(bach->audiotop, 0x28, 0x00000000);
	regmap_write(bach->audiotop, 0x2C, 0x00000000);
	regmap_write(bach->audiotop, 0x30, 0x00000000);
	regmap_write(bach->audiotop, 0x34, 0x00000000);
	regmap_write(bach->audiotop, 0x38, 0x00000000);
	regmap_write(bach->audiotop, 0x3C, 0x00000000);
	regmap_write(bach->audiotop, 0x40, 0x00000000);
	regmap_write(bach->audiotop, 0x44, 0x00000000);
	regmap_write(bach->audiotop, 0x48, 0x00000000);
	regmap_write(bach->audiotop, 0x4C, 0x00000000);
	regmap_write(bach->audiotop, 0x50, 0x00000000);
	regmap_write(bach->audiotop, 0x54, 0x00000000);
	regmap_write(bach->audiotop, 0x58, 0x00000000);
	regmap_write(bach->audiotop, 0x5C, 0x00000000);
	regmap_write(bach->audiotop, 0x60, 0x00000000);
	regmap_write(bach->audiotop, 0x64, 0x00000000);
	regmap_write(bach->audiotop, 0x68, 0x00000000);
	regmap_write(bach->audiotop, 0x6C, 0x00000000);
	regmap_write(bach->audiotop, 0x70, 0x00000000);
	regmap_write(bach->audiotop, 0x74, 0x00000000);
	regmap_write(bach->audiotop, 0x78, 0x00000000);
	regmap_write(bach->audiotop, 0x7C, 0x00000000);
	regmap_write(bach->audiotop, 0x80, 0x00000000);
	regmap_write(bach->audiotop, 0x84, 0x00003C1E);
	regmap_write(bach->audiotop, 0x88, 0x00000000);
	regmap_write(bach->audiotop, 0x8C, 0x00000000);
	regmap_write(bach->audiotop, 0x90, 0x00000000);
	regmap_write(bach->audiotop, 0x94, 0x00000000);
	regmap_write(bach->audiotop, 0x98, 0x00000000);
	regmap_write(bach->audiotop, 0x9C, 0x00000000);
	regmap_write(bach->audiotop, 0xA0, 0x00000000);
	regmap_write(bach->audiotop, 0xA4, 0x00000000);
	regmap_write(bach->audiotop, 0xA8, 0x00000000);
	regmap_write(bach->audiotop, 0xAC, 0x00000000);
	regmap_write(bach->audiotop, 0xB0, 0x00000000);
	regmap_write(bach->audiotop, 0xB4, 0x00000000);
	regmap_write(bach->audiotop, 0xB8, 0x00000000);
	regmap_write(bach->audiotop, 0xBC, 0x00000000);
	regmap_write(bach->audiotop, 0xC0, 0x00000000);
	regmap_write(bach->audiotop, 0xC4, 0x00000000);
	regmap_write(bach->audiotop, 0xC8, 0x00000000);
	regmap_write(bach->audiotop, 0xCC, 0x00000000);
	regmap_write(bach->audiotop, 0xD0, 0x00000000);
	regmap_write(bach->audiotop, 0xD4, 0x00000000);
	regmap_write(bach->audiotop, 0xD8, 0x00000000);
	regmap_write(bach->audiotop, 0xDC, 0x00000000);
	regmap_write(bach->audiotop, 0xE0, 0x00000000);
	regmap_write(bach->audiotop, 0xE4, 0x00000000);
	regmap_write(bach->audiotop, 0xE8, 0x00000000);
	regmap_write(bach->audiotop, 0xEC, 0x00000000);
	regmap_write(bach->audiotop, 0xF0, 0x00000000);
	regmap_write(bach->audiotop, 0xF4, 0x00000000);
	regmap_write(bach->audiotop, 0xF8, 0x00000000);
	regmap_write(bach->audiotop, 0xFC, 0x00000000);

	regmap_write(bach->bach, 0x00, 0x000089FF);

	// sr_sel0
	regmap_write(bach->bach, 0x04, 0x0000FF00);

	regmap_write(bach->bach, 0x08, 0x00000003);
	regmap_write(bach->bach, REG_MUX0SEL, 0x000019B4);
	regmap_write(bach->bach, 0x10, 0x0000F000);
	regmap_write(bach->bach, 0x14, 0x00008000);
	regmap_write(bach->bach, 0x18, 0x0000C09A);
	// MIX config?
	regmap_write(bach->bach, 0x1C, 0x0000555A);
	regmap_write(bach->bach, 0x20, 0x00000000);
	regmap_write(bach->bach, 0x24, 0x00000209);
	regmap_write(bach->bach, 0x28, 0x00000000);
	regmap_write(bach->bach, 0x2C, 0x0000007D);
	regmap_write(bach->bach, 0x30, 0x00000000);
	regmap_write(bach->bach, 0x34, 0x00000000);
	regmap_write(bach->bach, 0x38, 0x00003017);
	regmap_write(bach->bach, 0x3C, 0x00000002);
	regmap_write(bach->bach, 0x40, 0x00009400);
	regmap_write(bach->bach, 0x44, 0x00009400);
	regmap_write(bach->bach, 0x48, 0x00009400);
	regmap_write(bach->bach, 0x4C, 0x0000D400);
	regmap_write(bach->bach, 0x50, 0x00008400);
	regmap_write(bach->bach, 0x54, 0x0000D000);
	regmap_write(bach->bach, 0x58, 0x00009400);
	regmap_write(bach->bach, 0x5C, 0x00009400);
	regmap_write(bach->bach, 0x60, 0x00008400);
	regmap_write(bach->bach, 0x64, 0x00000000);
	regmap_write(bach->bach, 0x68, 0x00000000);
	regmap_write(bach->bach, 0x6C, 0x00000000);
	regmap_write(bach->bach, 0x70, 0x00000000);
	regmap_write(bach->bach, 0x74, 0x00000000);
	regmap_write(bach->bach, 0x78, 0x00000000);
	regmap_write(bach->bach, 0x7C, 0x00000000);
	regmap_write(bach->bach, 0x80, 0x00000005);
	//regmap_write(bach->bach, 0x84, 0x0000ECEC;
	regmap_write(bach->bach, 0x88, 0x00000007);
	regmap_write(bach->bach, 0x8C, 0x00000000);
	regmap_write(bach->bach, 0x90, 0x00000037);
	regmap_write(bach->bach, 0x94, 0x00000000);
	regmap_write(bach->bach, 0x98, 0x00000007);
	regmap_write(bach->bach, 0x9C, 0x00000000);
	regmap_write(bach->bach, 0xA0, 0x00000037);
	regmap_write(bach->bach, 0xA4, 0x00000000);
	regmap_write(bach->bach, 0xA8, 0x00000007);
	regmap_write(bach->bach, 0xAC, 0x00000000);
	regmap_write(bach->bach, 0xB0, 0x00000007);
	regmap_write(bach->bach, 0xB4, 0x00000000);
	regmap_write(bach->bach, 0xB8, 0x00000007);
	regmap_write(bach->bach, 0xBC, 0x00000000);
	regmap_write(bach->bach, 0xC0, 0x00000037);
	regmap_write(bach->bach, 0xC4, 0x00000000);
	regmap_write(bach->bach, 0xC8, 0x00000007);
	regmap_write(bach->bach, 0xCC, 0x00000000);
	regmap_write(bach->bach, 0xD0, 0x00000000);
	regmap_write(bach->bach, 0xD4, 0x00000000);
	regmap_write(bach->bach, 0xD8, 0x00000000);
	regmap_write(bach->bach, 0xDC, 0x00000000);
	regmap_write(bach->bach, 0xE0, 0x00000000);
	regmap_write(bach->bach, 0xE4, 0x00000000);
	regmap_write(bach->bach, 0xE8, 0x00000000);
	regmap_write(bach->bach, 0xEC, 0x00000000);
	regmap_write(bach->bach, 0xF0, 0x00000000);
	regmap_write(bach->bach, 0xF4, 0x00000000);
	regmap_write(bach->bach, 0xF8, 0x00000000);
	regmap_write(bach->bach, 0xFC, 0x00000000);
	//#regmap_write(bach->bach, 0x100, 0x00000496;
	//#regmap_write(bach->bach, 0x104, 0x00008000);
	regmap_write(bach->bach, 0x108, 0x00000FE8);
	regmap_write(bach->bach, 0x10C, 0x00002000);
	regmap_write(bach->bach, 0x110, 0x00000800);
	regmap_write(bach->bach, 0x114, 0x00000000);
	regmap_write(bach->bach, 0x118, 0x00001FE0);
	regmap_write(bach->bach, 0x11C, 0x00000F88);
	regmap_write(bach->bach, 0x120, 0x0000000F);
	regmap_write(bach->bach, 0x124, 0x00000000);
	regmap_write(bach->bach, 0x128, 0x00000000);
	regmap_write(bach->bach, 0x12C, 0x00000000);
	regmap_write(bach->bach, 0x130, 0x00000000);
	regmap_write(bach->bach, 0x134, 0x00000000);
	regmap_write(bach->bach, 0x138, 0x00000000);
	regmap_write(bach->bach, 0x13C, 0x00000000);
	regmap_write(bach->bach, 0x140, 0x00000000);
	regmap_write(bach->bach, 0x144, 0x00000000);
	regmap_write(bach->bach, 0x148, 0x00000000);
	regmap_write(bach->bach, 0x14C, 0x00000000);
	regmap_write(bach->bach, 0x150, 0x00000000);
	regmap_write(bach->bach, 0x154, 0x00000000);
	regmap_write(bach->bach, 0x158, 0x00000000);
	regmap_write(bach->bach, 0x15C, 0x00000000);
	regmap_write(bach->bach, 0x160, 0x00000000);
	regmap_write(bach->bach, 0x164, 0x00000000);
	regmap_write(bach->bach, 0x168, 0x00000000);
	regmap_write(bach->bach, 0x16C, 0x00000000);
	regmap_write(bach->bach, 0x170, 0x00000000);
	regmap_write(bach->bach, 0x174, 0x00000000);
	regmap_write(bach->bach, 0x178, 0x00000000);
	regmap_write(bach->bach, 0x17C, 0x00000000);
	regmap_write(bach->bach, 0x180, 0x00000000);
	regmap_write(bach->bach, 0x184, 0x00000000);
	regmap_write(bach->bach, 0x188, 0x00000000);
	regmap_write(bach->bach, 0x18C, 0x00000000);
	regmap_write(bach->bach, 0x190, 0x00000000);
	regmap_write(bach->bach, 0x194, 0x00000000);
	regmap_write(bach->bach, 0x198, 0x00000000);
	regmap_write(bach->bach, 0x19C, 0x00000000);
	regmap_write(bach->bach, 0x1A0, 0x00000000);
	regmap_write(bach->bach, 0x1A4, 0x00000000);
	regmap_write(bach->bach, 0x1A8, 0x00000000);
	regmap_write(bach->bach, 0x1AC, 0x00000000);
	regmap_write(bach->bach, 0x1B0, 0x00000000);
	regmap_write(bach->bach, 0x1B4, 0x00000000);
	regmap_write(bach->bach, 0x1B8, 0x00000000);
	regmap_write(bach->bach, 0x1BC, 0x00000000);
	regmap_write(bach->bach, 0x1C0, 0x00000000);
	regmap_write(bach->bach, 0x1C4, 0x00000000);
	regmap_write(bach->bach, 0x1C8, 0x00000000);
	regmap_write(bach->bach, 0x1CC, 0x000000E3);
	regmap_write(bach->bach, 0x1D0, 0x00000097);
	regmap_write(bach->bach, 0x1D4, 0x00000000);
	regmap_write(bach->bach, 0x1D8, 0x00000000);
	regmap_write(bach->bach, 0x1DC, 0x00000400);
	regmap_write(bach->bach, 0x1E0, 0x00000000);
	regmap_write(bach->bach, 0x1E4, 0x00000000);
	regmap_write(bach->bach, 0x1E8, 0x00000000);
	regmap_write(bach->bach, 0x1EC, 0x00000000);
	regmap_write(bach->bach, 0x1F0, 0x00000000);
	regmap_write(bach->bach, 0x1F4, 0x00000000);
	regmap_write(bach->bach, 0x1F8, 0x00000000);
	regmap_write(bach->bach, 0x1FC, 0x00000000);
	regmap_write(bach->bach, 0x200, 0x00000000);
	regmap_write(bach->bach, 0x204, 0x00000000);
	regmap_write(bach->bach, 0x208, 0x00000000);
	regmap_write(bach->bach, 0x20C, 0x00000000);
	regmap_write(bach->bach, 0x210, 0x00004000);
	regmap_write(bach->bach, 0x214, 0x00000100);
	regmap_write(bach->bach, 0x218, 0x000003E8);
	//#regmap_write(bach->bach, 0x21C, 0x00000002;
	regmap_write(bach->bach, 0x220, 0x00000000);
	regmap_write(bach->bach, 0x224, 0x00000000);
	regmap_write(bach->bach, 0x228, 0x00000000);
	regmap_write(bach->bach, 0x22C, 0x00000000);
	regmap_write(bach->bach, 0x230, 0x00000000);
	regmap_write(bach->bach, 0x234, 0x00000000);
	regmap_write(bach->bach, 0x238, 0x00000003);
	regmap_write(bach->bach, 0x23C, 0x00000000);
	regmap_write(bach->bach, 0x240, 0x000038C0);
	regmap_write(bach->bach, 0x244, 0x00003838);
	regmap_write(bach->bach, 0x248, 0x00000C04);
	regmap_write(bach->bach, 0x24C, 0x00001C14);
	regmap_write(bach->bach, 0x250, 0x00000001);
	regmap_write(bach->bach, 0x254, 0x00000000);
	regmap_write(bach->bach, 0x258, 0x00000003);
	regmap_write(bach->bach, 0x25C, 0x00000000);
	regmap_write(bach->bach, 0x260, 0x00000000);
	regmap_write(bach->bach, 0x264, 0x00000000);
	regmap_write(bach->bach, 0x268, 0x00000000);
	regmap_write(bach->bach, 0x26C, 0x00000202);
	regmap_write(bach->bach, 0x270, 0x00000000);
	regmap_write(bach->bach, 0x274, 0x00000000);
	regmap_write(bach->bach, 0x278, 0x00000000);
	regmap_write(bach->bach, 0x27C, 0x00000000);
	regmap_write(bach->bach, 0x280, 0x00000000);
	regmap_write(bach->bach, 0x284, 0x00000000);
	regmap_write(bach->bach, 0x288, 0x00000000);
	regmap_write(bach->bach, 0x28C, 0x00000000);
	regmap_write(bach->bach, 0x290, 0x00000000);
	regmap_write(bach->bach, 0x294, 0x00001234);
	regmap_write(bach->bach, 0x298, 0x00005678);
	regmap_write(bach->bach, 0x29C, 0x00000000);
	regmap_write(bach->bach, 0x2A0, 0x00000000);
	regmap_write(bach->bach, 0x2A4, 0x00000000);
	regmap_write(bach->bach, 0x2A8, 0x00000000);
	regmap_write(bach->bach, 0x2AC, 0x00000000);
	regmap_write(bach->bach, 0x2B0, 0x00000000);
	regmap_write(bach->bach, 0x2B4, 0x00000000);
	regmap_write(bach->bach, 0x2B8, 0x00000000);
	regmap_write(bach->bach, 0x2BC, 0x00000000);
	regmap_write(bach->bach, 0x2C0, 0x00000000);
	regmap_write(bach->bach, 0x2C4, 0x00000000);
	regmap_write(bach->bach, 0x2C8, 0x00000000);
	regmap_write(bach->bach, 0x2CC, 0x00000000);
	regmap_write(bach->bach, 0x2D0, 0x00000000);
	regmap_write(bach->bach, 0x2D4, 0x00000000);
	regmap_write(bach->bach, 0x2D8, 0x00000000);
	regmap_write(bach->bach, 0x2DC, 0x00000000);
	regmap_write(bach->bach, 0x2E0, 0x00000000);
	regmap_write(bach->bach, 0x2E4, 0x00000000);
	regmap_write(bach->bach, 0x2E8, 0x00000000);
	regmap_write(bach->bach, 0x2EC, 0x00000000);
	regmap_write(bach->bach, 0x2F0, 0x00000000);
	regmap_write(bach->bach, 0x2F4, 0x00000000);
	regmap_write(bach->bach, 0x2F8, 0x00000000);
	regmap_write(bach->bach, 0x2FC, 0x00000000);
	regmap_write(bach->bach, 0x300, 0x00000000);
	regmap_write(bach->bach, 0x304, 0x00000000);
	regmap_write(bach->bach, 0x308, 0x00000000);
	regmap_write(bach->bach, 0x30C, 0x00000000);
	regmap_write(bach->bach, 0x310, 0x00000000);
	regmap_write(bach->bach, 0x314, 0x00000000);
	regmap_write(bach->bach, 0x318, 0x00000000);
	regmap_write(bach->bach, 0x31C, 0x00000000);
	regmap_write(bach->bach, 0x320, 0x00000000);
	regmap_write(bach->bach, 0x324, 0x00000000);
	regmap_write(bach->bach, 0x328, 0x00000000);
	regmap_write(bach->bach, 0x32C, 0x00000001);
	regmap_write(bach->bach, 0x330, 0x00000000);
	regmap_write(bach->bach, 0x334, 0x00000000);
	regmap_write(bach->bach, 0x338, 0x00000000);
	regmap_write(bach->bach, 0x33C, 0x00000000);
	regmap_write(bach->bach, 0x340, 0x00000000);
	regmap_write(bach->bach, 0x344, 0x00000000);
	regmap_write(bach->bach, 0x348, 0x00000000);
	regmap_write(bach->bach, 0x34C, 0x00000000);
	regmap_write(bach->bach, 0x350, 0x00000000);
	regmap_write(bach->bach, 0x354, 0x00000000);
	regmap_write(bach->bach, 0x358, 0x00000000);
	regmap_write(bach->bach, 0x35C, 0x00000000);
	regmap_write(bach->bach, 0x360, 0x00000000);
	regmap_write(bach->bach, 0x364, 0x00000000);
	regmap_write(bach->bach, 0x368, 0x00000000);
	regmap_write(bach->bach, 0x36C, 0x00000000);
	regmap_write(bach->bach, 0x370, 0x00000000);
	regmap_write(bach->bach, 0x374, 0x00000000);
	regmap_write(bach->bach, 0x378, 0x00000000);
	regmap_write(bach->bach, 0x37C, 0x00000080);
	regmap_write(bach->bach, 0x380, 0x00000000);
	regmap_write(bach->bach, 0x384, 0x00000000);
	regmap_write(bach->bach, 0x388, 0x0000FF34);
	regmap_write(bach->bach, 0x38C, 0x00000000);
	regmap_write(bach->bach, 0x390, 0x00007FFF);
	regmap_write(bach->bach, 0x394, 0x00007FE9);
	regmap_write(bach->bach, 0x398, 0x00000000);
	regmap_write(bach->bach, 0x39C, 0x00000000);
	regmap_write(bach->bach, 0x3A0, 0x00000000);
	regmap_write(bach->bach, 0x3A4, 0x00000000);
	regmap_write(bach->bach, 0x3A8, 0x00000000);
	regmap_write(bach->bach, 0x3AC, 0x0000FEA6);
	regmap_write(bach->bach, 0x3B0, 0x0000019D);
	regmap_write(bach->bach, 0x3B4, 0x00000000);
	regmap_write(bach->bach, 0x3B8, 0x00000000);
	regmap_write(bach->bach, 0x3BC, 0x000078F4);
	regmap_write(bach->bach, 0x3C0, 0x00000000);
	regmap_write(bach->bach, 0x3C4, 0x00000000);
	regmap_write(bach->bach, 0x3C8, 0x000010D3);
	regmap_write(bach->bach, 0x3CC, 0x00000942);
	regmap_write(bach->bach, 0x3D0, 0x00000000);
	regmap_write(bach->bach, 0x3D4, 0x00000000);
	regmap_write(bach->bach, 0x3D8, 0x0000FDB6);
	regmap_write(bach->bach, 0x3DC, 0x0000F291);
	regmap_write(bach->bach, 0x3E0, 0x000078F4);
	regmap_write(bach->bach, 0x3E4, 0x00000000);
	regmap_write(bach->bach, 0x3E8, 0x00000000);
	regmap_write(bach->bach, 0x3EC, 0x00000000);
	regmap_write(bach->bach, 0x3F0, 0x00007FFF);
	regmap_write(bach->bach, 0x3F4, 0x00000000);
	regmap_write(bach->bach, 0x3F8, 0x00000001);
	regmap_write(bach->bach, 0x3FC, 0x00000000);
	regmap_write(bach->bach, 0x400, 0x00000000);
	regmap_write(bach->bach, 0x404, 0x00000021);
	regmap_write(bach->bach, 0x408, 0x00000000);
	regmap_write(bach->bach, 0x40C, 0x00000000);
	regmap_write(bach->bach, 0x410, 0x0000000A);
	regmap_write(bach->bach, 0x414, 0x00008000);
	regmap_write(bach->bach, 0x418, 0x0000011F);
	regmap_write(bach->bach, 0x41C, 0x00000000);
	regmap_write(bach->bach, 0x420, 0x00000000);
	regmap_write(bach->bach, 0x424, 0x00000000);
	regmap_write(bach->bach, 0x428, 0x00000000);
	regmap_write(bach->bach, 0x42C, 0x00000000);
	regmap_write(bach->bach, 0x430, 0x00000000);
	regmap_write(bach->bach, 0x434, 0x00000000);
	regmap_write(bach->bach, 0x438, 0x00000000);
	regmap_write(bach->bach, 0x43C, 0x0000FFFF);
	regmap_write(bach->bach, 0x440, 0x00000000);
	regmap_write(bach->bach, 0x444, 0x00000001);
	regmap_write(bach->bach, 0x448, 0x00008000);
	regmap_write(bach->bach, 0x44C, 0x00000001);
	regmap_write(bach->bach, 0x450, 0x00008000);
	regmap_write(bach->bach, 0x454, 0x00000000);
	regmap_write(bach->bach, 0x458, 0x00000000);
	regmap_write(bach->bach, 0x45C, 0x00000000);
	regmap_write(bach->bach, 0x460, 0x00000000);
	regmap_write(bach->bach, 0x464, 0x00000000);
	regmap_write(bach->bach, 0x468, 0x00000000);
	regmap_write(bach->bach, 0x46C, 0x00000000);
	regmap_write(bach->bach, 0x470, 0x00000000);
	regmap_write(bach->bach, 0x474, 0x00000000);
	regmap_write(bach->bach, 0x478, 0x00000000);
	regmap_write(bach->bach, 0x47C, 0x00000000);
	regmap_write(bach->bach, 0x480, 0x00000001);
	regmap_write(bach->bach, 0x484, 0x00000000);
	regmap_write(bach->bach, 0x488, 0x00000000);
	regmap_write(bach->bach, 0x48C, 0x00000000);
	regmap_write(bach->bach, 0x490, 0x00000000);
	regmap_write(bach->bach, 0x494, 0x00000000);
	regmap_write(bach->bach, 0x498, 0x00000000);
	regmap_write(bach->bach, 0x49C, 0x00000000);
	regmap_write(bach->bach, 0x4A0, 0x00000000);
	regmap_write(bach->bach, 0x4A4, 0x00000000);
	regmap_write(bach->bach, 0x4A8, 0x00000000);
	regmap_write(bach->bach, 0x4AC, 0x00000000);
	regmap_write(bach->bach, 0x4B0, 0x00000000);
	regmap_write(bach->bach, 0x4B4, 0x00000000);
	regmap_write(bach->bach, 0x4B8, 0x00000000);
	regmap_write(bach->bach, 0x4BC, 0x00000000);
	regmap_write(bach->bach, 0x4C0, 0x00000000);
	regmap_write(bach->bach, 0x4C4, 0x00000000);
	regmap_write(bach->bach, 0x4C8, 0x00000000);
	regmap_write(bach->bach, 0x4CC, 0x00000000);
	regmap_write(bach->bach, 0x4D0, 0x00000000);
	regmap_write(bach->bach, 0x4D4, 0x00000000);
	regmap_write(bach->bach, 0x4D8, 0x00000000);
	regmap_write(bach->bach, 0x4DC, 0x00000000);
	regmap_write(bach->bach, 0x4E0, 0x00000000);
	regmap_write(bach->bach, 0x4E4, 0x00000000);
	regmap_write(bach->bach, 0x4E8, 0x00000000);
	regmap_write(bach->bach, 0x4EC, 0x00000000);
	regmap_write(bach->bach, 0x4F0, 0x00000000);
	regmap_write(bach->bach, 0x4F4, 0x00000000);
	regmap_write(bach->bach, 0x4F8, 0x00000000);
	regmap_write(bach->bach, 0x4FC, 0x00000000);
	regmap_write(bach->bach, 0x500, 0x00000080);
	regmap_write(bach->bach, 0x504, 0x00000078);
	regmap_write(bach->bach, 0x508, 0x00000000);
	regmap_write(bach->bach, 0x50C, 0x00000000);
	regmap_write(bach->bach, 0x510, 0x00000000);
	regmap_write(bach->bach, 0x514, 0x00000000);
	regmap_write(bach->bach, 0x518, 0x00000000);
	regmap_write(bach->bach, 0x51C, 0x00000000);
	regmap_write(bach->bach, 0x520, 0x00000000);
	regmap_write(bach->bach, 0x524, 0x00000000);
	regmap_write(bach->bach, 0x528, 0x00000000);
	regmap_write(bach->bach, 0x52C, 0x00000000);
	regmap_write(bach->bach, 0x530, 0x00000000);
	regmap_write(bach->bach, 0x534, 0x00000000);
	regmap_write(bach->bach, 0x538, 0x00000000);
	regmap_write(bach->bach, 0x53C, 0x00000000);
	regmap_write(bach->bach, 0x540, 0x00000000);
	regmap_write(bach->bach, 0x544, 0x00000000);
	regmap_write(bach->bach, 0x548, 0x00000000);
	regmap_write(bach->bach, 0x54C, 0x00000000);
	regmap_write(bach->bach, 0x550, 0x00000000);
	regmap_write(bach->bach, 0x554, 0x00000000);
	regmap_write(bach->bach, 0x558, 0x00000000);
	regmap_write(bach->bach, 0x55C, 0x00000000);
	regmap_write(bach->bach, 0x560, 0x00000000);
	regmap_write(bach->bach, 0x564, 0x00000000);
	regmap_write(bach->bach, 0x568, 0x00000000);
	regmap_write(bach->bach, 0x56C, 0x00000000);
	regmap_write(bach->bach, 0x570, 0x00000000);
	regmap_write(bach->bach, 0x574, 0x00000000);
	regmap_write(bach->bach, 0x578, 0x00000000);
	regmap_write(bach->bach, 0x57C, 0x00000000);
	regmap_write(bach->bach, 0x580, 0x00000000);
	regmap_write(bach->bach, 0x584, 0x00000000);
	regmap_write(bach->bach, 0x588, 0x00000000);
	regmap_write(bach->bach, 0x58C, 0x00000000);
	regmap_write(bach->bach, 0x590, 0x00000000);
	regmap_write(bach->bach, 0x594, 0x00000000);
	regmap_write(bach->bach, 0x598, 0x00000000);
	regmap_write(bach->bach, 0x59C, 0x00000000);
	regmap_write(bach->bach, 0x5A0, 0x00000000);
	regmap_write(bach->bach, 0x5A4, 0x00000000);
	regmap_write(bach->bach, 0x5A8, 0x00000000);
	regmap_write(bach->bach, 0x5AC, 0x00000000);
	regmap_write(bach->bach, 0x5B0, 0x00000000);
	regmap_write(bach->bach, 0x5B4, 0x00000000);
	regmap_write(bach->bach, 0x5B8, 0x00000000);
	regmap_write(bach->bach, 0x5BC, 0x00000000);
	regmap_write(bach->bach, 0x5C0, 0x00000000);
	regmap_write(bach->bach, 0x5C4, 0x00000B0B);
	regmap_write(bach->bach, 0x5C8, 0x00000000);
	regmap_write(bach->bach, 0x5CC, 0x00004A4A);
	regmap_write(bach->bach, 0x5D0, 0x00004A4A);
	regmap_write(bach->bach, 0x5D4, 0x00000000);
	regmap_write(bach->bach, 0x5D8, 0x00000000);
	regmap_write(bach->bach, 0x5DC, 0x00004949);
	regmap_write(bach->bach, 0x5E0, 0x00004949);
	regmap_write(bach->bach, 0x5E4, 0x00000000);
	regmap_write(bach->bach, 0x5E8, 0x00000000);
	regmap_write(bach->bach, 0x5EC, 0x00000000);
	regmap_write(bach->bach, 0x5F0, 0x00000000);
	regmap_write(bach->bach, 0x5F4, 0x00000000);
	regmap_write(bach->bach, 0x5F8, 0x00000000);
	regmap_write(bach->bach, 0x5FC, 0x00000000);

}

#define MSC313_BACH_SUBCHANNEL_OFFSET	0x4
#define MSC313_BACH_SUBCHANNEL_SIZE	0x20

static int msc313_bach_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_dai_link *link;
	struct dma_device *dma_dev;
	struct snd_soc_card *card;
	struct msc313_bach *bach;
	struct msc313_bach_data *match_data;
	void __iomem *base;
	int i, j, ret, irq;

	match_data = device_get_match_data(dev);
	if (!match_data)
		return -EINVAL;

	/*
	 * Arrays for per-channel controls that are embedded in registers with
	 * controls for other channels.
	 */
	struct regmap_field *dma_rate_sels[ARRAY_SIZE(bach->dma_channels)];

	const struct reg_field src2_sel_field = REG_FIELD(MSC313_BACH_SR0_SEL, 0, 3);
	const struct reg_field src1_sel_field = REG_FIELD(MSC313_BACH_SR0_SEL, 4, 7);
	const struct reg_field dma1_rd_mono_field = REG_FIELD(MSC313_BACH_DMA_TEST_CTRL7, 15, 15);
	const struct reg_field dma1_wr_mono_field = REG_FIELD(MSC313_BACH_DMA_TEST_CTRL7, 14, 14);
	const struct reg_field dma1_rd_mono_copy_field = REG_FIELD(MSC313_BACH_DMA_TEST_CTRL7, 13, 13);
	const struct reg_field dma_int_en_field = REG_FIELD(REG_DMA_INT, 1, 1);

	/* Get the resources we need to probe the components */
	bach = devm_kzalloc(dev, sizeof(*bach), GFP_KERNEL);
	if(IS_ERR(bach))
		return PTR_ERR(bach);

	bach->dev = dev;
	bach->data = match_data;

	bach->clk = devm_clk_get(&pdev->dev, NULL);
	clk_prepare_enable(bach->clk);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	bach->bach = devm_regmap_init_mmio(dev, base, &msc313_bach_regmap_config);
	if (IS_ERR(bach->bach))
		return PTR_ERR(bach->bach);

	dma_rate_sels[0] = devm_regmap_field_alloc(dev, bach->bach, src1_sel_field);
	dma_rate_sels[1] = devm_regmap_field_alloc(dev, bach->bach, src2_sel_field);

	bach->dma_int_en = devm_regmap_field_alloc(dev, bach->bach, dma_int_en_field);

	bach->audiotop = syscon_regmap_lookup_by_phandle(dev->of_node, "mstar,audiotop");
	if(IS_ERR(bach->audiotop))
		return PTR_ERR(bach->audiotop);

	for (i = 0; i < ARRAY_SIZE(bach->dma_channels); i++) {
		struct msc313_bach_dma_channel *chan = &bach->dma_channels[i];
		unsigned int chan_offset = 0x100 + (0x40 * i);
		const struct reg_field chan_rst_field =
				REG_FIELD(chan_offset + MSC313_BACH_DMA_CHANNEL_CTRL0, 0, 0);
		const struct reg_field chan_en_field =
				REG_FIELD(chan_offset + MSC313_BACH_DMA_CHANNEL_CTRL0, 1, 1);
		const struct reg_field live_count_en_field =
				REG_FIELD(chan_offset + MSC313_BACH_DMA_CHANNEL_CTRL0, 2, 2);

		/* interrupt controls */
		const struct reg_field chan_rd_int_clear_field = REG_FIELD(chan_offset +
				MSC313_BACH_DMA_CHANNEL_CTRL0, 8, 8);
		const struct reg_field chan_rd_empty_int_en_field = REG_FIELD(chan_offset +
				MSC313_BACH_DMA_CHANNEL_CTRL0, 10, 10);
		const struct reg_field chan_rd_overrun_int_en_field = REG_FIELD(chan_offset +
						MSC313_BACH_DMA_CHANNEL_CTRL0, 12, 12);
		const struct reg_field chan_rd_underrun_int_en_field = REG_FIELD(chan_offset +
						MSC313_BACH_DMA_CHANNEL_CTRL0, 13, 13);

		/* flags */
		const struct reg_field chan_wd_underrun_flag_field = REG_FIELD(chan_offset +
				MSC313_BACH_DMA_CHANNEL_CTRL8, 0, 0);
		const struct reg_field chan_wd_overrun_flag_field = REG_FIELD(chan_offset +
				MSC313_BACH_DMA_CHANNEL_CTRL8, 1, 1);
		const struct reg_field chan_rd_underrun_flag_field = REG_FIELD(chan_offset +
				MSC313_BACH_DMA_CHANNEL_CTRL8, 2, 2);
		const struct reg_field chan_rd_overrun_flag_field = REG_FIELD(chan_offset +
				MSC313_BACH_DMA_CHANNEL_CTRL8, 3, 3);
		const struct reg_field chan_rd_empty_flag_field = REG_FIELD(chan_offset +
				MSC313_BACH_DMA_CHANNEL_CTRL8, 4, 4);
		const struct reg_field chan_wr_full_flag_field = REG_FIELD(chan_offset +
				MSC313_BACH_DMA_CHANNEL_CTRL8, 5, 5);
		const struct reg_field chan_wr_localbuf_full_flag_field = REG_FIELD(chan_offset +
				MSC313_BACH_DMA_CHANNEL_CTRL8, 6, 6);
		const struct reg_field chan_rd_localbuf_empty_flag_field = REG_FIELD(chan_offset +
				MSC313_BACH_DMA_CHANNEL_CTRL8, 7, 7);

		chan->bach = bach;

		spin_lock_init(&chan->lock);

		chan->rst = devm_regmap_field_alloc(dev, bach->bach, chan_rst_field);
		chan->en = devm_regmap_field_alloc(dev, bach->bach, chan_en_field);
		chan->live_count_en = devm_regmap_field_alloc(dev, bach->bach, live_count_en_field);
		chan->rd_int_clear = devm_regmap_field_alloc(dev, bach->bach, chan_rd_int_clear_field);
		chan->rd_empty_int_en = devm_regmap_field_alloc(dev, bach->bach, chan_rd_empty_int_en_field);
		chan->rd_overrun_int_en = devm_regmap_field_alloc(dev, bach->bach, chan_rd_overrun_int_en_field);
		chan->rd_underrun_int_en = devm_regmap_field_alloc(dev, bach->bach, chan_rd_underrun_int_en_field);

		chan->wr_underrun_flag = devm_regmap_field_alloc(dev, bach->bach, chan_wd_underrun_flag_field);
		chan->wr_overrun_flag = devm_regmap_field_alloc(dev, bach->bach, chan_wd_overrun_flag_field);
		chan->rd_underrun_flag = devm_regmap_field_alloc(dev, bach->bach, chan_rd_underrun_flag_field);
		chan->rd_overrun_flag = devm_regmap_field_alloc(dev, bach->bach, chan_rd_overrun_flag_field);
		chan->rd_empty_flag = devm_regmap_field_alloc(dev, bach->bach, chan_rd_empty_flag_field);
		chan->wr_full_flag = devm_regmap_field_alloc(dev, bach->bach, chan_wr_full_flag_field);
		chan->wr_localbuf_full_flag = devm_regmap_field_alloc(dev, bach->bach, chan_wr_localbuf_full_flag_field);
		chan->rd_localbuf_empty_flag = devm_regmap_field_alloc(dev, bach->bach, chan_rd_localbuf_empty_flag_field);

		/* Wire up the per-channel stuff that shares registers between channels*/
		chan->rate_sel = dma_rate_sels[i];

		if (i == 0) {
			chan->dma_rd_mono = devm_regmap_field_alloc(dev, bach->bach, dma1_rd_mono_field);
			chan->dma_wr_mono = devm_regmap_field_alloc(dev, bach->bach, dma1_wr_mono_field);
			chan->dma_rd_mono_copy = devm_regmap_field_alloc(dev, bach->bach, dma1_rd_mono_copy_field);
		}

		for (j = 0; j < ARRAY_SIZE(chan->reader_writer); j++){
			struct msc313_bach_dma_sub_channel *sub = &chan->reader_writer[j];
			unsigned int sub_chan_offset = chan_offset +
					MSC313_BACH_SUBCHANNEL_OFFSET +
					(MSC313_BACH_SUBCHANNEL_SIZE * j);

			sub->dma_channel = chan;

			/* Sub channel ctrl  fields */
			const struct reg_field sub_chan_count_field =
					REG_FIELD(sub_chan_offset + MSC313_BACH_DMA_SUB_CHANNEL_EN, 12, 12);
			const struct reg_field sub_chan_trigger_field =
					REG_FIELD(sub_chan_offset + MSC313_BACH_DMA_SUB_CHANNEL_EN, 13, 13);
			const struct reg_field sub_chan_init_field =
					REG_FIELD(sub_chan_offset + MSC313_BACH_DMA_SUB_CHANNEL_EN, 14, 14);
			const struct reg_field sub_chan_en_field =
					REG_FIELD(sub_chan_offset + MSC313_BACH_DMA_SUB_CHANNEL_EN, 15, 15);
			/* Buffer address */
			const struct reg_field sub_chan_addr_lo_field =
					REG_FIELD(sub_chan_offset + MSC313_BACH_DMA_SUB_CHANNEL_EN, 0, 11);
			const struct reg_field sub_chan_addr_hi_field =
					REG_FIELD(sub_chan_offset + MSC313_BACH_DMA_SUB_CHANNEL_ADDR, 0, 14);
			/* The rest .. */
			const struct reg_field sub_chan_size_field =
					REG_FIELD(sub_chan_offset + MSC313_BACH_DMA_SUB_CHANNEL_SIZE, 0, 15);
			const struct reg_field sub_chan_trigger_level_field =
					REG_FIELD(sub_chan_offset + MSC313_BACH_DMA_SUB_CHANNEL_TRIGGER, 0, 15);
			const struct reg_field sub_chan_overrunthreshold_field =
					REG_FIELD(sub_chan_offset + 0x10, 0, 15);
			const struct reg_field sub_chan_underrunthreshold_field =
					REG_FIELD(sub_chan_offset + 0x14, 0, 15);
			const struct reg_field sub_chan_level_field =
					REG_FIELD(sub_chan_offset + MSC313_BACK_DMA_SUB_CHANNEL_LEVEL, 0, 15);

			sub->count = devm_regmap_field_alloc(dev, bach->bach, sub_chan_count_field);
			sub->trigger = devm_regmap_field_alloc(dev, bach->bach, sub_chan_trigger_field);
			sub->init = devm_regmap_field_alloc(dev, bach->bach, sub_chan_init_field);
			sub->en = devm_regmap_field_alloc(dev, bach->bach, sub_chan_en_field);
			sub->addr_hi = devm_regmap_field_alloc(dev, bach->bach, sub_chan_addr_hi_field);
			sub->addr_lo = devm_regmap_field_alloc(dev, bach->bach, sub_chan_addr_lo_field);
			sub->size = devm_regmap_field_alloc(dev, bach->bach, sub_chan_size_field);
			sub->trigger_level = devm_regmap_field_alloc(dev, bach->bach, sub_chan_trigger_level_field);
			sub->overrunthreshold = devm_regmap_field_alloc(dev, bach->bach, sub_chan_overrunthreshold_field);
			sub->underrunthreshold = devm_regmap_field_alloc(dev, bach->bach, sub_chan_underrunthreshold_field);
			sub->level = devm_regmap_field_alloc(dev, bach->bach, sub_chan_level_field);

			regmap_field_write(sub->en, 0);
		}

		regmap_field_write(chan->en, 0);
		regmap_field_write(chan->rst, 1);
	}

	/* probe the components */
	ret = devm_snd_soc_register_component(dev,
			&msc313_bach_codec_drv,
			&msc313_bach_codec_dai_drv,
			1);
	if(ret)
		return ret;

	ret = devm_snd_soc_register_component(dev,
			&msc313_bach_cpu_component,
			&msc313_bach_cpu_dai_drv,
			1);
	if(ret)
		return ret;

	ret = devm_snd_soc_register_component(dev,
			&msc313_soc_pcm_drv,
			NULL,
			0);
	if(ret)
		return ret;

	link = &bach->dai_link;

	link->cpus = &bach->cpu_dai_component;
	link->codecs = &bach->codec_component;
	link->platforms = &bach->platform_component;

	link->num_cpus = 1;
	link->num_codecs = 1;
	link->num_platforms = 1;

	link->name = "cdc";
	link->stream_name = "CDC PCM";
	link->codecs->dai_name = "Codec";
	//link->cpus->dai_name = dev_name(dev);
	link->cpus->dai_name = "msc313-bach-cpu-dai";
	link->codecs->name = dev_name(dev);
	link->platforms->name = dev_name(dev);

	card = &bach->card;
	card->dev = dev;
	card->owner = THIS_MODULE;
	card->name = DRIVER_NAME;
	card->dai_link = link;
	card->num_links = 1;
	card->fully_routed = true;

	snd_soc_card_set_drvdata(&bach->card, bach);

	ret = snd_soc_of_parse_aux_devs(&bach->card, "audio-aux-devs");
	if(ret)
		return ret;

	ret = devm_snd_soc_register_card(dev, &bach->card);
	if(ret)
		return ret;

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!irq)
		return -EINVAL;
	ret = devm_request_irq(&pdev->dev, irq, msc313_bach_irq, IRQF_SHARED,
		dev_name(&pdev->dev), bach);

	regmap_field_write(bach->dma_int_en, 1);

	msc313_bach_the_horror(bach);

	return ret;
}

static const struct msc313_bach_data msc313_data = {
	.addr_sz_shift = 3,
};

static const struct msc313_bach_data ssd210_data = {
	.addr_sz_shift = 4,
};

static const struct of_device_id msc313_bach_of_match[] = {
		{
			.compatible = "mstar,msc313-bach",
			.data = &msc313_data,
		},
#ifdef CONFIG_MACH_PIONEER3
		{
			.compatible = "mstar,ssd210-bach",
			.data = &ssd210_data,
		},
#endif
		{ },
};
MODULE_DEVICE_TABLE(of, msc313_bach_of_match);

static struct platform_driver msc313_bach_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = msc313_bach_of_match,
	},
	.probe = msc313_bach_probe,
};
module_platform_driver(msc313_bach_driver);

MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_DESCRIPTION("MStar MSC313 BACH sound");
MODULE_LICENSE("GPL v2");
