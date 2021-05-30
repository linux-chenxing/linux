struct msc313_mux {
	struct clk_hw deglitch_hw;
	struct clk_hw mux_hw;
	struct clk_parent_data deglitch_parents[2];
	struct regmap_field *gate;
	struct regmap_field *mux;
	struct regmap_field *deglitch;
};

struct msc313_muxes {
	const struct msc313_muxes_data* muxes_data;
	struct msc313_mux muxes[];
};

struct msc313_mux_data {
	const char *name;
	union {
		const void *parent_data;
		const struct clk_parent_data *clk_parent_data;
	};
	u8 num_parents;
	int offset;
	int gate_shift;
	int mux_shift;
	int mux_width;
	int deglitch_shift;
	unsigned long flags;
	unsigned long mux_flags;
};

/*
 * For defining parent data for muxes that need the
 * clk_parent_data filled at runtime.
 */
#define MSC313_MUX_PARENT_DATA_FLAGS(_name, _parents, _offset, _gate_shift, _mux_shift, _mux_width, _deglitch_shift, _flags, _mux_flags) \
	{						\
		.name = _name,				\
		.parent_data = _parents,		\
		.num_parents = ARRAY_SIZE(_parents),	\
		.offset = _offset,			\
		.gate_shift = _gate_shift,		\
		.mux_shift = _mux_shift,		\
		.mux_width = _mux_width,		\
		.deglitch_shift = _deglitch_shift,	\
		.flags = _flags,			\
		.mux_flags = _mux_flags,		\
	}

#define MSC313_MUX_PARENT_DATA(_name, _parents, _offset, _gate_shift, _mux_shift, _mux_width, _deglitch_shift) \
		MSC313_MUX_PARENT_DATA_FLAGS(_name, _parents, _offset, _gate_shift, _mux_shift, _mux_width, _deglitch_shift, 0, 0)

/*
 * For defining parent data that is static so clk_parent_data
 * can be used directly.
 */
#define MSC313_MUX_CLK_PARENT_DATA_FLAGS(_name, _parents, _offset, _gate_shift, _mux_shift, _mux_width, _deglitch_shift, _flags, _mux_flags) \
	{						\
		.name = _name,				\
		.clk_parent_data = _parents,		\
		.num_parents = ARRAY_SIZE(_parents),	\
		.offset = _offset,			\
		.gate_shift = _gate_shift,		\
		.mux_shift = _mux_shift,		\
		.mux_width = _mux_width,		\
		.deglitch_shift = _deglitch_shift,	\
	}

#define MSC313_MUX_CLK_PARENT_DATA(_name, _parents, _offset, _gate_shift, _mux_shift, _mux_width, _deglitch_shift) \
	MSC313_MUX_CLK_PARENT_DATA_FLAGS(_name, _parents, _offset, _gate_shift, _mux_shift, _mux_width, _deglitch_shift, 0, 0)

#define MSC313_MUX_GAP() \
		{ \
		}

struct msc313_muxes_data {
	unsigned int num_muxes;
	const struct msc313_mux_data *muxes;
};

#define MSC313_MUXES_DATA(_muxes) \
	{ \
		.num_muxes = ARRAY_SIZE(_muxes), \
		.muxes = _muxes, \
	}

struct msc313_muxes *msc313_mux_register_muxes(struct device *dev,
		struct regmap *regmap, const struct msc313_muxes_data *muxes_data,
		int (*fill_clk_parent_data)(struct clk_parent_data *clk_parent_data,
				void *data, const void* parent_data, const struct msc313_muxes *muxes, unsigned int mux_idx, unsigned int parent_idx), void *data);
struct clk_hw *msc313_mux_xlate(struct of_phandle_args *clkspec, void *data);
