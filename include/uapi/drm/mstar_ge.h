/*
 * mstar_ge.h
 */

#ifndef INCLUDE_UAPI_DRM_MSTAR_GE_H_
#define INCLUDE_UAPI_DRM_MSTAR_GE_H_

/*
 * Get driver info from the kernel side, takes a pointer
 * to a struct mstar_ge_info.
 */
#define MSTAR_GE_IOCTL_INFO	0
/*
 * Queue a job into the GE queue, takes a pointer to a
 * struct mstar_ge_job_request.
 */
#define MSTAR_GE_IOCTL_QUEUE	1
/*
 * Query the status of a job, takes a job tag
 */
#define MSTAR_GE_IOCTL_QUERY	2


#define MSTAR_GE_ROTATION_0	(1 << 0)
#define MSTAR_GE_ROTATION_90	(1 << 1)
#define MSTAR_GE_ROTATION_180	(1 << 2)
#define MSTAR_GE_ROTATION_270	(1 << 3)
#define MSTAR_GE_ROTATION_MASK	0xf

#define MSTAR_GE_FLIP_SRC_V	(1 << 4)
#define MSTAR_GE_FLIP_DST_H	(1 << 5)
#define MSTAR_GE_FLIP_DST_V	(1 << 6)

struct mstar_ge_info {
	__u32 caps;
};

/* The desired operation */
enum mstar_ge_op {
	MSTAR_GE_OP_INVALID,
	MSTAR_GE_OP_LINE,
	MSTAR_GE_OP_RECTFILL,
	MSTAR_GE_OP_BITBLT,
	MSTAR_GE_OP_STRBLT,
};

struct mstar_ge_color {
	unsigned int r, g, b, a;
};

/* Extra parameters for a LINE */
struct mstar_ge_line_params {
	unsigned int x0, y0, x1, y1;
	struct mstar_ge_color start_color;
};

/* Extra parameters for a RECTFILL */
struct mstar_ge_rectfill_params {
	unsigned int x0, y0, x1, y1;
	struct mstar_ge_color start_color;
};

/* Extra parameters for a BITBLT */
struct mstar_ge_bitblt {
	/* top left corner of the src */
	__u32 src_x0, src_y0;
	/* top left corner of the dst */
	__u32 dst_x0, dst_y0;
	/* bottom right corner of the src */
	__u32 dst_x1, dst_y1;

	__u32 flags;
};

/* Extra parameters for a STRBLT */
struct mstar_ge_strblt {
	/* top left corner of the src */
	__u32 src_x0, src_y0;
	/* bottom right corner of the src */
	__u32 src_x1, src_y1;
	/* top left corner of the dst */
	__u32 dst_x0, dst_y0;
	/* bottom right corner of the src */
	__u32 dst_x1, dst_y1;

	__u32 flags;
};

struct mstar_ge_buf_cfg {
	__u32 width;
	__u32 height;
	__u32 pitch;
	__u32 fourcc;
};

struct mstar_ge_buf {
	union {
		/* PRIME fd of the buffer - valid for userspace callers */
		int fd;
		/* pointer to the buffer if allocated inside the kernel - not valid for userspace callers */
		void *buf;
	};
	struct mstar_ge_buf_cfg cfg;
};

struct mstar_ge_opdata {
	enum mstar_ge_op op;
	union {
		struct mstar_ge_line_params line;
		struct mstar_ge_rectfill_params rectfill;
		struct mstar_ge_bitblt bitblt;
		struct mstar_ge_strblt strblt;
	};
};

#define MSTAR_GE_MAX_JOBS	32

struct mstar_ge_job_request {
	/*
	 * Address to write the job tag into.
	 * Tag is used to track the progress of your
	 * job.
	 */
	unsigned long *tag;

	/* pointers to the operation(s) you want performed */
	const struct mstar_ge_opdata *ops;
	int num_ops;

	/*
	 * pointers to the buffer(s) you want the operations
	 * to happen on.
	 * You need at least for drawing, and 2 for blitting.
	 */
	const struct mstar_ge_buf *bufs;
	int num_bufs;
};
#endif /* INCLUDE_UAPI_DRM_MSTAR_GE_H_ */
