/*
 * RCM SoC video capture device driver
 *
 * 
 */

#include <linux/types.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include <linux/videodev2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-rect.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-subdev.h>
#include <media/media-device.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include "rcm-vdu-grabber.h"

#define RCM_VDU_GRB_DBG

#ifdef RCM_VDU_GRB_DBG
	#define GRB_DBG_PRINT(...) printk( KERN_DEBUG "[VDU_GRABBER] " __VA_ARGS__ )
#else
	#define GRB_DBG_PRINT(...) while(0)
#endif

#define GRB_DBG_PRINT_PROC_CALL GRB_DBG_PRINT("%s\n",__FUNCTION__)

enum {
	RCM_VDU_GRB_SINK,
	RCM_VDU_GRB_NR_PADS
};

static const struct v4l2_pix_format fmt_default = {
	.width 		= XGA_WIDTH,
	.height 	= XGA_HEIGHT,
	.pixelformat 	= V4L2_PIX_FMT_YUV422P,
	.field 		= V4L2_FIELD_NONE,
	.colorspace 	= V4L2_COLORSPACE_DEFAULT,
	.bytesperline   = XGA_WIDTH,
	.sizeimage	= XGA_WIDTH * XGA_HEIGHT * 2,
	.ycbcr_enc	= V4L2_YCBCR_ENC_DEFAULT,
	.quantization	= V4L2_QUANTIZATION_DEFAULT,
	.priv		= 0,
	.flags		= 0,
	.xfer_func	= V4L2_XFER_FUNC_DEFAULT,
};

static const struct grb_pix_map grb_pix_map_list[] = {
	/* TODO: add all missing formats */

	/* RGB formats */
	{ // 0
		.code = MEDIA_BUS_FMT_ARGB8888_1X32, // without alpha, HW add 
		.pixelformat = V4L2_PIX_FMT_BGR32,
		.bpp = 4,
	},
	{ // 1
		.code = MEDIA_BUS_FMT_RGB888_1X24,
		.pixelformat = V4L2_PIX_FMT_RGB24, // 3 planes, no packed 
		.bpp = 3,
	},
	/* YCBCR formats */
	{ // 2
		.code = MEDIA_BUS_FMT_UYVY8_2X8,
		.pixelformat = V4L2_PIX_FMT_YUV422P,
		.bpp = 2,
	},
	{ // 3
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.pixelformat = V4L2_PIX_FMT_NV16,
		.bpp = 2,
	},
	{
		.code = MEDIA_BUS_FMT_UYVY8_2X8,
		.pixelformat = V4L2_PIX_FMT_NV12,
		.bpp = 2,
	},
	/* Bayer formats */
	{ // 4
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.pixelformat = V4L2_PIX_FMT_SBGGR8,
		.bpp = 1,
	},
	{ // 5
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.pixelformat = V4L2_PIX_FMT_SGRBG8,
		.bpp = 1,
	},
	{ // 6
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.pixelformat = V4L2_PIX_FMT_SGBRG8,
		.bpp = 1,
	},
	{ // 7
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.pixelformat = V4L2_PIX_FMT_SRGGB8,
		.bpp = 1,
	},
};

const struct grb_pix_map *grb_pix_map_by_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(grb_pix_map_list); i++) {
		if (grb_pix_map_list[i].code == code)
			return &grb_pix_map_list[i];
	}

	return NULL;
}

const struct grb_pix_map *grb_pix_map_by_pixelformat(u32 pixelformat)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(grb_pix_map_list); i++) {
		if (grb_pix_map_list[i].pixelformat == pixelformat)
			return &grb_pix_map_list[i];
	}

	return NULL;
}

static inline struct grb_graph_entity *
to_grb_entity(struct v4l2_async_subdev *asd)
{
	return container_of(asd, struct grb_graph_entity, asd);
}

static void print_detected_frame_info(unsigned int size, unsigned int  param) {

		GRB_DBG_PRINT( "detected frame size: width=%u,height=%u\n", size & 0xFFF, (size >> 16) & 0xFFF);
		GRB_DBG_PRINT( "detected frame param: active_bus=%u,vi_ena=%u,interlace=%u,vsync_pol=%u,hsync_pol=%u,dv0_act=%u,dv1_act=%u,dv0_act=%u\n",
		                             param & 0x3, (param >> 2) & 1, (param >> 4) & 1, (param >> 5) & 1, (param >> 6) & 1, 
									 (param >> 8) & 1, (param >> 9) & 1, (param >> 10) & 1);
}

static void print_v4l2_pix_format( struct v4l2_pix_format* fmt ) {
	if(fmt) {
		GRB_DBG_PRINT( "pix format: width=%u,height=%u,pixelformat=%u,field=%u,bytesperline=%u,sizeimage=%u,colorspace=0x%x\n",
						fmt->width,fmt->height,fmt->pixelformat,fmt->field,fmt->bytesperline,fmt->sizeimage,fmt->colorspace );
	}
}

/*
static void print_videobuf_queue_param( struct videobuf_queue* queue, unsigned int num ) {
	unsigned int i=0 ;
	struct videobuf_dma_contig_memory* mem;
	struct videobuf_buffer* vb;
	for( i=0; i<num; i++ ) {
		vb = queue->bufs[i];		// buffer ptr
		if( vb ) {
			mem = vb->priv;			// memory area
			GRB_DBG_PRINT( "videobuf_buffer(%u): memory=%u,vaddr=0x%x,dma_handle=0x%x,size=0x%lx: %*ph\n",
						   i, vb->memory, (u32)mem->vaddr, (u32)mem->dma_handle, mem->size, 16, mem->vaddr )
		}
	}
}

static void print_videobuf_queue_param2( struct videobuf_queue* queue, int num, int off0, int off1, int off2 ) {
	struct videobuf_dma_contig_memory* mem;
	struct videobuf_buffer* vb = queue->bufs[num];
	if( vb ) {
		u32 p0, p1, p2;
		mem = vb->priv;
		p0 = (u32)(mem->vaddr+off0), p1 = (u32)(mem->vaddr+off1), p2 = (u32)(mem->vaddr+off2);
		GRB_DBG_PRINT( "videobuf_buffer: memory=%u,vaddr=0x%x,dma_handle=0x%x,size=0x%lx:\n%08x:%*ph\n%08x:%*ph\n%08x:%*ph\n",
				 	   vb->memory, (u32)mem->vaddr, (u32)mem->dma_handle, mem->size,
					   p0, 8, (void*)p0, p1, 8, (void*)p1, p2, 8, (void*)p2 )
	} 
}
*/
/*
static void print_four_cc( const char* info, unsigned int f ) {
	GRB_DBG_PRINT( "%s: '%c%c%c%c'", info, (u8)(f>>0), (u8)(f>>8), (u8)(f>>16), (u8)(f>>24) );
}
*/

/*
static void print_v4l2_selection( const char* info, int arg, const struct v4l2_selection* s ) {
		GRB_DBG_PRINT( "%s(%08x): target=%x,flag=%x,rect=%x,%x,%x,%x\n",
						info, arg, s->target, s->flags, s->r.left, s->r.top, s->r.width, s->r.height );
}
*/

static void print_v4l2_format( const char* info, int arg, const struct v4l2_format* f ) {
	GRB_DBG_PRINT( "%s(%08x): type=%x,fmt: width=%u,height=%u,pixelformat=%x\n"
					"field=%x,bytesperline=%u,sizeimage=%x,colorspace=%x,priv=%x,flags=%x\n"
					"ycbcr_enc/hsv_enc=%x,quantization=%x,xfer_func=%x\n",
					info, arg, f->type, f->fmt.pix.width, f->fmt.pix.height, f->fmt.pix.pixelformat,
					f->fmt.pix.field, f->fmt.pix.bytesperline, f->fmt.pix.sizeimage, f->fmt.pix.colorspace,f->fmt.pix.priv, f->fmt.pix.flags,
					f->fmt.pix.ycbcr_enc, f->fmt.pix.quantization, f->fmt.pix.xfer_func );
}
/*
static void print_v4l2_buffer( const char* info, const struct v4l2_buffer* b ) {
	GRB_DBG_PRINT( "%s: index=0x%x,type=0x%x,bytesused=0x%x,flags=0x%x,field=0x%x,sequence=0x%x,memory=0x%x,offset=0x%x,length=0x%x\n",
					info, b->index, b->type, b->bytesused, b->flags, b->field, b->sequence, b->memory, b->m.offset, b->length )
}
*/
static inline void write_register( u32 val, void __iomem *addr, u32 offset ) {
	iowrite32( val, addr + offset );
}

static inline u32 read_register( void __iomem *addr, u32 offset ) {
	return ioread32( addr + offset );
}

static inline u32 clr_register( u32 mask, void __iomem *addr, u32 offset ) {
	u32 val = ioread32( addr + offset );
	iowrite32( val &= ~mask, addr + offset );
	return val;
}

static inline u32 set_register( u32 mask, void __iomem *addr, u32 offset ) {
	u32 val = ioread32( addr + offset );
	iowrite32( val |= mask, addr + offset );
	return val;
}

static inline void print_registers( void __iomem *addr, unsigned int begin, unsigned int end ) {
	unsigned int reg, val;
	for( reg=begin; reg<=end; reg+=4 ) {
		val = read_register( addr, reg );
		GRB_DBG_PRINT( "get(0x%08X, 0x%08X)\n", reg, val );
	}
}

static inline void capture_start(struct grb_info *grb)
{
	write_register(CAPTURE_START, grb->regs, ADDR_ENABLE);
}

static void capture_stop(struct grb_info *grb)
{
	write_register(CAPTURE_STOP, grb->regs, ADDR_ENABLE);
}

static int reset_grab( void __iomem *addr ) {
	unsigned int i;
	write_register( 0x01 , addr, ADDR_PR_RESET );
	for( i = 0; i < 50000; i++ ) {
		if( read_register( addr, ADDR_PR_RESET ) == 0 ) {
			return 0;
		}
	}
	GRB_DBG_PRINT( "Grabber reset: timeout\n" );
	return -1;
}

static int set_input_format( struct grb_info *grb ) {
	struct grb_parameters *param = &grb->param;
	u32 active_bus;
	u32 format_data;

	if( param->d_format == D_FMT_YCBCR422 ) {
		format_data  = 0x0 ;							// YCbCr(0)
		GRB_DBG_PRINT( "input format data is YCbCr(0)\n" );
		if( param->std_in == STD_CLR_SD ) {
			if( param->v_if == V_IF_SERIAL ) {			// d0
				active_bus = 0x1;
				GRB_DBG_PRINT( "input active bus is d0\n" );
			}
			else if( param->v_if == V_IF_PARALLEL )	{	// d0,d1,d2
				active_bus = 0x3;
				GRB_DBG_PRINT( "input active bus is d0,d1,d2\n" );
			}
			else {
				active_bus = 0x0;
				GRB_DBG_PRINT( "invalid parameter param.v_if!\n" ) ;
				return -EINVAL;
			}
		}
		else if( param->std_in == STD_CLR_HD ) {
			active_bus = 0x2 ;							// d0,d1
			GRB_DBG_PRINT( "input active bus is d0,d1\n" );
		}
		else {
			active_bus = 0x0 ;
			GRB_DBG_PRINT( "invalid parameter param.std_in!\n" );
			return -EINVAL;
		}
		grb->in_f.color = YCBCR;
		GRB_DBG_PRINT( "input format is YCBCR 4:2:2\n" );
	}
	else if( param->d_format == D_FMT_YCBCR444 ) {		// 1
		format_data = 0x0 ;								// YCbCr
		GRB_DBG_PRINT( "input format data is YCbCr\n" );
		active_bus  = 0x3 ;								// d0,d1,d2
		GRB_DBG_PRINT( "input active bus is d0,d1,d2\n" );
		grb->in_f.color = YCBCR;
		GRB_DBG_PRINT( "input format is YCBCR 4:4:4\n" );
	} 
	else if( param->d_format == D_FMT_RGB888 ) {		// 2
		format_data = 0x1 ;								// RGB
		GRB_DBG_PRINT( "input format data is RGB\n" );
		active_bus  = 0x3 ;								// d0,d1,d2
		GRB_DBG_PRINT( "input active bus is d0,d1,d2\n" );
		grb->in_f.color = RGB ;
		GRB_DBG_PRINT( "input format is RGB 8:8:8\n" );
	}
	else {
		active_bus  = 0x0;
		format_data = 0x0;
		grb->in_f.color = 0x0;
		GRB_DBG_PRINT( "std_grb: unknown standard \n" );
		return -EINVAL;
	}

	grb->in_f.format_din = format_data;			// 1–RGB,0-YCbCr
	grb->in_f.format_din |= (active_bus << 1) ;	// 1–dv0;2–dv0,dv1;3–dv0,dv1,dv2
							//
	//if ( active_bus != 1 )
	grb->in_f.format_din |= (param->std_in << 3) ;	// 0-not duplication,1–duplication (SDTV)

	grb->in_f.format_din |= (param->sync << 4) ;	// synchronization: 0–external(hsync,vsync,field,data_enable); 1–internal(EAV,SAV)

	grb->in_f.color_std = param->std_in;			// input mode
	GRB_DBG_PRINT( "input mode is %s (%x)\n", param->std_in == STD_CLR_SD ? "SD" : "HD", param->std_in);
	grb->out_f.color_std = param->std_out;			// output mode (SD,HD)
	GRB_DBG_PRINT( "output mode is %s (%x)\n", param->std_out == STD_CLR_SD ? "SD" : "HD", param->std_out);

	if( param->alpha > 255 ) { 
		GRB_DBG_PRINT( "alpha > 255\n" );
		return -EINVAL;
	}
	return 0;
}

static int set_output_format( struct grb_info *grb ) {
// 0:   0-YCBCR, 1-RGB
// 2,1: 01–ARGB8888, 10–YCbCr422, 11-YCbCr444,YCbCr422 too?,RGB888
// 3:   0–YCbCr422, 1–YCbCr444
	struct v4l2_pix_format *pix_fmt = &grb->format.fmt.pix;

	grb->out_f.y_hor_size = pix_fmt->width;
	grb->out_f.y_ver_size = pix_fmt->height;

	switch( pix_fmt->pixelformat ) {
	case V4L2_PIX_FMT_BGR32:									// 'BGR4'
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SRGGB8:
		grb->out_f.format_dout = 0x03;
		grb->out_f.color = RGB;
		grb->out_f.y_full_size = pix_fmt->bytesperline/4;
		grb->out_f.c_hor_size = pix_fmt->width;
		grb->out_f.c_ver_size = pix_fmt->height;
		grb->out_f.c_full_size = pix_fmt->bytesperline/4;

		if (pix_fmt->pixelformat != V4L2_PIX_FMT_BGR32) {
			grb->out_f.y_hor_size = (grb->out_f.y_hor_size + 2)/3;
			grb->out_f.c_hor_size = (grb->out_f.c_hor_size + 2)/3;

			GRB_DBG_PRINT( "output pixelformat: Raw Bayer\n" );
		}
		else
			GRB_DBG_PRINT( "output pixelformat: ARGB8888\n" );

		break;
	case V4L2_PIX_FMT_NV16:										// 'NV16'
		grb->out_f.format_dout = 0x04;
		grb->out_f.color = YCBCR;
		grb->out_f.y_full_size = pix_fmt->bytesperline;
		grb->out_f.c_hor_size = pix_fmt->width;
		grb->out_f.c_ver_size = pix_fmt->height;
		grb->out_f.c_full_size = pix_fmt->bytesperline;
		GRB_DBG_PRINT( "output pixelformat: YCBCR422 two planes\n" );
		break;
    case V4L2_PIX_FMT_NV12:                                     // 'NV12'
        grb->out_f.format_dout = 0x04;
        grb->out_f.color = YCBCR;
        grb->out_f.y_full_size = pix_fmt->bytesperline * 2;
        grb->out_f.c_hor_size = pix_fmt->width;
        grb->out_f.c_ver_size = pix_fmt->height;
        grb->out_f.c_full_size = pix_fmt->bytesperline;
        GRB_DBG_PRINT( "output pixelformat: YCBCR420 two planes\n" );
        break;
	case V4L2_PIX_FMT_YUV422P:									// '422P'='Y42B'
		grb->out_f.format_dout = 0x06;
		grb->out_f.color = YCBCR;
		grb->out_f.y_full_size = pix_fmt->bytesperline;
		grb->out_f.c_hor_size = pix_fmt->width/2;
		grb->out_f.c_ver_size = pix_fmt->height;
		grb->out_f.c_full_size = pix_fmt->bytesperline/2;
		GRB_DBG_PRINT( "output pixelformat: YCBCR422 three planes\n" );
		break;
	case V4L2_PIX_FMT_YUV444M:									// 'YM24'
		grb->out_f.format_dout = 0x0e;
		grb->out_f.color = YCBCR;
		grb->out_f.y_full_size = pix_fmt->bytesperline;
		grb->out_f.c_hor_size = pix_fmt->width;
		grb->out_f.c_ver_size = pix_fmt->height;
		grb->out_f.c_full_size = pix_fmt->bytesperline;
		GRB_DBG_PRINT( "output pixelformat: YCBCR444 three planes.\n" );
		break;
	case V4L2_PIX_FMT_RGB24:									// 'RGB3',but planar,no packet!!!Is's bad.
		grb->out_f.format_dout = 0x07;
		grb->out_f.color = RGB;
		grb->out_f.y_full_size = pix_fmt->bytesperline;
		grb->out_f.c_hor_size = pix_fmt->width;
		grb->out_f.c_ver_size = pix_fmt->height;
		grb->out_f.c_full_size = pix_fmt->bytesperline;
		GRB_DBG_PRINT( "output pixelformat: RGB888\n" );
		break;
	default:
		grb->out_f.format_dout = 0x00;
		GRB_DBG_PRINT( "output pixelformat: unknown pixelformat!\n" );
		return -EINVAL;
	}

	if ( (grb->out_f.y_hor_size > grb->out_f.y_full_size) ||
		 (grb->out_f.c_hor_size > grb->out_f.c_full_size) ) {
		GRB_DBG_PRINT( "output size mismatch!\n" );
		return -EINVAL;
	} 
	return 0;
}

static int setup_color( struct grb_info *grb, void __iomem* base_addr ) {
	//void __iomem* base_addr = grb->regs;

	u32 color_in = grb->in_f.color,
		color_std_in = grb->in_f.color_std,
		color_out = grb->out_f.color,
		color_std_out = grb->out_f.color_std;

	if( ( ( color_in != RGB ) && ( color_in != YCBCR ) ) ||
		  ( ( color_std_in != STD_CLR_SD ) && ( color_std_in != STD_CLR_HD ) ) ||
		  ( ( color_out != RGB ) && ( color_out != YCBCR ) ) ||
		  ( ( color_std_out != STD_CLR_SD ) && ( color_std_out != STD_CLR_HD ) ) ) {
		GRB_DBG_PRINT( "color format is wrong: input-format=%u,standard=%u;output-format=%u,standard=%u\n",
					   color_in, color_std_in, color_out, color_std_out );
		return -EINVAL;
	}

	if( color_in != color_out ) {
		if( color_in == RGB ) {
			if( color_std_out == STD_CLR_SD ) {
				GRB_DBG_PRINT( "RGB->YCBCR,SD\n" );
				grb->c_conv = &RGB_TO_YCBCR_SD;
			} 
			else {
				GRB_DBG_PRINT( "RGB->YCBCR,HD\n" );
				grb->c_conv = &RGB_TO_YCBCR_HD;
			}
		}
		else if( color_in == YCBCR ) {
			if( color_std_in == STD_CLR_SD ) {
				GRB_DBG_PRINT( "YCBCR->RGB,SD\n" );
				grb->c_conv = &YCBCR_TO_RGB_SD;
			} 
			else {
				GRB_DBG_PRINT( "YCBCR->RGB,HD\n" );
				grb->c_conv = &YCBCR_TO_RGB_HD;
			}
		}
	}
	else if( (color_in == YCBCR) && (color_out == YCBCR) ) {
		if( (color_std_in == STD_CLR_SD) && (color_std_out == STD_CLR_HD) ) {
			GRB_DBG_PRINT( "YCBCR SD->HD\n" );
			grb->c_conv = &YCBCR_SD_TO_HD;
		}
		else if( (color_std_in == STD_CLR_HD) & (color_std_out == STD_CLR_SD) ) {
			GRB_DBG_PRINT( "YCBCR HD->SD\n" );
			grb->c_conv = &YCBCR_HD_TO_SD;
		}
		else {
			GRB_DBG_PRINT( "not need color conversion\n" );
			grb->c_conv = NULL;
	}
	}
	else { // not need color conversion
		GRB_DBG_PRINT( "not need color conversion\n" );
		grb->c_conv = NULL;
	}
	
	write_register( 0, base_addr, ADDR_CONV_ENABLE );

	if( grb->c_conv ) {
		write_register( grb->c_conv->coef[0][0], base_addr, ADDR_C_0_0 );
		write_register( grb->c_conv->coef[0][1], base_addr, ADDR_C_0_1 );
		write_register( grb->c_conv->coef[0][2], base_addr, ADDR_C_0_2 );
		write_register( grb->c_conv->coef[0][3], base_addr, ADDR_C_0_3 );

		write_register( grb->c_conv->coef[1][0], base_addr, ADDR_C_1_0 );
		write_register( grb->c_conv->coef[1][1], base_addr, ADDR_C_1_1 );
		write_register( grb->c_conv->coef[1][2], base_addr, ADDR_C_1_2 );
		write_register( grb->c_conv->coef[1][3], base_addr, ADDR_C_1_3 );

		write_register( grb->c_conv->coef[2][0], base_addr, ADDR_C_2_0 );
		write_register( grb->c_conv->coef[2][1], base_addr, ADDR_C_2_1 );
		write_register( grb->c_conv->coef[2][2], base_addr, ADDR_C_2_2 );
		write_register( grb->c_conv->coef[2][3], base_addr, ADDR_C_2_3 );

		write_register( grb->c_conv->range[0], base_addr, ADDR_CH0_RANGE );
		write_register( grb->c_conv->range[1], base_addr, ADDR_CH1_RANGE );
		write_register( grb->c_conv->range[2], base_addr, ADDR_CH2_RANGE );
		write_register( 1, base_addr, ADDR_CONV_ENABLE );
	}
	// print_registers( base_addr, ADDR_C_0_0, ADDR_CH2_RANGE );
	return 0;
}

static void setup_gamma( void __iomem* base_addr, struct grb_gamma *gam ) {
	int i;
	u32 vY_G, vC_R, vC_B;
	int *pY_G = gam->table_Y_G, *pC_R = gam->table_C_R, *pC_B = gam->table_C_B;

	write_register( 0, base_addr, ADDR_GAM_ENABLE );

	if( gam->active_gamma == ON ) {
		for ( i=0; i<256; i+=4 ) {											// todo check it!
				vY_G = U8x4_TO_U32( pY_G[i], pY_G[i+1], pY_G[i+2], pY_G[i+3] );
				vC_R = U8x4_TO_U32( pC_R[i], pC_R[i+1], pC_R[i+2], pC_R[i+3] );
				vC_B = U8x4_TO_U32( pC_B[i], pC_B[i+1], pC_B[i+2], pC_B[i+3] );
				write_register( vY_G, base_addr, BASE_ADDR_TABLE_0+i );		// dv0( Y,G)
				write_register( vC_R, base_addr, BASE_ADDR_TABLE_1+i );		// dv1(Cr,R)
				write_register( vC_B, base_addr, BASE_ADDR_TABLE_2+i );		// dv2(Cb,B)
				
		}
		//GRB_DBG_PRINT( "init gamma table, last word yg=%08x, cr=%08x, cb=%08x\n",
		//			   read_register( base_addr, BASE_ADDR_TABLE_0+252 ),
		//			   read_register( base_addr, BASE_ADDR_TABLE_1+252 ),
		//			   read_register( base_addr, BASE_ADDR_TABLE_2+252 ) );
		write_register( 1, base_addr, ADDR_GAM_ENABLE );
	}
}

static inline dma_addr_t buf_to_dma_contig_rcm( dma_addr_t dma_addr ) {
	 return PHYS_TO_DMA( dma_addr );
}

static void setup_dma_addr( u32 format_dout, u32 mem_offs1, u32 mem_offs2, u32 ba_dma0, u32* ba_dma1, u32* ba_dma2 ) {
	switch( format_dout ) {
	case 0x04:			// YCBCR422_2
	case 0x07:			// RGB888
		*ba_dma1 = ba_dma0 + mem_offs1;
		*ba_dma2 = ba_dma0 + mem_offs2;
		return;
	case 0x03:			// ARGB8888
	case 0x06:			// YCBCR422_3
	case 0x0e:			// YCBCR444
	default:
		*ba_dma1 = ba_dma0 + mem_offs2;
		*ba_dma2 = ba_dma0 + mem_offs1;
		return;
	}
}

static void dma_addr_fill(struct grb_info *grb, dma_addr_t dma_addr, unsigned int slot) {
	void __iomem* base_addr = grb->regs;
	u32 base_addr_dma0, base_addr_dma1, base_addr_dma2;

	base_addr_dma0 = buf_to_dma_contig_rcm( dma_addr);
	setup_dma_addr( grb->out_f.format_dout, grb->mem_offset1, grb->mem_offset2,
					base_addr_dma0, &base_addr_dma1, &base_addr_dma2 );

	if(slot == 0) {
		write_register( base_addr_dma0, base_addr, ADDR_DMA0_ADDR0 );	// luminance, odd
		write_register( base_addr_dma1, base_addr, ADDR_DMA1_ADDR0 );	// color difference 1
		write_register( base_addr_dma2, base_addr, ADDR_DMA2_ADDR0 );	// color difference 2
	}
	else {
		write_register( base_addr_dma0, base_addr, ADDR_DMA0_ADDR1 );	// luminance, even
		write_register( base_addr_dma1, base_addr, ADDR_DMA1_ADDR1 );	// color difference 1
		write_register( base_addr_dma2, base_addr, ADDR_DMA2_ADDR1 );	// color difference 2
	}
}

static void convert_rect_params(struct grb_info *grb, struct v4l2_rect *tmp, struct v4l2_rect *cur)
{
	struct v4l2_pix_format *pix_fmt = &grb->format.fmt.pix;
	switch (pix_fmt->pixelformat) {
	default:
		return;
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		tmp->left = (tmp->left + 2)/3;
		if (tmp->left & 0x1)
			tmp->left -= 1;

		tmp->width = (tmp->width + 2)/3;
		if (tmp->width & 0x1)
			tmp->width += 1;

		cur->left = (cur->left + 2)/3;
		if (cur->left & 0x1)
			cur->left -= 1;

		cur->width = (cur->width + 2)/3;
		if (cur->width & 0x1)
			cur->width += 1;


		break;
	}

	return;
}

static int buffer_fill_all(struct grb_info *grb);

int setup_registers( struct grb_info *grb ) {
	void __iomem* base_addr = grb->regs;
	u32 y_hor_size, y_ver_size, y_full_size;
	u32 c_ver_size, c_hor_size, c_full_size;
	u32 base_point_y, base_point_x;
	u32 format_din;
	u32 format_dout;
	int ret;
	struct v4l2_rect r = grb->cropping;
	struct v4l2_rect cur_rect;

	set_input_format( grb );
	set_output_format( grb );

	format_din  = grb->in_f.format_din;
	format_dout = grb->out_f.format_dout;
	GRB_DBG_PRINT( "format_din: 0x%0x, format_dout: 0x%0x\n", format_din, format_dout );

	/*Make the intersection between current size and crop request */
	cur_rect.top = 0;
	cur_rect.left = 0;
	cur_rect.width = grb->format.fmt.pix.width;
	cur_rect.height = grb->format.fmt.pix.height;
	v4l2_rect_map_inside(&r, &cur_rect);
	r.top  = clamp_t(s32, r.top, 0, grb->format.fmt.pix.height - r.height);
	r.left = clamp_t(s32, r.left, 0, grb->format.fmt.pix.width - r.width);

	convert_rect_params(grb, &cur_rect, &r);

	if (!(r.top == cur_rect.top && r.left == cur_rect.left &&
	      r.width == cur_rect.width && r.height == cur_rect.height))
	{
		grb->out_f.y_hor_size = r.width;
		grb->out_f.y_ver_size = r.height;
		grb->out_f.c_hor_size = r.width;
		grb->out_f.c_ver_size = r.height;

		if (format_dout == 0x6) {
			grb->out_f.c_hor_size /= 2;
			grb->out_f.c_full_size -= r.left/2;
		}
		else
			grb->out_f.c_full_size -= r.left;

		grb->out_f.y_full_size -= r.left;

		GRB_DBG_PRINT("crop %ux%u@(%u,%u) from %ux%u\n",
			r.width, r.height, r.left, r.top, cur_rect.width, cur_rect.height);
	}

	y_hor_size = grb->out_f.y_hor_size;
	y_ver_size = grb->out_f.y_ver_size;
	c_hor_size = grb->out_f.c_hor_size;
	c_ver_size = grb->out_f.c_ver_size;

	y_full_size = grb->out_f.y_full_size;
	c_full_size = grb->out_f.c_full_size;

	base_point_y = r.top;
	base_point_x = r.left;

	grb->mem_offset1 = y_full_size*y_ver_size;									// plane for color component 1
	grb->mem_offset2 = grb->mem_offset1 + c_full_size*c_ver_size;				// plane for color component 2
	
	GRB_DBG_PRINT( "y_hor_size=%d,y_ver_size=%d,c_hor_size=%d,c_ver_size=%d,y_full_size=%d,c_full_size=%d,mem_offset1=%d,mem_offset2=%d,alpha=%d\n",
			 y_hor_size, y_ver_size, c_hor_size, c_ver_size, y_full_size, c_full_size,
			 grb->mem_offset1, grb->mem_offset2, grb->param.alpha );
	//print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 2 ); 						// print vaddr,dma_handle,size for each buffer

	grb->sequence = 0;

	reset_grab( base_addr );
	write_register( 1, base_addr, ADDR_BASE_SW_ENA );											// enable switching base addresses

	ret = buffer_fill_all(grb);
	if (ret) {
		return ret;
	}

	write_register( U16x2_TO_U32( y_ver_size, y_hor_size ), base_addr, ADDR_Y_SIZE );			// 27:16-height,11:0-width for luminance
	write_register( U16x2_TO_U32( c_ver_size, c_hor_size ), base_addr, ADDR_C_SIZE );			// 27:16-height,11:0-width color difference
	write_register( U16x2_TO_U32( c_full_size, y_full_size ), base_addr, ADDR_FULL_LINE_SIZE );	// 27:16-height,11:0-width full string
	write_register( U16x2_TO_U32( base_point_y, base_point_x ), base_addr, ADDR_BASE_POINT );	// 27:16-base point vert coord,11:0-ase point hor coord
	write_register( grb->param.alpha, base_addr, ADDR_TRANSPARENCY );

// ADDR_MODE:
// 4: 0-hsync, vsync, field and data_enable),1-EAV and SAV
// 3: 0-not duplication,1-duplication (SDTV)
// 2,1: 1–dv0;2–dv0,dv1;3–dv0,dv1,dv2
// 0: input format: 0 – YCbCr,1 – RGB
	write_register( format_din , base_addr, ADDR_MODE );

// ADDR_LOCATION_DATA:
// бит 3:  YCbCr only: 0 – YCbCr 4:2:2; 1 – YCbCr 4:4:4
// биты 2,1: output plane count: 1–ARGB8888; 2–YCbCr 4:2:2; 3–YCbCr 4:4:4,YCbCr 4:2:2 and RGB888
// бит 0: output format 0 – YCbCr; 1 – RGB
	write_register( format_dout , base_addr, ADDR_LOCATION_DATA );
	setup_gamma( base_addr, &grb->gam );
	setup_color( grb, base_addr );
	set_register( INT_BIT_END_WRITE, base_addr, ADDR_INT_MASK  );
	
	capture_start(grb);

	return 0;
}

/* ------------------------------------------------------------------
	Videobuf operations
   ------------------------------------------------------------------*/
/*
 * Setup the constraints of the queue: besides setting the number of planes
 * per buffer and the size and allocation context of each plane, it also
 * checks if sufficient buffers have been allocated. Usually 3 is a good
 * minimum number: many DMA engines need a minimum of 2 buffers in the
 * queue and you need to have another available for userspace processing.
 */
static int queue_setup(struct vb2_queue *vq,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], struct device *alloc_devs[])
{
	struct grb_info *grb = vb2_get_drv_priv(vq);
	unsigned long size;
	int max_buff;

	GRB_DBG_PRINT_PROC_CALL;

	size = grb->format.fmt.pix.sizeimage;
	GRB_DBG_PRINT( "%s entry: buff_length=%u,size=%lu,count=%u\n", __func__, grb->buff_length, size, *nbuffers );

	if( size == 0 ) {
		dev_err(grb->dev, "%s image size is zero (%lu)\n",	__func__, size);
		return -EINVAL;
	}

	max_buff = grb->buff_length / size;

	if (vq->num_buffers + *nbuffers < 3)
		*nbuffers = 3 - vq->num_buffers;

	if( *nbuffers >  16 )
		*nbuffers = 16;
	
	if( *nbuffers > max_buff )
		*nbuffers = max_buff;

	grb->reqv_buf_cnt = *nbuffers;	// save available buffers count

	/* Make sure the image size is large enough. */
	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = size;

	GRB_DBG_PRINT( "%s return: buff_length=%u,size=%lu,count=%u\n", __func__, grb->buff_length, size, *nbuffers );
	return 0;
}

static int buffer_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct frame_buffer *buf = container_of(vbuf, struct frame_buffer, vb);

	GRB_DBG_PRINT_PROC_CALL;
	GRB_DBG_PRINT("%s: idx %u\n", __func__, vb->index);

	INIT_LIST_HEAD(&buf->list);

	return 0;
}

/*
 * Prepare the buffer for queueing to the DMA engine: check and set the
 * payload size.
 */
static int buffer_prepare(struct vb2_buffer *vb)
{
	struct grb_info *grb = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size;

	GRB_DBG_PRINT_PROC_CALL;
	GRB_DBG_PRINT("%s: idx %u\n", __func__, vb->index);

	size = grb->format.fmt.pix.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(grb->dev, "%s data will not fit into plane (%lu < %lu)\n",
				__func__, vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	GRB_DBG_PRINT( "%s: set plane payload size=%lu\n", __func__, size );

	vb2_set_plane_payload(vb, 0, size);
	return 0;
}

/*
 * Queue this buffer to the DMA engine.
 */
static void buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct grb_info *grb = vb2_get_drv_priv(vb->vb2_queue);
	struct frame_buffer *buf = container_of(vbuf, struct frame_buffer, vb);
	unsigned long flags = 0;

	GRB_DBG_PRINT_PROC_CALL;
	GRB_DBG_PRINT("%s: idx %u\n", __func__, vb->index);

	spin_lock_irqsave(&grb->irq_lock, flags);
	list_add_tail(&buf->list, &grb->buf_list);
	spin_unlock_irqrestore(&grb->irq_lock, flags);
}

static int setup_scratch_buffer(struct grb_info *grb, unsigned int slot)
{

	dev_dbg(grb->dev,
		"No more available buffer, need use the scratch buffer!\n");

	grb->current_buf[slot] = NULL;
	return 0;
}

static int buffer_fill_slot(struct grb_info *grb, unsigned int slot)
{
	struct frame_buffer *c_buf;
	struct vb2_v4l2_buffer *v_buf;
	dma_addr_t buf_addr;

	GRB_DBG_PRINT_PROC_CALL;

	/*
	 * We should never end up in a situation where we overwrite an
	 * already filled slot.
	 */
	if (WARN_ON(grb->current_buf[slot]))
		return -EINVAL;

	if (list_empty(&grb->buf_list))
		return setup_scratch_buffer(grb, slot);

	c_buf = list_first_entry(&grb->buf_list, struct frame_buffer, list);
	list_del_init(&c_buf->list);

	v_buf = &c_buf->vb;
	grb->current_buf[slot] = v_buf;

	buf_addr = vb2_dma_contig_plane_dma_addr(&v_buf->vb2_buf, 0);
	dma_addr_fill(grb, buf_addr, slot);

	return 0;
}

static int buffer_fill_all(struct grb_info *grb)
{
	unsigned int slot;
	int ret = 0;
	unsigned long flags;

	GRB_DBG_PRINT_PROC_CALL;

	spin_lock_irqsave(&grb->irq_lock, flags);
	for (slot = 0; slot < RCM_GRB_MAX_FRAME; slot++) {
		ret = buffer_fill_slot(grb, slot);
		if (ret) {
			dev_err(grb->dev, "Error buffer fill slot %d\n", slot);
			goto unlock_irq;
		}
	}
unlock_irq:	
	spin_unlock_irqrestore(&grb->irq_lock, flags);
	return ret;
}

static void buffer_mark_done(struct grb_info *grb, unsigned int slot, unsigned int sequence)
{
	struct vb2_v4l2_buffer *v_buf;

	GRB_DBG_PRINT_PROC_CALL;

	if (!grb->current_buf[slot]) {
		dev_dbg(grb->dev, "Scratch buffer was used, ignoring..\n");
		return;
	}

	v_buf = grb->current_buf[slot];
	v_buf->field = grb->format.fmt.pix.field;
	v_buf->sequence = sequence;
	v_buf->vb2_buf.timestamp = ktime_get_ns();
	vb2_buffer_done(&v_buf->vb2_buf, VB2_BUF_STATE_DONE);

	GRB_DBG_PRINT("%s: idx %u\n", __func__, v_buf->vb2_buf.index);

	grb->current_buf[slot] = NULL;
}

static int buffer_flip(struct grb_info *grb, unsigned int sequence)
{
	unsigned int next;
	u32 reg = read_register( grb->regs, ADDR_GRB_STATUS);

	/* Our next buffer is not the current buffer */
	next = !(reg & BIT_ACTIVE_FRAME);

	/* Report the previous buffer as done */
	buffer_mark_done(grb, next, sequence);

	GRB_DBG_PRINT( "buffer_flip: sequence = %d; next = %d\n", sequence , next );

	/* Put a new buffer in there */
	return buffer_fill_slot(grb, next);
}

static void return_all_buffers(struct grb_info *grb, enum vb2_buffer_state state)
{
	struct frame_buffer *buf, *node;
	unsigned int slot;
	unsigned long flags;

	GRB_DBG_PRINT_PROC_CALL;

	spin_lock_irqsave(&grb->irq_lock, flags);
	/* release all active buffers */
	for (slot = 0; slot < RCM_GRB_MAX_FRAME; slot++) {
		struct vb2_v4l2_buffer *v_buf = grb->current_buf[slot];

		if (!v_buf)
			continue;

		GRB_DBG_PRINT("%s slot: idx %u\n", __func__, v_buf->vb2_buf.index);
		vb2_buffer_done(&v_buf->vb2_buf, state);
		grb->current_buf[slot] = NULL;
	}

	list_for_each_entry_safe(buf, node, &grb->buf_list, list) {
		GRB_DBG_PRINT("%s: idx %u\n", __func__, buf->vb.vb2_buf.index);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&grb->irq_lock, flags);
}

static void buffer_finish(struct vb2_buffer *vb)
{
	struct grb_info *grb = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format *pix_fmt = &grb->format.fmt.pix;
	void *mem = vb2_plane_vaddr(vb, 0);
	unsigned long frame_full_size, y_hor_size, y_full_size;
	unsigned long y_ver_size, origin_size;
	unsigned long i, j, k;


#if 0
	if (pix_fmt->pixelformat == V4L2_PIX_FMT_NV12)
	{
		printk("FixBufSize\n");
		frame_full_size = vb2_get_plane_payload(vb, 0);
		vb2_set_plane_payload(vb, 0, 12 * frame_full_size / 16 );
		return;
	}
#endif

	if ((pix_fmt->pixelformat != V4L2_PIX_FMT_SBGGR8) && (pix_fmt->pixelformat != V4L2_PIX_FMT_SGBRG8) &&
	    (pix_fmt->pixelformat != V4L2_PIX_FMT_SGRBG8) && (pix_fmt->pixelformat != V4L2_PIX_FMT_SRGGB8))
		return;

	frame_full_size = vb2_get_plane_payload(vb, 0);
	y_hor_size = grb->out_f.y_hor_size *4;
	y_full_size = grb->out_f.y_full_size*4;
	y_ver_size = grb->out_f.y_ver_size;
	origin_size = grb->out_f.y_hor_size*3;

	if(y_full_size * y_ver_size > frame_full_size)
		return;

	if (origin_size > pix_fmt->width)
		origin_size = pix_fmt->width;

	for (i = 0; i < y_ver_size; i++) {
		for (k = 0, j = 0; j < y_hor_size && k < origin_size; j+=4, k+=3) {
			unsigned char tmp[3];

			tmp[0] = *((unsigned char *)(mem + y_full_size*i + j + 0));
			tmp[1] = *((unsigned char *)(mem + y_full_size*i + j + 1)); 
			tmp[2] = *((unsigned char *)(mem + y_full_size*i + j + 2));

			*((unsigned char *)(mem + origin_size*i + k + 0)) = tmp[2];
			*((unsigned char *)(mem + origin_size*i + k + 1)) = tmp[1];
			*((unsigned char *)(mem + origin_size*i + k + 2)) = tmp[0];
		}
	}

	vb2_set_plane_payload(vb, 0, y_ver_size*origin_size);

	return; 
}

static void buffer_cleanup(struct vb2_buffer *vb)
{
//	struct grb_info *grb = vb2_get_drv_priv(vb->vb2_queue);

	GRB_DBG_PRINT_PROC_CALL;
	GRB_DBG_PRINT("%s: idx %u\n", __func__, vb->index);

//	INIT_LIST_HEAD(&grb->buf_list);
}

/*
 * Start streaming. First check if the minimum number of buffers have been
 * queued. If not, then return -ENOBUFS and the vb2 framework will call
 * this function again the next time a buffer has been queued until enough
 * buffers are available to actually start the DMA engine.
 */
static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct grb_info *grb = vb2_get_drv_priv(vq);
	int ret;

	GRB_DBG_PRINT_PROC_CALL;

	dev_dbg(grb->dev, "Starting capture\n");

	ret = media_pipeline_start(&grb->video_dev.entity, &grb->video_dev.pipe);
	if (ret < 0)
		goto err_start_stream;


	/* Enable stream on the sub device */
	ret = v4l2_subdev_call(grb->entity.subdev, video, s_stream, 1);
	if (ret && ret != -ENOIOCTLCMD) {
		dev_err(grb->dev, "stream on failed in subdev\n");
		goto err_disable_pipeline;
	}

	ret = v4l2_ctrl_handler_setup(&grb->hdl);
	if( ret < 0 ) {
		dev_err(grb->dev,  "v4l2_ctrl_handler_setup failed \n");
		goto err_stop_subdev;
	}

	ret = setup_registers( grb );
	if( ret < 0 ) {
		dev_err(grb->dev,  "set_register failed \n");
		goto err_disable_device;
	}

	return 0;

err_disable_device:
	capture_stop(grb);

err_stop_subdev:
	v4l2_subdev_call(grb->entity.subdev, video, s_stream, 0);

err_disable_pipeline:
	media_pipeline_stop(&grb->video_dev.entity);

err_start_stream:
	return_all_buffers(grb, VB2_BUF_STATE_QUEUED);

	return ret;
}

static void grb_capture_stop(struct grb_info *grb)
{
	GRB_DBG_PRINT_PROC_CALL;

	/* Disable stream on the sub device */
	v4l2_subdev_call(grb->entity.subdev, video, s_stream, 0);

	capture_stop(grb);
	reset_grab( grb->regs );

	/* Release all active buffers */
	return_all_buffers(grb, VB2_BUF_STATE_ERROR);
}

/*
 * Stop the DMA engine. Any remaining buffers in the DMA queue are dequeued
 * and passed on to the vb2 framework marked as STATE_ERROR.
 */
static void stop_streaming(struct vb2_queue *vq)
{
	struct grb_info *grb = vb2_get_drv_priv(vq);
	GRB_DBG_PRINT_PROC_CALL;
	dev_dbg(grb->dev, "Stopping capture\n");
	grb_capture_stop(grb);
}

/*
 * The vb2 queue ops. Note that since q->lock is set we can use the standard
 * vb2_ops_wait_prepare/finish helper functions. If q->lock would be NULL,
 * then this driver would have to provide these ops.
 */
static const struct vb2_ops grb_video_qops = {
	.queue_setup		= queue_setup,
	.buf_init		= buffer_init,
	.buf_prepare		= buffer_prepare,
	.buf_queue		= buffer_queue,
	.buf_finish		= buffer_finish,
	.buf_cleanup 		= buffer_cleanup,
	.start_streaming	= start_streaming,
	.stop_streaming		= stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static int drv_vidioc_auto_detect( struct grb_info *grb, void *arg );

static int device_open( struct file *file ) {
	struct grb_info *grb = video_drvdata( file) ;
//	struct v4l2_subdev *sd = grb->entity.subdev;
	int ret;

	dev_dbg( grb->dev, "Open video4linux2 device. Name: %s, base: %x \n" ,
			  grb->video_dev.name, (u32)grb->phys_addr_regs );

	ret = mutex_lock_interruptible(&grb->lock);
	if (ret)
		return ret;

	ret = v4l2_fh_open(file);
	if (ret < 0) {
		dev_err(grb->dev, "Error! v4l2_fh_open fail.\n");
		goto unlock;
	}

//	ret = v4l2_subdev_call(sd, core, s_power, 1);
//	if (ret < 0 && ret != -ENOIOCTLCMD){
//		dev_err(grb->dev, "Error power up subdev\n");
//		goto fh_rel;
//	}

//	drv_vidioc_auto_detect( grb, NULL );

//	v4l2_subdev_call(sd, core, s_power, 0);

//fh_rel:
	if (ret)
		v4l2_fh_release(file);
unlock:
	mutex_unlock(&grb->lock);
	return ret;
}

static int device_release(struct file *file ) {
	struct video_device *video_dev = video_devdata( file );
	struct grb_info *grb = video_drvdata( file );

	dev_dbg( grb->dev, "Close video4linux2 device. Name: %s \n" , video_dev->name );

	mutex_lock(&grb->lock);

	grb_capture_stop(grb);
	vb2_queue_release(&grb->queue);
	grb->queue.owner = NULL;

	v4l2_fh_release(file);

	mutex_unlock(&grb->lock);
	return 0;
}

static void video_dev_release(struct video_device *video_dev ) {
}

static struct v4l2_file_operations fops =
{
	.owner          = THIS_MODULE,
	.open           = device_open,
	.release        = device_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap			= vb2_fop_mmap,
	.poll			= vb2_fop_poll,
	.read			= vb2_fop_read,
};

static int vidioc_querycap_grb ( struct file* file, void* fh, struct v4l2_capability* v4l2_cap_ptr )
{
	struct grb_info *grb = video_drvdata( file );
	char bus_info[32];

	strlcpy( v4l2_cap_ptr->driver, RCM_GRB_DRIVER_NAME, sizeof(v4l2_cap_ptr->driver) );
	strlcpy( v4l2_cap_ptr->card, RCM_GRB_DEVICE_NAME, sizeof(v4l2_cap_ptr->card) );
	snprintf( bus_info, sizeof(bus_info), "platform:vdu-grabber@0x%x", (u32)virt_to_phys(grb->regs) );
	strlcpy( v4l2_cap_ptr->bus_info, bus_info, sizeof(v4l2_cap_ptr->bus_info) ) ;
//	v4l2_cap_ptr->version = RCM_GRB_DRIVER_VERSION;
	v4l2_cap_ptr->device_caps = grb->video_dev.device_caps;
	v4l2_cap_ptr->capabilities = v4l2_cap_ptr->device_caps | V4L2_CAP_DEVICE_CAPS;
	v4l2_cap_ptr->reserved[0] = v4l2_cap_ptr->reserved[1] = v4l2_cap_ptr->reserved[2] = 0;
	//GRB_DBG_PRINT( "vidioc_querycap_grb return: device_caps=%x,capabilities=%x\n", v4l2_cap_ptr->device_caps, v4l2_cap_ptr->capabilities );
	return 0;
}

static int vidioc_fmt( struct grb_info *grb, struct v4l2_format *f ) {
	if( f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ) {
		//f->fmt.pix.width  = grb->recognize_format.width;
		//f->fmt.pix.height = grb->recognize_format.height;
		//f->fmt.pix.field  = grb->recognize_format.field;
		//print_four_cc( "vidioc_fmt entry: ", f->fmt.pix.pixelformat );
		switch( f->fmt.pix.pixelformat  ) {
		default:
			f->fmt.pix.pixelformat  = V4L2_PIX_FMT_YUV422P;
			f->fmt.pix.bytesperline = f->fmt.pix.width;
			f->fmt.pix.sizeimage    = f->fmt.pix.bytesperline*f->fmt.pix.height*2;
			break;
		case V4L2_PIX_FMT_SBGGR8:
		case V4L2_PIX_FMT_SGBRG8:
		case V4L2_PIX_FMT_SGRBG8:
		case V4L2_PIX_FMT_SRGGB8:
			f->fmt.pix.bytesperline = ((f->fmt.pix.width + 2) / 3) * 4;
			f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * f->fmt.pix.height;
			break;
		case V4L2_PIX_FMT_BGR32:
			f->fmt.pix.bytesperline = f->fmt.pix.width*4;
			f->fmt.pix.sizeimage    = f->fmt.pix.bytesperline*f->fmt.pix.height;
			break;
		case V4L2_PIX_FMT_RGB24:
		case V4L2_PIX_FMT_YUV444M:
			f->fmt.pix.bytesperline = f->fmt.pix.width;
			f->fmt.pix.sizeimage    = f->fmt.pix.bytesperline*f->fmt.pix.height*3;
			break;
		case V4L2_PIX_FMT_NV12:
		case V4L2_PIX_FMT_NV16:
		case V4L2_PIX_FMT_YUV422P:
			f->fmt.pix.bytesperline = f->fmt.pix.width;
			f->fmt.pix.sizeimage    = f->fmt.pix.bytesperline*f->fmt.pix.height*2;
			break;
		}
		f->fmt.pix.colorspace = V4L2_COLORSPACE_DEFAULT;
		f->fmt.pix.priv = 0;
		f->fmt.pix.flags = 0;
		f->fmt.pix.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		f->fmt.pix.quantization = V4L2_QUANTIZATION_DEFAULT;
		f->fmt.pix.xfer_func = 0;
		//print_four_cc( "vidioc_fmt return: ", f->fmt.pix.pixelformat );
		return 0;
	}
	else
		return -EINVAL;
}

static int vidioc_enum_fmt_vid_cap_grb( struct file *file, void *fh, struct v4l2_fmtdesc *fmt ) {
	
	if( fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE )
		return -EINVAL;

	switch( fmt->index ) {
	case 0: // V4L2_PIX_FMT_BGR32
		fmt->flags = 0;
		strlcpy( fmt->description, "ARGB8888", sizeof(fmt->description) );
		fmt->pixelformat = v4l2_fourcc( 'B','G','R', '4' );
		break;
	case 1: // V4L2_PIX_FMT_RGB24
		fmt->flags = 0;
		strlcpy(fmt->description, "RGB888", sizeof(fmt->description));
		fmt->pixelformat = v4l2_fourcc('R', 'G', 'B', '3');
		break;
	case 2: // V4L2_PIX_FMT_YUV422P The three components are separated into three sub-images or planes. Cb,Cr two pixel.
		fmt->flags = 0;
		strlcpy( fmt->description, "YUV 4:2:2", sizeof(fmt->description) );
		fmt->pixelformat = v4l2_fourcc( '4','2','2', 'P' );
		break;
	case 3: // V4L2_PIX_FMT_NV16,format with ½ horizontal chroma resolution, also known as YUV 4:2:2. One luminance and one chrominance plane.
		fmt->flags = 0;
		strlcpy( fmt->description, "YUV 4:2:2", sizeof(fmt->description) );
		fmt->pixelformat = v4l2_fourcc( 'N','V','1', '6' );
		break;
	case 4:
		fmt->flags = 0;
		strlcpy(fmt->description, "BGGR8", sizeof(fmt->description));
		fmt->pixelformat = v4l2_fourcc( 'B', 'A', '8', '1');
		break;
	case 5:
		fmt->flags = 0;
		strlcpy(fmt->description, "GRBG8", sizeof(fmt->description));
		fmt->pixelformat = v4l2_fourcc('G', 'R', 'B', 'G');
		break;
	case 6:
		fmt->flags = 0;
		strlcpy(fmt->description, "GBRG8", sizeof(fmt->description));
		fmt->pixelformat = v4l2_fourcc('G', 'B', 'R', 'G');
		break;
	case 7:
		fmt->flags = 0;
		strlcpy(fmt->description, "RGGB8", sizeof(fmt->description));
		fmt->pixelformat = v4l2_fourcc('R', 'G', 'G', 'B');
		break;
	case 8: // V4L2_PIX_FMT_YUV444M Planar formats with full horizontal resolution, also known as YUV and YVU 4:4:4
		fmt->flags = 0;
		strlcpy( fmt->description, "YUV 4:4:4", sizeof(fmt->description) );
		fmt->pixelformat = v4l2_fourcc( 'Y','M','2', '4' );
		break;
    case 9: // V4L2_PIX_FMT_NV12
            fmt->flags = 0;
            strlcpy( fmt->description, "YUV 4:2:0", sizeof(fmt->description) );
            fmt->pixelformat = v4l2_fourcc( 'N','V','1', '2' );
            break;
	default:
		return -EINVAL;
	}
	GRB_DBG_PRINT( "vidioc_enum_fmt_vid_cap_grb: type=0x%08x,index=0x%08x,pixfmt=0x%08x\n", fmt->type, fmt->index, fmt->pixelformat );
	return 0;
}

static int grb_try_fmt(struct grb_info *grb, struct v4l2_format *f, u32 *code)
{
	const struct grb_pix_map *vpix;
	struct v4l2_pix_format *pixfmt = &f->fmt.pix;
//	struct v4l2_subdev_format sd_fmt;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		GRB_DBG_PRINT( "Incorrect format type!\n" );
		return -EINVAL;
	}

	/* Don't accept a pixelformat that is not on the table */
	vpix = grb_pix_map_by_pixelformat(pixfmt->pixelformat);
	if (!vpix) {
		pixfmt->pixelformat = fmt_default.pixelformat;
		GRB_DBG_PRINT("%s: pixelformat not found. use default\n", __func__);
	}

	if (pixfmt->field == V4L2_FIELD_ANY) {
		pixfmt->field = fmt_default.field;
		GRB_DBG_PRINT("%s: use default field\n", __func__);
	}

	/* Limit to hardware capabilities */
	if (pixfmt->width > RCM_GRB_MAX_SUPPORT_WIDTH) {
		pixfmt->width = RCM_GRB_MAX_SUPPORT_WIDTH;
		GRB_DBG_PRINT("%s: set RCM_GRB_MAX_SUPPORT_WIDTH\n", __func__);
	}
	if (pixfmt->height > RCM_GRB_MAX_SUPPORT_HEIGHT) {
		pixfmt->height = RCM_GRB_MAX_SUPPORT_HEIGHT;
		GRB_DBG_PRINT("%s: set RCM_GRB_MAX_SUPPORT_HEIGHT\n", __func__);
	}
	
	if (pixfmt->width < RCM_GRB_MIN_SUPPORT_WIDTH) {
		pixfmt->width = fmt_default.width;
		GRB_DBG_PRINT("%s: set fmt_default.width (%d)\n", __func__, pixfmt->width);
	}
	if (pixfmt->height < RCM_GRB_MIN_SUPPORT_HEIGHT) {
		pixfmt->height = fmt_default.height;
		GRB_DBG_PRINT("%s: set fmt_default.height (%d)\n", __func__, pixfmt->height);
	}
	GRB_DBG_PRINT("%s: cur size %dX%d\n", __func__, pixfmt->width, pixfmt->height);


//	sd_fmt.pad = grb->src_pad;
//	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
//	v4l2_subdev_call(grb->src_subdev, pad, get_fmt, NULL, &sd_fmt);
//	v4l2_fill_pix_format(pixfmt, &sd_fmt.format);

	pixfmt->field = V4L2_FIELD_NONE;
	if((pixfmt->colorspace == V4L2_COLORSPACE_DEFAULT) || 
	   (pixfmt->colorspace > V4L2_COLORSPACE_DCI_P3)) {
		pixfmt->colorspace = V4L2_COLORSPACE_SRGB;
	}
	pixfmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pixfmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	pixfmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	switch( pixfmt->pixelformat  ) {
	default:
		pixfmt->pixelformat  = V4L2_PIX_FMT_YUV422P;
		pixfmt->bytesperline = pixfmt->width;
		pixfmt->sizeimage    = pixfmt->bytesperline*pixfmt->height*2;
		GRB_DBG_PRINT( "Unsupported pixelformat format. Set to default.\n" );
		break;
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		pixfmt->bytesperline = ((pixfmt->width + 2) / 3) * 4;
		pixfmt->sizeimage = pixfmt->bytesperline * pixfmt->height;
		break;
	case V4L2_PIX_FMT_BGR32:
		pixfmt->bytesperline = pixfmt->width*4;
		pixfmt->sizeimage    = pixfmt->bytesperline*pixfmt->height;
		break;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_YUV444M:
		pixfmt->bytesperline = pixfmt->width;
		pixfmt->sizeimage    = pixfmt->bytesperline*pixfmt->height*3;
		break;
	case V4L2_PIX_FMT_NV12:		
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_YUV422P:
		pixfmt->bytesperline = pixfmt->width;
		pixfmt->sizeimage    = pixfmt->bytesperline*pixfmt->height*2;
		break;
	}

	GRB_DBG_PRINT("%s: cur bytesperline %d, sizeimage %d, field %d\n", __func__, pixfmt->bytesperline, pixfmt->sizeimage, pixfmt->field);

	return 0;
}

static int vidioc_g_fmt_vid_cap_grb ( struct file *file, void *fh, struct v4l2_format *f ) {
	int ret = 0;
	struct grb_info *grb = video_drvdata(file);
	ret = vidioc_fmt( grb, f );
	*f = grb->format;
	print_v4l2_format( "Vidioc_g_fmt_vid_cap_grb return", (int)grb->phys_addr_regs, f );
	return ret;
}

static int vidioc_try_fmt_vid_cap_grb( struct file *file, void *fh, struct v4l2_format *f ) {
	int ret;
	struct grb_info *grb = video_drvdata( file );
	ret = grb_try_fmt(grb, f, NULL);
	print_v4l2_format( "Vidioc_try_fmt_vid_cap_grb return", (int)grb->phys_addr_regs, f );
	return ret;
}

static int vidioc_s_fmt_vid_cap_grb ( struct file *file, void *fh, struct v4l2_format *f ) {
	int ret;
	struct grb_info *grb = video_drvdata( file );

	ret = grb_try_fmt(grb, f, NULL);
	if(ret)
		return ret;

	grb->format = *f;

	ret = set_output_format(grb);	//negotiate the format of data (typically image format) exchanged between driver and application
	print_v4l2_format( "Vidioc_s_fmt_vid_cap_grb return", (int)grb->phys_addr_regs, f );
	return ret;
}

/*
static void print_gamma_cs( struct grb_gamma* g ) {
	int i;
	unsigned int cs = 0;
	for( i=0; i<256; i++ )
		cs += (g->table_C_R[i]<<16) + (g->table_Y_G[i]<<8) + (g->table_C_B[i]<<0);
	GRB_DBG_PRINT( "Gamma table checksum=%08x", cs );
}
*/
static void drv_set_gamma( struct grb_info *grb, void *arg ) {
	struct grb_gamma* gam = (struct grb_gamma*)arg;
	grb->gam = *gam;
	//print_gamma_cs( &grb->gam );
}

static void drv_vidioc_g_params( struct grb_info *grb, void *arg ) {
	struct grb_parameters* param = (struct grb_parameters*)arg;
	GRB_DBG_PRINT_PROC_CALL;
	param = &grb->param;
}

static int drv_vidioc_s_params( struct grb_info *grb, void *arg ) { // VIDIOC_S_PARAMS
	struct grb_parameters* param = (struct grb_parameters*)arg;
	GRB_DBG_PRINT_PROC_CALL;
	grb->param = *param;
	return set_input_format( grb );
}

static int drv_vidioc_auto_detect( struct grb_info *grb, void *arg )
{ // todo mutex
	struct v4l2_pix_format* recognize_format = (struct v4l2_pix_format*)arg;
	int retval;

	GRB_DBG_PRINT_PROC_CALL;

	/* Enable stream on the sub device */
	retval = v4l2_subdev_call(grb->entity.subdev, video, s_stream, 1);
	if (retval && retval != -ENOIOCTLCMD) {
		GRB_DBG_PRINT( "stream on failed in subdev\n" );
		return retval;
	}

	if( reset_grab( grb->regs ) ) {
		GRB_DBG_PRINT( "reset failed\n" );
		return -EIO;
	}

	init_completion( &grb->cmpl );
	set_register( INT_BIT_DONE, grb->regs, ADDR_INT_STATUS );
	set_register( INT_BIT_DONE, grb->regs, ADDR_INT_MASK );
	write_register( 1 , grb->regs, ADDR_REC_ENABLE );

	if( wait_for_completion_timeout( &grb->cmpl, (HZ/2) ) == 0 ) {
		GRB_DBG_PRINT( "Vidioc autodetection: timeout\n" );
		v4l2_subdev_call(grb->entity.subdev, video, s_stream, 0);
		return -ETIMEDOUT;
	}

	print_v4l2_pix_format(&grb->recognize_format);

	if( recognize_format )
		*recognize_format = grb->recognize_format;

	v4l2_subdev_call(grb->entity.subdev, video, s_stream, 0);

	return 0;
}

static long vidioc_default_grb( struct file *file, void *fh, bool valid_prio, unsigned int cmd, void *arg )
{
	struct grb_info *grb = video_drvdata( file );
	GRB_DBG_PRINT( "Vidioc ioctl default: cmd=%08x\n", cmd );
	switch (cmd) {
	case VIDIOC_SET_GAMMA:
		GRB_DBG_PRINT( "Vidioc ioctl default: VIDIOC_SET_GAMMA\n");
		drv_set_gamma( grb, arg );
		return 0;
	case VIDIOC_G_PARAMS:
		GRB_DBG_PRINT( "Vidioc ioctl default: VIDIOC_G_PARAMS\n");
		drv_vidioc_g_params( grb, arg );
		return 0;
    case VIDIOC_S_PARAMS:
		GRB_DBG_PRINT( "Vidioc ioctl default: VIDIOC_S_PARAMS\n");
		drv_vidioc_s_params( grb, arg );
		return 0;
	case VIDIOC_AUTO_DETECT:
		GRB_DBG_PRINT( "Vidioc ioctl default: VIDIOC_AUTO_DETECT\n");
		return drv_vidioc_auto_detect( grb, arg );
	default:
		return -ENOTTY;
	}
}

static void grb_try_compose(struct grb_info *grb, struct v4l2_rect *r)
{

	r->width = clamp_t(u32, r->width, 0, grb->format.fmt.pix.width);
	r->height = clamp_t(u32, r->height, 0, grb->format.fmt.pix.height);

	r->left = clamp_t(u32, r->left, 0, grb->format.fmt.pix.width - r->width);
	r->top  = clamp_t(u32, r->top, 0, grb->format.fmt.pix.height - r->height);

}


static int vidioc_g_selection_grb( struct file *file, void *fh, struct v4l2_selection *s ) {
	struct grb_info *grb = video_drvdata(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = grb->format.fmt.pix.width;
		s->r.height = grb->format.fmt.pix.height;
		return 0;

	case V4L2_SEL_TGT_CROP:
		s->r = grb->cropping;
		return 0;
	}

	return -EINVAL;
}

static int vidioc_s_selection_grb( struct file *file, void *fh, struct v4l2_selection *s ) {
	struct grb_info *grb = video_drvdata(file);
	struct v4l2_rect rect = s->r;

	GRB_DBG_PRINT_PROC_CALL;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    s->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	if (vb2_is_busy(&grb->queue))
		return -EBUSY;

	if (rect.top < 0 || rect.left < 0) {
		v4l2_err(&grb->v4l2_dev,
			"doesn't support negative values for top & left\n");
		return -EINVAL;
	}

	grb_try_compose(grb, &rect);

	s->r = rect;
	grb->cropping = rect; // rect
	return 0;
}

static int grb_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct grb_info *grb =
		container_of(ctrl->handler, struct grb_info, hdl);

	switch (ctrl->id) {
	case RCM_GRB_CID_SYNC:
		GRB_DBG_PRINT( "%s: sync=%d\n", __func__, ctrl->val);
		grb->param.sync = ctrl->val;
		break;
	case RCM_GRB_CID_STD_IN:
		GRB_DBG_PRINT( "%s: std_in=%d\n", __func__, ctrl->val);
		grb->param.std_in = ctrl->val;
		break;
	case RCM_GRB_CID_D_V_IF:
		GRB_DBG_PRINT( "%s: v_if=%d\n", __func__, ctrl->val);
		grb->param.v_if = ctrl->val;
		break;
	case RCM_GRB_CID_D_FORMAT:
		GRB_DBG_PRINT( "%s: d_format=%d\n", __func__, ctrl->val);
		grb->param.d_format = ctrl->val;
		break;
	case RCM_GRB_CID_STD_OUT:
		GRB_DBG_PRINT( "%s: std_out=%d\n", __func__, ctrl->val);
		grb->param.std_out = ctrl->val;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int grb_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct grb_info *grb =
		container_of(ctrl->handler, struct grb_info, hdl);

	switch (ctrl->id) {
	case RCM_GRB_CID_SYNC:
		ctrl->val = grb->param.sync;
		GRB_DBG_PRINT( "%s: sync=%d\n", __func__, ctrl->val);
		break;
	case RCM_GRB_CID_STD_IN:
		ctrl->val = grb->param.std_in;
		GRB_DBG_PRINT( "%s: std_in=%d\n", __func__, ctrl->val);
		break;
	case RCM_GRB_CID_D_V_IF:
		ctrl->val = grb->param.v_if;
		GRB_DBG_PRINT( "%s: v_if=%d\n", __func__, ctrl->val);
		break;
	case RCM_GRB_CID_D_FORMAT:
		ctrl->val = grb->param.d_format;
		GRB_DBG_PRINT( "%s: d_format=%d\n", __func__, ctrl->val);
		break;
	case RCM_GRB_CID_STD_OUT:
		ctrl->val = grb->param.std_out;
		GRB_DBG_PRINT( "%s: std_out=%d\n", __func__, ctrl->val);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops grb_ctrl_ops = {
	.s_ctrl 			= grb_s_ctrl,
	.g_volatile_ctrl	= grb_g_volatile_ctrl,
};

static const struct v4l2_ctrl_config grb_sync = {
	.ops = &grb_ctrl_ops,
	.id = RCM_GRB_CID_SYNC,
	.name = "sync",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.def = SYNC_EXTERNAL,
	.min = 0,
	.max = 1,
	.step = 1,
};

static const struct v4l2_ctrl_config grb_stdin = {
	.ops = &grb_ctrl_ops,
	.id = RCM_GRB_CID_STD_IN,
	.name = "stdin",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.def = STD_CLR_SD,
	.min = 0,
	.max = 1,
	.step = 1,
};

static const struct v4l2_ctrl_config grb_dvif = {
	.ops = &grb_ctrl_ops,
	.id = RCM_GRB_CID_D_V_IF,
	.name = "dvif",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.def = V_IF_SERIAL,
	.min = 0,
	.max = 1,
	.step = 1,
};

static const struct v4l2_ctrl_config grb_dformat = {
	.ops = &grb_ctrl_ops,
	.id = RCM_GRB_CID_D_FORMAT,
	.name = "dformat",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.def = D_FMT_YCBCR422,
	.min = 0,
	.max = 2,
	.step = 1,
};

static const struct v4l2_ctrl_config grb_stdout = {
	.ops = &grb_ctrl_ops,
	.id = RCM_GRB_CID_STD_OUT,
	.name = "stdout",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.def = STD_CLR_SD,
	.min = 0,
	.max = 1,
	.step = 1,
};

static int vidioc_enum_frameintervals_grb( struct file *file, void *fh, struct v4l2_frmivalenum *fival ) {
	GRB_DBG_PRINT( "vidioc_enum_frameintervals: index=%u,pixel_format=%08x,width=%u,height=%u,type=%u\n",
					fival->index, fival->pixel_format, fival->width, fival->height, fival->type );
	return -EINVAL;
}

static int vidioc_enum_framesizes_grb( struct file *file, void *fh, struct v4l2_frmsizeenum *fsize ) {
	GRB_DBG_PRINT( "vidioc_enum_framesizes: index=%u,pixel_format=%08x,type=%u\n",
					fsize->index, fsize->pixel_format, fsize->type );
	return -EINVAL;
}

static int vidioc_g_parm_grb( struct file *file, void *fh, struct v4l2_streamparm *a ) {
	struct grb_info *grb = video_drvdata(file);
	int ret;

	GRB_DBG_PRINT( "vidioc_g_parm_grb: type=%u\n", a->type );

	ret = v4l2_g_parm_cap(video_devdata(file), grb->entity.subdev, a);
	a->parm.capture.readbuffers = grb->reqv_buf_cnt;
	return ret;

}

static int vidioc_s_parm_grb( struct file *file, void *fh, struct v4l2_streamparm *a ) {
	struct grb_info *grb = video_drvdata(file);
	int ret;

	GRB_DBG_PRINT( "vidioc_s_parm_grb: type=%u\n", a->type );

	ret = v4l2_s_parm_cap(video_devdata(file), grb->entity.subdev, a);
	a->parm.capture.readbuffers = grb->reqv_buf_cnt;
	return ret;
}

static int vidioc_enum_input_grb( struct file *file, void *fh, struct v4l2_input *inp ) {
	struct grb_info *grb = video_drvdata(file);

	GRB_DBG_PRINT( "vidioc_enum_input_grb: index=%u\n", inp->index );
	if( inp->index != 0 )
		return -EINVAL;

	snprintf( inp->name, sizeof(inp->name), "%s(%08x)", RCM_GRB_DEVICE_NAME, (u32)grb->phys_addr_regs );
	inp->type = V4L2_INPUT_TYPE_CAMERA;
//	inp->audioset = 0;
//	inp->tuner = 0;
//	inp->std = V4L2_STD_ALL;
//	inp->status = 0;		// todo V4L2_IN_ST_NO_SIGNAL,if it's so
	inp->capabilities = 0;
	inp->reserved[0] = inp->reserved[1] = inp->reserved[2] = 0;
	return 0;
}

static int vidioc_g_input_grb( struct file *file, void *fh, unsigned int *i ) {
	GRB_DBG_PRINT( "vidioc_g_input_grb: i=%u\n", *i );
	*i = 0;
	return 0;
}

static int vidioc_s_input_grb( struct file *file, void *fh, unsigned int i ) {
	GRB_DBG_PRINT( "vidioc_s_input_grb: i=%u\n", i );
	if( i == 0)
		return 0;
	return -EINVAL;
}

// grb capture ioctl operations 
static const struct v4l2_ioctl_ops grb_ioctl_ops = {
	.vidioc_querycap			= vidioc_querycap_grb,

	.vidioc_enum_fmt_vid_cap	= vidioc_enum_fmt_vid_cap_grb,
	.vidioc_g_fmt_vid_cap		= vidioc_g_fmt_vid_cap_grb,
	.vidioc_s_fmt_vid_cap		= vidioc_s_fmt_vid_cap_grb,
	.vidioc_try_fmt_vid_cap		= vidioc_try_fmt_vid_cap_grb,

	.vidioc_g_parm				= vidioc_g_parm_grb,
	.vidioc_s_parm				= vidioc_s_parm_grb,
	.vidioc_enum_frameintervals	= vidioc_enum_frameintervals_grb,
	.vidioc_enum_framesizes		= vidioc_enum_framesizes_grb,

	.vidioc_default				= vidioc_default_grb,
	.vidioc_g_selection			= vidioc_g_selection_grb,
	.vidioc_s_selection			= vidioc_s_selection_grb,

	.vidioc_enum_input			= vidioc_enum_input_grb,
	.vidioc_g_input				= vidioc_g_input_grb,
	.vidioc_s_input				= vidioc_s_input_grb,

	.vidioc_reqbufs				= vb2_ioctl_reqbufs,
	.vidioc_create_bufs			= vb2_ioctl_create_bufs,
	.vidioc_querybuf			= vb2_ioctl_querybuf,
	.vidioc_qbuf				= vb2_ioctl_qbuf,
	.vidioc_dqbuf				= vb2_ioctl_dqbuf,
	.vidioc_expbuf				= vb2_ioctl_expbuf,
	.vidioc_prepare_buf			= vb2_ioctl_prepare_buf,
	.vidioc_streamon			= vb2_ioctl_streamon,
	.vidioc_streamoff			= vb2_ioctl_streamoff,

//	.vidioc_log_status			= v4l2_ctrl_log_status,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe
};


static irqreturn_t proc_interrupt (struct grb_info *grb) {
	int rd_data;
	void __iomem *base_addr;
	int switch_page;

	base_addr = grb->regs;
	rd_data = read_register( base_addr, ADDR_INTERRUPTION );
	if( rd_data != 1 )
		return IRQ_NONE;

	rd_data = read_register( base_addr, ADDR_INT_STATUS );

	if( rd_data & INT_BIT_TEST ) {
		GRB_DBG_PRINT( "irq_handler: test detected \n" );

		write_register( 0 , base_addr, ADDR_TEST_INT );
		set_register( INT_BIT_TEST, base_addr, ADDR_INT_STATUS );
		clr_register( INT_BIT_TEST, base_addr, ADDR_INT_MASK );
		complete_all( &grb->cmpl );
	}
	else if( rd_data & INT_BIT_END_WRITE ) {
		GRB_DBG_PRINT( "irq_handler: end write detected\n" );

		set_register( INT_BIT_END_WRITE, base_addr, ADDR_INT_STATUS );

//		if (list_empty(&grb->buf_list)) {
//			GRB_DBG_PRINT( "irq_handler: next buffer 0\n" );
//			clr_register( INT_BIT_END_WRITE, base_addr, ADDR_INT_MASK );
//			write_register( 0, base_addr, ADDR_ENABLE );
//			return IRQ_HANDLED;
//		}

		switch_page = grb->sequence & 1;

		if (buffer_flip(grb, grb->sequence++)) {
			dev_warn(grb->dev, "%s: Flip failed\n", __func__);
			capture_stop(grb);
		}

		GRB_DBG_PRINT( "irq_handler: sequence = %d; switch_page = %d\n", grb->sequence - 1, switch_page );
		//print_videobuf_queue_param2( &grb_info_ptr->videobuf_queue_grb, grb_info_ptr->frame_count-1, 0, grb_info_ptr->mem_offset1/*-4*/, grb_info_ptr->mem_offset2/*-4*/ );

	}
	else if( rd_data & INT_BIT_DONE ) {
		unsigned int frame_size, frame_param;

		GRB_DBG_PRINT( "irq_handler: scan detected\n" );

		set_register( INT_BIT_DONE, base_addr, ADDR_INT_STATUS );
		clr_register( INT_BIT_DONE, base_addr, ADDR_INT_MASK );
		frame_size = read_register( base_addr, ADDR_FRAME_SIZE );
		grb->recognize_format.height  = (frame_size >> 16) & 0xFFF;	// 27:16-height
		grb->recognize_format.width = frame_size & 0xFFF;				// 11:0-width
		frame_param = read_register( base_addr, ADDR_FRAME_PARAM );
		grb->recognize_format.pixelformat = 0;						// set format default?
		grb->recognize_format.field = ( frame_param & 0x10 ) ? V4L2_FIELD_INTERLACED : V4L2_FIELD_NONE;
		complete_all( &grb->cmpl );

		print_detected_frame_info(frame_size, frame_param);
	}
	else {																	// we do not must be here
		GRB_DBG_PRINT( "irq_handler: unhandled status %08x\n", rd_data );
		write_register( rd_data, base_addr, ADDR_INT_STATUS );				// let it be now
	}
	return IRQ_HANDLED;
}

static irqreturn_t irq_handler( int irq, void* dev ) {
	struct grb_info *grb = (struct grb_info*) dev;
	irqreturn_t grb_irq;

	spin_lock( &grb->irq_lock );
	grb_irq = proc_interrupt( grb );
	spin_unlock( &grb->irq_lock );

	return grb_irq;
}

static int grb_graph_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct grb_info *grb = notifier_to_grb(notifier);
	int ret;

	if(grb->entity.subdev)
		v4l2_ctrl_add_handler(grb->v4l2_dev.ctrl_handler, grb->entity.subdev->ctrl_handler, NULL, true);

	ret = video_register_device(&grb->video_dev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		dev_err(grb->dev, "Failed to register video device\n");
		return ret;
	}

	dev_info(grb->dev, "Device registered as '%s'.\n", video_device_node_name(&grb->video_dev));

	dev_info(grb->dev, "Notify complete, all subdevs registered\n");

	/* Create the media link. */
	dev_info(grb->dev, "creating %s:%u -> %s:%u link\n",
			grb->src_subdev->entity.name, grb->src_pad,
			grb->video_dev.entity.name, RCM_VDU_GRB_SINK);

	ret = media_create_pad_link(&grb->src_subdev->entity, grb->src_pad,
				    &grb->video_dev.entity, RCM_VDU_GRB_SINK,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);

	if (ret) {
		dev_err(grb->dev, "Couldn't create media link %d", ret);
		return ret;
	}

	dev_info(grb->dev, "Media link with subdev %s was created\n", grb->src_subdev->name);

	ret = v4l2_device_register_subdev_nodes(&grb->v4l2_dev);
	if (ret < 0)
		dev_err(grb->dev, "failed to register subdev nodes\n");

	return media_device_register(&grb->media_dev);
}

static void grb_graph_notify_unbind(struct v4l2_async_notifier *notifier,
				     struct v4l2_subdev *sd,
				     struct v4l2_async_subdev *asd)
{
	struct grb_info *grb = notifier_to_grb(notifier);

	/* Checks internally if video_dev have been init or not */
	video_unregister_device(&grb->video_dev);

	dev_info(grb->dev, "Removing %s\n", video_device_node_name(&grb->video_dev));
}

static int grb_graph_notify_bound(struct v4l2_async_notifier *notifier,
				   struct v4l2_subdev *subdev,
				   struct v4l2_async_subdev *unused)
{
	struct grb_info *grb = notifier_to_grb(notifier);

	struct grb_graph_entity *entity;
	struct v4l2_async_subdev *asd;

	grb->src_subdev = subdev;
	grb->src_pad = media_entity_get_fwnode_pad(&subdev->entity, subdev->fwnode, MEDIA_PAD_FL_SOURCE);
	if (grb->src_pad < 0) {
		dev_err(grb->dev, "Couldn't find output pad for subdev %s\n", subdev->name);
		return grb->src_pad;
	}

	/* Locate the entity corresponding to the bound subdev and store the
	 * subdev pointer.
	 */
	list_for_each_entry(asd, &grb->notifier.asd_list, asd_list) {
		entity = to_grb_entity(asd);

		if (entity->asd.match.fwnode != subdev->fwnode)
			continue;

		if (entity->subdev) {
			dev_err(grb->dev, "duplicate subdev for node %p\n",
				entity->asd.match.fwnode);
			return -EINVAL;
		}

		dev_dbg(grb->dev, "subdev %s bound\n", subdev->name);
		entity->entity = &subdev->entity;
		entity->subdev = subdev;
		return 0;
	}

	dev_err(grb->dev, "no entity for subdev %s\n", subdev->name);
	return -EINVAL;
}

static const struct v4l2_async_notifier_operations grb_graph_notify_ops = {
	.bound = grb_graph_notify_bound,
	.unbind = grb_graph_notify_unbind,
	.complete = grb_graph_notify_complete,
};

static int grb_graph_parse(struct grb_info *grb, struct device_node *node)
{
	struct device_node *ep = NULL;
	struct device_node *remote;

	ep = of_graph_get_next_endpoint(node, ep);
	if (!ep) {
		dev_err(grb->dev, "of_graph_get_next_endpoint failed\n");
		return -EINVAL;
	}

	remote = of_graph_get_remote_port_parent(ep);
	of_node_put(ep);
	if (!remote) {
		dev_err(grb->dev, "of_graph_get_remote_port_parent failed\n");
		return -EINVAL;
	}

	/* Remote node to connect */
	grb->entity.node = remote;
	grb->entity.asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
	grb->entity.asd.match.fwnode = of_fwnode_handle(remote);
	return 0;
}

static int grb_graph_init(struct grb_info *grb)
{
	int ret;

	/* Parse the graph to extract a list of subdevice DT nodes. */
	ret = grb_graph_parse(grb, grb->dev->of_node);
	if (ret < 0) {
		dev_err(grb->dev, "Graph parsing failed\n");
		return ret;
	}

	v4l2_async_notifier_init(&grb->notifier);

	ret = v4l2_async_notifier_add_subdev(&grb->notifier, &grb->entity.asd);
	if (ret) {
		dev_err(grb->dev, "v4l2_async_notifier_add_subdev failed\n");
		of_node_put(grb->entity.node);
		return ret;
	}

	grb->notifier.ops = &grb_graph_notify_ops;

	ret = v4l2_async_notifier_register(&grb->v4l2_dev, &grb->notifier);
	if (ret < 0) {
		dev_err(grb->dev, "Notifier registration failed\n");
		v4l2_async_notifier_cleanup(&grb->notifier);
		return ret;
	}

	return 0;
}

static int get_memory_buffer_address( const struct platform_device* pdev,  const char* res_name, struct resource* res_ptr ) {
	int ret;
	struct device_node* np;
	if( ( np  = of_parse_phandle( pdev->dev.of_node, res_name, 0 ) ) == NULL ) {
		dev_err( &pdev->dev, "can't get the device node of the memory-region\n" );
		return -ENODEV;
	}
	if( ( ret = of_address_to_resource( np, 0, res_ptr ) ) != 0 ) {
		dev_err( &pdev->dev, "can't get the resource of the memory area\n" );
		return ret;
	}
	return 0;
}

static int test_interrupt( struct grb_info *grb ) {	// test interrupt started,handler must be calling 
	init_completion( &grb->cmpl );
	set_register( INT_BIT_TEST, grb->regs, ADDR_INT_MASK );
	write_register( 1, grb->regs, ADDR_TEST_INT );
	if( wait_for_completion_timeout( &grb->cmpl, HZ ) == 0 ) {
		dev_err( grb->dev, "Test interrupt: timeout\n" );
		return -ETIMEDOUT;
	}
	return 0;
}

static int grb_set_default_fmt(struct grb_info *grb, const struct v4l2_pix_format *fmt)
{
	struct v4l2_format f = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	};
	int ret;

	f.fmt.pix = *fmt;

	ret = grb_try_fmt(grb, &f, NULL);
	if (ret) {
		dev_err( grb->dev, "Failed set default format! \n" );
		return ret;
	}

	grb->recognize_format.width 	= fmt->width;		// it's too, but autodetect will set new values
	grb->recognize_format.height 	= fmt->height;
	grb->recognize_format.field 	= fmt->field;

	grb->format = f;
	return 0;
}

static int device_probe( struct platform_device *pdev ) {
	int irq;
	struct grb_info *grb;
	struct resource* res;
	struct vb2_queue *q;
	int i, err;

	grb = devm_kzalloc(&pdev->dev, sizeof(struct grb_info), GFP_KERNEL);
	if( grb == NULL ) {
		dev_err( &pdev->dev, "Can't allocated memory!\n" );
		return -ENOMEM;
	}

	for (i = 0; i < RCM_GRB_MAX_FRAME; i++)
		grb->current_buf[i] = NULL;

	res = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
	grb->phys_addr_regs = res->start; // it is for information only
	grb->regs = devm_ioremap_resource( &pdev->dev, res );
	if( IS_ERR( grb->regs ) ) {
		dev_err( &pdev->dev, "can't ioremap grabber resource");
		return PTR_ERR( grb->regs );
	}

	if( read_register( grb->regs, ADDR_ID_REG) != RCM_GRB_DEVID ) {
		dev_err( &pdev->dev, "invalid identificator, device not found! \n" );
		return -ENODEV;
	}

	if( get_memory_buffer_address( pdev, "memory-region", res ) ) {
		dev_err( &pdev->dev, "can't get the base address of the memory area" );
		return -EINVAL;
	}

	grb->dev = &pdev->dev;
	mutex_init(&grb->lock);
	spin_lock_init(&grb->irq_lock);
	INIT_LIST_HEAD(&grb->buf_list);

	q = &grb->queue;

	grb->buff_phys_addr = res->start;
	grb->buff_length = res->end - res->start + 1;
	grb->buff_dma_addr = (dma_addr_t)(grb->buff_phys_addr - (pdev->dev.dma_pfn_offset << PAGE_SHIFT));

	pdev->dma_mask = DMA_BIT_MASK(32);
	pdev->dev.archdata.dma_offset = 0;

	err = dma_declare_coherent_memory( &pdev->dev, grb->buff_phys_addr, grb->buff_phys_addr, grb->buff_length );
	if( err ) {
		dev_err( &pdev->dev, "declare coherent memory %d\n" , err );
		goto err_free_mutex;
	}
	grb->kern_virt_addr = phys_to_virt( grb->buff_phys_addr );
	GRB_DBG_PRINT( "Declare coherent memory: for phys addr %llx, dma addr %llx, kern virt addr %x, size %x\n",
			 	   grb->buff_phys_addr, grb->buff_dma_addr, (u32)grb->kern_virt_addr, grb->buff_length );

	platform_set_drvdata( pdev, grb ); 

	grb->video_dev.dev_parent = grb->dev;
	grb->video_dev.fops = &fops;
	grb->video_dev.ioctl_ops = &grb_ioctl_ops;
	grb->video_dev.queue = &grb->queue;
	grb->video_dev.lock = &grb->lock;
	strlcpy( grb->video_dev.name, RCM_GRB_DRIVER_NAME, sizeof(grb->video_dev.name) );
	grb->video_dev.release = video_dev_release;

	//grb->video_dev.vfl_dir	= VFL_DIR_M2M;
	//grb->video_dev.tvnorms = V4L2_STD_ATSC_8_VSB + V4L2_STD_ATSC_16_VSB;

	grb->param.sync = SYNC_EXTERNAL;
	grb->param.std_in = STD_CLR_SD;
	grb->param.v_if = V_IF_SERIAL;
	grb->param.d_format = D_FMT_YCBCR422;
	grb->param.std_out = STD_CLR_SD;
	grb->param.alpha = 255;
	set_input_format( grb );

	grb_set_default_fmt(grb, &fmt_default);

	set_output_format( grb );

	grb->cropping.left = 0;
	grb->cropping.top  = 0;
	grb->cropping.width  = grb->format.fmt.pix.width;
	grb->cropping.height = grb->format.fmt.pix.height;

	grb->media_dev.dev = grb->dev;
	strscpy(grb->media_dev.model, "RCM Video Capture Device",
		sizeof(grb->media_dev.model));
	grb->media_dev.hw_revision = 0;

	media_device_init(&grb->media_dev);

	grb->v4l2_dev.mdev = &grb->media_dev;

	err = v4l2_device_register( grb->dev, &grb->v4l2_dev );
	if (err) {
		dev_err( grb->dev, "failed v4l2_device register %d\n", err );
		goto err_media_clean;
	}
	grb->video_dev.v4l2_dev = &grb->v4l2_dev;
	grb->video_dev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	grb->pad.flags = MEDIA_PAD_FL_SINK;
	err = media_entity_pads_init(&grb->video_dev.entity, 1, &grb->pad);
	if (err) {
		dev_err( grb->dev, "failed media pad init %d\n", err );
		goto err_release_dev;
	}

	err = v4l2_ctrl_handler_init(&grb->hdl, 5);
	if (err) {
		goto err_release_dev;
	}

	v4l2_ctrl_new_custom(&grb->hdl, &grb_sync, NULL);
	v4l2_ctrl_new_custom(&grb->hdl, &grb_stdin, NULL);
	v4l2_ctrl_new_custom(&grb->hdl, &grb_dvif, NULL);
	v4l2_ctrl_new_custom(&grb->hdl, &grb_dformat, NULL);
	v4l2_ctrl_new_custom(&grb->hdl, &grb_stdout, NULL);
	grb->v4l2_dev.ctrl_handler = &grb->hdl;

	if (grb->hdl.error) {
		err = grb->hdl.error;
		dev_err(grb->dev, "Failed to add ctrl\n");
		goto err_free_ctrl;
	}

	v4l2_ctrl_handler_setup(&grb->hdl);

	video_set_drvdata( &grb->video_dev , grb );
//	err = video_register_device( &grb->video_dev, VFL_TYPE_GRABBER, -1 );
//	if( err ) {
//		dev_err( grb->dev, "failed video_dev register %d\n", err );
//		goto err_release_dev;
//	}

	/* buffer queue */
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP;
	q->lock = &grb->lock;
	q->drv_priv = grb;
	q->buf_struct_size = sizeof(struct frame_buffer);
	q->ops = &grb_video_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 2;
	q->dev = &pdev->dev;

	err = vb2_queue_init(q);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to initialize VB2 queue\n");
		goto err_free_ctrl;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		err = irq;
		goto err_vb2_queue;
	}

	err = devm_request_irq(&pdev->dev, irq, irq_handler, IRQF_SHARED, RCM_GRB_DEVICE_NAME, grb);
	if (err) {
		dev_err(&pdev->dev, "Unable to request irq %d\n", irq);
		goto err_dev_irq;
	}
	grb->irq = irq;


	err = test_interrupt(grb);
	if( err ) {
		dev_err(&pdev->dev, "test interrupt failed\n");
		goto err_dev_irq;
	}

	err = grb_graph_init(grb);
	if (err < 0) {
		dev_err(&pdev->dev, "graph init failed\n");
		goto err_graph_init;
	}

	dev_info(&pdev->dev, "probe succesfully completed (base %08x)\n", (u32)grb->phys_addr_regs);
	return 0;

err_dev_irq:
err_graph_init:
err_vb2_queue:
	vb2_queue_release(&grb->queue);
err_free_ctrl:
	v4l2_ctrl_handler_free(&grb->hdl);
err_release_dev:
	video_unregister_device( &grb->video_dev ); 
err_media_clean:
	media_device_cleanup(&grb->media_dev);
err_free_mutex:
	mutex_destroy(&grb->lock);
	return err;
}

static int device_remove( struct platform_device* pdev )
{
	struct grb_info* grb;
	grb = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "grabber removed (base %08x)\n", (u32)grb->phys_addr_regs);

	reset_grab( grb->regs );

	v4l2_ctrl_handler_free(&grb->hdl);
	video_unregister_device( &grb->video_dev );
	media_device_unregister(&grb->media_dev);
	vb2_queue_release(&grb->queue);
	mutex_destroy(&grb->lock);
	media_device_cleanup(&grb->media_dev);
	return 0;
}

static struct of_device_id grb_match[] = 
{
	{ .compatible = "rcm,vdu-grabber" },
	{}
};

static struct platform_driver module_grb_driver =
{
	.driver = {
		.owner = THIS_MODULE,
		.name = RCM_GRB_MODULE_NAME,
		.of_match_table = grb_match
	},
	.probe	= device_probe	,
	.remove	= device_remove
};

module_platform_driver( module_grb_driver );

MODULE_LICENSE("GPL"); 
MODULE_AUTHOR("Vladimir Shalyt <Vladimir.Shalyt@astrosoft.ru>");
MODULE_DESCRIPTION("RCM SoC video capture device driver");
MODULE_DEVICE_TABLE(of, grb_match);
