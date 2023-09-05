#include <linux/list.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gfp.h>  //??? may be don"t need
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mm.h>  //??? may be don"t need
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>

#include <media/videobuf-dma-contig.h>
#include <media/videobuf-core.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>

#include <asm/delay.h>
#include <asm/io.h>
#include <asm/page.h>
//#include <asm/segment.h>
#include <asm/uaccess.h>

#include "rcm-vdu-grabber.h"

//#define RCM_VDU_GRB_DBG

#ifdef RCM_VDU_GRB_DBG
	#define GRB_DBG_PRINT(...) printk( KERN_DEBUG "[VDU_GRABER] " __VA_ARGS__ );
#else
	#define GRB_DBG_PRINT(...) while(0);
#endif

#define GRB_DBG_PRINT_PROC_CALL GRB_DBG_PRINT("%s\n",__FUNCTION__)

enum {
	RCM_VDU_GRB_SINK,
	RCM_VDU_GRB_NR_PADS
};

struct videobuf_dma_contig_memory {
	u32 magic;
	void *vaddr;
	dma_addr_t dma_handle;
	unsigned long size;
};

static const struct v4l2_pix_format fmt_default = {
	.width 			= XGA_WIDTH,
	.height 		= XGA_HEIGHT,
	.pixelformat 	= V4L2_PIX_FMT_YUV422P,
	.field 			= V4L2_FIELD_NONE,
	.colorspace 	= V4L2_COLORSPACE_DEFAULT,
	.bytesperline   = XGA_WIDTH,
	.sizeimage		= XGA_WIDTH * XGA_HEIGHT * 2,
	.ycbcr_enc		= V4L2_YCBCR_ENC_DEFAULT,
	.quantization	= V4L2_QUANTIZATION_DEFAULT,
	.priv			= 0,
	.flags			= 0,
	.xfer_func		= V4L2_XFER_FUNC_DEFAULT,
};

static const struct grb_pix_map grb_pix_map_list[] = {
	/* TODO: add all missing formats */

	/* RGB formats */
	{
		.code = MEDIA_BUS_FMT_ARGB8888_1X32,
		.pixelformat = V4L2_PIX_FMT_BGR32,
		.bpp = 4,
	},
	/* YCBCR formats */
	{
		.code = MEDIA_BUS_FMT_YVYU8_2X8,
		.pixelformat = V4L2_PIX_FMT_YUV422P,
		.bpp = 3,
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

static void print_four_cc( const char* info, unsigned int f ) {
	GRB_DBG_PRINT( "%s: '%c%c%c%c'", info, (u8)(f>>0), (u8)(f>>8), (u8)(f>>16), (u8)(f>>24) )
}

/*
static void print_v4l2_selection( const char* info, int arg, const struct v4l2_selection* s ) {
		GRB_DBG_PRINT( "%s(%08x): target=%x,flag=%x,rect=%x,%x,%x,%x\n",
						info, arg, s->target, s->flags, s->r.left, s->r.top, s->r.width, s->r.height )
}
*/

static void print_v4l2_format( const char* info, int arg, const struct v4l2_format* f ) {
	GRB_DBG_PRINT( "%s(%08x): type=%x,fmt: width=%u,height=%u,pixelformat=%x\n"
					"field=%x,bytesperline=%u,sizeimage=%x,colorspace=%x,priv=%x,flags=%x\n"
					"ycbcr_enc/hsv_enc=%x,quantization=%x,xfer_func=%x\n",
					info, arg, f->type, f->fmt.pix.width, f->fmt.pix.height, f->fmt.pix.pixelformat,
					f->fmt.pix.field, f->fmt.pix.bytesperline, f->fmt.pix.sizeimage, f->fmt.pix.colorspace,f->fmt.pix.priv, f->fmt.pix.flags,
					f->fmt.pix.ycbcr_enc, f->fmt.pix.quantization, f->fmt.pix.xfer_func )
}

static void print_v4l2_buffer( const char* info, const struct v4l2_buffer* b ) {
	GRB_DBG_PRINT( "%s: index=0x%x,type=0x%x,bytesused=0x%x,flags=0x%x,field=0x%x,sequence=0x%x,memory=0x%x,offset=0x%x,length=0x%x\n",
					info, b->index, b->type, b->bytesused, b->flags, b->field, b->sequence, b->memory, b->m.offset, b->length )
}

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
		GRB_DBG_PRINT( "get(0x%08X, 0x%08X)\n", reg, val )
	}
}

static void dump_all_regs(char *info, void __iomem *addr) {
#if 0
	GRB_DBG_PRINT("*s:\n", info);
	GRB_DBG_PRINT("CONTROL group:\n");
	print_registers(addr, ADDR_ID_REG, ADDR_REC_ENABLE);
	GRB_DBG_PRINT("STATUS group:\n");
	print_registers(addr, ADDR_ACTIVE_FRAME, ADDR_DMA_ERROR);
	GRB_DBG_PRINT("INTERRUPT group:\n");
	print_registers(addr, ADDR_INT_STATUS, ADDR_INTERRUPTION);
	GRB_DBG_PRINT("COLOR_CONVERSION group:\n");
	print_registers(addr, ADDR_GAM_ENABLE, ADDR_CH2_RANGE);
	GRB_DBG_PRINT("DMA_WR group:\n");
	print_registers(addr, ADDR_BASE_SW_ENA, ADDR_AXI_PARAM);
#endif
}

static int reset_grab( void __iomem *addr ) {
	unsigned int i;
	write_register( 0x01 , addr, ADDR_PR_RESET );
	for( i = 0; i < 50000; i++ ) {
		if( read_register( addr, ADDR_PR_RESET ) == 0 ) {
			return 0;
		}
	}
	GRB_DBG_PRINT( "Grabber reset: timeout\n" )
	return -1;
}

static int set_input_format( struct grb_info *grb_info_ptr ) {
	struct grb_parameters *param = &grb_info_ptr->param;
	u32 active_bus;
	u32 format_data;

	if( param->d_format == D_FMT_YCBCR422 ) {
		format_data  = 0x0 ;							// YCbCr(0)
		GRB_DBG_PRINT( "input format data is YCbCr(0)\n" )
		if( param->std_in == STD_CLR_SD ) {
			if( param->v_if == V_IF_SERIAL ) {			// d0
				active_bus = 0x1;
				GRB_DBG_PRINT( "input active bus is d0\n" )
			}
			else if( param->v_if == V_IF_PARALLEL )	{	// d0,d1,d2
				active_bus = 0x3;
				GRB_DBG_PRINT( "input active bus is d0,d1,d2\n" )
			}
			else {
				active_bus = 0x0;
				GRB_DBG_PRINT( "invalid parameter param.v_if!\n" ) 
				return -EINVAL;
			}
		}
		else if( param->std_in == STD_CLR_HD ) {
			active_bus = 0x2 ;							// d0,d1
			GRB_DBG_PRINT( "input active bus is d0,d1\n" )
		}
		else {
			active_bus = 0x0 ;
			GRB_DBG_PRINT( "invalid parameter param.std_in!\n" )
			return -EINVAL;
		}
		grb_info_ptr->in_f.color = YCBCR;
		GRB_DBG_PRINT( "input format is YCBCR 4:2:2\n" )
	}
	else if( param->d_format == D_FMT_YCBCR444 ) {		// 1
		format_data = 0x0 ;								// YCbCr
		GRB_DBG_PRINT( "input format data is YCbCr\n" )
		active_bus  = 0x3 ;								// d0,d1,d2
		GRB_DBG_PRINT( "input active bus is d0,d1,d2\n" )
		grb_info_ptr->in_f.color = YCBCR;
		GRB_DBG_PRINT( "input format is YCBCR 4:4:4\n" ) 
	} 
	else if( param->d_format == D_FMT_RGB888 ) {		// 1
		format_data = 0x1 ;								// RGB
		GRB_DBG_PRINT( "input format data is RGB\n" )
		active_bus  = 0x3 ;								// d0,d1,d2
		GRB_DBG_PRINT( "input active bus is d0,d1,d2\n" )
		grb_info_ptr->in_f.color = RGB ;
		GRB_DBG_PRINT( "input format is RGB 8:8:8\n" ) 
	}
	else {
		active_bus  = 0x0;
		format_data = 0x0;
		grb_info_ptr->in_f.color = 0x0;
		GRB_DBG_PRINT( "std_grb: unknown standard \n" )
		return -EINVAL;
	}

	grb_info_ptr->in_f.format_din = format_data;			// 1–RGB,0-YCbCr
	grb_info_ptr->in_f.format_din |= (active_bus << 1) ;	// 1–dv0;2–dv0,dv1;3–dv0,dv1,dv2
	grb_info_ptr->in_f.format_din |= (param->std_in << 3) ;	// 0-not duplication,1–duplication (SDTV)
	grb_info_ptr->in_f.format_din |= (param->sync << 4) ;	// synchronization: 0–external(hsync,vsync,field,data_enable); 1–internal(EAV,SAV)

	grb_info_ptr->in_f.color_std = param->std_in;			// input mode
	GRB_DBG_PRINT( "input mode is %s (%x)\n", param->std_in == STD_CLR_SD ? "SD" : "HD", param->std_in)
	grb_info_ptr->out_f.color_std = param->std_out;			// output mode (SD,HD)
	GRB_DBG_PRINT( "output mode is %s (%x)\n", param->std_out == STD_CLR_SD ? "SD" : "HD", param->std_out)

	if( param->alpha > 255 ) { 
		GRB_DBG_PRINT( "alpha > 255\n" )
		return -EINVAL;
	}
	return 0;
}

static int set_output_format( struct grb_info *grb_info_ptr, struct v4l2_format *v4l2_format_ptr ) {
// 0:   0-YCBCR, 1-RGB
// 2,1: 01–ARGB8888, 10–YCbCr422, 11-YCbCr444,YCbCr422 too?,RGB888
// 3:   0–YCbCr422, 1–YCbCr444

	grb_info_ptr->format.fmt.pix = v4l2_format_ptr->fmt.pix;

	grb_info_ptr->out_f.y_hor_size = v4l2_format_ptr->fmt.pix.width;
	grb_info_ptr->out_f.y_ver_size = v4l2_format_ptr->fmt.pix.height;

	switch( v4l2_format_ptr->fmt.pix.pixelformat ) {
	case V4L2_PIX_FMT_BGR32:									// 'BGR4'='BGRx'
		grb_info_ptr->out_f.format_dout = 0x03;
		grb_info_ptr->out_f.color = RGB;
		grb_info_ptr->out_f.y_full_size = v4l2_format_ptr->fmt.pix.bytesperline/4;
		grb_info_ptr->out_f.c_hor_size = v4l2_format_ptr->fmt.pix.width;
		grb_info_ptr->out_f.c_ver_size = v4l2_format_ptr->fmt.pix.height;
		grb_info_ptr->out_f.c_full_size = v4l2_format_ptr->fmt.pix.bytesperline/4;
		GRB_DBG_PRINT( "output pixelformat: ARGB8888\n" )
		break;
	case V4L2_PIX_FMT_NV16:										// 'NV16'
		grb_info_ptr->out_f.format_dout = 0x04;
		grb_info_ptr->out_f.color = YCBCR;
		grb_info_ptr->out_f.y_full_size = v4l2_format_ptr->fmt.pix.bytesperline;
		grb_info_ptr->out_f.c_hor_size = v4l2_format_ptr->fmt.pix.width;
		grb_info_ptr->out_f.c_ver_size = v4l2_format_ptr->fmt.pix.height;
		grb_info_ptr->out_f.c_full_size = v4l2_format_ptr->fmt.pix.bytesperline;
		GRB_DBG_PRINT( "output pixelformat: YCBCR422 two planes\n" )
		break;
	case V4L2_PIX_FMT_YUV422P:									// '422P'='Y42B'
		grb_info_ptr->out_f.format_dout = 0x06;
		grb_info_ptr->out_f.color = YCBCR;
		grb_info_ptr->out_f.y_full_size = v4l2_format_ptr->fmt.pix.bytesperline;
		grb_info_ptr->out_f.c_hor_size = v4l2_format_ptr->fmt.pix.width;
		grb_info_ptr->out_f.c_ver_size = v4l2_format_ptr->fmt.pix.height;
		grb_info_ptr->out_f.c_full_size = v4l2_format_ptr->fmt.pix.bytesperline;
		GRB_DBG_PRINT( "output pixelformat: YCBCR422 three planes\n" )
		break;
	case V4L2_PIX_FMT_YUV444M:									// 'YM24'
		grb_info_ptr->out_f.format_dout = 0x0e;
		grb_info_ptr->out_f.color = YCBCR;
		grb_info_ptr->out_f.y_full_size = v4l2_format_ptr->fmt.pix.bytesperline;
		grb_info_ptr->out_f.c_hor_size = v4l2_format_ptr->fmt.pix.width;
		grb_info_ptr->out_f.c_ver_size = v4l2_format_ptr->fmt.pix.height;
		grb_info_ptr->out_f.c_full_size = v4l2_format_ptr->fmt.pix.bytesperline;
		GRB_DBG_PRINT( "output pixelformat: YCBCR444 three planes.\n" )
		break;
	case V4L2_PIX_FMT_RGB24:									// 'RGB3',but planar,no packet!!!Is's bad.
		grb_info_ptr->out_f.format_dout = 0x07;
		grb_info_ptr->out_f.color = RGB;
		grb_info_ptr->out_f.y_full_size = v4l2_format_ptr->fmt.pix.bytesperline;
		grb_info_ptr->out_f.c_hor_size = v4l2_format_ptr->fmt.pix.width;
		grb_info_ptr->out_f.c_ver_size = v4l2_format_ptr->fmt.pix.height;
		grb_info_ptr->out_f.c_full_size = v4l2_format_ptr->fmt.pix.bytesperline;
		GRB_DBG_PRINT( "output pixelformat: RGB888\n" )
		break;
	default:
		grb_info_ptr->out_f.format_dout = 0x00;
		GRB_DBG_PRINT( "output pixelformat: unknown pixelformat!\n" )
		return -EINVAL;
	}

	if ( (grb_info_ptr->out_f.y_hor_size > grb_info_ptr->out_f.y_full_size) ||
		 (grb_info_ptr->out_f.c_hor_size > grb_info_ptr->out_f.c_full_size) ) {
		GRB_DBG_PRINT( "output size mismatch!\n" )
		return -EINVAL;
	} 
	return 0;
}

static int setup_color( struct grb_info *grb_info_ptr, void __iomem* base_addr ) {
	//void __iomem* base_addr = grb_info_ptr->base_addr_regs_grb;

	u32 color_in = grb_info_ptr->in_f.color,
		color_std_in = grb_info_ptr->in_f.color_std,
		color_out = grb_info_ptr->out_f.color,
		color_std_out = grb_info_ptr->out_f.color_std;

	if( ( ( color_in != RGB ) && ( color_in != YCBCR ) ) ||
		  ( ( color_std_in != STD_CLR_SD ) && ( color_std_in != STD_CLR_HD ) ) ||
		  ( ( color_in != RGB ) && ( color_in != YCBCR ) ) ||
		  ( ( color_std_in != STD_CLR_SD ) && ( color_std_in != STD_CLR_HD ) ) ) {
		GRB_DBG_PRINT( "color format is wrong: input-format=%u,standard=%u;output-format=%u,standard=%u\n",
					   color_in, color_std_in, color_out, color_std_out );
		return -EINVAL;
	}

	if( color_in != color_out ) {
		if( color_in == RGB ) {
			if( color_std_out == STD_CLR_SD ) {
				GRB_DBG_PRINT( "RGB->YCBCR,SD\n" )
				grb_info_ptr->c_conv = &RGB_TO_YCBCR_SD;
			} 
			else {
				GRB_DBG_PRINT( "RGB->YCBCR,HD\n" )
				grb_info_ptr->c_conv = &RGB_TO_YCBCR_HD;
			}
		}
		else if( color_in == YCBCR ) {
			if( color_std_in == STD_CLR_SD ) {
				GRB_DBG_PRINT( "YCBCR->RGB,SD\n" )
				grb_info_ptr->c_conv = &YCBCR_TO_RGB_SD;
			} 
			else {
				GRB_DBG_PRINT( "YCBCR->RGB,HD\n" )
				grb_info_ptr->c_conv = &YCBCR_TO_RGB_HD;
			}
		}
	}
	else if( (color_in == YCBCR) && (color_out == YCBCR) ) {
		GRB_DBG_PRINT( "enable conversion colour standard  for YCBCR.\n" )

		if( (color_std_in == STD_CLR_SD) && (color_std_out == STD_CLR_HD) ) {
			GRB_DBG_PRINT( "YCBCR SD->HD\n" )
			grb_info_ptr->c_conv = &YCBCR_SD_TO_HD;
		}
		else if( (color_std_in == STD_CLR_HD) & (color_std_out == STD_CLR_SD) ) {
			GRB_DBG_PRINT( "YCBCR HD->SD\n" )
			grb_info_ptr->c_conv = &YCBCR_HD_TO_SD;
		}
	}
	else // not need color conversion
		grb_info_ptr->c_conv = NULL;
	
	write_register( 0, base_addr, ADDR_CONV_ENABLE );

	if( grb_info_ptr->c_conv ) {
		write_register( grb_info_ptr->c_conv->coef[0][0], base_addr, ADDR_C_0_0 );
		write_register( grb_info_ptr->c_conv->coef[0][1], base_addr, ADDR_C_0_1 );
		write_register( grb_info_ptr->c_conv->coef[0][2], base_addr, ADDR_C_0_2 );
		write_register( grb_info_ptr->c_conv->coef[0][3], base_addr, ADDR_C_0_3 );

		write_register( grb_info_ptr->c_conv->coef[1][0], base_addr, ADDR_C_1_0 );
		write_register( grb_info_ptr->c_conv->coef[1][1], base_addr, ADDR_C_1_1 );
		write_register( grb_info_ptr->c_conv->coef[1][2], base_addr, ADDR_C_1_2 );
		write_register( grb_info_ptr->c_conv->coef[1][3], base_addr, ADDR_C_1_3 );

		write_register( grb_info_ptr->c_conv->coef[2][0], base_addr, ADDR_C_2_0 );
		write_register( grb_info_ptr->c_conv->coef[2][1], base_addr, ADDR_C_2_1 );
		write_register( grb_info_ptr->c_conv->coef[2][2], base_addr, ADDR_C_2_2 );
		write_register( grb_info_ptr->c_conv->coef[2][3], base_addr, ADDR_C_2_3 );

		write_register( grb_info_ptr->c_conv->range[0], base_addr, ADDR_CH0_RANGE );
		write_register( grb_info_ptr->c_conv->range[1], base_addr, ADDR_CH1_RANGE );
		write_register( grb_info_ptr->c_conv->range[2], base_addr, ADDR_CH2_RANGE );
		write_register( 1, base_addr, ADDR_CONV_ENABLE );
	}
	// print_registers( base_addr, ADDR_C_0_0, ADDR_CH2_RANGE );
	return 0;
}

static int check_and_correct_out_format( u32 format_dout, u32* c_hor_size, u32* c_full_size ) {
	switch( format_dout ) {
	case 0x03:			// ARGB8888
	case 0x07:			// RGB888
	case 0x04:			// YCBCR422_2
	case 0x0e:			// YCBCR444
		return 0;
	case 0x06:			// YCBCR422_3
		*c_hor_size /= 2, *c_full_size /= 2;
		return 0;
	default:			// unknown pixelformat
		return -EINVAL;
	}
}

static inline dma_addr_t videobuf_to_dma_contig_rcm( struct videobuf_buffer *buf ) {
	 dma_addr_t dma_addr = videobuf_to_dma_contig( buf );
	 return PHYS_TO_DMA( dma_addr );
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
		//			   read_register( base_addr, BASE_ADDR_TABLE_2+252 ) )
		write_register( 1, base_addr, ADDR_GAM_ENABLE );
	}
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

int setup_registers( struct grb_info *grb_info_ptr ) {
	void __iomem* base_addr = grb_info_ptr->base_addr_regs_grb;
	u32 base_addr0_dma0, base_addr1_dma0, base_addr0_dma1, base_addr1_dma1, base_addr0_dma2, base_addr1_dma2;
	u32 y_hor_size, y_ver_size, y_full_size;
	u32 c_ver_size, c_hor_size, c_full_size;
	u32 base_point_y, base_point_x;
	u32 format_din;
	u32 format_dout;

	format_din  = grb_info_ptr->in_f.format_din;
	format_dout = grb_info_ptr->out_f.format_dout;
	GRB_DBG_PRINT( "format_din: 0x%0x, format_dout: 0x%0x\n", format_din, format_dout )

	if( grb_info_ptr->cropping.width != 0 && grb_info_ptr->cropping.height !=0 ) {
		y_hor_size	= grb_info_ptr->cropping.width;
		y_ver_size	= grb_info_ptr->cropping.height;
		c_hor_size	= grb_info_ptr->cropping.width;
		c_ver_size	= grb_info_ptr->cropping.height;
	}
	else {
		y_hor_size	= grb_info_ptr->out_f.y_hor_size;
		y_ver_size	= grb_info_ptr->out_f.y_ver_size;
		c_hor_size	= grb_info_ptr->out_f.c_hor_size;
		c_ver_size	= grb_info_ptr->out_f.c_ver_size;
	}
	y_full_size = grb_info_ptr->out_f.y_full_size;
	c_full_size	= grb_info_ptr->out_f.c_full_size;
	base_point_y = grb_info_ptr->cropping.top;
	base_point_x = grb_info_ptr->cropping.left;

	if( check_and_correct_out_format( grb_info_ptr->out_f.format_dout, &c_hor_size, &c_full_size ) )
		return -EINVAL;

	grb_info_ptr->mem_offset1 = y_full_size*y_ver_size;											// plane for color component 1
	grb_info_ptr->mem_offset2 = grb_info_ptr->mem_offset1 + c_full_size*c_ver_size;				// plane for color component 2
	
	GRB_DBG_PRINT( "y_hor_size=%d,y_ver_size=%d,c_hor_size=%d,c_ver_size=%d,y_full_size=%d,c_full_size=%d,mem_offset1=%d,mem_offset2=%d,alpha=%d\n",
			 y_hor_size, y_ver_size, c_hor_size, c_ver_size, y_full_size, c_full_size,
			 grb_info_ptr->mem_offset1, grb_info_ptr->mem_offset2, grb_info_ptr->param.alpha )
	print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 2 ); 						// print vaddr,dma_handle,size for each buffer

	base_addr0_dma0 = videobuf_to_dma_contig_rcm( grb_info_ptr->videobuf_queue_grb.bufs[0] );	// just return dma address
	//base_addr0_dma1 = base_addr0_dma0 + grb_info_ptr->mem_offset1;
	//base_addr0_dma2 = base_addr0_dma0 + grb_info_ptr->mem_offset2;
	setup_dma_addr( grb_info_ptr->out_f.format_dout, grb_info_ptr->mem_offset1, grb_info_ptr->mem_offset2,
					base_addr0_dma0, &base_addr0_dma1, &base_addr0_dma2 );

	base_addr1_dma0 = videobuf_to_dma_contig_rcm( grb_info_ptr->videobuf_queue_grb.bufs[1] );
	//base_addr1_dma1 = base_addr1_dma0 + grb_info_ptr->mem_offset1;
	//base_addr1_dma2 = base_addr1_dma0 + grb_info_ptr->mem_offset2;
	setup_dma_addr( grb_info_ptr->out_f.format_dout, grb_info_ptr->mem_offset1, grb_info_ptr->mem_offset2,
					base_addr1_dma0, &base_addr1_dma1, &base_addr1_dma2 );

	grb_info_ptr->frame_count = 0;
	grb_info_ptr->next_buf_num = 2;

	GRB_DBG_PRINT( "base_addr0_dma0=%08x,base_addr1_dma0=%08x,base_addr0_dma1=%08x,base_addr1_dma1=%08x,base_addr0_dma2=%08x,base_addr1_dma2=%08x\n",
			 base_addr0_dma0, base_addr1_dma0, base_addr0_dma1, base_addr1_dma1, base_addr0_dma2, base_addr1_dma2 )

	reset_grab( base_addr );
	write_register( 1, base_addr, ADDR_BASE_SW_ENA );											// enable switching base addresses
	write_register( base_addr0_dma0, base_addr, ADDR_DMA0_ADDR0 );								// luminance,odd
	write_register( base_addr0_dma1, base_addr, ADDR_DMA1_ADDR0 );								// color difference 1
	write_register( base_addr0_dma2, base_addr, ADDR_DMA2_ADDR0 );								// color difference 2
	write_register( base_addr1_dma0, base_addr, ADDR_DMA0_ADDR1 );								// luminance
	write_register( base_addr1_dma1, base_addr, ADDR_DMA1_ADDR1 );								// color difference 1
	write_register( base_addr1_dma2, base_addr, ADDR_DMA2_ADDR1 );								// color difference 2
	write_register( U16x2_TO_U32( y_ver_size, y_hor_size ), base_addr, ADDR_Y_SIZE );			// 27:16-height,11:0-width for luminance
	write_register( U16x2_TO_U32( c_ver_size, c_hor_size ), base_addr, ADDR_C_SIZE );			// 27:16-height,11:0-width color difference
	write_register( U16x2_TO_U32( c_full_size, y_full_size ), base_addr, ADDR_FULL_LINE_SIZE );	// 27:16-height,11:0-width full string
	write_register( U16x2_TO_U32( base_point_y, base_point_x ), base_addr, ADDR_BASE_POINT );	// 27:16-base point vert coord,11:0-ase point hor coord
	write_register( grb_info_ptr->param.alpha, base_addr, ADDR_TRANSPARENCY );
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
	setup_gamma( base_addr, &grb_info_ptr->gam );
	setup_color( grb_info_ptr, base_addr );
	set_register( INT_BIT_END_WRITE, base_addr, ADDR_INT_MASK  );
	write_register( 1 , base_addr, ADDR_ENABLE );

	return 0;
}

static int buf_setup_grb ( struct videobuf_queue *q, unsigned int *count, unsigned int *size ) {
	struct grb_info *grb_info_ptr = q->priv_data;
	int max_buff;
	*size = grb_info_ptr->format.fmt.pix.sizeimage;
	GRB_DBG_PRINT( "buf setup entry: buff_length=%u,size=%u,count=%u\n", grb_info_ptr->buff_length, *size, *count )
	if( *size == 0 )
		return -EINVAL;
	max_buff = grb_info_ptr->buff_length / *size;
	if( *count < 2 )
		*count = 2;
	else if( *count > 32 )
		*count = 32;
	if( *count > max_buff )
		*count = max_buff;
	grb_info_ptr->reqv_buf_cnt = *count;	// save available buffers count
	//print_videobuf_queue_param( q, 4 );
	GRB_DBG_PRINT( "buf setup return: buff_length=%u,size=%u,count=%u\n", grb_info_ptr->buff_length, *size, *count )
	return 0;
}

static int buf_prepare_grb ( struct videobuf_queue *q, struct videobuf_buffer *vb, enum v4l2_field field ) {
	struct grb_info *grb_info_ptr = q->priv_data;
	GRB_DBG_PRINT_PROC_CALL
	vb->size = grb_info_ptr->format.fmt.pix.sizeimage;
	vb->width = grb_info_ptr->format.fmt.pix.bytesperline;
	vb->height = grb_info_ptr->format.fmt.pix.height;
	vb->field = field;

	if( vb->state == VIDEOBUF_NEEDS_INIT ) {
		int ret = videobuf_iolock( q, vb, NULL );
		if( ret ) {
			GRB_DBG_PRINT( "videobuf_iolock failed (%d)\n", ret )
			return ret;
		}
	}
	vb->state = VIDEOBUF_PREPARED;
	//print_videobuf_queue_param( q, 4 );
	return 0;
}

static void buf_queue_grb ( struct videobuf_queue *q, struct videobuf_buffer *vb ) {
	struct grb_info* grb_info_ptr = q->priv_data;
	GRB_DBG_PRINT_PROC_CALL
	list_add_tail( &vb->queue, &grb_info_ptr->buffer_queue );	// Note that videobuf holds the lock when it calls us, so we need not (indeed, cannot) take it here
	vb->state = VIDEOBUF_QUEUED;
	//print_videobuf_queue_param( q, 4 );
}

static void buf_release_grb ( struct videobuf_queue *q, struct videobuf_buffer *vb ) {
	struct grb_info* grb_info_ptr = q->priv_data;
	unsigned long flags;
	GRB_DBG_PRINT_PROC_CALL
	spin_lock_irqsave( &grb_info_ptr->irq_lock, flags );
	INIT_LIST_HEAD( &grb_info_ptr->buffer_queue );				// We need to flush the buffer from the dma queue since hey are de-allocated
	spin_unlock_irqrestore( &grb_info_ptr->irq_lock, flags );
	videobuf_dma_contig_free( q, vb );
	vb->state = VIDEOBUF_NEEDS_INIT;
	//print_videobuf_queue_param( q, 4 );
}

static const struct videobuf_queue_ops videobuf_queue_ops_grb =
{
	.buf_setup   = buf_setup_grb,	// Is called early in the I/O process, when streaming is being initiated, its purpose is to tell videobuf about the I/O stream.
									// The count parameter will be a suggested number of buffers to use; the driver should check it for rationality and adjust it if need be.
									// As a practical rule, a minimum of two buffers are needed for proper streaming, and there is usually a maximum (which cannot exceed 32) which makes sense for each device.
									// The size parameter should be set to the expected (maximum) size for each frame of data.
	.buf_prepare = buf_prepare_grb,	// Each buffer (in the form of a struct videobuf_buffer pointer) will be passed to buf_prepare().
									// which should set the buffer’s size, width, height, and field fields properly.
	.buf_queue   = buf_queue_grb,	// When a buffer is queued for I/O, it is passed to buf_queue(), which should put it onto the driver’s list of available buffers and set its state to VIDEOBUF_QUEUED.
									// Note also that videobuf may wait on the first buffer in the queue; placing other buffers in front of it could again gum up the works.
									// So use list_add_tail() to enqueue buffers.
	.buf_release = buf_release_grb,	// Finally, buf_release() is called when a buffer is no longer intended to be used.
};									// The driver should ensure that there is no I/O active on the buffer, then pass it to the appropriate free routine(s):

static int drv_vidioc_auto_detect( struct grb_info *grb_info_ptr, void *arg );

static int device_open( struct file *file_ptr ) {
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr) ;
	struct v4l2_subdev *sd = grb_info_ptr->entity.subdev;
	int ret;

	void __iomem* base_addr = grb_info_ptr->base_addr_regs_grb;

	dev_dbg( grb_info_ptr->dev, "Open video4linux2 device. Name: %s, base: %x \n" ,
			  grb_info_ptr->video_dev.name, (u32)grb_info_ptr->phys_addr_regs_grb );


//	ret = v4l2_fh_open(file_ptr);
//	if (ret < 0) {
//		dev_err(grb_info_ptr->dev, "Error! v4l2_fh_open fail.\n");
//		return ret;
//	}

//	if (!v4l2_fh_is_singular_file(file_ptr)) {
//		dev_err(grb_info_ptr->dev, "Error! v4l2_fh_file are not singular.\n");
//		goto fh_rel;
//	}

	ret = v4l2_subdev_call(sd, core, s_power, 1);
	if (ret < 0 && ret != -ENOIOCTLCMD){
		dev_err(grb_info_ptr->dev, "Error power up subdev\n");
		goto fh_rel;
	}


	drv_vidioc_auto_detect( grb_info_ptr, NULL );

	videobuf_queue_dma_contig_init( &grb_info_ptr->videobuf_queue_grb,
									&videobuf_queue_ops_grb,
									grb_info_ptr->dev,
									&grb_info_ptr->irq_lock,
									V4L2_BUF_TYPE_VIDEO_CAPTURE,
									V4L2_FIELD_NONE,
									sizeof(struct videobuf_buffer),
									grb_info_ptr,
									NULL );

	dump_all_regs("Dump regs after device_open", base_addr);

	return 0;

fh_rel:
//	v4l2_fh_release(file_ptr);
	return ret;
}

static int device_release(struct file *file_ptr ) {
	struct video_device *video_dev = video_devdata( file_ptr );
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	int err;
	struct v4l2_subdev *sd = grb_info_ptr->entity.subdev;
//	bool fh_singular;

	dev_dbg( grb_info_ptr->dev, "Close video4linux2 device. Name: %s \n" , video_dev->name );

//	fh_singular = v4l2_fh_is_singular_file(file_ptr);

	//_vb2_fop_release(file, NULL);
//	v4l2_fh_release(file_ptr);

//	if (fh_singular)
		v4l2_subdev_call(sd, core, s_power, 0);

	videobuf_stop(  &grb_info_ptr->videobuf_queue_grb );			// The call to videobuf_stop() terminates any I/O in progress-though it is still up to the driver to stop the capture engine.
	err = videobuf_mmap_free( &grb_info_ptr->videobuf_queue_grb );	// The call to videobuf_mmap_free() will ensure that all buffers have been unmapped.
	if( err )														// If so, they will all be passed to the buf_release() callback. If buffers remain mapped, videobuf_mmap_free() returns an error code instead.
		dev_err( grb_info_ptr->dev, "videobuf_mmap_free failed,err=%d\n", err );
	return 0;
}

static int device_mmap( struct file *file_ptr, struct vm_area_struct *vma ) {
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	int ret = videobuf_mmap_mapper( &grb_info_ptr->videobuf_queue_grb, vma );
	GRB_DBG_PRINT( "device_mmap return: vm_start=%lx,vm_end=%lx\n", vma->vm_start, vma->vm_end )
	return ret;
}

static void video_dev_release(struct video_device *video_dev ) {
}

static struct v4l2_file_operations fops =
{
	.owner          = THIS_MODULE,
	.open           = device_open,
	.release        = device_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = device_mmap
};

static int vidioc_querycap_grb ( struct file* file_ptr, void* fh, struct v4l2_capability* v4l2_cap_ptr )
{
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	char bus_info[32];

	strlcpy( v4l2_cap_ptr->driver, RCM_GRB_DRIVER_NAME, sizeof(v4l2_cap_ptr->driver) );
	strlcpy( v4l2_cap_ptr->card, RCM_GRB_DEVICE_NAME, sizeof(v4l2_cap_ptr->card) );
	snprintf( bus_info, sizeof(bus_info), "platform:vdu-grabber@0x%x", (u32)virt_to_phys(grb_info_ptr->base_addr_regs_grb) );
	strlcpy( v4l2_cap_ptr->bus_info, bus_info, sizeof(v4l2_cap_ptr->bus_info) ) ;
//	v4l2_cap_ptr->version = RCM_GRB_DRIVER_VERSION;
	v4l2_cap_ptr->device_caps = grb_info_ptr->video_dev.device_caps;
	v4l2_cap_ptr->capabilities = v4l2_cap_ptr->device_caps | V4L2_CAP_DEVICE_CAPS;
	v4l2_cap_ptr->reserved[0] = v4l2_cap_ptr->reserved[1] = v4l2_cap_ptr->reserved[2] = 0;
	//GRB_DBG_PRINT( "vidioc_querycap_grb return: device_caps=%x,capabilities=%x\n", v4l2_cap_ptr->device_caps, v4l2_cap_ptr->capabilities )
	return 0;
}

/*
static int vidioc_cropcap_grb ( struct file *file_ptr, void *fh, struct v4l2_cropcap *v4l2_cropcap_ptr ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	return 0;
}

static int vidioc_g_crop_grb ( struct file *file_ptr, void *fh, struct v4l2_crop *v4l2_crop_ptr ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	if (v4l2_crop_ptr->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	v4l2_crop_ptr->c = grb_info_ptr->cropping;
	return 0;
}

static int vidioc_s_crop_grb ( struct file *file_ptr,void *fh, const struct v4l2_crop *v4l2_crop_ptr ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	if (v4l2_crop_ptr->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	grb_info_ptr->cropping = v4l2_crop_ptr->c;
	return 0;
}
*/

static int vidioc_fmt( struct grb_info *grb_info_ptr, struct v4l2_format *f ) {
	if( f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ) {
		//f->fmt.pix.width  = grb_info_ptr->recognize_format.width;
		//f->fmt.pix.height = grb_info_ptr->recognize_format.height;
		//f->fmt.pix.field  = grb_info_ptr->recognize_format.field;
		//print_four_cc( "vidioc_fmt entry: ", f->fmt.pix.pixelformat );
		switch( f->fmt.pix.pixelformat  ) {
		default:
			f->fmt.pix.pixelformat  = V4L2_PIX_FMT_YUV422P;
			f->fmt.pix.bytesperline = f->fmt.pix.width;
			f->fmt.pix.sizeimage    = f->fmt.pix.bytesperline*f->fmt.pix.height*2;
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
	case 1: // V4L2_PIX_FMT_YUV422P The three components are separated into three sub-images or planes. Cb,Cr two pixel.
		fmt->flags = 0;
		strlcpy( fmt->description, "YUV 4:2:2", sizeof(fmt->description) );
		fmt->pixelformat = v4l2_fourcc( '4','2','2', 'P' );
		break;
#if 1 /* only two formats are support gstreamer */
	case 2: // V4L2_PIX_FMT_NV16,format with ½ horizontal chroma resolution, also known as YUV 4:2:2. One luminance and one chrominance plane.
		fmt->flags = 0;
		strlcpy( fmt->description, "YUV 4:2:2", sizeof(fmt->description) );
		fmt->pixelformat = v4l2_fourcc( 'N','V','1', '6' );
		break;
	case 3: // V4L2_PIX_FMT_YUV444M Planar formats with full horizontal resolution, also known as YUV and YVU 4:4:4
		fmt->flags = 0;
		strlcpy( fmt->description, "YUV 4:4:4", sizeof(fmt->description) );
		fmt->pixelformat = v4l2_fourcc( 'Y','M','2', '4' );
		break;
#endif
	default:
		return -EINVAL;
	}
	GRB_DBG_PRINT( "vidioc_enum_fmt_vid_cap_grb: type=0x%08x,index=0x%08x,pixfmt=0x%08x\n", fmt->type, fmt->index, fmt->pixelformat );
	return 0;
}

static int grb_try_fmt(struct grb_info *grb_info_ptr, struct v4l2_format *f, u32 *code)
{
	const struct grb_pix_map *vpix;
	struct v4l2_pix_format *pixfmt = &f->fmt.pix;
//	struct v4l2_subdev_format sd_fmt;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		GRB_DBG_PRINT( "Incorrect format type!\n" )
		return -EINVAL;
	}

	/* Don't accept a pixelformat that is not on the table */
	vpix = grb_pix_map_by_pixelformat(pixfmt->pixelformat);
	if (!vpix) {
		pixfmt->pixelformat = fmt_default.pixelformat;
	}

	if (pixfmt->field == V4L2_FIELD_ANY)
		pixfmt->field = fmt_default.field;

	/* Limit to hardware capabilities */
	if (pixfmt->width > RCM_GRB_MAX_SUPPORT_WIDTH)
		pixfmt->width = RCM_GRB_MAX_SUPPORT_WIDTH;
	if (pixfmt->height > RCM_GRB_MAX_SUPPORT_HEIGHT)
		pixfmt->height = RCM_GRB_MAX_SUPPORT_HEIGHT;
	
	if (pixfmt->width < RCM_GRB_MIN_SUPPORT_WIDTH)
		pixfmt->width = fmt_default.width;
	if (pixfmt->height < RCM_GRB_MIN_SUPPORT_HEIGHT)
		pixfmt->height = fmt_default.height;

//	sd_fmt.pad = grb_info_ptr->src_pad;
//	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
//	v4l2_subdev_call(grb_info_ptr->src_subdev, pad, get_fmt, NULL, &sd_fmt);
//	v4l2_fill_pix_format(pixfmt, &sd_fmt.format);

	switch( pixfmt->pixelformat  ) {
	default:
		pixfmt->pixelformat  = V4L2_PIX_FMT_YUV422P;
		pixfmt->bytesperline = pixfmt->width;
		pixfmt->sizeimage    = pixfmt->bytesperline*pixfmt->height*2;
		GRB_DBG_PRINT( "Unsupported pixelformat format. Set to default.\n" )
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
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_YUV422P:
		pixfmt->bytesperline = pixfmt->width;
		pixfmt->sizeimage    = pixfmt->bytesperline*pixfmt->height*2;
		break;
	}

	pixfmt->field = V4L2_FIELD_NONE;

	return 0;
}

static int vidioc_g_fmt_vid_cap_grb ( struct file *file_ptr, void *fh, struct v4l2_format *f ) {
	int ret = 0;
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	ret = vidioc_fmt( grb_info_ptr, f );
	*f = grb_info_ptr->format;
	print_v4l2_format( "Vidioc_g_fmt_vid_cap_grb return", (int)grb_info_ptr->phys_addr_regs_grb, f );
	return ret;
}

static int vidioc_try_fmt_vid_cap_grb( struct file *file_ptr, void *fh, struct v4l2_format *f ) {
	int ret;
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
//	ret = vidioc_fmt( grb_info_ptr, f ) || set_output_format( grb_info_ptr, f );
	ret = grb_try_fmt(grb_info_ptr, f, NULL);
	print_v4l2_format( "Vidioc_try_fmt_vid_cap_grb return", (int)grb_info_ptr->phys_addr_regs_grb, f );
	return ret;
}

static int vidioc_s_fmt_vid_cap_grb ( struct file *file_ptr, void *fh, struct v4l2_format *f ) {
	int ret;
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );

	ret = grb_try_fmt(grb_info_ptr, f, NULL);
	if(ret)
		return ret;

	grb_info_ptr->format = *f;

	ret = set_output_format( grb_info_ptr, f );	//negotiate the format of data (typically image format) exchanged between driver and application
	print_v4l2_format( "Vidioc_s_fmt_vid_cap_grb return", (int)grb_info_ptr->phys_addr_regs_grb, f );
	return ret;
}

static int vidioc_reqbufs_grb ( struct file *file_ptr, void *fh, struct v4l2_requestbuffers *req ) {
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	unsigned long flags;
	int ret;
	GRB_DBG_PRINT_PROC_CALL
	spin_lock_irqsave( &grb_info_ptr->irq_lock, flags );
	INIT_LIST_HEAD( &grb_info_ptr->buffer_queue );
	spin_unlock_irqrestore( &grb_info_ptr->irq_lock, flags );
	ret = videobuf_reqbufs( &grb_info_ptr->videobuf_queue_grb, req );
	print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 4 );
	return ret;
}

static int vidioc_querybuf_grb( struct file *file_ptr, void *fh, struct v4l2_buffer *buf ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	int ret = videobuf_querybuf(&grb_info_ptr->videobuf_queue_grb, buf);
	print_v4l2_buffer( "vidioc_querybuf_grb", buf );
	print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 4 );
	return ret;
}

static int vidioc_qbuf_grb( struct file *file_ptr, void *fh, struct v4l2_buffer *buf ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	int ret = videobuf_qbuf(&grb_info_ptr->videobuf_queue_grb, buf);
	print_v4l2_buffer( "vidioc_qbuf_grb", buf );
	print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 4 );
	return ret;
}

static int vidioc_dqbuf_grb( struct file *file_ptr, void *fh, struct v4l2_buffer *buf ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	struct videobuf_queue* videobuf_queue = &grb_info_ptr->videobuf_queue_grb;
	int ret = videobuf_dqbuf( videobuf_queue, buf, file_ptr->f_flags & O_NONBLOCK );
	print_v4l2_buffer( "vidioc_dqbuf_grb", buf );
	print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 4 );
	return ret;
}

static int vidioc_streamon_grb( struct file *file_ptr, void *fh, enum v4l2_buf_type type ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	int retval;
	void __iomem* base_addr = grb_info_ptr->base_addr_regs_grb;

	GRB_DBG_PRINT_PROC_CALL
	if( type != V4L2_BUF_TYPE_VIDEO_CAPTURE )
		return -EINVAL;

	dump_all_regs("Dump regs before vidioc_streamon_grb", base_addr);

	/* Enable stream on the sub device */
	retval = v4l2_subdev_call(grb_info_ptr->entity.subdev, video, s_stream, 1);
	if (retval && retval != -ENOIOCTLCMD) {
		dev_err(grb_info_ptr->dev, "stream on failed in subdev\n");
		return retval;
	}

	dump_all_regs("Dump regs after Enable stream on the sub device", base_addr);


	retval = setup_registers( grb_info_ptr );
	if( retval < 0 ) {
		dev_err(grb_info_ptr->dev,  "set_register failed \n");
		goto err_start_stream;
	}
	retval = videobuf_streamon( &grb_info_ptr->videobuf_queue_grb );
	if( retval < 0 ) {
		dev_err(grb_info_ptr->dev,  "videobuf_streamon failed \n");
		goto err_start_stream;
	}

	dump_all_regs("Dump regs after vidioc_streamon_grb", base_addr);
	return retval;

err_start_stream:
	v4l2_subdev_call(grb_info_ptr->entity.subdev, video, s_stream, 0);
	return retval;
}

static int vidioc_streamoff_grb( struct file *file_ptr, void *__fh, enum v4l2_buf_type type ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	int retval;
	GRB_DBG_PRINT_PROC_CALL
	if ( type != V4L2_BUF_TYPE_VIDEO_CAPTURE )
		return -EINVAL;

	/* Disable stream on the sub device */
	retval = v4l2_subdev_call(grb_info_ptr->entity.subdev, video, s_stream, 0);
	if (retval && retval != -ENOIOCTLCMD)
		dev_err(grb_info_ptr->dev, "stream off failed in subdev\n");

	retval = videobuf_streamoff( &grb_info_ptr->videobuf_queue_grb );
	reset_grab( grb_info_ptr->base_addr_regs_grb );
	return retval;
}
/*
static void print_gamma_cs( struct grb_gamma* g ) {
	int i;
	unsigned int cs = 0;
	for( i=0; i<256; i++ )
		cs += (g->table_C_R[i]<<16) + (g->table_Y_G[i]<<8) + (g->table_C_B[i]<<0);
	GRB_DBG_PRINT( "Gamma table checksum=%08x", cs )
}
*/
static void drv_set_gamma( struct grb_info *grb_info_ptr, void *arg ) {
	struct grb_gamma* gam = (struct grb_gamma*)arg;
	grb_info_ptr->gam = *gam;
	//print_gamma_cs( &grb_info_ptr->gam );
}

static void drv_vidioc_g_params( struct grb_info *grb_info_ptr, void *arg ) {
	struct grb_parameters* param = (struct grb_parameters*)arg;
	GRB_DBG_PRINT_PROC_CALL
	param = &grb_info_ptr->param;
}

static int drv_vidioc_s_params( struct grb_info *grb_info_ptr, void *arg ) { // VIDIOC_S_PARAMS
	struct grb_parameters* param = (struct grb_parameters*)arg;
	GRB_DBG_PRINT_PROC_CALL
	grb_info_ptr->param = *param;
	return set_input_format( grb_info_ptr );
}

static int drv_vidioc_auto_detect( struct grb_info *grb_info_ptr, void *arg )
{ // todo mutex
	struct v4l2_pix_format* recognize_format = (struct v4l2_pix_format*)arg;
	int retval;

	GRB_DBG_PRINT_PROC_CALL

	/* Enable stream on the sub device */
	retval = v4l2_subdev_call(grb_info_ptr->entity.subdev, video, s_stream, 1);
	if (retval && retval != -ENOIOCTLCMD) {
		GRB_DBG_PRINT( "stream on failed in subdev\n" );
		return retval;
	}


	if( reset_grab( grb_info_ptr->base_addr_regs_grb ) ) {
		GRB_DBG_PRINT( "reset failed\n" )
		return -EIO;
	}

	init_completion( &grb_info_ptr->cmpl );
	set_register( INT_BIT_DONE, grb_info_ptr->base_addr_regs_grb, ADDR_INT_STATUS );
	set_register( INT_BIT_DONE, grb_info_ptr->base_addr_regs_grb, ADDR_INT_MASK );
	write_register( 1 , grb_info_ptr->base_addr_regs_grb, ADDR_REC_ENABLE );
	//print_registers( grb_info_ptr->base_addr_regs_grb, 0x100, 0x108 );

	if( wait_for_completion_timeout( &grb_info_ptr->cmpl, (HZ/2) ) == 0 ) {
		GRB_DBG_PRINT( "Vidioc autodetection: timeout\n" )
		v4l2_subdev_call(grb_info_ptr->entity.subdev, video, s_stream, 0);
		return -ETIMEDOUT;
	}

	//TODO
	//Need get format from subdev!!!

	print_v4l2_pix_format(&grb_info_ptr->recognize_format);

	if( recognize_format )
		*recognize_format = grb_info_ptr->recognize_format;

	v4l2_subdev_call(grb_info_ptr->entity.subdev, video, s_stream, 0);

	return 0;
}

static long vidioc_default_grb( struct file *file_ptr, void *fh, bool valid_prio, unsigned int cmd, void *arg )
{
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	GRB_DBG_PRINT( "Vidioc ioctl default: cmd=%08x\n", cmd )
	switch (cmd) {
	case VIDIOC_SET_GAMMA:
		GRB_DBG_PRINT( "Vidioc ioctl default: VIDIOC_SET_GAMMA\n")
		drv_set_gamma( grb_info_ptr, arg );
		return 0;
	case VIDIOC_G_PARAMS:
		GRB_DBG_PRINT( "Vidioc ioctl default: VIDIOC_G_PARAMS\n")
		drv_vidioc_g_params( grb_info_ptr, arg );
		return 0;
    case VIDIOC_S_PARAMS:
		GRB_DBG_PRINT( "Vidioc ioctl default: VIDIOC_S_PARAMS\n")
		drv_vidioc_s_params( grb_info_ptr, arg );
		return 0;
	case VIDIOC_AUTO_DETECT:
		GRB_DBG_PRINT( "Vidioc ioctl default: VIDIOC_AUTO_DETECT\n")
		return drv_vidioc_auto_detect( grb_info_ptr, arg );
	default:
		return -ENOTTY;
	}
}
// 0-target=2,flag=0,rect=0,0,0,0; 1-target=1,flag=2,rect=0,0,0x500, 0x2d0
static int vidioc_g_selection_grb( struct file *file, void *fh, struct v4l2_selection *s ) {
	int ret = 0;
	struct grb_info *grb_info_ptr = video_drvdata(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		s->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	}

	if( s->target == V4L2_SEL_TGT_CROP ) {				// 0: Crop rectangle. Defines the cropped area.
		s->r = grb_info_ptr->cropping;
	}
	else if( s->target == V4L2_SEL_TGT_CROP_BOUNDS ) {	// 2: Bounds of the crop rectangle.
		s->r.left = 0;									// All valid crop rectangles fit inside the crop bounds rectangle.
		s->r.top = 0;
		s->r.width = grb_info_ptr->format.fmt.pix.width;
		s->r.height = grb_info_ptr->format.fmt.pix.height;
	}
	else if( s->target == V4L2_SEL_TGT_CROP_DEFAULT ) {	// 1:Suggested cropping rectangle that covers the “whole picture”.
		s->r.left = 0;									// This includes only active pixels and excludes other non-active pixels such as black pixels.
		s->r.top = 0;
		s->r.width = grb_info_ptr->format.fmt.pix.width;
		s->r.height = grb_info_ptr->format.fmt.pix.height;
	}
	else
		ret = -EINVAL;
//exit:
	if( !ret ) memset( s->reserved, 0, sizeof(s->reserved) );
	return ret;
}

static int vidioc_s_selection_grb( struct file *file, void *fh, struct v4l2_selection *s ) {
	struct grb_info *grb_info_ptr = video_drvdata(file);
	GRB_DBG_PRINT_PROC_CALL
	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	grb_info_ptr->cropping = s->r; // rect
	return 0;
}

static int vidioc_queryctrl_grb( struct file *file, void *fh, struct v4l2_queryctrl *a ) {
	GRB_DBG_PRINT( "vidioc_queryctrl_grb entry: id=%x,type=%x,name=%s\n", a->id, a->type, a->name )
	switch( a->id ) {
	case RCM_GRB_CID_SYNC:
		strlcpy( a->name, "sync", sizeof(a->name) );
		a->minimum = 0, a->maximum = 1, a->step = 1, a->default_value = SYNC_EXTERNAL;
		break;
	case RCM_GRB_CID_STD_IN:
		strlcpy( a->name, "stdin", sizeof(a->name) );
		a->minimum = 0, a->maximum = 1, a->step = 1, a->default_value = STD_CLR_HD;
		break;
	case RCM_GRB_CID_D_V_IF:
		strlcpy( a->name, "dvif", sizeof(a->name) );
		a->minimum = 0, a->maximum = 1, a->step = 1, a->default_value = V_IF_SERIAL;
		break;
	case RCM_GRB_CID_D_FORMAT:
		strlcpy( a->name, "dformat", sizeof(a->name) );
		a->minimum = 0, a->maximum = 2, a->step = 1, a->default_value = D_FMT_YCBCR422;
		break;
	case RCM_GRB_CID_STD_OUT:
		strlcpy( a->name, "stdout", sizeof(a->name) );
		a->minimum = 0, a->maximum = 1, a->step = 1, a->default_value = STD_CLR_HD;
		break;
	default:
		return -EINVAL;
	}
	a->type = V4L2_CTRL_TYPE_INTEGER;
	a->flags = 0;
	a->reserved[0] = a->reserved[1] = 0;
	return 0;
}

static int vidioc_s_ctrl_grb( struct file *file, void *fh, struct v4l2_control *a ) {
	struct grb_info *grb_info_ptr = video_drvdata(file);
	struct grb_parameters *grb_parameters_ptr = &grb_info_ptr->param;

	GRB_DBG_PRINT( "vidioc_s_ctrl_grb: id=%x,value=%x\n", a->id, a->value )
	switch( a->id ) {
	case RCM_GRB_CID_SYNC:
		grb_parameters_ptr->sync = a->value;
		break;
	case RCM_GRB_CID_STD_IN:
		grb_parameters_ptr->std_in = a->value;
		break;
	case RCM_GRB_CID_D_V_IF:
		grb_parameters_ptr->v_if = a->value;
		break;
	case RCM_GRB_CID_D_FORMAT:
		grb_parameters_ptr->d_format = a->value;
		break;
	case RCM_GRB_CID_STD_OUT:
		grb_parameters_ptr->std_out = a->value;
		break;
	case V4L2_CID_ALPHA_COMPONENT:
		grb_parameters_ptr->alpha = a->value;
		break;
	default:
		return -EINVAL;
	}
	set_input_format( grb_info_ptr );
	return 0;
}

static int vidioc_g_ctrl_grb( struct file *file, void *fh, struct v4l2_control *a ) {
	struct grb_info *grb_info_ptr = video_drvdata(file);
	struct grb_parameters *grb_parameters_ptr = &grb_info_ptr->param;

	//GRB_DBG_PRINT( "vidioc_g_ctrl_grb: id=%x,value=%x\n", a->id, a->value )
	switch( a->id ) {
	case RCM_GRB_CID_SYNC:
		a->value = grb_parameters_ptr->sync;
		break;
	case RCM_GRB_CID_STD_IN:
		a->value = grb_parameters_ptr->std_in;
		break;
	case RCM_GRB_CID_D_V_IF:
		a->value = grb_parameters_ptr->v_if;
		break;
	case RCM_GRB_CID_D_FORMAT:
		a->value = grb_parameters_ptr->d_format;
		break;
	case RCM_GRB_CID_STD_OUT:
		a->value = grb_parameters_ptr->std_out;
		break;
	case V4L2_CID_ALPHA_COMPONENT:
		a->value = grb_parameters_ptr->alpha;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int vidioc_enum_frameintervals_grb( struct file *file, void *fh, struct v4l2_frmivalenum *fival ) {
	GRB_DBG_PRINT( "vidioc_enum_frameintervals: index=%u,pixel_format=%08x,width=%u,height=%u,type=%u\n",
					fival->index, fival->pixel_format, fival->width, fival->height, fival->type )
	return -EINVAL;
}

static int vidioc_enum_framesizes_grb( struct file *file, void *fh, struct v4l2_frmsizeenum *fsize ) {
	GRB_DBG_PRINT( "vidioc_enum_framesizes: index=%u,pixel_format=%08x,type=%u\n",
					fsize->index, fsize->pixel_format, fsize->type )
	return -EINVAL;
}

static int vidioc_g_parm_grb( struct file *file, void *fh, struct v4l2_streamparm *a ) {
	struct grb_info *grb = video_drvdata(file);

	GRB_DBG_PRINT( "vidioc_g_parm_grb: type=%u\n", a->type )

	return v4l2_g_parm_cap(video_devdata(file), grb->entity.subdev, a);

}

static int vidioc_s_parm_grb( struct file *file, void *fh, struct v4l2_streamparm *a ) {
	struct grb_info *grb = video_drvdata(file);

	GRB_DBG_PRINT( "vidioc_s_parm_grb: type=%u\n", a->type )

	return v4l2_s_parm_cap(video_devdata(file), grb->entity.subdev, a);
}

static int vidioc_enum_input_grb( struct file *file, void *fh, struct v4l2_input *inp ) {
	struct grb_info *grb_info_ptr = video_drvdata(file);

	GRB_DBG_PRINT( "vidioc_enum_input_grb: index=%u\n", inp->index )
	if( inp->index != 0 )
		return -EINVAL;

	snprintf( inp->name, sizeof(inp->name), "%s(%08x)", RCM_GRB_DEVICE_NAME, (u32)grb_info_ptr->phys_addr_regs_grb );
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->audioset = 0;
	inp->tuner = 0;
	inp->std = V4L2_STD_ALL;
	inp->status = 0;		// todo V4L2_IN_ST_NO_SIGNAL,if it's so
	inp->capabilities = 0;
	inp->reserved[0] = inp->reserved[1] = inp->reserved[2] = 0;
	return 0;
}

static int vidioc_g_input_grb( struct file *file, void *fh, unsigned int *i ) {
	GRB_DBG_PRINT( "vidioc_g_input_grb: i=%u\n", *i )
	*i = 0;
	return 0;
}

static int vidioc_s_input_grb( struct file *file, void *fh, unsigned int i ) {
	GRB_DBG_PRINT( "vidioc_s_input_grb: i=%u\n", i )
	if( i == 0)
		return 0;
	return -EINVAL;
}

// grb capture ioctl operations 
static const struct v4l2_ioctl_ops grb_ioctl_ops = {
	.vidioc_querycap			= vidioc_querycap_grb,
//	.vidioc_cropcap				= vidioc_cropcap_grb,					// VIDIOC_CROPCAP
//	.vidioc_g_crop				= vidioc_g_crop_grb,					// VIDIOC_G_CROP
//	.vidioc_s_crop				= vidioc_s_crop_grb,					// VIDIOC_S_CROP
	.vidioc_enum_fmt_vid_cap	= vidioc_enum_fmt_vid_cap_grb,
	.vidioc_g_fmt_vid_cap		= vidioc_g_fmt_vid_cap_grb,
	.vidioc_s_fmt_vid_cap		= vidioc_s_fmt_vid_cap_grb,
	.vidioc_try_fmt_vid_cap		= vidioc_try_fmt_vid_cap_grb,
	.vidioc_reqbufs				= vidioc_reqbufs_grb,
	.vidioc_querybuf			= vidioc_querybuf_grb,
	.vidioc_qbuf				= vidioc_qbuf_grb,
	.vidioc_dqbuf				= vidioc_dqbuf_grb,
	.vidioc_streamon			= vidioc_streamon_grb,
	.vidioc_streamoff			= vidioc_streamoff_grb,
	.vidioc_default				= vidioc_default_grb,
	.vidioc_g_selection			= vidioc_g_selection_grb,
	.vidioc_s_selection			= vidioc_s_selection_grb,
	.vidioc_queryctrl			= vidioc_queryctrl_grb,
	.vidioc_g_ctrl				= vidioc_g_ctrl_grb,					// for future modification
	.vidioc_s_ctrl				= vidioc_s_ctrl_grb,					// ...
	.vidioc_enum_frameintervals	= vidioc_enum_frameintervals_grb,
	.vidioc_enum_framesizes		= vidioc_enum_framesizes_grb,
	.vidioc_g_parm				= vidioc_g_parm_grb,
	.vidioc_s_parm				= vidioc_s_parm_grb,
	.vidioc_enum_input			= vidioc_enum_input_grb,
	.vidioc_g_input				= vidioc_g_input_grb,
	.vidioc_s_input				= vidioc_s_input_grb
};

static struct videobuf_buffer* grb_next_buffer( struct list_head* buffer_queue ) { // from interrupt context,because without spinlock
	struct videobuf_buffer* vb = NULL;

	if( list_empty( buffer_queue ) )
		goto out;

	vb = list_entry( buffer_queue->next, struct videobuf_buffer, queue );
	list_del( &vb->queue );
	vb->state = VIDEOBUF_ACTIVE;
out:
	return vb;
}

static irqreturn_t proc_interrupt (struct grb_info *grb_info_ptr) {
	int rd_data;
	void __iomem *base_addr;
	struct videobuf_buffer *vb;
	u32 base_addr0_dma, base_addr1_dma, base_addr2_dma;
	int switch_page;

	base_addr = grb_info_ptr->base_addr_regs_grb;
	rd_data = read_register( base_addr, ADDR_INTERRUPTION );
	if( rd_data != 1 )
		return IRQ_NONE;

	rd_data = read_register( base_addr, ADDR_INT_STATUS );

	if( rd_data & INT_BIT_TEST ) {
		GRB_DBG_PRINT( "irq_handler: test detected \n" )

		write_register( 0 , base_addr, ADDR_TEST_INT );
		set_register( INT_BIT_TEST, base_addr, ADDR_INT_STATUS );
		clr_register( INT_BIT_TEST, base_addr, ADDR_INT_MASK );
		complete_all( &grb_info_ptr->cmpl );
	}
	else if( rd_data & INT_BIT_END_WRITE ) {
		GRB_DBG_PRINT( "irq_handler: end write detected\n" )

		set_register( INT_BIT_END_WRITE, base_addr, ADDR_INT_STATUS );

		if( ( vb = grb_next_buffer( &grb_info_ptr->buffer_queue ) ) == NULL ) {
			GRB_DBG_PRINT( "irq_handler: next buffer 0\n" )
			clr_register( INT_BIT_END_WRITE, base_addr, ADDR_INT_MASK );
			write_register( 0, base_addr, ADDR_ENABLE );
			return IRQ_HANDLED;
		}

		vb->state = VIDEOBUF_DONE;
		wake_up( &vb->done );
		switch_page = grb_info_ptr->frame_count & 1;

		grb_info_ptr->frame_count = grb_info_ptr->frame_count + 1;

		GRB_DBG_PRINT( "irq_handler: frame_count = %d; switch_page = %d\n", grb_info_ptr->frame_count , switch_page )
		print_videobuf_queue_param2( &grb_info_ptr->videobuf_queue_grb, grb_info_ptr->frame_count-1, 0, grb_info_ptr->mem_offset1/*-4*/, grb_info_ptr->mem_offset2/*-4*/ );

		if( grb_info_ptr->next_buf_num < grb_info_ptr->reqv_buf_cnt ) { // prepare next buffer,if it avalaible
			base_addr0_dma = videobuf_to_dma_contig_rcm( grb_info_ptr->videobuf_queue_grb.bufs[grb_info_ptr->next_buf_num] );
			//base_addr1_dma = base_addr0_dma + grb_info_ptr->mem_offset1;
			//base_addr2_dma = base_addr0_dma + grb_info_ptr->mem_offset2;
			setup_dma_addr( grb_info_ptr->out_f.format_dout, grb_info_ptr->mem_offset1, grb_info_ptr->mem_offset2,
									base_addr0_dma, &base_addr1_dma, &base_addr2_dma );

			if( switch_page == 0 ) {
				write_register( base_addr0_dma, base_addr, ADDR_DMA0_ADDR0 );
				write_register( base_addr1_dma, base_addr, ADDR_DMA1_ADDR0 );
				write_register( base_addr2_dma, base_addr, ADDR_DMA2_ADDR0 );
			}
			else {
				write_register( base_addr0_dma, base_addr, ADDR_DMA0_ADDR1 );
				write_register( base_addr1_dma, base_addr, ADDR_DMA1_ADDR1 );
				write_register( base_addr2_dma, base_addr, ADDR_DMA2_ADDR1 );
			}
			grb_info_ptr->next_buf_num = grb_info_ptr->next_buf_num + 1;
		}
	}
	else if( rd_data & INT_BIT_DONE ) {
		unsigned int frame_size, frame_param;

		GRB_DBG_PRINT( "irq_handler: scan detected\n" )

		set_register( INT_BIT_DONE, base_addr, ADDR_INT_STATUS );
		clr_register( INT_BIT_DONE, base_addr, ADDR_INT_MASK );
		frame_size = read_register( base_addr, ADDR_FRAME_SIZE );
		grb_info_ptr->recognize_format.height  = (frame_size >> 16) & 0xFFF;	// 27:16-height
		grb_info_ptr->recognize_format.width = frame_size & 0xFFF;				// 11:0-width
		frame_param = read_register( base_addr, ADDR_FRAME_PARAM );
		grb_info_ptr->recognize_format.pixelformat = 0;						// set format default?
		grb_info_ptr->recognize_format.field = ( frame_param & 0x10 ) ? V4L2_FIELD_INTERLACED : V4L2_FIELD_NONE;
		complete_all( &grb_info_ptr->cmpl );

		print_detected_frame_info(frame_size, frame_param);
	}
	else {																	// we do not must be here
		GRB_DBG_PRINT( "irq_handler: unhandled status %08x\n", rd_data )
		write_register( rd_data, base_addr, ADDR_INT_STATUS );				// let it be now
	}
	return IRQ_HANDLED;
}

static irqreturn_t irq_handler( int irq, void* dev ) {
	struct grb_info *grb_info_ptr = (struct grb_info*) dev;
	irqreturn_t grb_irq;
	spin_lock( &grb_info_ptr->irq_lock );
	grb_irq = proc_interrupt( grb_info_ptr );
	spin_unlock( &grb_info_ptr->irq_lock );
	return grb_irq;
}

static int grb_graph_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct grb_info *grb = notifier_to_grb(notifier);
	int ret;

	grb->video_dev.ctrl_handler = grb->entity.subdev->ctrl_handler;
#if 0 //need correct
	ret = grb_formats_init(grb);
	if (ret) {
		dev_err(grb->video_dev, "No supported mediabus format found\n");
		return ret;
	}
	grb_camera_set_bus_param(grb);

	ret = grb_set_default_fmt(grb);
	if (ret) {
		dev_err(grb->dev, "Could not set default format\n");
		return ret;
	}
#endif

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

static int get_memory_buffer_address( const struct platform_device* grb_device,  const char* res_name, struct resource* res_ptr ) {
	int ret;
	struct device_node* np;
	if( ( np  = of_parse_phandle( grb_device->dev.of_node, res_name, 0 ) ) == NULL ) {
		//GRB_DBG_PRINT( "ERROR : can't get the device node of the memory-region") 
		return -ENODEV;
	}
	if( ( ret = of_address_to_resource( np, 0, res_ptr ) ) != 0 ) {
		//GRB_DBG_PRINT( "ERROR : can't get the resource of the memory area")
		return ret;
	}
	return 0;
}

static int test_interrupt( struct grb_info *grb_info_ptr ) {	// test interrupt started,handler must be calling 
	init_completion( &grb_info_ptr->cmpl );
	set_register( INT_BIT_TEST, grb_info_ptr->base_addr_regs_grb, ADDR_INT_MASK );
	write_register( 1, grb_info_ptr->base_addr_regs_grb, ADDR_TEST_INT );
	if( wait_for_completion_timeout( &grb_info_ptr->cmpl, HZ ) == 0 ) {
		GRB_DBG_PRINT( "Test interrupt: timeout\n" )
		return -ETIMEDOUT;
	}
	return 0;
}

static int get_resources( struct platform_device *grb_device, struct grb_info *grb_info_ptr ) {
	struct device* dev = &grb_device->dev;
	struct resource* grb_res_ptr;

	grb_res_ptr = platform_get_resource( grb_device, IORESOURCE_MEM, 0 );
	if( !grb_res_ptr ) {
		dev_err( dev, "can't get the base address registers grabber" );
		return -EINVAL;
	}
	grb_info_ptr->phys_addr_regs_grb = grb_res_ptr->start; // it is for information only

	grb_info_ptr->base_addr_regs_grb = devm_ioremap_resource( &grb_device->dev, grb_res_ptr );
	if( IS_ERR( grb_info_ptr->base_addr_regs_grb ) ) {
		dev_err( dev, "can't ioremap grabber resource");
		return PTR_ERR( grb_info_ptr->base_addr_regs_grb );
	}

	if( read_register( grb_info_ptr->base_addr_regs_grb, ADDR_ID_REG) != RCM_GRB_DEVID ) {
		dev_err( dev, "invalid identificator, device not found! \n" );
		return -ENODEV;
	}

	if( get_memory_buffer_address( grb_device, "memory-region", grb_res_ptr ) ) {
		dev_err( dev, "can't get the base address of the memory area" );
		return -EINVAL;
	}

	grb_info_ptr->buff_phys_addr = grb_res_ptr->start;
	grb_info_ptr->buff_length = grb_res_ptr->end - grb_res_ptr->start + 1;
	grb_info_ptr->buff_dma_addr = (dma_addr_t)(grb_info_ptr->buff_phys_addr - (grb_device->dev.dma_pfn_offset << PAGE_SHIFT));

	grb_res_ptr = platform_get_resource( grb_device, IORESOURCE_IRQ, 0 );
	if( !grb_res_ptr ) {
		dev_err( dev,  "can't get base irq" );
		return -EINVAL;;
	}
	grb_info_ptr->num_irq = grb_res_ptr->start;
	return 0;
}

static int device_probe( struct platform_device *grb_device ) {
	struct grb_info *grb_info_ptr;
	int err;
	void __iomem* base_addr;


	grb_info_ptr = kzalloc(sizeof(struct grb_info), GFP_KERNEL);
	if( grb_info_ptr == NULL ) {
		dev_err( &grb_device->dev, "Can't allocated memory!\n" );
		return -ENOMEM;
	}
	grb_info_ptr->dev = &grb_device->dev;

	err = get_resources (grb_device , grb_info_ptr);
	if( err )
		goto err_free_mem;

	grb_device->dma_mask = DMA_BIT_MASK(32);
	grb_device->dev.archdata.dma_offset = 0;								// (grb_device->dev.dma_pfn_offset << PAGE_SHIFT);

	err = dma_declare_coherent_memory( &grb_device->dev,					// dev->dma_mem was filled
									   grb_info_ptr->buff_phys_addr,
									   grb_info_ptr->buff_phys_addr,		// grb_info_ptr->buff_dma_addr
									   grb_info_ptr->buff_length );			// DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE
	if( err ) {
		dev_err( grb_info_ptr->dev, "declare coherent memory %d\n" , err );
		goto err_free_mem;
	}
	grb_info_ptr->kern_virt_addr = phys_to_virt( grb_info_ptr->buff_phys_addr );
	GRB_DBG_PRINT( "Declare coherent memory: for phys addr %llx, dma addr %llx, kern virt addr %x, size %x\n",
			 	   grb_info_ptr->buff_phys_addr, grb_info_ptr->buff_dma_addr, (u32)grb_info_ptr->kern_virt_addr, grb_info_ptr->buff_length )

	platform_set_drvdata( grb_device, grb_info_ptr ); 

	grb_info_ptr->video_dev.dev_parent = grb_info_ptr->dev;
	grb_info_ptr->video_dev.fops = &fops;
	grb_info_ptr->video_dev.ioctl_ops = &grb_ioctl_ops;
	strlcpy( grb_info_ptr->video_dev.name, RCM_GRB_DRIVER_NAME, sizeof(grb_info_ptr->video_dev.name) );
	grb_info_ptr->video_dev.release = video_dev_release;
	//grb_info_ptr->video_dev.vfl_dir	= VFL_DIR_M2M;
	//grb_info_ptr->video_dev.tvnorms = V4L2_STD_ATSC_8_VSB + V4L2_STD_ATSC_16_VSB;

//	grb_info_ptr->in_f.format_din = xx;			// it'default value
	grb_info_ptr->param.sync = SYNC_EXTERNAL;
	grb_info_ptr->param.std_in = STD_CLR_HD;
	grb_info_ptr->param.v_if = V_IF_SERIAL;
	grb_info_ptr->param.d_format = D_FMT_YCBCR422;
	grb_info_ptr->param.std_out = STD_CLR_HD;
	grb_info_ptr->param.alpha = 255;
	set_input_format( grb_info_ptr );

	grb_info_ptr->recognize_format.width = 640;		// it's too,but autodetect will set new values
	grb_info_ptr->recognize_format.height = 480;
	grb_info_ptr->recognize_format.field = V4L2_FIELD_NONE;

	grb_info_ptr->cropping.left = 0;
	grb_info_ptr->cropping.top = 0;
	grb_info_ptr->cropping.width = 1;
	grb_info_ptr->cropping.height = 1;

	grb_info_ptr->media_dev.dev = grb_info_ptr->dev;
	strscpy(grb_info_ptr->media_dev.model, "RCM Video Capture Device",
		sizeof(grb_info_ptr->media_dev.model));
	grb_info_ptr->media_dev.hw_revision = 0;

	media_device_init(&grb_info_ptr->media_dev);

	grb_info_ptr->v4l2_dev.mdev = &grb_info_ptr->media_dev;

	err = v4l2_device_register( grb_info_ptr->dev, &grb_info_ptr->v4l2_dev );
	if (err) {
		dev_err( grb_info_ptr->dev, "failed v4l2_device register %d\n", err );
		goto err_media_clean;
	}
	grb_info_ptr->video_dev.v4l2_dev = &grb_info_ptr->v4l2_dev;
	grb_info_ptr->video_dev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	grb_info_ptr->pad.flags = MEDIA_PAD_FL_SINK;
	err = media_entity_pads_init(&grb_info_ptr->video_dev.entity, 1, &grb_info_ptr->pad);
	if (err) {
		dev_err( grb_info_ptr->dev, "failed media pad init %d\n", err );
		goto err_release_dev;
	}

	video_set_drvdata( &grb_info_ptr->video_dev , grb_info_ptr );
//	err = video_register_device( &grb_info_ptr->video_dev, VFL_TYPE_GRABBER, -1 );
//	if( err ) {
//		dev_err( grb_info_ptr->dev, "failed video_dev register %d\n", err );
//		goto err_release_dev;
//	}

	err = request_irq( grb_info_ptr->num_irq, irq_handler, IRQF_SHARED, RCM_GRB_DEVICE_NAME, grb_info_ptr );
	if( err ) {
		dev_err( grb_info_ptr->dev, "request_irq %u isn't availiable\n", grb_info_ptr->num_irq );
		goto err_release_dev;
	}

	err = test_interrupt( grb_info_ptr );
	if( err ) {
		dev_err( grb_info_ptr->dev, "test interrupt failed\n" );
		goto err_release_dev;
	}

	err = grb_graph_init(grb_info_ptr);
	if (err < 0) {
		dev_err( grb_info_ptr->dev, "graph init failed\n" );
		goto err_release_dev;
	}

	spin_lock_init( &grb_info_ptr->irq_lock );

	dev_info( grb_info_ptr->dev, "probe succesfully completed (base %08x)\n", (u32)grb_info_ptr->phys_addr_regs_grb );

	base_addr = grb_info_ptr->base_addr_regs_grb;
	dump_all_regs("Dump regs after probe", base_addr);

	return 0;

err_release_dev:
	video_unregister_device( &grb_info_ptr->video_dev ); 
err_media_clean:
	media_device_cleanup(&grb_info_ptr->media_dev);
err_free_mem:
	kfree( grb_info_ptr );
	return err;
}

static int device_remove( struct platform_device* grb_device )
{
	struct grb_info* grb_info_ptr;

	GRB_DBG_PRINT( "In grabber_remove function : %p\n", grb_device )

	grb_info_ptr = platform_get_drvdata(grb_device);
	reset_grab( grb_info_ptr->base_addr_regs_grb );

	video_unregister_device( &grb_info_ptr->video_dev );
	media_device_unregister(&grb_info_ptr->media_dev);
	media_device_cleanup(&grb_info_ptr->media_dev);

	if( grb_info_ptr->num_irq )
		free_irq( grb_info_ptr->num_irq, grb_info_ptr );

	if(  grb_info_ptr )
		kfree( grb_info_ptr );

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
		.owner	= THIS_MODULE,
		.name = "grabber",
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
