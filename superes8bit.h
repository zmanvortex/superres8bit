/*
 * RCS info
 * $Author: steves $
 * $Locker:  $
 * $Date: 2018/02/20 18:33:52 $
 * $Id: superes8bit.h,v 1.7 2018/02/20 18:33:52 steves Exp $
 * $Revision: 1.7 $
 * $State: Exp $
 */

#ifndef SUPERES8BIT_H
#define SUPERES8BIT_H

#include <rpgc.h>
#include <rpgcs.h>
#include <coldat.h>
#include <a309.h>
#include <packet_16.h>
#include <siteadp.h>

/* Data value special values. */
#define RDBLTH			0
#define RDRNGF			1

/* Maximum number of different products generated by this task. */
#define SUPER_RES_NPRODS	5
#define MAX_REQUESTS		1

/* These are buffer sizes and must be defined in terms of # of bytes. */
#define DR_PROD_IND		0
#define DR_PROD_NAME		"SR_BREF8BIT"
#define DR_OBUF_OVERHEAD	(sizeof(Graphic_product)+sizeof(Packet_16_hdr_t)+sizeof(Packet_16_data_t))
#define DR_OBUF_SIZE    	((MAX_BASEDATA_REF_SIZE*800)+DR_OBUF_OVERHEAD)

#define DV_PROD_IND		1
#define DV_PROD_NAME		"SR_BVEL8BIT"
#define DV_OBUF_OVERHEAD	(sizeof(Graphic_product)+sizeof(Packet_16_hdr_t)+sizeof(Packet_16_data_t))
#define DV_OBUF_SIZE    	((BASEDATA_DOP_SIZE*800)+DV_OBUF_OVERHEAD)

#define DW_PROD_IND		2
#define DW_PROD_NAME		"SR_BSPW8BIT"
#define DW_OBUF_OVERHEAD	(sizeof(Graphic_product)+sizeof(Packet_16_hdr_t)+sizeof(Packet_16_data_t))
#define DW_OBUF_SIZE    	((BASEDATA_DOP_SIZE*800)+DW_OBUF_OVERHEAD)

#define DC_PROD_IND		3
#define DC_PROD_NAME		"SR_BRHO8BIT"
#define DC_OBUF_OVERHEAD	(sizeof(Graphic_product)+sizeof(Packet_16_hdr_t)+sizeof(Packet_16_data_t))
#define DC_OBUF_SIZE    	((BASEDATA_RHO_SIZE*800)+DC_OBUF_OVERHEAD)

#define DP_PROD_IND		4
#define DP_PROD_NAME		"SR_BPHI8BIT"
#define DP_OBUF_OVERHEAD	(sizeof(Graphic_product)+sizeof(Packet_16_hdr_t)+sizeof(Packet_16_data_t))
#define DP_OBUF_SIZE    	((BASEDATA_PHI_SIZE*800)+DP_OBUF_OVERHEAD)

/* Global variables. */
Siteadp_adpt_t Siteadp;		/* Site adaptation data. */

int Endelcut;			/* End of elevation cut flag. */

int Proc_rad;			/* Process radial flag. */

int Num_prods;			/* Number of products to generate this scan. */

#ifdef SUPERES8BIT_MAIN
int All_products = 0;		/* By default only super resolution products
				   are generated.  If this flag is set,
                                   then super res as well as non super res
                                   products are generated.  */
#else
extern int All_products;
#endif

/* Product Information structure. */
typedef struct Product_info {

   int pcode;			/* Product code. */

   char *pname;			/* Product name string ... matches TAT name. */

   int prod_id;			/* Product ID. */

   int max_size;		/* Maximum size of product, in bytes. */

   int actual_size;		/* Actual size of product, in bytes. */

   unsigned short type;		/* Message type needed for this product (e.g., REFLDATA_TYPE
				   COMBBASE_TYPE, BASEDATA_TYPE). */

   unsigned short spare;	/* Unused. */

   int vc;			/* Flat-earth projection scaling (cos(elev)*1000.0). */

   int elmeas;			/* Elevation angle of product, in degree*10.0. */

   char *outbuf;		/* Pointer to output buffer. */

   User_array_t request[MAX_REQUESTS];	/* Pointer to this products request data. */

   short *packet16_data;	/* Pointer to output bufferi where packet 16 data
                                   is to be placed. */
   
   int max_dl;			/* Maximum data level, if applicable. */

   int min_dl;			/* Minimum data level, if applicable. */

   float scale;			/* Used for data conversion. */

   float offset;		/* Used for data conversion. */

   int pbuffind;		/* Current product index, in shorts. */

   int ndpb;			/* Current number of bytes of data in packet 16. */

   int max_numbins;		/* Maximum number of data bins in product. */

   int numbins;			/* Maximum number of data bins in product, clipped
				   to 70 Kft. */

   int last_bin;		/* Number of data bins in the radial. */

   int radcount;		/* Radial counter for this product. */

   char moment[8]; 		/* Name of data element. */

} Product_info_t;

Product_info_t Prod_info[SUPER_RES_NPRODS];

/* Function Prototypes. */
int SuperRes8bit_buffer_control();

# endif
