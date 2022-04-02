
/* RCS info */
/* $Author: steves $ */
/* $Locker:  $ */
/* $Date: 2018/07/17 16:37:01 $ */
/* $Id: superes8bit.c,v 1.26 2018/07/17 16:37:01 steves Exp $ */
/* $Revision: 1.26 $ */
/* $State: Exp $ */

#include <superes8bit.h>

/* Macro definitions. */
#define EST_PER_RADIAL_REF		1846
#define EST_PER_RADIAL_DOP		1206
#define EST_PER_RADIAL_RHO		1206
#define EST_PER_RADIAL_PHI		1206
#define DATA_MAX_CHAR			 255
#define NUM_LEADING_FLAGS		   2

#define PHI_SCALE			0.702777f
#define PHI_OFFSET		 	2.0

/* Static Global Variables. */
static int Suppl_scan = RPGCS_NOT_SUPPLEMENTAL_SCAN;
static int Delta_time = 0;

/* Static Function Prototypes. */
static void Release_output_buffers( int disposition, int status );
static int Product_generation_control( int pindx, char *bdataptr );
static int MaxMindl( int pindx, void *data, int data_size, int frst_ref, int last_ref );
static int Product_header( int pindx, int velreso );
static int End_of_product_processing( int pindx, int velreso, int vol_num );


/*********************************************************************

   Description:
      Buffer control module for the 8-bit Super Resolution Base
      products.

*********************************************************************/
int SuperRes8bit_buffer_control(){

    int ref_flag, vel_flag, wid_flag, pindx, opstat, status;
    int i, cc_flag, phi_flag, elev_index;
    char *bdataptr = NULL;
    Base_data_header *radhead = NULL;

    /* Initialization section. */
    opstat = 0;
    Endelcut = 0;
    Proc_rad = 1;
    Num_prods = 0;

    /* Initialize information for each product. */
    for( pindx = DR_PROD_IND; pindx <= DP_PROD_IND; ++pindx ) {

        /* Destroy  any output buffer not already released or
           destroyed. */
        if( Prod_info[pindx].outbuf != NULL )
           RPGC_rel_outbuf( Prod_info[pindx].outbuf, DESTROY );

        Prod_info[pindx].outbuf = NULL;

        /* Clear out all the product request data. */
        memset( &Prod_info[pindx].request, 0, sizeof(User_array_t) );

        /* Initialize product specific data. */
        Prod_info[pindx].max_dl = RDBLTH;
        Prod_info[pindx].min_dl = BASEDATA_INVALID;
        Prod_info[pindx].vc = 0;
        Prod_info[pindx].pbuffind = 0;
        Prod_info[pindx].ndpb = 0;
        Prod_info[pindx].numbins = Prod_info[pindx].max_numbins;
        Prod_info[pindx].last_bin = Prod_info[pindx].numbins;

    }

    /* Done with initialization. */

    /* Request first input buffer (Radial base data) and process it. */
    bdataptr = RPGC_get_inbuf_by_name( "SR_BASEDATA", &opstat);
 
    /* Check the status of the operation.  If not NORMAL, abort. */
    if( (opstat != NORMAL) || (bdataptr == NULL) ){

       RPGC_abort();
       return 0;

    }

    /* Get the elevation index of the radial message. */
    elev_index = RPGC_get_buffer_elev_index( bdataptr );
    if( elev_index < 0 ){

       /* Free the radial buffer, log an error message, then abort. */
       RPGC_rel_inbuf( bdataptr );
       RPGC_log_msg( GL_INFO, 
           "Bad Elevation Index Returned From RPGC_get_buffer_elev_index()\n" );
       RPGC_abort();

    }

    /* Request all possible output buffers to see which products need to
       be generated. */
    for( pindx = DR_PROD_IND; pindx <= DP_PROD_IND; pindx++ ){

        unsigned short radial_type, highres_refl_type, not_recombined;
        unsigned short superres_type, dualpol_type;

        /* Check to see if there are any requests this elevation for this product. */
        int num_requests = RPGC_get_request_by_name( elev_index, Prod_info[pindx].pname, 
                                                     Prod_info[pindx].request, MAX_REQUESTS );

        /* If no requests, check next product.  We assume if there are requests, then 
           there is only one request since each product only has one customizing piece
           of information .... elevation angle. */
        if( num_requests <= 0 )
            continue;

        /* Check if this product can be generated using this basedata type. */ 
        if( RPGC_check_cut_type( bdataptr, Prod_info[pindx].type) ){
           
           /* Get the radial types. */
           radial_type = RPGC_check_radial_type( bdataptr, RADIAL_TYPE_MASK );

           /* Set the various types of interest. */
           superres_type = (radial_type & SUPERRES_TYPE);
           dualpol_type = (radial_type & DUALPOL_TYPE);
           highres_refl_type = (radial_type & HIGHRES_REFL_TYPE);
           not_recombined = !(radial_type & RECOMBINED_TYPE);

           /* If "All Products" not set, then the data must be SUPERRES_TYPE but
              not RECOMBINED_TYPE. */
            if( (All_products) 
                      || 
                (superres_type && not_recombined)
                      ||
                ((pindx == DR_PROD_IND) 
                      && 
                (highres_refl_type && not_recombined))
                      ||
                (((pindx == DC_PROD_IND) || (pindx == DP_PROD_IND)) 
                                         && 
                (highres_refl_type && dualpol_type && not_recombined))
                      ||
                (pindx == DW_PROD_IND) ){

                /* Product can be generated from this data. Get output buffer .... */
                Prod_info[pindx].outbuf = RPGC_get_outbuf_by_name_for_req( Prod_info[pindx].pname,
                                                                           Prod_info[pindx].max_size, 
                                                                           Prod_info[pindx].request, 
                                                                           &status );

                /* If output buffer acquisition not NORMAL, abort datatype. */
                if( status != NORMAL ){

                    RPGC_log_msg( GL_INFO, "RPGC_get_outbuf Failed for %s\n", Prod_info[pindx].pname );
                    RPGC_abort_request( Prod_info[pindx].request, status );

                }
                else{
 
                    /* Increment the number of products to generate. */
                    Num_prods++;

                }

            }
            else{

                /* Abort this data type because input isn't of the correct radial type. */
                RPGC_abort_request( Prod_info[pindx].request, PGM_PROD_NOT_GENERATED );

            }

        }
        /* We don't need to abort based on basedata type. */

    }

    /* Are there any products we can generate this elevation. */
    if( Num_prods <= 0 ){

        /* No products to be generated this scan ..... cycle through all the
           radials this scan ... */
        RPGC_rel_inbuf( bdataptr );
        while(1){

            int radial_status;

            bdataptr = RPGC_get_inbuf_by_name( "SR_BASEDATA", &opstat);
            if( (opstat != NORMAL) || (bdataptr == NULL)){

                RPGC_abort();
                return 0;

            }

            /* Get the next radial in the elevation. */
            radhead = (Base_data_header *) bdataptr;
            radial_status = radhead->status;

            /* We don't need the buffer any longer. */
            RPGC_rel_inbuf( bdataptr );
            
            /* Continue until end of elevation or volume, then return to waiting
               for activation. */
            if( (radial_status == GENDEL) || (radial_status == GENDVOL) )
               return 0;

        } 

    }

    /* There must be some products to generate.  Check for moments disabled. */
    RPGC_what_moments( (Base_data_header *) bdataptr, &ref_flag, &vel_flag, &wid_flag );

    /* Check any disabled moments against products which need to be generated. */

    /* Check for availability of reflectivity data. */
    if( (Prod_info[DR_PROD_IND].outbuf != NULL) && (!ref_flag) ){

        /* Reflectivity Moment disabled .. Destroy output buffer and 
           abort data type. */

        /* Reflectivity Product. */
        RPGC_rel_outbuf( Prod_info[DR_PROD_IND].outbuf, DESTROY );
        Prod_info[DR_PROD_IND].outbuf = NULL;
        RPGC_abort_request( Prod_info[DR_PROD_IND].request, PROD_DISABLED_MOMENT );

        /* Decrement the number of products being generated. */
        Num_prods--;

    }

    /* Check for availability of Correlation Coefficient and Differential Phase data. */
    radhead = (Base_data_header *) bdataptr;
    cc_flag = 0;
    phi_flag = 0;
 
    /* Go through all data fields ...... */
    for( i = 0; i < radhead->no_moments; i++ ){

        Generic_moment_t *ghdr = (Generic_moment_t *) (bdataptr + radhead->offsets[i]);
        char name[8];

        /* The name in the message is not a string so convert it to a
           string. */
        memset( name, 0, 8 );
        memcpy( name, ghdr->name, 4 );

        /* Check for match on index name and data name. */
        if( strstr( ghdr->name, Prod_info[DC_PROD_IND].moment ) != NULL )
           cc_flag = 1;

        if( strstr( ghdr->name, Prod_info[DP_PROD_IND].moment ) != NULL )
            phi_flag = 1;

    }
 
    if( (Prod_info[DC_PROD_IND].outbuf != NULL) && (!cc_flag) ){

        /* Correlation Coefficient data not available .... Destroy output 
           buffer and abort data type. */
        RPGC_rel_outbuf( Prod_info[DC_PROD_IND].outbuf, DESTROY );
        Prod_info[DC_PROD_IND].outbuf = NULL;
        RPGC_abort_request( Prod_info[DC_PROD_IND].request, PROD_DISABLED_MOMENT );

        /* Decrement the number of products being generated. */
        Num_prods--;

    }

    if( (Prod_info[DP_PROD_IND].outbuf != NULL) && (!phi_flag) ){

        /* Correlation Coefficient data not available .... Destroy output 
           buffer and abort data type. */
        RPGC_rel_outbuf( Prod_info[DP_PROD_IND].outbuf, DESTROY );
        Prod_info[DP_PROD_IND].outbuf = NULL;
        RPGC_abort_request( Prod_info[DP_PROD_IND].request, PROD_DISABLED_MOMENT );

        /* Decrement the number of products being generated. */
        Num_prods--;

    }

    /* Check for availability of velocity data. */
    if( (Prod_info[DV_PROD_IND].outbuf != NULL) && (!vel_flag) ){

        /* Velocity Moment disabled .. Destroy output buffer and 
           abort data type. */
        RPGC_rel_outbuf( Prod_info[DV_PROD_IND].outbuf, DESTROY );
        Prod_info[DV_PROD_IND].outbuf = NULL;
        RPGC_abort_request( Prod_info[DV_PROD_IND].request, PROD_DISABLED_MOMENT );

        /* Decrement the number of products being generated. */
        Num_prods--;

    }

    /* Check for availability of spectrum width data. */
    if( (Prod_info[DW_PROD_IND].outbuf != NULL) && (!wid_flag) ){

        /* Spectrum Width Moment disabled .. Destroy output buffer and
           abort data type. */
        RPGC_rel_outbuf( Prod_info[DW_PROD_IND].outbuf, DESTROY );
        Prod_info[DW_PROD_IND].outbuf = NULL;
        RPGC_abort_request( Prod_info[DW_PROD_IND].request, PROD_DISABLED_MOMENT );

        /* Decrement the number of products being generated. */
        Num_prods--;

    }

    /* If no products to be generated, go to waiting for activation. */
    if( Num_prods <= 0 ){

       /* Release the input buffer. */
       RPGC_rel_inbuf( bdataptr );

       return 0;

    }

    /* Do For All input (base data) radials until end of elevation encountered. */
    while(1){

        if( opstat == NORMAL ){

            Base_data_header *radhead = (Base_data_header *) bdataptr;

            for( pindx = DR_PROD_IND; pindx <= DP_PROD_IND; pindx++ ){

                /* Call the product generation control routine. */
                if( Prod_info[pindx].outbuf != NULL )
	            Product_generation_control( pindx, bdataptr );

            }

            /* Test for pseudo-end of elev or pseudo-end of volume encountered. */
            if( (radhead->status == PGENDEL) || (radhead->status == PGENDVOL) )
                Proc_rad = 0;

            /* Test for last radial in the elevation cut. */
            if( (radhead->status == GENDVOL) || (radhead->status == GENDEL) ){

                Endelcut = 1;

                for( pindx = DR_PROD_IND; pindx <= DP_PROD_IND; pindx++ ){

                    /* If last radial encountered, fill remaining fields in 
                       product buffer. */
                    if( Prod_info[pindx].outbuf != NULL ){

                        int vol_num = RPGC_get_buffer_vol_num( bdataptr );
                        End_of_product_processing( pindx, (int) radhead->dop_resolution,
                                                   vol_num );

                   }

                }

            }

            /* Release the input radial. */
	    RPGC_rel_inbuf( bdataptr );

            /* Get another radial if not at the end of the elevation cut. */
            if( !Endelcut ){

                /* Retrieve next input radial. */
		bdataptr = RPGC_get_inbuf_by_name( "SR_BASEDATA", &opstat );
		continue;

            }
            else{

                /* Elevation cut completed. */
                Release_output_buffers( FORWARD, NORMAL );
                break;

	    }

	}
        else {

            /* If the input data stream has been canceled for some reason, 
               destroy all product buffers obtained and return to waiting
               for activation. */
            Release_output_buffers( DESTROY, opstat );
            break;

	}

    }
	 
    /* Return to wait for activation. */
    return 0;

}
 

/*********************************************************************

   Description:
      Convenience function for releasing output buffers.

   Inputs:
      disposition - FORWARD or DESTROY
      status - NORMAL or abort reason code.

   Assumptions:
      If disposition is FORWARD, assume status is NORMAL.   If
      disposition is DESTROY, assume status is error code.

*********************************************************************/
static void Release_output_buffers( int disposition, int status ){

   int pindx;

   for( pindx = DR_PROD_IND; pindx <= DP_PROD_IND; pindx++ ){

        if( Prod_info[pindx].outbuf != NULL ){

            if( disposition == DESTROY ){

                RPGC_rel_outbuf( Prod_info[pindx].outbuf, disposition );
                RPGC_abort_request( Prod_info[pindx].request, status );


            }                
            else if( disposition == FORWARD )
                RPGC_rel_outbuf( Prod_info[pindx].outbuf, 
                                 disposition | RPGC_EXTEND_ARGS, 
                                 Prod_info[pindx].actual_size );
 
            Prod_info[pindx].outbuf = NULL;

        }

    }

/* End of Release_output_buffers() */
}


/********************************************************************* 

   Description:
      Controls the processing of data for the 256-level Base products
      on a radial basis. 

   Inputs:
      pindx - index of the Product Information structure.
      bdataptr - pointer to radial message.

   Returns:
      Always returns 0.

******************************************************************* */
static int Product_generation_control( int pindx, char *bdataptr ){

    /* Local variables */
    int volnumber, vcpnumber, elevindex, frst_rf = 0; 
    int i, ndpbyts, delta, start, end_rf = 0, excess;
    int start_date = 0, end_date = 0, start_time = 0, end_time = 0;

    double coselev, elang;
    
    Scan_Summary *summary = NULL;
    Base_data_header *radhead = (Base_data_header *) bdataptr;
    Generic_moment_t *ghdr = NULL;
    Generic_moment_t *cc_ghdr = NULL;
    Generic_moment_t *phi_ghdr = NULL;

    if( (pindx == DC_PROD_IND) || (pindx == DP_PROD_IND) ){

        /* Go through all data fields ...... */
        for( i = 0; i < radhead->no_moments; i++ ){

            char name[8];

            /* The name in the message is not a string so convert it to a
               string. */
            ghdr = (Generic_moment_t *) (bdataptr + radhead->offsets[i]);
            memset( name, 0, 8 );
            memcpy( name, ghdr->name, 4 );

            if( (pindx == DC_PROD_IND) 
                       &&
                (strstr( name, Prod_info[DC_PROD_IND].moment ) != NULL) )
                cc_ghdr = ghdr;

            if( (pindx == DP_PROD_IND) 
                       &&
                (strstr( name, Prod_info[DP_PROD_IND].moment ) != NULL) )
                phi_ghdr = ghdr;

        }

    }

    /* Beginning of product initialization at start of volume or
       start of elevation. */
    if( (radhead->status == GOODBVOL) 
                    || 
        (radhead->status == GOODBEL) ){

        /* Initialize the max and min data levels. */
	Prod_info[pindx].max_dl = RDBLTH;
	Prod_info[pindx].min_dl = BASEDATA_INVALID;

        /* Initialize the count of the number of radials in product. */
	Prod_info[pindx].radcount = 0;

        /* Buffer index counter and number of packet bytes counter 
           initialization. */
	Prod_info[pindx].pbuffind = Prod_info[pindx].ndpb = 0;
        Prod_info[pindx].packet16_data = (short *) (Prod_info[pindx].outbuf + 
                                                    sizeof(Graphic_product) + 
                                                    sizeof(Symbology_block) + 
                                                    sizeof(Packet_16_hdr_t));

        /* Get elevation angle from volume coverage pattern and elevation 
           index number. */
	volnumber = RPGC_get_buffer_vol_num( bdataptr );
        summary = RPGC_get_scan_summary( volnumber );
	vcpnumber = summary->vcp_number;
	elevindex = RPGC_get_buffer_elev_index( bdataptr );

	Prod_info[pindx].elmeas = (int) RPGCS_get_target_elev_ang( vcpnumber, 
                                                                   elevindex );
	elang = Prod_info[pindx].elmeas * .1f;

        /* Cosine of elevation angle computation for combination with 
           scale factor for the flat earth projection. */
	coselev = cos( elang * DEGTORAD );
	Prod_info[pindx].vc = coselev * 1e3f;

        /* Compute slant range (km) to a height cutoff of 70K ft AGL.
           Compute last bin of radial to process. */
        if( pindx == DR_PROD_IND )
	   Prod_info[pindx].numbins = RPGC_bins_to_ceiling( bdataptr, radhead->surv_bin_size );

        else if( (pindx == DV_PROD_IND) || (pindx == DW_PROD_IND) )
	   Prod_info[pindx].numbins = RPGC_bins_to_ceiling( bdataptr, radhead->dop_bin_size );
  
        else if( pindx == DC_PROD_IND ){

            /* If the data field is available, get the number of bins to height ceiling. */
            if( cc_ghdr != NULL ){

	        Prod_info[pindx].numbins = RPGC_bins_to_ceiling( bdataptr, cc_ghdr->bin_size );
                Prod_info[pindx].scale = cc_ghdr->scale;
                Prod_info[pindx].offset = cc_ghdr->offset;

            }
            else{

                Prod_info[pindx].numbins = 0;
                Prod_info[pindx].scale = 0;
                Prod_info[pindx].offset = 0;

            }

        }
        else if( pindx == DP_PROD_IND ){

            /* If the data field is available, get the number of bins to height ceiling. */
            if( phi_ghdr != NULL ){

	        Prod_info[pindx].numbins = RPGC_bins_to_ceiling( bdataptr, radhead->dop_bin_size );

                /* Note: We scale PHI by 4 to convert 10 bit data to 8 bit data. */
                Prod_info[pindx].scale = PHI_SCALE;
                Prod_info[pindx].offset = PHI_OFFSET;

            }
            else{

                Prod_info[pindx].numbins = 0;
                Prod_info[pindx].scale = 0;
                Prod_info[pindx].offset = 0;

            }

        }

	if( Prod_info[pindx].numbins > Prod_info[pindx].max_numbins )
	    Prod_info[pindx].numbins = Prod_info[pindx].max_numbins;

        /* Pack product header fields. */
	Product_header( pindx, (int) radhead->dop_resolution );

        /* Determine if this is a supplemental scan. */
        Suppl_scan = RPGCS_is_supplemental_scan( vcpnumber, volnumber, elevindex );
        LE_send_msg( GL_INFO, "RPGCS_is_supplemental_scan( %d, %d, %d ) Returned: %d\n",
                     vcpnumber, volnumber, elevindex, Suppl_scan );

        /* End of product initialization. */

    }
    else if( (radhead->status == GENDEL) 
                    || 
        (radhead->status == GENDVOL) ){

        start_date = (int) radhead->begin_vol_date;
        end_date = (int) radhead->date;
        start_time = (int) radhead->begin_vol_time/1000;
        end_time = (int) radhead->time/1000;
        Delta_time = RPGCS_time_span( start_date, start_time, end_date, end_time );

        LE_send_msg( GL_INFO, "RPGCS_time_span( %d, %d, %d, %d ) Returns: %d\n",
                     start_date, start_time, end_date, end_time, Delta_time );

    }

    /* Perform individual radial processing if radial not flagged 'BAD'. */
    if( Proc_rad ){

        /* Retrieve start angle and delta angle measurements from the 
           input radial buffer header. */
	start = radhead->start_angle;
	delta = radhead->delta_angle;

        /* Calculate the start and end bin indices of the good data. */
        if( pindx == DR_PROD_IND ){

	    frst_rf = radhead->surv_range - 1;
	    end_rf = frst_rf + radhead->n_surv_bins - 1;

        }
        else if( (pindx == DV_PROD_IND) || (pindx == DW_PROD_IND)){

	    frst_rf = radhead->dop_range - 1;
	    end_rf = frst_rf + radhead->n_dop_bins - 1;

        }
        else if( pindx == DC_PROD_IND){

            if( cc_ghdr != NULL ){

               frst_rf = cc_ghdr->first_gate_range / cc_ghdr->bin_size;
               end_rf = frst_rf + cc_ghdr->no_of_gates - 1;

            }
            else {

               /* Assume there is no data. */
               frst_rf = 0;
               end_rf = -1;

            }

        }
        else if( pindx == DP_PROD_IND){

            if( phi_ghdr != NULL ){

                frst_rf = phi_ghdr->first_gate_range / phi_ghdr->bin_size;
                end_rf = frst_rf + phi_ghdr->no_of_gates - 1;

            }
            else {

                /* Assume there is no data. */
                frst_rf = 0;
                end_rf = -1;

            }

        }

	if( Prod_info[pindx].numbins < (end_rf + 1) )
           Prod_info[pindx].last_bin = Prod_info[pindx].numbins - 1;

        /* Calculate remaining words available in output buffer. */
	excess = Prod_info[pindx].max_size - 
                 (Prod_info[pindx].ndpb + sizeof(Graphic_product));
	if( ((pindx == DR_PROD_IND) && (excess > EST_PER_RADIAL_REF))
                                    ||
	    ((pindx == DC_PROD_IND) && (excess > EST_PER_RADIAL_RHO))
                                    ||
	    ((pindx == DP_PROD_IND) && (excess > EST_PER_RADIAL_PHI))
                                    ||
	    ((pindx != DR_PROD_IND) && (excess > EST_PER_RADIAL_DOP)) ){

            int pbuffind = Prod_info[pindx].pbuffind;
            int data_size = RPGC_BYTE_DATA;
            Base_data_header *bdh = (Base_data_header *) bdataptr;
            short *sdata = NULL;
            unsigned char *cdata = NULL;
            unsigned char *temp = NULL;

            if( pindx == DR_PROD_IND ){

                sdata = (short *) (bdataptr + bdh->ref_offset); 
                data_size = RPGC_SHORT_DATA;

            }
            else if( pindx == DV_PROD_IND ){

                sdata = (short *) (bdataptr + bdh->vel_offset); 
                data_size = RPGC_SHORT_DATA;

            }
            else if( pindx == DW_PROD_IND ){

                sdata = (short *) (bdataptr + bdh->spw_offset); 
                data_size = RPGC_SHORT_DATA;

            }
            else if( pindx == DC_PROD_IND ){

                if( cc_ghdr != NULL ){

                   cdata = (unsigned char *) &cc_ghdr->gate.b[0];
                   data_size = RPGC_BYTE_DATA;

                }
                else
                    cdata = NULL;

            }
            else if( pindx == DP_PROD_IND ){

                /* Note: PHI data needs to be converted to byte data by scaling by 4. */
                if( phi_ghdr != NULL ){

                   temp = calloc( 1, EST_PER_RADIAL_PHI );
                   for( i = 0; i < phi_ghdr->no_of_gates; i++ ){

                       /* Only transform data above BASEDATA_RDRNGF value. */
                       if( phi_ghdr->gate.u_s[i] > BASEDATA_RDRNGF ){

                           float f = (phi_ghdr->gate.u_s[i] - phi_ghdr->offset)/phi_ghdr->scale;
                           int t = roundf((f*PHI_SCALE) + PHI_OFFSET);

                           /* Ensure data is within bounds ... clip if necessary. */
                           if( t > 255 )
                              t = 255;

                           if( t < 2 )
                              t = 2;

                           /* Assign value to range array. */
                           temp[i] = t;

                       }
                       else
                           temp[i] = (unsigned char) phi_ghdr->gate.u_s[i];

                   }

                   cdata = temp;
                   data_size = RPGC_BYTE_DATA;

                }
                else
                    cdata = NULL;

            }
            else
               cdata = NULL;

            /* Increment the radial count. */
  	    Prod_info[pindx].radcount++;

            /* If sufficient space available, pack present radial in the output 
               buffer in packet 16 format. */
            if( data_size == RPGC_SHORT_DATA )
                ndpbyts = RPGC_digital_radial_data_array( (void *) sdata, RPGC_SHORT_DATA,
                                       frst_rf, end_rf, 0, Prod_info[pindx].numbins, 1, start, 
                                       delta, (void *) &Prod_info[pindx].packet16_data[pbuffind] );
            else
                ndpbyts = RPGC_digital_radial_data_array( (void *) cdata, RPGC_BYTE_DATA,
                                       frst_rf, end_rf, 0, Prod_info[pindx].numbins, 1, start, 
                                       delta, (void *) &Prod_info[pindx].packet16_data[pbuffind] );

            /* Update buffer counters and pointers. */
	    Prod_info[pindx].ndpb += ndpbyts;
	    Prod_info[pindx].pbuffind += ndpbyts / 2;

            /* Determine maximum positive/negative data value in the elevation   
               through the present radial. */
            if( data_size == RPGC_SHORT_DATA )
	        MaxMindl( pindx, sdata, RPGC_SHORT_DATA, frst_rf, end_rf );

            else
	        MaxMindl( pindx, cdata, RPGC_BYTE_DATA, frst_rf, end_rf );

            if( temp != NULL )
               free(temp);

	}

    }

    /* Return to buffer control routine. */
    return 0;

/* End of Product_generation_control() */
} 

/********************************************************************
   
   Description:
      Fills in product description block, symbology block information.

   Inputs:
      pindx - index into the Product Information structure.
      velreso = Velocity resolution.

   Returns:
      Always returns 0.

********************************************************************/
static int Product_header( int pindx, int velreso ){

    Graphic_product *phd = (Graphic_product *) Prod_info[pindx].outbuf;
    Symbology_block *sym = (Symbology_block *) 
                           (Prod_info[pindx].outbuf + sizeof(Graphic_product));

    /* Data level threshold codes. */
    if( pindx == DR_PROD_IND ){

        phd->level_1 = -320;
        phd->level_2 = 5;
        phd->level_3 = 254;

    }
    else if( pindx == DV_PROD_IND ){

        phd->level_1 = -635*velreso;
        phd->level_2 = 5*velreso;
        phd->level_3 = 254;


    }
    else if( pindx == DW_PROD_IND ){

        phd->level_1 = 0;
        phd->level_2 = 5;
        phd->level_3 = 43;

    }
    else if( pindx == DC_PROD_IND ){

        RPGC_set_product_float( (void *) &phd->level_1, Prod_info[pindx].scale );
        RPGC_set_product_float( (void *) &phd->level_3, Prod_info[pindx].offset );
        phd->level_6 = (short) DATA_MAX_CHAR;
        phd->level_7 = (short) NUM_LEADING_FLAGS;
        phd->level_8 = (short) 0;

    }
    else if( pindx == DP_PROD_IND ){

        RPGC_set_product_float( (void *) &phd->level_1, Prod_info[pindx].scale );
        RPGC_set_product_float( (void *) &phd->level_3, Prod_info[pindx].offset );
        phd->level_6 = (short) DATA_MAX_CHAR;
        phd->level_7 = (short) NUM_LEADING_FLAGS;
        phd->level_8 = (short) 0;

    }
    else{

        phd->level_1 = 0;
        phd->level_2 = 0;
        phd->level_3 = 0;

    }

    /* Store offset to symbology. */
    RPGC_set_prod_block_offsets( phd, sizeof(Graphic_product)/sizeof(short), 0, 0 );

    /* Store product block divider, block ID, number of layers and 
       layer divider. */
    sym->divider = -1;
    sym->block_id = 1;
    sym->n_layers = 1;
    sym->layer_divider = -1;

    /* Length of layer, length of block, length of message, scale factor,
       number of radials are stored in the end-of-product module. */
    return 0;

/* End of Product_header() */
}


/******************************************************************
   
   Description:
      Maximum/Minum data-level routine for 256-level Base Data 
      product generation program. 

   Inputs:
      pindx - index into the Product Information structure.
      data - Input buffer pointer to data.
      data_size - either RPGC_BYTE_DATA or RPGC_SHORT_DATA.
      frst_bin - index of first data bin.
      last_bin - index of last data bin.

   Returns:
      Always returns 0.

******************************************************************/
static int MaxMindl( int pindx, void *data, int data_size, 
                     int frst_bin, int last_bin ){

    short value;
    int max_dl, min_dl, binindx;

    /* Temporary storage for min and max data levels. */
    max_dl = Prod_info[pindx].max_dl;
    min_dl = Prod_info[pindx].min_dl;

    /* Do For All bins ..... */
    for( binindx = frst_bin;  binindx <= last_bin; binindx++ ){

        if( data_size == RPGC_BYTE_DATA )
            value = ((unsigned char *)data)[binindx];

        else
            value = ((short *)data)[binindx];

        /* Ensure the data value is above threshold, not range-folded
           and not invalid. */
        if( (value > RDRNGF) 
                  && 
            (value < BASEDATA_INVALID) ){

            /* Find the maximum data level. */
            if( value > max_dl ) 
		max_dl = value;

            /* Find the minimum data level. */
            if( value < min_dl ) 
		min_dl = value;
	     
        }

    }

    /* Set the minimum and maximum data levels. */
    Prod_info[pindx].max_dl = max_dl;
    Prod_info[pindx].min_dl = min_dl;

    return 0;

/* End of int MaxMindl() */
} 

#define PARAM_SUPPLEMENTAL_SCAN		 7
#define DELTA_TIME_SHIFT		 5

/**********************************************************************

   Description:
      Fill remaining Product Header fields for 256-level Base 
      Data product. 

   Inputs:
      pindx - index into the Product Information structure.
      velreso - velocity resolution.

   Returns:
      Always returns 0.

**********************************************************************/
static int End_of_product_processing( int pindx, int velreso, int vol_num ){

    int elev_ind, bytecnt;
    float maxval, minval;
    short params[10];

    Graphic_product *phd = (Graphic_product *) Prod_info[pindx].outbuf;
    Symbology_block *sym = (Symbology_block *) 
                           (Prod_info[pindx].outbuf + sizeof(Graphic_product));
    Packet_16_hdr_t *packet_16 = (Packet_16_hdr_t *) 
       (Prod_info[pindx].outbuf + sizeof(Graphic_product) + sizeof(Symbology_block));

    /* Complete the packet 16 header. */
    RPGC_digital_radial_data_hdr( 0, Prod_info[pindx].numbins, 0, 0, 
                                  Prod_info[pindx].vc, Prod_info[pindx].radcount, 
                                  (void *) packet_16 );

    /* Initialize the product dependent halfwords. */
    memset( params, 0, 10*sizeof(short) );

    /* Assign the maximum/minimum data level to the product header. */
    if( pindx == DR_PROD_IND ){

        maxval = RPGCS_reflectivity_to_dBZ( Prod_info[pindx].max_dl );
        params[3] = (short) maxval;

    }
    else if( pindx == DV_PROD_IND ){

        RPGCS_set_velocity_reso( velreso );

        minval = RPGCS_velocity_to_ms( Prod_info[pindx].min_dl );
        params[3] = (short) RPGC_NINT( minval * MPS_TO_KTS );

        maxval = RPGCS_velocity_to_ms( Prod_info[pindx].max_dl );
        params[4] = (short) RPGC_NINT( maxval * MPS_TO_KTS );

        RPGC_log_msg( GL_INFO, "SDV: minval: %d, maxval: %d\n", 
                      params[3], params[4] );
    }
    else if( pindx == DW_PROD_IND ){

        /* Can't use the loopup table for spectrum width since the
           lookup table stops at 10 m/s or about 20 knots. */
        maxval = ((float) Prod_info[pindx].max_dl / 2.0f) - 64.5f;
        params[3] = (short) RPGC_NINT( maxval * MPS_TO_KTS );
        RPGC_log_msg( GL_INFO, "SDW: maxval: %d\n", params[3] );

    }
    else if( pindx == DC_PROD_IND ){

        maxval = Prod_info[pindx].max_dl - Prod_info[pindx].offset;
        maxval /= Prod_info[pindx].scale;
        maxval /= 0.00333;

        minval = Prod_info[pindx].min_dl - Prod_info[pindx].offset;
        minval /= Prod_info[pindx].scale;
        minval /= 0.00333;

        params[3] = (short) RPGC_NINT( minval );
        params[4] = (short) RPGC_NINT( maxval );
        LE_send_msg( GL_INFO, "CC - Param 3 (Min Val): %d, Param 4 (Max Val): %d\n", 
                     params[3], params[4] );
        LE_send_msg( GL_INFO, "-->Prod_info[%d].min_dl: %d, Prod_info[%d].max_dl: %d\n",
                     pindx, Prod_info[pindx].min_dl, pindx, Prod_info[pindx].max_dl );
        LE_send_msg( GL_INFO, "-->minval: %f, maxval: %f\n", 
                     (float) params[3] * 0.00333, (float) params[4] * 0.00333 );

    }
    else if( pindx == DP_PROD_IND ){

        maxval = Prod_info[pindx].max_dl - Prod_info[pindx].offset;
        maxval /= Prod_info[pindx].scale;
        minval = Prod_info[pindx].min_dl - Prod_info[pindx].offset;
        minval /= Prod_info[pindx].scale;

        params[3] = (short) RPGC_NINT( minval );
        params[4] = (short) RPGC_NINT( maxval );
        LE_send_msg( GL_INFO, "PHI - Param 3 (Min Val): %d, Param 4 (Max Val): %d\n", 
                     params[3], params[4] );
        LE_send_msg( GL_INFO, "-->Prod_info[%d].min_dl: %d, Prod_info[%d].max_dl: %d\n",
                     pindx, Prod_info[pindx].min_dl, pindx, Prod_info[pindx].max_dl );
        LE_send_msg( GL_INFO, "-->minval: %f, max_val: %f\n", 
                     (float) params[3], (float) params[4] );

    }
    else{

       params[3] = 0;
       params[4] = 0;

    }

    /* Assign the Elevation Angle. */
    elev_ind = ORPGPAT_get_elevation_index( Prod_info[pindx].prod_id );
    if( elev_ind >= 0 )
        params[elev_ind] = Prod_info[pindx].elmeas;

    /* Set param_7, the supplemental scan product dependent parameter. */
    params[6] = (Delta_time << DELTA_TIME_SHIFT) | Suppl_scan;
    LE_send_msg( GL_INFO, "param[6]: %d (%x)\n", params[6], params[6] );

    /* Store the product dependent halfwords. */
    RPGC_set_dep_params( (char *) phd, params );

    /* Populate the Product Description Block. */
    /* Fill in product description block fields. */
    RPGC_prod_desc_block( phd, Prod_info[pindx].prod_id, vol_num );

    /* Calculate and store the product message length, the product block 
       length and the product layer length. */

    /* Length of product layer. */
    bytecnt = Prod_info[pindx].ndpb + sizeof(Packet_16_hdr_t);
    RPGC_set_product_int( &sym->data_len, bytecnt);

    /* Length of block. */
    bytecnt += sizeof(Symbology_block);
    RPGC_set_product_int( &sym->block_len, bytecnt);

    /* Complete the product header. */
    RPGC_prod_hdr( Prod_info[pindx].outbuf, Prod_info[pindx].prod_id, &bytecnt);

    /* Save the length of the product. */
    Prod_info[pindx].actual_size = bytecnt;

    /* Return to the product generation control routine. */
    return 0;

/* End of End_of_product_processing() */
} 

