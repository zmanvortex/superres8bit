/* RCS info */
/* $Author: steves $ */
/* $Locker:  $ */
/* $Date: 2018/02/20 18:34:52 $ */
/* $Id: superes8bit_main.c,v 1.5 2018/02/20 18:34:52 steves Exp $ */
/* $Revision: 1.5 $ */
/* $State: Exp $ */

#define SUPERES8BIT_MAIN
#include <superes8bit.h>

static char *User_defined_commands = "A";

/* Function Prototypes. */
int Service_options( int arg, char *optarg );

/*************************************************************************

    The main function for the Super Resolution Base Data Array (256 level) 
    Products Generation task.

*************************************************************************/
int main( int argc, char *argv[] ){

    /* Register command line arguments. */
    RPGC_reg_custom_options( User_defined_commands, Service_options );

    /* Specify inputs and outputs */
    RPGC_reg_io( argc, argv );

    /* Register scan summary array */
    RPGC_reg_scan_summary();

    /* Register for site info adaptation data. */
    RPGC_reg_site_info( &Siteadp.rda_lat );

    /* Tell system we are ready to go ...... */
    RPGC_task_init( ELEVATION_BASED, argc, argv );

    /* Get the product IDs for all product generated by the task. */
    if( ((Prod_info[DR_PROD_IND].prod_id = RPGC_get_id_from_name( DR_PROD_NAME )) < 0)
                          ||
        ((Prod_info[DV_PROD_IND].prod_id = RPGC_get_id_from_name( DV_PROD_NAME )) < 0)
                          ||
        ((Prod_info[DW_PROD_IND].prod_id = RPGC_get_id_from_name( DW_PROD_NAME )) < 0)
                          ||
        ((Prod_info[DC_PROD_IND].prod_id = RPGC_get_id_from_name( DC_PROD_NAME )) < 0)
                          ||
        ((Prod_info[DP_PROD_IND].prod_id = RPGC_get_id_from_name( DP_PROD_NAME )) < 0) ){ 
        RPGC_log_msg( GL_ERROR, "RPGC_get_id_from_name() Failed\n" );
        RPGC_hari_kiri();

    }

    /* Do some product information initialization.   The rest of the initialization
       will be done in the buffer control module. */
    /* --- 8 bit Super Resolution Reflectivity --- */
    Prod_info[DR_PROD_IND].pcode = RPGC_get_code_from_id( Prod_info[DR_PROD_IND].prod_id );
    Prod_info[DR_PROD_IND].pname = DR_PROD_NAME;
    Prod_info[DR_PROD_IND].max_size = DR_OBUF_SIZE;
    Prod_info[DR_PROD_IND].type = (BASEDATA_TYPE | REFLDATA_TYPE);
    Prod_info[DR_PROD_IND].max_numbins = MAX_BASEDATA_REF_SIZE;
    strncpy( &Prod_info[DR_PROD_IND].moment[0], "DREF", 4 );

    /* --- 8 bit Super Resolution Velocity --- */
    Prod_info[DV_PROD_IND].pcode = RPGC_get_code_from_id( Prod_info[DV_PROD_IND].prod_id );
    Prod_info[DV_PROD_IND].pname = DV_PROD_NAME;
    Prod_info[DV_PROD_IND].max_size = DV_OBUF_SIZE;
    Prod_info[DV_PROD_IND].type = (BASEDATA_TYPE | COMBBASE_TYPE);
    Prod_info[DV_PROD_IND].max_numbins = BASEDATA_DOP_SIZE;
    strncpy( &Prod_info[DV_PROD_IND].moment[0], "DVEL", 4 );

    /* --- 8 bit Super Resolution Spectrum Width --- */
    Prod_info[DW_PROD_IND].pcode = RPGC_get_code_from_id( Prod_info[DW_PROD_IND].prod_id );
    Prod_info[DW_PROD_IND].pname = DW_PROD_NAME;
    Prod_info[DW_PROD_IND].max_size = DW_OBUF_SIZE;
    Prod_info[DW_PROD_IND].type = (BASEDATA_TYPE | COMBBASE_TYPE);
    Prod_info[DW_PROD_IND].max_numbins = BASEDATA_DOP_SIZE;
    strncpy( &Prod_info[DW_PROD_IND].moment[0], "DSW ", 4 );

    /* --- 8 bit Super Resolution Correlation Coefficient --- */
    Prod_info[DC_PROD_IND].pcode = RPGC_get_code_from_id( Prod_info[DC_PROD_IND].prod_id );
    Prod_info[DC_PROD_IND].pname = DC_PROD_NAME;
    Prod_info[DC_PROD_IND].max_size = DC_OBUF_SIZE;
    Prod_info[DC_PROD_IND].type = (BASEDATA_TYPE | REFLDATA_TYPE);
    Prod_info[DC_PROD_IND].max_numbins = BASEDATA_RHO_SIZE;
    strncpy( &Prod_info[DC_PROD_IND].moment[0], "DRHO", 4 );

    /* --- 8 bit Super Resolution Differential Phase --- */
    Prod_info[DP_PROD_IND].pcode = RPGC_get_code_from_id( Prod_info[DP_PROD_IND].prod_id );
    Prod_info[DP_PROD_IND].pname = DP_PROD_NAME;
    Prod_info[DP_PROD_IND].max_size = DP_OBUF_SIZE;
    Prod_info[DP_PROD_IND].type = (BASEDATA_TYPE | REFLDATA_TYPE);
    Prod_info[DP_PROD_IND].max_numbins = BASEDATA_PHI_SIZE;
    strncpy( &Prod_info[DP_PROD_IND].moment[0], "DPHI", 4 );

    /* Do Forever .... */
    while(1){

        RPGC_wait_act( WAIT_DRIVING_INPUT );
        SuperRes8bit_buffer_control();

    }

    return 0;

} 

/*************************************************************************************

   Description:
      Routine for servicing command line options.

   Inputs:
      arg - the command line argument.
      optarg - pointer the the options arguments.

   Returns:
      0 on Success, -1 on error.

**************************************************************************************/
int Service_options( int arg, char *optarg ){

    switch( arg ){

        case 'A':
            RPGC_log_msg( GL_INFO, "Setting All Product Generation\n" );
            All_products = 1;
            break;

        default:
            break;

    }

    return 0;

/* End of Service_options() */
}
