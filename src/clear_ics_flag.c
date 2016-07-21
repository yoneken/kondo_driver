#include <stdio.h>
#include "kondo_driver/ics.h"

int main(int argc, char **argv)
{
    ICSData ics;
    int product_id = strtol(argv[1], 0, 16);
    int servo_id = atoi(argv[2]);
    int ret;
    fprintf (stderr, "product_id: %x\n", product_id);
    fprintf (stderr, "servo_id: %d\n", servo_id);
    // Initiallize ICS interface
    if (ics_init(&ics, product_id) < 0) {
	fprintf (stderr, "Could not init ICS: %s\n", ics.error);
	exit(0);
    }
    ics.debug = 1;
    // Get servo EEPROM
    ret = ics_clear_flag (&ics, servo_id);
    if(ret){
        fprintf (stderr, "Cleared all flags\n");
    }else{
        fprintf (stderr, "Failed to clear flags\n");
    }

    return 0;
}	
