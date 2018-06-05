
#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi.h>


#include <nuttx/analog/adc.h>
#include <drivers/drv_adc.h>
#include <systemlib/err.h>

#define ADC_33v1_CHANNEL      13
#define ADC_33v2_CHANNEL      14
#define ADC_66v1_CHANNEL      15


void MW_openADC(int *fd_adc)
{
    
    *fd_adc = open(ADC0_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
    
    if (*fd_adc < 0) {
		warnx("ERROR: can't open ADC device");
		return 1;
	}
}

void MW_readADC(int fd_adc, int *ADC_Data, unsigned char *DataValid)
{
   struct adc_msg_s data_read[12];
   
   ssize_t count = read(fd_adc, data_read, sizeof(data_read));
   
   if (count < 0) {
			warnx("ERROR: can't read ADC device");
		}
   *DataValid = (count>0);
   
   unsigned channels = count / sizeof(data_read[0]);
   for (unsigned j = 0; j < channels; j++) {
        
 //       printf("Printing channel: [%d]=%d \n",data_read[j].am_channel,data_read[j].am_data);
        
        if (data_read[j].am_channel == ADC_33v1_CHANNEL)
            ADC_Data[0] = data_read[j].am_data; //fill-up ADC Channel with data
        
        if (data_read[j].am_channel == ADC_33v2_CHANNEL) 
            ADC_Data[1] = data_read[j].am_data; //fill-up ADC Channel with data
        
        if (data_read[j].am_channel == ADC_66v1_CHANNEL) 
            ADC_Data[2] = data_read[j].am_data; //fill-up ADC Channel with data
        
        
    }
   
   
}

