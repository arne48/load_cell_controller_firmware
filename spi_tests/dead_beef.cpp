#include <bcm2835.h>
#include <ctime>
#include <stdio.h>
#include <iostream>

#define BUFFER_LENGTH 64
#define PATTERN_LENGTH 4

char pattern[4] = {0xDE, 0xAD, 0xBE, 0xEF};
char rx_buffer[BUFFER_LENGTH];

time_t now, last; 

bool equal(char* first, char* second, unsigned int length){
	for(unsigned idx = 0; idx < length; idx++){
		if(first[idx] != second[idx % PATTERN_LENGTH]) {
			return false;
		}
	}
	return true;
}

int main(int argc, char **argv)
{
    if (!bcm2835_init())
    {
      std::cout << "bcm2835_init failed. Are you running as root??" << std::endl;
      return 1;
    }
    if (!bcm2835_spi_begin())
    {
      std::cout << "bcm2835_spi_begin failed. Are you running as root??" << std::endl;
      return 1;
    }
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_512);    // The default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);                  // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS_NONE, LOW);  // the default
    
    bcm2835_gpio_fsel(RPI_GPIO_P1_07, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(RPI_GPIO_P1_07, HIGH);

    //1100 0000 1100 0000 0000 0000
    
    bcm2835_gpio_write(RPI_GPIO_P1_07, LOW);
    unsigned int good = 0;
    unsigned int mills = 0;
    std::cout.setf(std::ios::unitbuf);
    
    char beef_command[BUFFER_LENGTH];
    beef_command[0] = 0xFE;
    bcm2835_spi_transfernb(beef_command, rx_buffer, BUFFER_LENGTH);
    
    delay(1);
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32);
    last = std::time(0);
    
    while(1){
		bcm2835_spi_transfernb(beef_command, rx_buffer, BUFFER_LENGTH);
		
		if(equal(rx_buffer, pattern, BUFFER_LENGTH)){
			good++;
			if(good%10000 == 0){
				std::cout<<".";
			}
			if(good%1000000 == 0){
				now = std::time(0);
				mills++;
				std::cout << "\nNo errors in " << mills << " million transfers of 16 x DEADBEEF = 61.03MB at 7.8125MHz resulting in " << 61.03 / (now - last) << " MB/s" << std::endl;
				last = std::time(0);
			}
		}else{
			std::cout << "\n-----------ERROR HAPPENED----------" << std::endl;
			for (unsigned int idx = 0; idx < BUFFER_LENGTH; idx++) {
				if (!(idx % 8))
				    puts("");
				printf("%.2X ", rx_buffer[idx]);
			}
			std::cout << "\n-----------------------------------" << std::endl;
			good = 0;
			mills = 0;
		}
		
	
    }
    
    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}

