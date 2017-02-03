#include <bcm2835.h>
#include <ctime>
#include <stdio.h>
#include <iostream>

char rx_buffer[64];
char tx_buffer[64];

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
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);    // The default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);                  // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS_NONE, LOW);  // the default
    
    bcm2835_gpio_fsel(RPI_GPIO_P1_07, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(RPI_GPIO_P1_07, HIGH);
    
    bcm2835_gpio_write(RPI_GPIO_P1_07, LOW);
    std::cout.setf(std::ios::unitbuf);
    
    tx_buffer[0] = 0xFD;
    bcm2835_spi_transfernb(tx_buffer, rx_buffer, 64);
    delay(100);
    
    //bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32);
    
    
    for(int i=0; i<100 ; i++){
	bcm2835_spi_transfernb(tx_buffer, rx_buffer, 64);
	
	std::cout << "\n-----------OUTPUT-----------" << std::endl;
	for (unsigned int idx = 0; idx < 64; idx++) {
		if (!(idx % 8)) {
			puts("");
		}
		printf("%.2X ", rx_buffer[idx]);
	}
	std::cout << "\n----------------------------" << std::endl;
	delay(100);
    }
    
    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}
