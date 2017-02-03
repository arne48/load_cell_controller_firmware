#include <bcm2835.h>
#include <ctime>
#include <stdio.h>
#include <iostream>

#define BUFFER_SIZE 4

char rx_buffer[BUFFER_SIZE];
char tx_buffer[BUFFER_SIZE];

int main(int argc, char **argv) {
	std::cout.setf(std::ios::unitbuf);

    if (!bcm2835_init()) {
      std::cout << "bcm2835_init failed. Are you running as root??" << std::endl;
      return 1;
    }
    if (!bcm2835_spi_begin()) {
      std::cout << "bcm2835_spi_begin failed. Are you running as root??" << std::endl;
      return 1;
    }
    
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_512);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS_NONE, LOW);
    
    bcm2835_gpio_fsel(RPI_GPIO_P1_07, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(RPI_GPIO_P1_07, HIGH);
    
    bcm2835_gpio_write(RPI_GPIO_P1_07, LOW);
    
    tx_buffer[0] = 0;
	tx_buffer[1] = 0;
	tx_buffer[2] = 0;
	tx_buffer[3] = 0;

    for(int i=0; i<15; i++) {
		bcm2835_spi_transfernb(tx_buffer, rx_buffer, BUFFER_SIZE);
		
		for (unsigned int idx = 0; idx < BUFFER_SIZE; idx++) {
			printf("%.2X ", rx_buffer[idx]);
		}
		std::cout<<"\n-------------------------"<<std::endl;
	
		tx_buffer[0] += 1;
	    tx_buffer[1] += 1;
	    tx_buffer[2] += 1;
	    tx_buffer[3] += 1;
	}
	
    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}

