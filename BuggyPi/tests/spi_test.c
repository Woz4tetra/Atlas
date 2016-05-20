#include <bcm2835.h>
#include <stdio.h>

int main(int argc, char **argv)
{
	if (!bcm2835_init()) {
		printf("oops, could not init bcm2835\n");
		return 1;
	}

    printf("begin...");
	bcm2835_spi_begin();
    printf("done\n");

    printf("bit order...");
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    printf("done\n");

    printf("data mode...");
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    printf("done\n");

    printf("clock divider...");
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);    // ~ 4 MHz
    printf("done\n");

    printf("chip select...");
	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    printf("done\n");

    printf("polarity...");
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
    printf("done\n");

	uint8_t mosi[10] = { 0x60, 0x00 };
	uint8_t miso[10] = { 0 };
	// bcm2835_spi_transfernb(mosi, miso, 2);

    printf("transferring...");
    bcm2835_spi_transfern(mosi, 2);
    printf("done\n");

	// printf("Analogue level from SPI: %04x\n", miso[1] + ((miso[0] & 3) << 8));
    printf("ending...");
	bcm2835_spi_end();
    printf("done\n");

    printf("closing...");
	bcm2835_close();
    printf("done\n");
	return 0;
}
