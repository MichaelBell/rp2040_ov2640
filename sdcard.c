#include <stdio.h>
#include "hardware/spi.h"
#include "pico/stdlib.h"

// SPI control of SD card - heavily inspired by http://www.rjhcoding.com/avrc-sd-interface-1.php

#define SPI_CS 26
#define SPI_CLK 18
#define SPI_MOSI 19
#define SPI_MISO 20

#define cs_disable() gpio_put(SPI_CS, 1)
#define cs_enable()  gpio_put(SPI_CS, 0)

void sd_power_up()
{
    cs_disable();

    // give SD card time to power up
    sleep_ms(1);

    // send 88 clock cycles to synchronize
    const uint8_t blank_byte = 0xff;
    for (int i = 0; i < 11; ++i) {
        spi_write_blocking(spi0, &blank_byte, 1);
    }
}

// Switch the CS line while clocking the card
void sd_set_cs(bool enable)
{
    const uint8_t blank_byte = 0xff;
    spi_write_blocking(spi0, &blank_byte, 1);
    gpio_put(SPI_CS, enable ? 0 : 1);
    spi_write_blocking(spi0, &blank_byte, 1);
}

// Send a command with a 32-bit argument
void sd_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    cmd |= 0x40;
    spi_write_blocking(spi0, &cmd, 1);
    
    arg = __builtin_bswap32(arg);
    spi_write_blocking(spi0, (uint8_t*)&arg, 4);

    crc|= 1;
    spi_write_blocking(spi0, &crc, 1);
}

// Read a 1 byte result
uint8_t sd_read1()
{
    for (int i = 0; i < 10; ++i) {
        uint8_t res;
        spi_read_blocking(spi0, 0xff, &res, 1);

        if (res != 0xff) return res;
    }

    return 0xff;
}

// Read a 5 byte result
void sd_read5(uint8_t* res)
{
    res[0] = sd_read1();

    if (res[0] > 1) return;

    spi_read_blocking(spi0, 0xff, &res[1], 4);
}

#define PARAM_ERROR(X)      X & 0b01000000
#define ADDR_ERROR(X)       X & 0b00100000
#define ERASE_SEQ_ERROR(X)  X & 0b00010000
#define CRC_ERROR(X)        X & 0b00001000
#define ILLEGAL_CMD(X)      X & 0b00000100
#define ERASE_RESET(X)      X & 0b00000010
#define IN_IDLE(X)          X & 0b00000001

void sd_print_r1(uint8_t res)
{
    if(res & 0b10000000)
        { printf("\tError: MSB = 1\n"); return; }
    if(res == 0)
        { printf("\tCard Ready\n"); return; }
    if(PARAM_ERROR(res))
        printf("\tParameter Error\n");
    if(ADDR_ERROR(res))
        printf("\tAddress Error\n");
    if(ERASE_SEQ_ERROR(res))
        printf("\tErase Sequence Error\n");
    if(CRC_ERROR(res))
        printf("\tCRC Error\n");
    if(ILLEGAL_CMD(res))
        printf("\tIllegal Command\n");
    if(ERASE_RESET(res))
        printf("\tErase Reset Error\n");
    if(IN_IDLE(res))
        printf("\tIn Idle State\n");
}

#define CMD_VER(X)          ((X >> 4) & 0xF0)
#define VOL_ACC(X)          (X & 0x1F)

#define VOLTAGE_ACC_27_33   0b00000001
#define VOLTAGE_ACC_LOW     0b00000010
#define VOLTAGE_ACC_RES1    0b00000100
#define VOLTAGE_ACC_RES2    0b00001000

void sd_print_r7(uint8_t *res)
{
    sd_print_r1(res[0]);

    if (res[0] > 1) return;

    printf("\tCommand Version: ");
    printf("%x\n", CMD_VER(res[1]));

    printf("\tVoltage Accepted: ");
    if(VOL_ACC(res[3]) == VOLTAGE_ACC_27_33)
        printf("2.7-3.6V\n");
    else if(VOL_ACC(res[3]) == VOLTAGE_ACC_LOW)
        printf("LOW VOLTAGE\n");
    else if(VOL_ACC(res[3]) == VOLTAGE_ACC_RES1)
        printf("RESERVED\n");
    else if(VOL_ACC(res[3]) == VOLTAGE_ACC_RES2)
        printf("RESERVED\n");
    else
        printf("NOT DEFINED\n");

    printf("\tEcho: ");
    printf("%x\n", res[4]);
}

#define POWER_UP_STATUS(X)  X & 0x40
#define CCS_VAL(X)          X & 0x40
#define VDD_2728(X)         X & 0b10000000
#define VDD_2829(X)         X & 0b00000001
#define VDD_2930(X)         X & 0b00000010
#define VDD_3031(X)         X & 0b00000100
#define VDD_3132(X)         X & 0b00001000
#define VDD_3233(X)         X & 0b00010000
#define VDD_3334(X)         X & 0b00100000
#define VDD_3435(X)         X & 0b01000000
#define VDD_3536(X)         X & 0b10000000

void sd_print_r3(uint8_t *res)
{
    sd_print_r1(res[0]);

    if (res[0] > 1) return;

    printf("\tCard Power Up Status: ");
    if (POWER_UP_STATUS(res[1]))
    {
        printf("READY\n");
        printf("\tCCS Status: ");
        if(CCS_VAL(res[1])){ printf("1\n"); }
        else printf("0\n");
    }
    else
    {
        printf("BUSY\n");
    }

    printf("\tVDD Window: ");
    if(VDD_2728(res[3])) printf("2.7-2.8, ");
    if(VDD_2829(res[2])) printf("2.8-2.9, ");
    if(VDD_2930(res[2])) printf("2.9-3.0, ");
    if(VDD_3031(res[2])) printf("3.0-3.1, ");
    if(VDD_3132(res[2])) printf("3.1-3.2, ");
    if(VDD_3233(res[2])) printf("3.2-3.3, ");
    if(VDD_3334(res[2])) printf("3.3-3.4, ");
    if(VDD_3435(res[2])) printf("3.4-3.5, ");
    if(VDD_3536(res[2])) printf("3.5-3.6");
    printf("\n");
}

uint8_t sd_go_idle()
{
    sd_set_cs(true);

    sd_command(0, 0, 0x94);

    uint8_t res1 = sd_read1();

    sd_set_cs(false);

    return res1;
}

void sd_send_if_cond(uint8_t* res)
{
    sd_set_cs(true);

    sd_command(8, 0x1aa, 0x86);

    sd_read5(res);

    sd_set_cs(false);
}

void sd_read_ocr(uint8_t* res)
{
    sd_set_cs(true);

    sd_command(58, 0, 0);

    sd_read5(res);

    sd_set_cs(false);
}

uint8_t sd_send_op_cond()
{
    sd_set_cs(true);

    sd_command(55, 0, 0);
    uint8_t res = sd_read1();

    if (res <= 1) {
        sd_command(41, 0x40000000, 0);
        res = sd_read1();
    }

    sd_set_cs(false);
}

int set_sd_idle()
{
    // Set RAM CS high (disabled)
    gpio_init(17);
    gpio_set_dir(17, GPIO_OUT);
    gpio_put(17, 1);

    // SPI at 100kHz for now
    spi_init(spi_default, 100 * 1000);
    gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);

    // Should pull up MISO
    gpio_set_pulls(SPI_MISO, true, false);

    gpio_init(SPI_CS);
    gpio_set_dir(SPI_CS, GPIO_OUT);
    gpio_put(SPI_CS, 1);

    sd_power_up();

    uint8_t res[5];
    printf("Seding CMD0...\n");
    res[0] = sd_go_idle();
    printf("Response:\n");
    sd_print_r1(res[0]);

    if (res[0] > 1) return 1;
    return 0;
}
