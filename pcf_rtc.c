#include "hardware/rtc.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "pcf_rtc.h"

#define ADDR 0x51

static uint8_t bcd_encode(uint8_t v) {
	uint8_t tens = v / 10;
	uint8_t units = v - tens * 10;
	return (tens << 4) | units;
}

static int8_t bcd_decode(uint v) {
	return (v & 0xf) + (v >> 4) * 10;
}

void pcf_rtc_init(struct pcf_rtc_config* config) {
	i2c_init(i2c0, 100000);
	gpio_set_function(config->pin_sda, GPIO_FUNC_I2C); gpio_pull_up(config->pin_sda);
	gpio_set_function(config->pin_scl, GPIO_FUNC_I2C); gpio_pull_up(config->pin_scl);

	gpio_set_function(config->pin_interrupt, GPIO_FUNC_SIO);
	gpio_set_dir(config->pin_interrupt, GPIO_IN);
	gpio_set_pulls(config->pin_interrupt, false, true);

	// Turn off the clock output and clear any alarm
	uint8_t buffer[2] = {0x01};
	i2c_write_blocking(i2c0, ADDR, buffer, 1, true);
	i2c_read_blocking(i2c0, ADDR, &buffer[1], 1, false);
	buffer[1] = (buffer[1] & 0xb8) | 7;
	i2c_write_blocking(i2c0, ADDR, buffer, 2, false);
}

void pcf_rtc_set_time() {
	datetime_t t;
	rtc_get_datetime(&t);
	uint8_t buffer[8] = {
		0x04,
		bcd_encode(t.sec),
		bcd_encode(t.min),
		bcd_encode(t.hour),
		bcd_encode(t.day),
		bcd_encode(t.dotw),
		bcd_encode(t.month),
		bcd_encode(t.year - 2000)
	};

	i2c_write_blocking(i2c0, ADDR, buffer, 8, false);
}

void pcf_rtc_get_time(datetime_t* t) {
	uint8_t buffer[8] = {0x04};
	i2c_write_blocking(i2c0, ADDR, buffer, 1, true);
	i2c_read_blocking(i2c0, ADDR, &buffer[1], 7, false);

	t->sec = bcd_decode(buffer[1] & 0x7f);
	t->min = bcd_decode(buffer[2]);
	t->hour = bcd_decode(buffer[3]);
	t->day = bcd_decode(buffer[4]);
	t->dotw = bcd_decode(buffer[5]);
	t->month = bcd_decode(buffer[6]);
	t->year = (int16_t)bcd_decode(buffer[7]) + 2000;
}

void pcf_rtc_clear_alarm() {
	uint8_t buffer[2] = {0x01};
	i2c_write_blocking(i2c0, ADDR, buffer, 1, true);
	i2c_read_blocking(i2c0, ADDR, &buffer[1], 1, false);
	buffer[1] = (buffer[1] & 0xbf);
	i2c_write_blocking(i2c0, ADDR, buffer, 2, false);
}

void pcf_rtc_set_alarm(int second, int minute, int hour) {
	uint8_t buffer[6] = {
		0x0B,
		second >= 0 ? bcd_encode(second) : 0x80,
		minute >= 0 ? bcd_encode(minute) : 0x80,
		hour   >= 0 ? bcd_encode(hour)   : 0x80,
		0x80,
		0x80
	};

	i2c_write_blocking(i2c0, ADDR, buffer, 6, false);
}

bool pcf_rtc_get_alarm() {
	uint8_t buffer[2] = {0x01};
	i2c_write_blocking(i2c0, ADDR, buffer, 1, true);
	i2c_read_blocking(i2c0, ADDR, &buffer[1], 1, false);
	return buffer[1] & 0x40;
}
