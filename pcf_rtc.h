#pragma once

#include "pico.h"

struct pcf_rtc_config {
	uint pin_sda;
	uint pin_scl;
	uint pin_interrupt;
};

void pcf_rtc_init(struct pcf_rtc_config* config);
void pcf_rtc_get_time(datetime_t* t);
void pcf_rtc_set_time();
void pcf_rtc_clear_alarm();
void pcf_rtc_set_alarm(int second, int minute, int hour);
bool pcf_rtc_get_alarm();
