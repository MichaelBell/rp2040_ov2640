#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "ov2640.h"
#include "aps6404.h"
#include "pcf_rtc.h"

#include "pico/cyw43_arch.h"
#include "secrets.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

const int PIN_VSYS_EN = 2;
const int PIN_LED = 6;
const int PIN_POKE = 7;

const int PIN_EXT_INT = 3;

const int PIN_CAM_SIOC = 5; // I2C0 SCL
const int PIN_CAM_SIOD = 4; // I2C0 SDA
const int PIN_CAM_RESETB = 9;
const int PIN_CAM_XCLK = 0;
const int PIN_CAM_VSYNC = 19;
const int PIN_CAM_Y2_PIO_BASE = 10;

const uint8_t CMD_REG_WRITE = 0xAA;
const uint8_t CMD_REG_READ = 0xBB;
const uint8_t CMD_CAPTURE = 0xCC;

_Alignas(8192) uint8_t image_buf[8192];
//_Alignas(4) uint8_t image_buf[352*288*2];

struct pcf_rtc_config pcf_config;
struct ov2640_config ov_config;
struct aps6404_config ram_config;

#define TEST_TCP_SERVER_IP "192.168.1.248"
#define TCP_PORT 4242
#define DEBUG_printf printf
#define BUF_SIZE 1024

typedef struct TCP_CLIENT_T_ {
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;
    int unack_len;
    uint8_t* xfer_ptr;
    int xfer_count;
    bool connected;
} TCP_CLIENT_T;

static err_t tcp_client_close(void *arg) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    err_t err = ERR_OK;
    if (state->tcp_pcb != NULL) {
        tcp_arg(state->tcp_pcb, NULL);
        tcp_poll(state->tcp_pcb, NULL, 0);
        tcp_sent(state->tcp_pcb, NULL);
        tcp_recv(state->tcp_pcb, NULL);
        tcp_err(state->tcp_pcb, NULL);
        err = tcp_close(state->tcp_pcb);
        if (err != ERR_OK) {
            DEBUG_printf("close failed %d, calling abort\n", err);
            tcp_abort(state->tcp_pcb);
            err = ERR_ABRT;
        }
        state->tcp_pcb = NULL;
	state->connected = false;
    }
    return err;
}

// Called with results of operation
static err_t tcp_result(void *arg, int status) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    if (status == 0) {
        DEBUG_printf("test success\n");
    } else {
        DEBUG_printf("test failed %d\n", status);
    }
    return tcp_client_close(arg);
}

static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    DEBUG_printf("tcp_client_sent %u, ", len);
    state->unack_len -= len;

    printf("%d bytes left to send, unack len %d\n", state->xfer_count, state->unack_len);

    return ERR_OK;
}

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    if (err != ERR_OK) {
        printf("connect failed %d\n", err);
        return tcp_result(arg, err);
    }
    state->connected = true;
    DEBUG_printf("Client connected\n");
    return ERR_OK;
}

static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb) {
    DEBUG_printf("tcp_client_poll\n");
    return ERR_OK;
}

static void tcp_client_err(void *arg, err_t err) {
    if (err != ERR_ABRT) {
        DEBUG_printf("tcp_client_err %d\n", err);
        tcp_result(arg, err);
    }
}

err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    if (!p) {
        return tcp_result(arg, -1);
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    if (p->tot_len > 0) {
        //DEBUG_printf("recv %d err %d\n", p->tot_len, err);
#if 0
        for (struct pbuf *q = p; q != NULL; q = q->next) {
            DUMP_BYTES(q->payload, q->len);
        }
        // Receive the buffer
        const uint16_t buffer_left = BUF_SIZE - state->buffer_len;
        state->buffer_len += pbuf_copy_partial(p, state->buffer + state->buffer_len,
                                               p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
#endif
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);

    return ERR_OK;
}

static bool tcp_client_open(void *arg) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    DEBUG_printf("Connecting to %s port %u\n", ip4addr_ntoa(&state->remote_addr), TCP_PORT);
    state->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&state->remote_addr));
    if (!state->tcp_pcb) {
        DEBUG_printf("failed to create pcb\n");
        return false;
    }

    tcp_arg(state->tcp_pcb, state);
    tcp_poll(state->tcp_pcb, tcp_client_poll, 4);
    tcp_sent(state->tcp_pcb, tcp_client_sent);
    tcp_recv(state->tcp_pcb, tcp_client_recv);
    tcp_err(state->tcp_pcb, tcp_client_err);

    state->unack_len = 0;

    // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
    // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
    // these calls are a no-op and can be omitted, but it is a good practice to use them in
    // case you switch the cyw43_arch type later.
    cyw43_arch_lwip_begin();
    err_t err = tcp_connect(state->tcp_pcb, &state->remote_addr, TCP_PORT, tcp_client_connected);
    cyw43_arch_lwip_end();

    return err == ERR_OK;
}

// Perform initialisation
static TCP_CLIENT_T* tcp_client_init(void) {
    TCP_CLIENT_T *state = calloc(1, sizeof(TCP_CLIENT_T));
    if (!state) {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    ip4addr_aton(TEST_TCP_SERVER_IP, &state->remote_addr);
    return state;
}	

void capture_frame_to_sram(struct ov2640_config* ov_config, struct aps6404_config* ram_config) {
        dma_channel_config c = dma_channel_get_default_config(ov_config->dma_channel);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, true);
        channel_config_set_dreq(&c, pio_get_dreq(ov_config->pio, ov_config->pio_sm, false));
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_bswap(&c, true);
	channel_config_set_ring(&c, true, 13);  // Ring of 8192 bytes

        dma_channel_configure(
                ov_config->dma_channel, &c,
                ov_config->image_buf,
                &ov_config->pio->rxf[ov_config->pio_sm],
                ov_config->image_buf_size / 4,
                false
        );

	const int bytes_per_transfer = 1024;

	c = dma_channel_get_default_config(ram_config->dma_channel);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, pio_get_dreq(ram_config->pio, ram_config->pio_sm, true));
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
	channel_config_set_ring(&c, false, 13);  // Ring of 8192 bytes

        dma_channel_configure(
                ram_config->dma_channel, &c,
                &ram_config->pio->txf[ram_config->pio_sm],
                ov_config->image_buf,
                bytes_per_transfer / 4,
                false
        );

	int next_transfer_threshold = (ov_config->image_buf_size - bytes_per_transfer) / 4;

        // Wait for vsync rising edge to start frame
        while (gpio_get(ov_config->pin_vsync) == true);
        while (gpio_get(ov_config->pin_vsync) == false);
        pio_sm_clear_fifos(ov_config->pio, ov_config->pio_sm);

        dma_channel_start(ov_config->dma_channel);

	uint32_t write_cmd_and_addr = 0x02000000u;
	while (true) {
		while (dma_hw->ch[ov_config->dma_channel].transfer_count > next_transfer_threshold);

		pio_sm_put_blocking(ram_config->pio, ram_config->pio_sm, 0x8000201fu);
		pio_sm_put_blocking(ram_config->pio, ram_config->pio_sm, write_cmd_and_addr);
		dma_channel_start(ram_config->dma_channel);
		write_cmd_and_addr += bytes_per_transfer;
		dma_channel_wait_for_finish_blocking(ram_config->dma_channel);

		if (next_transfer_threshold == 0) break;
		else next_transfer_threshold -= bytes_per_transfer / 4;

		if (next_transfer_threshold < 0) next_transfer_threshold = 0;
	}
}

void core1_entry();

int main() {
	// The PSRAM supports up to 84MHz, so 168MHz is the fastest we can go
	set_sys_clock_khz(168000, true);

	gpio_init(PIN_VSYS_EN);
	gpio_set_dir(PIN_VSYS_EN, GPIO_OUT);
	gpio_put(PIN_VSYS_EN, 1);

	stdio_init_all();

	//sleep_ms(5000);

	printf("Init RTC\n");
	pcf_config.pin_sda = PIN_CAM_SIOD;
	pcf_config.pin_scl = PIN_CAM_SIOC;
	pcf_config.pin_interrupt = PIN_EXT_INT;

	pcf_rtc_init(&pcf_config);

	datetime_t t;
	pcf_rtc_get_time(&t);
	printf("RTC time: %d/%d/%d %d:%d:%d\n", t.day, t.month, t.year, t.hour, t.min, t.sec);

#if 0
	bool alarm = pcf_rtc_get_alarm();
	pcf_rtc_set_alarm(5, -1, -1);
	while (!alarm) {
		sleep_ms(1000);
		alarm = pcf_rtc_get_alarm();
		printf("RTC alarm: %s\n", alarm? "Y" : "N");
	}
#endif

	multicore_launch_core1(core1_entry);

	while (true) __wfe();
}

void core1_entry() {
	if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK)) {
		printf("failed to initialise\n");
		return;
	}

	cyw43_arch_enable_sta_mode();

	while (cyw43_arch_wifi_connect_timeout_ms(wifi_ssid, wifi_pass, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
		printf("failed to connect\n");
		sleep_ms(1000);
	}
	printf("\n\nConnected!\n");

	gpio_init(PIN_LED);
	gpio_set_dir(PIN_LED, GPIO_OUT);
	gpio_put(PIN_LED, 1);
	gpio_init(PIN_POKE);
	gpio_pull_down(PIN_POKE);
	gpio_set_dir(PIN_POKE, GPIO_IN);

	ov_config.sccb = i2c0;
	ov_config.pin_sioc = PIN_CAM_SIOC;
	ov_config.pin_siod = PIN_CAM_SIOD;

	ov_config.pin_resetb = PIN_CAM_RESETB;
	ov_config.pin_xclk = PIN_CAM_XCLK;
	ov_config.pin_vsync = PIN_CAM_VSYNC;
	ov_config.pin_y2_pio_base = PIN_CAM_Y2_PIO_BASE;

	ov_config.pio = pio0;
	ov_config.pio_sm = pio_claim_unused_sm(pio0, true);

	ov_config.dma_channel = dma_claim_unused_channel(true);
	ov_config.image_buf = image_buf;
	//ov_config.image_buf_size = 352*288*2;
	//ov_config.image_buf_size = 800*600*2;
	ov_config.image_buf_size = 1600*1200*2;

	ov2640_init(&ov_config);

	ov2640_reg_write(&ov_config, 0xff, 0x01);
	uint8_t midh = ov2640_reg_read(&ov_config, 0x1C);
	uint8_t midl = ov2640_reg_read(&ov_config, 0x1D);
	printf("MIDH = 0x%02x, MIDL = 0x%02x\n", midh, midl);

	ram_config.pin_csn = 20;
	ram_config.pin_mosi = 22;
	ram_config.pin_miso = 26;

	ram_config.pio = pio0;
	ram_config.pio_sm = pio_claim_unused_sm(pio0, true);

	ram_config.dma_channel = dma_claim_unused_channel(true);

	uint logic_sm = pio_claim_unused_sm(pio1, true);
	uint logic_dma_channel = dma_claim_unused_channel(true);

	aps6404_init(&ram_config);

	uint32_t data[16];
	for (int i = 0; i < 16; ++i) {
		data[i] = 0x12345670 + i;
	}
	aps6404_write(&ram_config, 0, data, 16);

	for (int i = 0; i < 16; ++i) {
		uint32_t read_data = 1;
		aps6404_read_blocking(&ram_config, i * 4, &read_data, 1);
		if (read_data != data[i]) {
			printf("RAM test failed: Wrote %x, read back %x\n", data[i], read_data);
		}
	}

	cyw43_arch_poll();

	TCP_CLIENT_T* cli = tcp_client_init();
	if (!cli) return;

	while (true) {
		while (!gpio_get(PIN_POKE))
		{
			cyw43_arch_poll();
			sleep_ms(1);
		}

		for (int i = 0; i < 1000; ++i)
		{
			cyw43_arch_poll();
			sleep_ms(1);
		}

		while (!cli->connected) {
			if (!tcp_client_open(cli)) {
				tcp_result(cli, -1);
			}
			else {
				for (int i = 0; i < 10000 &&(!cli->connected); ++i)
				{
					cyw43_arch_poll();
					sleep_ms(1);
				}

				if (!cli->connected) {
					tcp_result(cli, 2);
				}
			}
			for (int i = 0; i < 1000; ++i)
			{
				cyw43_arch_poll();
				sleep_ms(1);
			}
		}

		capture_frame_to_sram(&ov_config, &ram_config);
		printf("Frame captured\n");
		cyw43_arch_poll();

		uint32_t addr = 0;
		int len_to_send = BUF_SIZE;
		aps6404_read(&ram_config, addr, (uint32_t*)ov_config.image_buf, BUF_SIZE / 4);
		addr += BUF_SIZE;

		cli->xfer_ptr = ov_config.image_buf;
		cli->xfer_count = ov_config.image_buf_size;
		cli->unack_len = 0;

		while (cli->xfer_count > 0) {
			cyw43_arch_poll();
			if (cli->unack_len == 0 ||
			    (cli->unack_len <= BUF_SIZE * 6 && !dma_channel_is_busy(ram_config.dma_channel)))
			{
				dma_channel_wait_for_finish_blocking(ram_config.dma_channel);

				cli->unack_len += len_to_send;
				cli->xfer_count -= len_to_send;
				cyw43_arch_lwip_begin();
				tcp_write(cli->tcp_pcb, cli->xfer_ptr, len_to_send, 0);
				cyw43_arch_lwip_end();
				cli->xfer_ptr += BUF_SIZE;
				if (cli->xfer_ptr == ov_config.image_buf + BUF_SIZE * 8) {
					cli->xfer_ptr = ov_config.image_buf;
				}

				len_to_send = cli->xfer_count;
				if (len_to_send > BUF_SIZE) {
				    len_to_send = BUF_SIZE;
				}

				if (len_to_send > 0) {
					aps6404_read(&ram_config, addr, (uint32_t*)cli->xfer_ptr, len_to_send / 4);
					addr += len_to_send;
				}
			}
			else {
				sleep_us(100);
			}
		}

		while (cli->unack_len > 0) {
			sleep_ms(1);
			cyw43_arch_poll();
		}

		tcp_result(cli, 0);
	}
}
