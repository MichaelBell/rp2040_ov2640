#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "ov2640.h"

#include "pico/cyw43_arch.h"
#include "secrets.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

const int PIN_LED = 6;
const int PIN_POKE = 7;

const int PIN_CAM_SIOC = 5; // I2C0 SCL
const int PIN_CAM_SIOD = 4; // I2C0 SDA
const int PIN_CAM_RESETB = 9;
const int PIN_CAM_XCLK = 0;
const int PIN_CAM_VSYNC = 19;
const int PIN_CAM_Y2_PIO_BASE = 10;

const uint8_t CMD_REG_WRITE = 0xAA;
const uint8_t CMD_REG_READ = 0xBB;
const uint8_t CMD_CAPTURE = 0xCC;

_Alignas(4) uint8_t image_buf[352*288*2];
struct ov2640_config config;

#define TEST_TCP_SERVER_IP "192.168.1.248"
#define TCP_PORT 4242
#define DEBUG_printf printf
#define BUF_SIZE 1400

typedef struct TCP_CLIENT_T_ {
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;
    int unack_len;
    uint8_t* xfer_ptr;
    uint8_t* xfer_end;
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
    DEBUG_printf("tcp_client_sent %u\n", len);
    state->unack_len -= len;

    int len_to_send = state->xfer_end - state->xfer_ptr;
    printf("%d bytes left to send\n", len_to_send);

    if (state->unack_len == 0) {
        if (len_to_send > BUF_SIZE) {
	    len_to_send = BUF_SIZE;
        }
        state->unack_len = len_to_send;
        state->xfer_ptr += len_to_send;
        tcp_write(state->tcp_pcb, state->xfer_ptr, len_to_send, 0);
        tcp_output(state->tcp_pcb);
    }

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
        DEBUG_printf("recv %d err %d\n", p->tot_len, err);
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

void core1_entry();

int main() {
	stdio_init_all();

	multicore_launch_core1(core1_entry);

	while (true) __wfe();
}

void core1_entry() {
	if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK)) {
		printf("failed to initialise\n");
		return;
	}

	cyw43_arch_enable_sta_mode();

	if (cyw43_arch_wifi_connect_timeout_ms(wifi_ssid, wifi_pass, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
		printf("failed to connect\n");
		return;
	}
	printf("\n\nConnected!\n");

	gpio_init(PIN_LED);
	gpio_set_dir(PIN_LED, GPIO_OUT);
	gpio_init(PIN_POKE);
	gpio_pull_down(PIN_POKE);
	gpio_set_dir(PIN_POKE, GPIO_IN);

	config.sccb = i2c0;
	config.pin_sioc = PIN_CAM_SIOC;
	config.pin_siod = PIN_CAM_SIOD;

	config.pin_resetb = PIN_CAM_RESETB;
	config.pin_xclk = PIN_CAM_XCLK;
	config.pin_vsync = PIN_CAM_VSYNC;
	config.pin_y2_pio_base = PIN_CAM_Y2_PIO_BASE;

	config.pio = pio0;
	config.pio_sm = 0; //pio_claim_unused_sm(pio0, true);

	config.dma_channel = 0; //dma_claim_unused_channel(true);
	config.image_buf = image_buf;
	config.image_buf_size = sizeof(image_buf);

	ov2640_init(&config);

	ov2640_reg_write(&config, 0xff, 0x01);
	uint8_t midh = ov2640_reg_read(&config, 0x1C);
	uint8_t midl = ov2640_reg_read(&config, 0x1D);
	printf("MIDH = 0x%02x, MIDL = 0x%02x\n", midh, midl);

	cyw43_arch_poll();

	TCP_CLIENT_T* cli = tcp_client_init();
	if (!cli) return;

	while (true) {
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

		ov2640_capture_frame(&config);
		printf("Frame captured\n");
		cyw43_arch_poll();

		cli->xfer_ptr = config.image_buf;
		cli->xfer_end = cli->xfer_ptr + config.image_buf_size;
		cli->unack_len = BUF_SIZE;
		cyw43_arch_lwip_begin();
		tcp_write(cli->tcp_pcb, cli->xfer_ptr, BUF_SIZE, 0);
		tcp_output(cli->tcp_pcb);
		cyw43_arch_lwip_end();

		while (cli->unack_len > 0) {
			cyw43_arch_poll();
			sleep_ms(1);
		}	

		if (cli->xfer_ptr != cli->xfer_end)
		{
			tcp_result(cli, 1);
		}
		else
		{
			tcp_result(cli, 0);
		}

		while (!gpio_get(PIN_POKE))
		{
			cyw43_arch_poll();
			sleep_ms(1);
		}

		sleep_ms(500);
	}
}
