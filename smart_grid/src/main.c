#include <stdio.h>
#include <string.h>

#include "constants.h"
#include "lwipopts.h"

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/uart.h"

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/altcp_tcp.h"
#include "lwip/altcp_tls.h"
#include "lwip/apps/mqtt.h"

void _init_uart();

void _init_adc();
void _init_dma0();
void _init_dma1();
void _dma0_irh();
void _dma1_irh();
void _dma_irh();

void core1();
void core0();

int _wifi_connect();
void _print_data(const void *data, size_t data_len);

ip4_addr_t server_ip;
u16_t web_port, mqtt_port;

enum protocol_t {
    PROTOCOL_NULL,
    PROTOCOL_HTTP,
    PROTOCOL_MQTT,
};

enum protocol_t protocol = PROTOCOL_NULL;
bool use_tls = false;
bool client_connected = false;

struct payload {
    char data[MAX_DATA_LENGTH];
    size_t length;
};

struct altcp_tls_config *tls_conf = NULL;
altcp_allocator_t *tls_allocator = NULL;

volatile bool send_next = false;
void *send_buffer = NULL;

/*
    HTTP Setup START
*/

struct altcp_pcb *httpc = NULL;
volatile bool httpc_connected = false;
struct payload *http_req = NULL;
size_t http_header_len;

err_t httpc_close(){
    err_t err = ERR_OK;
    if(httpc){
        httpc_connected = false;
        altcp_sent(httpc, NULL);
        altcp_recv(httpc, NULL);
        altcp_err(httpc, NULL);
        err = altcp_close(httpc);
        printf("HTTPc Closed\n");
        client_connected = false;
        httpc = NULL;
    }
    return err;
}

err_t httpc_send_data(struct altcp_pcb *httpc, struct payload* req, uint16_t *data){
    memcpy(req->data + http_header_len, data, DMA_BUFFER_SIZE*2);
    printf("sending %u bytes...\n", req->length);
    err_t err;
    err = altcp_write(httpc, req->data, req->length, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        printf("tcp_write failed: %d\n", err);
        return err;
    }
    err = altcp_output(httpc);
    if (err != ERR_OK) {
        printf("tcp_output failed: %d\n", err);
        return err;
    }
    return err;
}

err_t httpc_recv(void *arg, struct altcp_pcb *pcb, struct pbuf *p, err_t err){
    if(p==NULL){
        httpc_close();
        return err;
    }
    u16_t len = p->len;
    altcp_recved(pcb, len);
    pbuf_free(p);
    printf("received %u bytes...\n", len);
    if(err != ERR_OK){
        printf("recv error: %d", err);
    }
    // send_next = true;
    return err;
}

void httpc_err(void *arg, err_t err){
    printf("http client error: %d\n", err);
}

err_t httpc_sent(void *arg, struct altcp_pcb *pcb, u16_t len){
    printf("Data sent successfully\n");
    return ERR_OK;
}

err_t httpc_conn_cb(void *arg, struct altcp_pcb *httpc, err_t err){
    if (err != ERR_OK) {
        printf("http connect failed %d\n", err);
        err = httpc_close(httpc);
        return err;
    }
    printf("Connected to Web Server.\n");
    altcp_sent(httpc, httpc_sent);
    altcp_recv(httpc, httpc_recv);

    httpc_connected = true;
    
    return err;
}

void httpc_init(){

    if(!http_req){
        http_req = (struct payload*) malloc(sizeof(struct payload));
        http_header_len = sizeof(HTTP_HEADER)-1;
        http_req -> length = http_header_len + DMA_BUFFER_SIZE*2;
        memcpy(http_req -> data, HTTP_HEADER, http_header_len);
    }

    if(tls_conf){
        printf("Using TLS\n");
        if(!tls_allocator){
            tls_allocator = (altcp_allocator_t *) malloc(sizeof(altcp_allocator_t));
            tls_allocator->alloc = altcp_tls_alloc;
            tls_allocator->arg = tls_conf;
        }
    }

    if(!httpc){
        httpc = altcp_new_ip_type(tls_allocator, IPADDR_TYPE_V4);
    }
}

err_t httpc_connect(){
    
    httpc_init();
    
    web_port = SERVER_PORT_HTTP;
    if(tls_conf){
        mbedtls_ssl_set_hostname(altcp_tls_context(httpc), SERVER_HOSTNAME);
        web_port = SERVER_PORT_HTTPS;
    }

    altcp_err(httpc, httpc_err);

    printf("Connecting...");

    err_t err = altcp_connect(httpc, &server_ip, web_port, httpc_conn_cb);
    if (err != ERR_OK)
    {
        printf("HTTP connect failed, err=%d\n", err);
        return err;
    }

    printf("___\n");
    while(!httpc_connected);

    return err;
}

/*
    HTTP Setup END
*/

/*
    MQTT Setup START
*/
struct mqtt_client_s *mqttc = NULL;
struct mqtt_connect_client_info_t *mqttc_info = NULL;
volatile bool mqttc_connected = false;
char *mqtt_topic = UPLOAD_PATH;

void mqttc_close(){
    mqttc_connected = false;
    if(mqttc){
        mqtt_disconnect(mqttc);
        mqtt_client_free(mqttc);
        mqttc = NULL;
    }
    printf("MQTTc Closed\n");
    client_connected = false;
}

void mqttc_cb(void *arg, err_t err){
    if(err != ERR_OK){
        printf("MQTT error: %d\n", err);
        mqttc_close();
    }
}

err_t mqttc_send_data(char *topic, uint16_t *data){
    err_t err;
    if(!tls_conf){
        err = mqtt_publish(mqttc, topic, data, DMA_BUFFER_SIZE*2, 2, 0, mqttc_cb, NULL);
    }else{
        err = mqtt_publish(mqttc, topic, data, DMA_BUFFER_SIZE/2, 2, 0, mqttc_cb, NULL);
        // data += DMA_BUFFER_SIZE/4;
        // err = mqtt_publish(mqttc, topic, data, DMA_BUFFER_SIZE/2, 2, 0, mqttc_cb, NULL);
        // data += DMA_BUFFER_SIZE/4;
        // err = mqtt_publish(mqttc, topic, data, DMA_BUFFER_SIZE/2, 2, 0, mqttc_cb, NULL);
        // data += DMA_BUFFER_SIZE/4;
        // err = mqtt_publish(mqttc, topic, data, DMA_BUFFER_SIZE/2, 2, 0, mqttc_cb, NULL);
    }
    return err;
}

void mqttc_conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status){
    if(status == MQTT_CONNECT_ACCEPTED){
        printf("Connected to MQTT Broker.\n");
        mqttc_connected = true;
    }else{
        printf("MQTT connection error: %d\n", status);
        mqttc_close();
    }
}

void mqttc_init(){
    if(!mqttc){
        mqttc = mqtt_client_new();
    }
    if(!mqttc_info){
        mqttc_info = (struct mqtt_connect_client_info_t *) malloc(sizeof(struct mqtt_connect_client_info_t));
    }
}

err_t mqttc_connect(){

    mqttc_init();

    mqtt_port = SERVER_PORT_MQTT;
    if(tls_conf != NULL){
        printf("Using TLS\n");
        mqtt_port = SERVER_PORT_MQTTS;
    }
    
    memset(mqttc_info, 0, sizeof(mqttc_info));
    mqttc_info->client_id = CLIENT_ID;
    mqttc_info->client_user = NULL;
    mqttc_info->client_pass = NULL;
    mqttc_info->keep_alive = 5;
    mqttc_info->will_topic = NULL;
    mqttc_info->will_msg = NULL;
    mqttc_info->will_retain = 0;
    mqttc_info->will_qos = 0;
    mqttc_info->tls_config = tls_conf;
    mqttc_info->server_name = SERVER_HOSTNAME;

    printf("Connecting...");

    err_t err;
    err = mqtt_client_connect(mqttc, &server_ip, mqtt_port, mqttc_conn_cb, NULL, mqttc_info);
    if(err != ERR_OK){
        printf("MQTT connect failed: %d\n", err);
        return err;
    }

    printf("___\n");
    while(!mqttc_connected);

    return err;
}

/*
    MQTT Setup END
*/

/*
    ADC & DMA Setup START
*/

uint dma0_chn, dma1_chn;
dma_channel_config dma0_cfg, dma1_cfg;
uint16_t adc_sample;
uint16_t tmp_buffer[DMA_BUFFER_SIZE];
uint16_t dma_buffer[2][DMA_BUFFER_SIZE];

void _init_adc(){

    // Initialize PIN 26 for ADC
    // disable digital functions for the pin
    adc_gpio_init(27);

    // Initialize ADC, reset and enable clock (48MHz)
    adc_init();

    // select input for ADC MUX (0...3 are PINs 26...29, 4 is internal temperature)
    adc_select_input(1);

    adc_fifo_setup(
        true,  // write each completed conversion to FIFO
        true,  // enable DMA Data Request (DREQ)
        1,     // number of samples OR DREQ_THRESHOLD
        false, // error bit
        false   // shift each sample to 8 bits
    );

    adc_fifo_drain();

    // adc_irq_set_enabled(true);
    // irq_set_exclusive_handler(ADC_IRQ_FIFO, _adc_irh);
    // irq_set_enabled(ADC_IRQ_FIFO, true);

    // ADC is triggered every CLOCK_DIV+1 cycles, it takes 96 cycles for 1 conversion
    // Cycles are wrt to 48MHz ADC clock
    // CLOCK_DIV set to 0 implies free running mode
    // set any value > 96 for slowing down the ADC sampling rate
    adc_set_clkdiv(CLOCK_DIV);

    return;
}

void _init_dma0(){
    
    dma0_cfg = dma_channel_get_default_config(dma0_chn);

    channel_config_set_transfer_data_size(&dma0_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma0_cfg, false);
    channel_config_set_write_increment(&dma0_cfg, false);
    channel_config_set_chain_to(&dma0_cfg, dma1_chn);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&dma0_cfg, DREQ_ADC);

    dma_channel_configure(
        dma0_chn,           // DMA channel
        &dma0_cfg,          // DMA configuration
        &adc_sample,        // initial write address
        &adc_hw->fifo,      // initial read address
        1,                  // transfer count
        false               // start immediate trigger
    );

    dma_channel_set_irq0_enabled(dma0_chn, true);
    irq_set_exclusive_handler(DMA_IRQ_0, _dma_irh);
    irq_set_enabled(DMA_IRQ_0, true);

    return;
}

void _init_dma1(){
    
    dma1_cfg = dma_channel_get_default_config(dma1_chn);

    channel_config_set_transfer_data_size(&dma1_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma1_cfg, false);
    channel_config_set_write_increment(&dma1_cfg, false);
    channel_config_set_chain_to(&dma1_cfg, dma0_chn);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&dma1_cfg, DREQ_ADC);

    dma_channel_configure(
        dma1_chn,           // DMA channel
        &dma1_cfg,          // DMA configuration
        &adc_sample,        // initial write address
        &adc_hw->fifo,      // initial read address
        1,                  // transfer count
        false               // start immediate trigger
    );

    dma_channel_set_irq1_enabled(dma1_chn, true);
    irq_set_exclusive_handler(DMA_IRQ_1, _dma_irh);
    irq_set_enabled(DMA_IRQ_1, true);

    return;
}

void _init_adc_dma(){

    // ADC SETUP
    _init_adc();
    sleep_ms(1000);

    // DMA SETUP
    dma0_chn = dma_claim_unused_channel(true);
    dma1_chn = dma_claim_unused_channel(true);
    _init_dma0();
    _init_dma1();

}

bool dma_bi = 0;
size_t dma_si = 0;
size_t dma_count = 0;

void _dma_irh(){
    dma_channel_acknowledge_irq0(dma0_chn);
    dma_channel_acknowledge_irq1(dma1_chn);
    dma_buffer[dma_bi][dma_si] = adc_sample;
    dma_si = (dma_si+1) % (DMA_BUFFER_SIZE);
    if(dma_si == 0){
        send_buffer = dma_buffer[dma_bi];
        send_next = true;
        dma_bi != dma_bi;
        dma_count++;
    }
}

size_t dma0_count = 0;

void _dma0_irh(){
    dma_channel_acknowledge_irq0(dma0_chn);
    // send_buffer = dma_buffer[0];
    send_next = true;
    dma0_count++;
}

size_t dma1_count = 0;

void _dma1_irh(){
    dma_channel_acknowledge_irq1(dma1_chn);
    // send_buffer = dma_buffer[1];
    send_next = true;
    dma1_count++;
}

/*
    ADC & DMA Setup END
*/

void core1(){

    // printf("core 1 started \n");

    _init_adc_dma();
    adc_run(true);
    dma_channel_start(dma0_chn);

    // while (true)
    // {
    //     sleep_ms(199);
    //     send_next = true;
    // }

    while (true)
    {
        tight_loop_contents();
    }

    printf("\nCORE 1 END\n");
}

void _init_(){
    stdio_init_all();
    _init_uart();

    for (size_t i = 0; i < DMA_BUFFER_SIZE; i++)
    {
        tmp_buffer[i] = i;
    }
}


void core0(){

    bool adc_buffer_print = false;
    err_t err;

    while (true){
        while(client_connected){
            if(httpc_connected && send_next){
                printf("hs\n");
                send_next = false;
                err = httpc_send_data(httpc, http_req, send_buffer);
                if(err != ERR_OK){
                    printf("HTTP send failed, error: %d\n", err);
                    httpc_close();
                }
            }else if(mqttc_connected && send_next){
                printf("mp\n");
                send_next = false;
                err = mqttc_send_data(mqtt_topic, send_buffer);
                if(err != ERR_OK){
                    printf("MQTT publish failed, error: %d\n", err);
                    mqttc_close();
                }
            }else if(adc_buffer_print && dma_count == 2){
                printf("\nADC BUFFER 0:\n");
                _print_data(dma_buffer[0], DMA_BUFFER_SIZE);
                printf("\nADC BUFFER 1:\n");
                _print_data(dma_buffer[1], DMA_BUFFER_SIZE);
                printf("\n");
                adc_buffer_print = false;
            }else{
                // printf(".%u,%u.", dma0_count, dma1_count);
                printf(".%u.", dma_count);
            }
        }

        switch (protocol){
            case PROTOCOL_HTTP:
                printf("Connecting to Web Server...\n");
                httpc_connect();
                while(!httpc_connected);
                client_connected = true;
                break;
            case PROTOCOL_MQTT:
                printf("Connecting to MQTT Broker...\n");
                mqttc_connect();
                while(!mqttc_connected);
                client_connected = true;
                break;
            default:
                printf(".PROTOCOL_UNKNOWN.");
        }
    }

    printf("\nCORE 0 END\n");

}

int main(void)
{

    _init_();
    sleep_ms(3000);
    
    for (int i = 5; i > 0; i--){
        printf("%d ", i);
        sleep_ms(1000);
    }
    printf("\n");

    while(_wifi_connect()){
        cyw43_arch_deinit();
    }

    ip4addr_aton(SERVER_IP, &server_ip);
    
    protocol = PROTOCOL_HTTP;
    use_tls = true;
    if(use_tls){
        tls_conf = altcp_tls_create_config_client(CA_CERT, sizeof(CA_CERT));
    }

    send_buffer = tmp_buffer;
    multicore_launch_core1(core1);
    core0();

    printf("\nMAIN END\n");
    return 0;
}

int _wifi_connect(){
    int werr;
    if(werr = cyw43_arch_init_with_country(CYW43_COUNTRY_INDIA)){
        printf("Wi-Fi initialisation failed, err=%d\n", werr);
        return 1;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    if(werr = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, WIFI_TIMEOUT_MS)){
        printf("Wi-Fi connection failed, err=%d\n", werr);
    }else{
        printf("Wi-Fi Connected.\n");
    }
    return werr;
}

// Initialize UART
void _init_uart(){
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

// Print data
void _print_data(const void *data, size_t data_len){
    uint16_t *c = (uint16_t *)data;
    printf("Printing data: \n");
    for (size_t i = 0; i < data_len; i++){
        printf("0x%04x ", c[i]);
    }
    printf("\n");
}
