#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/tcp.h"
#include "hardware/i2c.h"
#include "aht20.h"
#include "bmp280.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ssd1306.h"
#include "font.h"
#include <math.h>


#define LED_PIN 12
#define BOTAO_A 5
#define BOTAO_JOY 22
#define JOYSTICK_X 26
#define JOYSTICK_Y 27

#define WIFI_SSID "COMMON-C24Q1"
#define WIFI_PASS "aabb142536"


#define I2C_PORT i2c0               // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA 0                   // 0 ou 2
#define I2C_SCL 1                   // 1 ou 3
#define SEA_LEVEL_PRESSURE 101325.0 // Pressão ao nível do mar em Pa

#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C

const char HTML_BODY[] =
"<!DOCTYPE html>\n"
"<html lang=\"pt\">\n"
"<head>\n"
"  <meta charset=\"UTF-8\">\n"
"  <title>Leituras dos Sensores: AHT10 & BMP280</title>\n"
"  <script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script>\n"
"  <style>\n"
"    body {\n"
"      font-family: Arial, sans-serif;\n"
"      text-align: center;\n"
"      padding: 20px;\n"
"    }\n"
"    canvas {\n"
"      max-width: 600px;\n"
"      margin: 30px auto;\n"
"      display: block;\n"
"    }\n"
"  </style>\n"
"</head>\n"
"<body>\n"
"\n"
"  <h1>Leituras dos Sensores</h1>\n"
"\n"
"  <h2>BMP280 - Temperatura & Pressão</h2>\n"
"  <canvas id=\"bmpChart\"></canvas>\n"
"\n"
"  <h2>AHT10 - Temperatura & Humidade</h2>\n"
"  <canvas id=\"ahtChart\"></canvas>\n"
"\n"
"  <script>\n"
"    const labels = [];\n"
"\n"
"    const bmpChart = new Chart(document.getElementById('bmpChart').getContext('2d'), {\n"
"      type: 'line',\n"
"      data: {\n"
"        labels,\n"
"        datasets: [\n"
"          {\n"
"            label: 'Temperatura BMP280 (°C)',\n"
"            data: [],\n"
"            borderColor: 'green',\n"
"            fill: false\n"
"          },\n"
"          {\n"
"            label: 'Pressão (kPa)',\n"
"            data: [],\n"
"            borderColor: 'orange',\n"
"            fill: false\n"
"          }\n"
"        ]\n"
"      },\n"
"      options: { responsive: true }\n"
"    });\n"
"\n"
"    const ahtChart = new Chart(document.getElementById('ahtChart').getContext('2d'), {\n"
"      type: 'line',\n"
"      data: {\n"
"        labels,\n"
"        datasets: [\n"
"          {\n"
"            label: 'Temperatura AHT10 (°C)',\n"
"            data: [],\n"
"            borderColor: 'red',\n"
"            fill: false\n"
"          },\n"
"          {\n"
"            label: 'Humidade (%)',\n"
"            data: [],\n"
"            borderColor: 'blue',\n"
"            fill: false\n"
"          }\n"
"        ]\n"
"      },\n"
"      options: { responsive: true }\n"
"    });\n"
"\n"
"    function atualizar() {\n"
"      fetch('/estado').then(res => res.json()).then(data => {\n"
"        const now = new Date().toLocaleTimeString();\n"
"\n"
"        if (labels.length >= 20) {\n"
"          labels.shift();\n"
"          bmpChart.data.datasets.forEach(ds => ds.data.shift());\n"
"          ahtChart.data.datasets.forEach(ds => ds.data.shift());\n"
"        }\n"
"\n"
"        labels.push(now);\n"
"\n"
"        const tempBMP = parseFloat(data.temp);\n"
"        const presBMP = parseFloat(data.pres);\n"
"        const tempAHT = parseFloat(data.atemp);\n"
"        const humAHT  = parseFloat(data.ahum);\n"
"\n"
"        bmpChart.data.datasets[0].data.push(tempBMP);\n"
"        bmpChart.data.datasets[1].data.push(presBMP);\n"
"\n"
"        ahtChart.data.datasets[0].data.push(tempAHT);\n"
"        ahtChart.data.datasets[1].data.push(humAHT);\n"
"\n"
"        bmpChart.update();\n"
"        ahtChart.update();\n"
"      });\n"
"    }\n"
"\n"
"    setInterval(atualizar, 2000);\n"
"  </script>\n"
"\n"
"</body>\n"
"</html>\n";


// Estrutura para armazenar os dados do sensor
AHT20_Data data;
int32_t raw_temp_bmp;
int32_t raw_pressure;
struct bmp280_calib_param params;



char str_tmp1[5];  // Buffer para armazenar a string
char str_alt[5];  // Buffer para armazenar a string  
char str_tmp2[5];  // Buffer para armazenar a string
char str_umi[5];  // Buffer para armazenar a string    

// Função para calcular a altitude a partir da pressão atmosférica
double calculate_altitude(double pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

struct http_state
{
    char response[4096];
    size_t len;
    size_t sent;
};

static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    struct http_state *hs = (struct http_state *)arg;
    hs->sent += len;
    if (hs->sent >= hs->len)
    {
        tcp_close(tpcb);
        free(hs);
    }
    return ERR_OK;
}

static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (!p)
    {
        tcp_close(tpcb);
        return ERR_OK;
    }

    char *req = (char *)p->payload;
    struct http_state *hs = malloc(sizeof(struct http_state));
    if (!hs)
    {
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }
    hs->sent = 0;

    if (strstr(req, "GET /estado"))
    {

        // Leitura do BMP280
        bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure);
        int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

        // Cálculo da altitude
        double altitude = calculate_altitude(pressure);


        printf("Pressao = %.3f kPa\n", pressure / 1000.0);
        printf("Temperatura BMP: = %.2f C\n", temperature / 100.0);
        printf("Altitude estimada: %.2f m\n", altitude);

        // Leitura do AHT20
        if (aht20_read(I2C_PORT, &data))
        {
            printf("Temperatura AHT: %.2f C\n", data.temperature);
            printf("Umidade: %.2f %%\n\n\n", data.humidity);
        }
        else
        {
            printf("Erro na leitura do AHT10!\n\n\n");
        }

        sprintf(str_tmp1, "%.1fC", temperature / 100.0);  // Converte o inteiro em string
        sprintf(str_alt, "%.0fm", altitude);  // Converte o inteiro em string
        sprintf(str_tmp2, "%.1fC", data.temperature);  // Converte o inteiro em string
        sprintf(str_umi, "%.1f%%", data.humidity);  // Converte o inteiro em string 
 
         
        char json_payload[96];
        int json_len = snprintf(json_payload, sizeof(json_payload),
                                "{\"temp\":%.1f,\"pres\":%.3f,\"alt\":%.0f,\"atemp\":%.1f,\"ahum\":%.1f}\r\n",
                                    temperature/100.0, pressure/1000.0, altitude, data.temperature, data.humidity);

        printf("[DEBUG] JSON: %s\n", json_payload);

        hs->len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: application/json\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           json_len, json_payload);
    }
    else
    {
        hs->len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/html\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           (int)strlen(HTML_BODY), HTML_BODY);
    }

    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent);

    tcp_write(tpcb, hs->response, hs->len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    pbuf_free(p);
    return ERR_OK;
}

static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, http_recv);
    return ERR_OK;
}

static void start_http_server(void)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb)
    {
        printf("Erro ao criar PCB TCP\n");
        return;
    }
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK)
    {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    printf("Servidor HTTP rodando na porta 80...\n");
}

#include "pico/bootrom.h"
#define BOTAO_B 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

int main()
{
    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();
    sleep_ms(2000);

    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Iniciando Wi-Fi", 0, 0);
    ssd1306_draw_string(&ssd, "Aguarde...", 0, 30);    
    ssd1306_send_data(&ssd);

    // Inicializa o I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o BMP280
    bmp280_init(I2C_PORT);
    bmp280_get_calib_params(I2C_PORT, &params);

    // Inicializa o AHT20
    aht20_reset(I2C_PORT);
    aht20_init(I2C_PORT);
    
    if (cyw43_arch_init())
    {
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "WiFi => FALHA", 0, 0);
        ssd1306_send_data(&ssd);
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000))
    {
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "WiFi => ERRO", 0, 0);
        ssd1306_send_data(&ssd);
        return 1;
    }

    uint8_t *ip = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
    char ip_str[24];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "WiFi => OK", 0, 0);
    ssd1306_draw_string(&ssd, ip_str, 0, 10);
    ssd1306_send_data(&ssd);

    start_http_server();
 
    char str_x[5]; // Buffer para armazenar a string
    char str_y[5]; // Buffer para armazenar a string
    bool cor = true;
    while (true)
    {
        cyw43_arch_poll();

        // Leitura do BMP280
        bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure);
        int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

        // Cálculo da altitude
        double altitude = calculate_altitude(pressure);


        printf("Pressao = %.3f kPa\n", pressure / 1000.0);
        printf("Temperatura BMP: = %.2f C\n", temperature / 100.0);
        printf("Altitude estimada: %.2f m\n", altitude);

        // Leitura do AHT20
        if (aht20_read(I2C_PORT, &data))
        {
            printf("Temperatura AHT: %.2f C\n", data.temperature);
            printf("Umidade: %.2f %%\n\n\n", data.humidity);
        }
        else
        {
            printf("Erro na leitura do AHT10!\n\n\n");
        }

        sprintf(str_tmp1, "%.1fC", temperature / 100.0);  // Converte o inteiro em string
        sprintf(str_alt, "%.0fm", altitude);  // Converte o inteiro em string
        sprintf(str_tmp2, "%.1fC", data.temperature);  // Converte o inteiro em string
        sprintf(str_umi, "%.1f%%", data.humidity);  // Converte o inteiro em string        
    
        //  Atualiza o conteúdo do display com animações
        ssd1306_fill(&ssd, !cor);                           // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);       // Desenha um retângulo
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);            // Desenha uma linha
        ssd1306_line(&ssd, 3, 37, 123, 37, cor);            // Desenha uma linha
        ssd1306_draw_string(&ssd, "CEPEDI   TIC37", 8, 6);  // Desenha uma string
        ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 16);   // Desenha uma string
        ssd1306_draw_string(&ssd, "BMP280  AHT10", 10, 28); // Desenha uma string
        ssd1306_line(&ssd, 63, 25, 63, 60, cor);            // Desenha uma linha vertical
        ssd1306_draw_string(&ssd, str_tmp1, 14, 41);             // Desenha uma string
        ssd1306_draw_string(&ssd, str_alt, 14, 52);             // Desenha uma string
        ssd1306_draw_string(&ssd, str_tmp2, 73, 41);             // Desenha uma string
        ssd1306_draw_string(&ssd, str_umi, 73, 52);            // Desenha uma string
        ssd1306_send_data(&ssd);                            // Atualiza o display

        sleep_ms(300);
    }

    cyw43_arch_deinit();
    return 0;
}
