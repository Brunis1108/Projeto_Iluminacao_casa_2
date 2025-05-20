/*
 * WebServer para controle de iluminação com RP2040
 * 
 * Funcionalidades principais:
 * - Conexão Wi-Fi como cliente (station mode)
 * - Servidor HTTP na porta 80
 * - Controle de matriz de LEDs WS2812 (5x5)
 * - Display OLED para status via I2C
 * - Buzzer para feedback sonoro
 * - Interface web com botões para controle
 * 
 * Organização do código:
 * 1. Inclusão de bibliotecas
 * 2. Definições e constantes
 * 3. Variáveis globais
 * 4. Protótipos de funções
 * 5. Função principal (main)
 * 6. Implementação das funções
 */

// ==================== BIBLIOTECAS ====================
#include <stdio.h>           // Funções padrão de entrada/saída
#include <string.h>          // Manipulação de strings
#include <stdlib.h>          // Alocação dinâmica e utilitários
#include "pico/stdlib.h"     // SDK básico da Pico
#include "hardware/adc.h"    // Conversor Analógico-Digital
#include "pico/cyw43_arch.h" // Driver Wi-Fi da Pico W
#include "lwip/pbuf.h"       // Buffers de rede (LWIP)
#include "lwip/tcp.h"        // Protocolo TCP (LWIP)
#include "lwip/netif.h"      // Interface de rede (LWIP)
#include "ws2812.pio.h"      // Controle de LEDs WS2812 via PIO
#include "hardware/i2c.h"    // Comunicação I2C para o display
#include "lib/ssd1306.h"    // Biblioteca para display OLED
#include "hardware/pwm.h"    // Controle PWM para o buzzer

// ==================== DEFINIÇÕES ====================
// Credenciais da rede Wi-Fi
#define WIFI_SSID "DESKTOP-0G1AS8V 7081" // SSID da rede Wi-Fi
#define WIFI_PASSWORD "12345678"         // Senha da rede Wi-Fi

// Display OLED (I2C)
#define I2C_PORT i2c1           // Controlador I2C utilizado
#define PIN_I2C_SDA 14          // Pino de dados I2C
#define PIN_I2C_SCL 15          // Pino de clock I2C
#define OLED_ADDRESS 0x3C       // Endereço I2C do display

// Configuração da matriz de LEDs WS2812
#define WS2812_PIN 7  // Pino de dados da matriz
#define NUM_LEDS 25   // Quantidade de LEDs (matriz 5x5)
PIO pio = pio0;       // Controlador PIO (0 ou 1)
int sm = 0;           // Máquina de estado PIO (0-3)

#define buzzer 10 // Pino GPIO conectado ao buzzer

ssd1306_t ssd; // Estrutura para controle do display OLED

// Estados dos cômodos (ligado/desligado)
bool sl = false;    // Estado da luz da sala
bool q1 = false;    // Estado da luz do quarto 1
bool cz = false;    // Estado da luz da cozinha
bool q2 = false;    // Estado da luz do quarto 2
bool sl_ng = false; // Estado da luz noturna da sala
bool q1_ng = false; // Estado da luz noturna do quarto 1
bool cz_ng = false; // Estado da luz noturna da cozinha
bool q2_ng = false; // Estado da luz noturna do quarto 2

// Estados anteriores para comparação
bool prev_sl = false;
bool prev_q1 = false;
bool prev_cz = false;
bool prev_q2 = false;

// ==================== PROTÓTIPOS DE FUNÇÕES ====================
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err); // Callback de aceitação de conexão
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err); // Callback de recebimento de dados
void user_request(char **request); // Processa as requisições HTTP
void acender_leds(); // Controla a matriz de LEDs
void put_pixel(uint32_t pixel_grb); // Envia cor para um LED específico
uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b); // Converte RGB para formato WS2812
void display_init(); // Inicializa o display OLED
void atualizar_display(); // Atualiza o conteúdo do display
bool mudanca_display(); // Verifica se houve mudança nos estados
void configurar_pwm_buzzer(uint buzzer_pin); // Configura o PWM para o buzzer
void tocar_tom(uint buzzer_pin, int freq_hz, int duracao_ms, int intervalo_ms); // Toca um tom específico
void tocar_buzzer_ligado(); // Toca o som de confirmação

// ==================== FUNÇÃO PRINCIPAL ====================
int main()
{
    // Inicializa a comunicação serial via USB
    stdio_init_all();


    // Inicializa a matriz de LEDs WS2812
    uint offset = pio_add_program(pio, &ws2812_program); // Adiciona o programa PIO
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, false); // Configura o hardware

    // Inicializa o hardware Wi-Fi (CYW43)
    while (cyw43_arch_init())
    {
        printf("Falha ao inicializar Wi-Fi\n");
        sleep_ms(100);
        return -1; // Encerra se falhar
    }


    // Configura como station (conecta a uma rede existente)
    cyw43_arch_enable_sta_mode();

    // Tenta conectar ao Wi-Fi
    printf("Conectando ao Wi-Fi...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                              CYW43_AUTH_WPA2_AES_PSK, 20000))
    {
        printf("Falha ao conectar ao Wi-Fi\n");
        sleep_ms(100);
        return -1; // Encerra se falhar
    }
    printf("Conectado ao Wi-Fi\n");

    // Mostra o IP atribuído ao dispositivo
    if (netif_default)
    {
        printf("IP do dispositivo: %s\n", ipaddr_ntoa(&netif_default->ip_addr));
    }

    // Cria um novo PCB (Protocol Control Block) TCP
    struct tcp_pcb *server = tcp_new();
    if (!server)
    {
        printf("Falha ao criar servidor TCP\n");
        return -1;
    }

    // Associa o servidor à porta 80 (HTTP)
    if (tcp_bind(server, IP_ADDR_ANY, 80) != ERR_OK)
    {
        printf("Falha ao associar servidor TCP à porta 80\n");
        return -1;
    }

    // Coloca o servidor em modo de escuta
    server = tcp_listen(server);

    // Configura o callback para conexões recebidas
    tcp_accept(server, tcp_server_accept);
    printf("Servidor ouvindo na porta 80\n");

    // Inicializa o ADC para leitura de temperatura
    adc_init();
    adc_set_temp_sensor_enabled(true);
    
    // Inicializa o display OLED
    display_init();
    
    // Configura o PWM para o buzzer
    configurar_pwm_buzzer(buzzer);
    
    // Atualiza o display com o estado inicial
    atualizar_display();
    
    // Loop principal do programa
    while (true)
    {
        acender_leds(); // Atualiza a matriz de LEDs

        // Verifica se houve mudança nos estados para atualizar o display
        if (mudanca_display())
        {
            atualizar_display();
            // Atualiza os estados anteriores
            prev_sl = sl;
            prev_q1 = q1;
            prev_cz = cz;
            prev_q2 = q2;
        }

        cyw43_arch_poll(); // Mantém a conexão Wi-Fi ativa
        sleep_ms(100);     // Pequena pausa para reduzir uso da CPU
    }

    // Desliga o Wi-Fi (não executado devido ao loop infinito)
    cyw43_arch_deinit();
    return 0;
}

// ==================== IMPLEMENTAÇÃO DAS FUNÇÕES ====================

// Callback chamado quando uma nova conexão TCP é aceita
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    // Configura o callback para quando dados forem recebidos
    tcp_recv(newpcb, tcp_server_recv);
    return ERR_OK;
}

// Processa as requisições HTTP recebidas
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    // Se p for NULL, a conexão foi fechada
    if (!p)
    {
        tcp_close(tpcb);      // Fecha a conexão
        tcp_recv(tpcb, NULL); // Remove o callback
        return ERR_OK;
    }

    // Aloca memória para armazenar a requisição
    char *request = (char *)malloc(p->len + 1);
    memcpy(request, p->payload, p->len); // Copia os dados recebidos
    request[p->len] = '\0';              // Adiciona terminador de string

    printf("Request: %s\n", request); // Log da requisição

    // Processa a requisição (liga/desliga os cômodos)
    user_request(&request);

    // Buffer para a resposta HTML
    char html[1024];

    // Gera a página HTML dinamicamente
    snprintf(html, sizeof(html),
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: text/html\r\n"
             "\r\n"
             "<!DOCTYPE html><html><head><title>Controle</title><style>"
             "body{background:#004f5b;font-family:Arial,sans-serif;text-align:center;margin-top:40px;}"
             "h1{font-size:56px;margin-bottom:25px;color:#fff;}"
             "button{background:#ccc;font-size:18px;margin:10px;padding:14px 28px;border:none;border-radius:10px;box-shadow:4px 4px 8px #000;color:#003;}"
             "button:hover{background:#08c;color:#fff;cursor:pointer;box-shadow:4px 4px 8px #fff;}"
             ".s{background:#7affa7;}"
             "</style></head><body>"
             "<h1>Controle Iluminacao</h1>"
             "<form action=\"./sl_on\"><button class=\"%s\">%s Sala</button></form>"
             "<form action=\"./sl_ng\"><button>Noturna</button></form>"
             "<form action=\"./q1_on\"><button class=\"%s\">%s Quarto1</button></form>"
             "<form action=\"./q1_ng\"><button>Noturna</button></form>"
             "<form action=\"./cz_on\"><button class=\"%s\">%s Cozinha</button></form>"
             "<form action=\"./cz_ng\"><button>Noturna</button></form>"
             "<form action=\"./q2_on\"><button class=\"%s\">%s Quarto2</button></form>"
             "<form action=\"./q2_ng\"><button>Noturna</button></form>"
             "</body></html>",
             sl ? "s" : "", sl ? "Desligar" : "Ligar",
             q1 ? "s" : "", q1 ? "Desligar" : "Ligar",
             cz ? "s" : "", cz ? "Desligar" : "Ligar",
             q2 ? "s" : "", q2 ? "Desligar" : "Ligar");

    // Envia a resposta HTML
    tcp_write(tpcb, html, strlen(html), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb); // Força o envio

    // Libera os recursos
    free(request); // Libera a memória da requisição
    pbuf_free(p);  // Libera o buffer de rede

    return ERR_OK;
}

// Processa as requisições HTTP e altera os estados dos cômodos
void user_request(char **request)
{
    // Verifica qual botão foi pressionado
    if (strstr(*request, "GET /sl_on") != NULL)
    {
        sl = !sl; // Alterna o estado da sala

        tocar_buzzer_ligado(); // Toca o som de confirmação

        if (!sl)
            sl_ng = false; // Desliga a luz noturna se a principal estiver desligada
    }
    else if (strstr(*request, "GET /sl_ng") != NULL)
    {
        if (sl)
            sl_ng = !sl_ng; // Alterna a luz noturna da sala (só se a principal estiver ligada)
    }
    else if (strstr(*request, "GET /q1_on") != NULL)
    {
        q1 = !q1; // Alterna o estado do quarto1

        tocar_buzzer_ligado();
        
        if (!q1)
            q1_ng = false;
    }
    else if (strstr(*request, "GET /q1_ng") != NULL)
    {
        if (q1)
            q1_ng = !q1_ng; // Alterna a luz noturna do quarto1
    }
    else if (strstr(*request, "GET /cz_on") != NULL)
    {
        cz = !cz; // Alterna o estado da cozinha

        tocar_buzzer_ligado();

        if (!cz)
            cz_ng = false;
    }
    else if (strstr(*request, "GET /cz_ng") != NULL)
    {
        if (cz)
            cz_ng = !cz_ng; // Alterna a luz noturna da cozinha
    }
    else if (strstr(*request, "GET /q2_on") != NULL)
    {
        q2 = !q2; // Alterna o estado do quarto2

        tocar_buzzer_ligado();

        if (!q2)
            q2_ng = false;
    }
    else if (strstr(*request, "GET /q2_ng") != NULL)
    {
        if (q2)
            q2_ng = !q2_ng; // Alterna a luz noturna do quarto2
    }
}

// Envia uma cor para um LED específico da matriz
void put_pixel(uint32_t pixel_grb)
{
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u); // Envia os dados via PIO
}

// Converte valores RGB para o formato esperado pelos WS2812
uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b);
}

// Atualiza todos os LEDs da matriz baseado nos estados dos cômodos
void acender_leds()
{
    for (int i = 0; i < NUM_LEDS; ++i)
    {
        uint8_t r = 0, g = 0, b = 0; // Inicia com LED desligado

        // Sala (canto superior esquerdo)
        if (sl && (i == 24 || i == 23 || i == 16 || i == 15))
        {
            if (sl_ng)
            {
                r = 5; // Luz noturna (amarelo suave)
                g = 3;
            }
            else
            {
                r = g = b = 5; // Branco suave
            }
        }
        // Quarto1 (canto superior direito)
        else if (q1 && (i == 18 || i == 19 || i == 20 || i == 21))
        {
            if (q1_ng)
            {
                r = 5; // Luz noturna (amarelo suave)
                g = 3;
            }
            else
            {
                r = g = b = 5; // Branco suave
            }
        }
        // Cozinha (canto inferior esquerdo)
        else if (cz && (i == 3 || i == 4 || i == 5 || i == 6))
        {
            if (cz_ng)
            {
                r = 5; // Luz noturna (amarelo suave)
                g = 3;
            }
            else
            {
                r = g = b = 5; // Branco suave
            }
        }
        // Quarto2 (canto superior esquerdo)
        else if (q2 && (i == 0 || i == 1 || i == 8 || i == 9))
        {
            if (q2_ng)
            {
                r = 5; // Luz noturna (amarelo suave)
                g = 3;
            }
            else
            {
                r = g = b = 5; // Branco suave
            }
        }
        // Cruz central (sempre vermelha)
        else if ((i % 5 == 2) || (i >= 10 && i <= 14))
        {
            r = 10;
            g = 0;
            b = 0;
        }

        // Envia a cor para o LED atual
        put_pixel(urgb_u32(r, g, b));
    }
}

// Inicializa o display OLED
void display_init()
{
    i2c_init(I2C_PORT, 400 * 1000); // Inicializa I2C a 400kHz
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C); // Configura pino SDA
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C); // Configura pino SCL
    gpio_pull_up(PIN_I2C_SDA); // Ativa resistor pull-up
    gpio_pull_up(PIN_I2C_SCL); // Ativa resistor pull-up

    // Inicializa o display SSD1306 (128x64 pixels)
    ssd1306_init(&ssd, 128, 64, false, OLED_ADDRESS, I2C_PORT);
    ssd1306_config(&ssd); // Configurações iniciais
    ssd1306_send_data(&ssd); // Envia os dados para o display
}

// Atualiza o conteúdo do display OLED
void atualizar_display()
{
    ssd1306_fill(&ssd, false); // Limpa o display
    
    char linha[22]; // Buffer para as linhas de texto

    // Primeira linha: Sala
    snprintf(linha, sizeof(linha), "Sala: %s", sl ? "ON " : "OFF");
    ssd1306_draw_string(&ssd, linha, 5, 5);

    // Segunda linha: Quarto 1
    snprintf(linha, sizeof(linha), "Quarto1: %s", q1 ? "ON " : "OFF");
    ssd1306_draw_string(&ssd, linha, 5, 15);
    
    // Terceira linha: Cozinha
    snprintf(linha, sizeof(linha), "Cozinha: %s", cz ? "ON " : "OFF");
    ssd1306_draw_string(&ssd, linha, 5, 25);

    // Quarta linha: Quarto 2
    snprintf(linha, sizeof(linha), "Quarto2: %s", q2 ? "ON " : "OFF");
    ssd1306_draw_string(&ssd, linha, 5, 35);

    ssd1306_send_data(&ssd); // Atualiza o display com os novos dados
}

// Verifica se houve mudança nos estados dos cômodos
bool mudanca_display()
{
    return sl != prev_sl || q1 != prev_q1 || cz != prev_cz || q2 != prev_q2;
}

// Configura o PWM para o buzzer
void configurar_pwm_buzzer(uint buzzer_pin)
{
    gpio_set_function(buzzer_pin, GPIO_FUNC_PWM); // Configura o pino como PWM
    uint slice = pwm_gpio_to_slice_num(buzzer_pin); // Obtém o slice PWM
    pwm_set_wrap(slice, 12500); // Configura para 1kHz (125MHz/12500)
    pwm_set_enabled(slice, true); // Habilita o slice PWM
}

// Toca um tom específico no buzzer
void tocar_tom(uint buzzer_pin, int freq_hz, int duracao_ms, int intervalo_ms)
{
    uint slice = pwm_gpio_to_slice_num(buzzer_pin);
    uint32_t clock = 125000000; // Clock padrão do RP2040 (125MHz)
    uint32_t wrap = clock / freq_hz;

    // Limita o wrap para o máximo de 16 bits (65535)
    if (wrap > 65535) {
        wrap = 65535;
        freq_hz = clock / wrap; // Ajusta a frequência para o máximo possível
    }

    pwm_set_wrap(slice, wrap);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(buzzer_pin), wrap / 2); // 50% duty cycle

    for (int i = 0; i < 2; i++)
    {
        pwm_set_enabled(slice, true); // Liga o som
        sleep_ms(duracao_ms);
        pwm_set_enabled(slice, false); // Desliga o som
        if (i == 0)
            sleep_ms(intervalo_ms); // Intervalo entre os bips
    }
}

// Toca o som de confirmação quando um cômodo é ligado/desligado
void tocar_buzzer_ligado()
{
    tocar_tom(buzzer, 300, 100, 0); // Toca um tom de 300Hz por 100ms com intervalo de 0ms
}