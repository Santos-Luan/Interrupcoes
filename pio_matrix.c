#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "pico/bootrom.h"

// arquivo .pio
#include "pio_matrix.pio.h"

// Limite X
double desenhoX[25] = {1.0, 0.0, 0.0, 0.0, 1.0,
                       0.0, 1.0, 0.0, 1.0, 0.0,
                       0.0, 0.0, 1.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 1.0, 0.0,
                       1.0, 0.0, 0.0, 0.0, 1.0};
// Numero 0
double desenho0[25] = {0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.2, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0};
// Numero 1
double desenho1[25] = {0.0, 0.0, 0.2, 0.0, 0.0,
                       0.0, 0.0, 0.2, 0.2, 0.0,
                       0.0, 0.0, 0.2, 0.0, 0.0,
                       0.0, 0.0, 0.2, 0.0, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0};
// vetor para criar imagem na matriz de led - 2
double desenho2[25] = {0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.0, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.0, 0.0, 0.2, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0};
// Numero 3
double desenho3[25] = {0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.0, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.0, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0};
// Nuemro 4
double desenho4[25] = {0.0, 0.2, 0.0, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.2, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.2, 0.0};
// Numero 5
double desenho5[25] = {0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.0, 0.0, 0.2, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.0, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0};
// Numero 6
double desenho6[25] = {0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.0, 0.0, 0.2, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.2, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0};
// Numero 7
double desenho7[25] = {0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.2, 0.0};
// Numero 8
double desenho8[25] = {0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.2, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.2, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0};
// Numero 9
double desenho9[25] = {0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.2, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0,
                       0.0, 0.2, 0.0, 0.0, 0.0,
                       0.0, 0.2, 0.2, 0.2, 0.0};

double *desenhos[] = {desenho0, desenho1, desenho2, desenho3, desenho4, desenho5, desenho6, desenho7, desenho8, desenho9, desenhoX};

// rotina para acionar a matrix de leds - ws2812b
void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b);

// Definição dos pinos dos LEDs e dos botões
#define LED_PIN_R 13
#define LED_PIN_G 11
#define LED_PIN_B 12
#define BTN_PIN_A 5
#define BTN_PIN_B 6
// número de LEDs
#define NUM_PIXELS 25
#define DEBOUNCE_DELAY 350 // Delay de debounce em milissegundos

// pino de saída
#define OUT_PIN 7

// Variável para debounce da interrupção
static volatile uint64_t last_interrupt_time = 0;
static volatile bool interrupt_flag = false; // Sinalizador da interrupção
static volatile int16_t a = 0;
static volatile bool btnA = false, btnB = false;
static volatile uint64_t last_interrupt_time_A = 0;
static volatile uint64_t last_interrupt_time_B = 0;

// Função para inicialização dos pinos
void inicializar_pinos();

// Função para definir cores dos LEDs
void set_leds(bool red, bool green, bool blue);

// Função de interrupção
static void gpio_irq_handler(uint gpio, uint32_t events);

// Função para leds
void leds();

// Função para definir a intensidade das cores --> exemplo aula
uint32_t matrix_rgb(double b, double r, double g);

// função principal
int main()
{
    PIO pio = pio0;
    uint16_t i;
    uint32_t valor_led;
    double r = 0.0, b = 0.0, g = 0.0;

    // Inicializa todos os códigos stdio padrão que estão ligados ao binário.
    stdio_init_all();
    inicializar_pinos();

    // configurações da PIO
    uint offset = pio_add_program(pio, &pio_matrix_program);
    uint sm = pio_claim_unused_sm(pio, true);
    pio_matrix_program_init(pio, sm, offset, OUT_PIN);

    desenho_pio(desenho0, valor_led, pio, sm, r, g, b);
    while (true)
    {
        leds();
        if (!gpio_get(BTN_PIN_A) && a < 10)
        {
            btnA = 1;
            // Configuração da interrupção para o botão A (borda de subida e descida)
            gpio_set_irq_enabled_with_callback(BTN_PIN_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
        }
        else if (!gpio_get(BTN_PIN_B) && a > -1)
        {
            btnB = 1;
            // Configuração da interrupção para o botão B (borda de subida e descida)
            gpio_set_irq_enabled_with_callback(BTN_PIN_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
        }

        if (a >= 0 && a <= 9)
        {
            // Chamada da função para imprimir os desenhos
            desenho_pio(desenhos[a], valor_led, pio, sm, r, g, b);
        }
    }
}

void inicializar_pinos()
{
    stdio_init_all();

    // Inicialização dos LEDs como saída
    gpio_init(LED_PIN_R);
    gpio_set_dir(LED_PIN_R, GPIO_OUT);
    gpio_init(LED_PIN_G);
    gpio_set_dir(LED_PIN_G, GPIO_OUT);
    gpio_init(LED_PIN_B);
    gpio_set_dir(LED_PIN_B, GPIO_OUT);

    // Inicialização dos botões como entrada
    gpio_init(BTN_PIN_A);
    gpio_set_dir(BTN_PIN_A, GPIO_IN);
    gpio_pull_up(BTN_PIN_A); // Pull-up para o botão A

    gpio_init(BTN_PIN_B);
    gpio_set_dir(BTN_PIN_B, GPIO_IN);
    gpio_pull_up(BTN_PIN_B); // Pull-up para o botão B
}
// Função para ligar os leds solicitados
void set_leds(bool red, bool green, bool blue)
{
    gpio_put(LED_PIN_R, red);
    gpio_put(LED_PIN_G, green);
    gpio_put(LED_PIN_B, blue);
}
// Tratamento de interrupções dos botões
static void gpio_irq_handler(uint gpio, uint32_t events)
{
    uint64_t current_time = to_ms_since_boot(get_absolute_time());// Obtém o tempo atual em milissegundos desde o boot do Raspberry Pi Pico

    if (gpio == BTN_PIN_A && (current_time - last_interrupt_time_A) > DEBOUNCE_DELAY)
    {
        if (a <= 9)
            a++; // Incrementa o número
        last_interrupt_time_A = current_time;
        printf("a incrementado para: %i\n", a);
    }
    else if (gpio == BTN_PIN_B && (current_time - last_interrupt_time_B) > DEBOUNCE_DELAY)
    {
        if (a >= 0)
            a--; // Decrementa o número
        last_interrupt_time_B = current_time;
        printf("a decrementado para: %i\n", a);
    }
}
// Pisca LEDs para indicar que o sistema está rodando
void leds()
{
    set_leds(1, 0, 0);
    sleep_ms(200);
    set_leds(0, 0, 0);
    sleep_ms(200);
}
// Converte valores RGB para o formato aceito pela matriz
uint32_t matrix_rgb(double b, double r, double g)
{
    unsigned char R, G, B;
    R = r * 255;
    G = g * 255;
    B = b * 255;
    return (G << 24) | (R << 16) | (B << 8);
}
// Função para enviar o desenho para a matriz de LEDs
void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b)
{
    for (int16_t i = 0; i < NUM_PIXELS; i++)
    {
        valor_led = matrix_rgb(desenho[24 - i], desenho[24 - i], g = 0.0);
        pio_sm_put_blocking(pio, sm, valor_led);
    }
}
