# 💡 WebServer de Controle de Iluminação com Raspberry Pi Pico W

Este projeto implementa um sistema de controle de iluminação utilizando o microcontrolador **RP2040** (Raspberry Pi Pico W). O sistema permite controlar remotamente uma **matriz de LEDs WS2812** representando diferentes cômodos da casa por meio de uma **interface web**. Inclui ainda **display OLED** para status e **buzzer** para feedback sonoro.

---

## 🔧 Funcionalidades

- 📶 Conexão Wi-Fi no modo cliente (Station Mode)
- 🌐 Servidor HTTP na porta 80
- 🎇 Controle de matriz de LEDs WS2812 (5x5)
- 📺 Display OLED via I2C para exibição do status dos cômodos
- 🔊 Buzzer para feedback sonoro ao alternar luzes
- 🖥️ Interface Web com botões para controle individual dos cômodos e modos de iluminação (normal e noturno)

---

## 🗂️ Organização do Código

1. Inclusão de bibliotecas
2. Definições e constantes
3. Variáveis globais
4. Protótipos de funções
5. Função principal (`main`)
6. Implementação das funções

---

## ⚙️ Hardware Utilizado

- Raspberry Pi Pico W (RP2040)
- Matriz de LEDs WS2812 (5x5)
- Display OLED (I2C - SSD1306)
- Buzzer passivo
- Fonte de alimentação 5V

---

## 🌈 Cômodos Representados

| Cômodo       | LEDs na Matriz | Luz Noturna |
|--------------|----------------|-------------|
| Sala         | LEDs 24, 23, 16, 15 | Sim |
| Quarto 1     | LEDs 14, 13, 8      | Sim |
| Cozinha      | LEDs 0, 1, 2, 3     | Sim |
| Quarto 2     | LEDs 10, 11, 6, 7   | Sim |

---

## 🌐 Interface Web

A interface HTML é gerada diretamente no código C e exibida ao acessar o IP do dispositivo no navegador. Ela inclui botões para:

- Ligar/Desligar luzes dos cômodos
- Ativar luz noturna (se a principal estiver ligada)

