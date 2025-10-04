#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define UART_TX_PIN PD3
#define UART_RX_PIN PD2

#define TX_BUFFER_SIZE 64
#define RX_BUFFER_SIZE 64

typedef struct {
    volatile uint8_t head;
    volatile uint8_t tail;
    char buffer[RX_BUFFER_SIZE];
} RingBuffer;

RingBuffer rx_buffer = {0, 0};
RingBuffer tx_buffer = {0, 0};

volatile uint16_t timer_ticks_per_bit;

enum TxState { TX_IDLE, TX_START_BIT, TX_DATA_BITS, TX_STOP_BIT };
volatile TxState tx_state = TX_IDLE;
volatile uint8_t tx_byte;
volatile uint8_t tx_bit_index;

enum RxState { RX_IDLE, RX_DATA_BITS, RX_STOP_BIT };
volatile RxState rx_state = RX_IDLE;
volatile uint8_t rx_byte;
volatile uint8_t rx_bit_index;

void uart_set_baudrate(uint16_t baud) {
    uint32_t prescaler = 8;
    timer_ticks_per_bit = (F_CPU / prescaler) / baud;

    DDRD |= (1 << UART_TX_PIN);
    PORTD |= (1 << UART_TX_PIN);

    DDRD &= ~(1 << UART_RX_PIN);
    PORTD |= (1 << UART_RX_PIN);

    TCCR1A = 0;
    TCCR1B = (1 << CS11);

    EICRA |= (1 << ISC01);
    EICRA &= ~(1 << ISC00);
    EIMSK |= (1 << INT0); 

    sei();
}

// Поместить байт в буфер передачи
void uart_send(char b) {
    uint8_t next_head = (tx_buffer.head + 1) % TX_BUFFER_SIZE;
    while (next_head == tx_buffer.tail) {
        // Буфер полон, ждем (или можно вернуть ошибку)
    }
    tx_buffer.buffer[tx_buffer.head] = b;
    tx_buffer.head = next_head;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (tx_state == TX_IDLE) {
            tx_state = TX_START_BIT;
            OCR1B = TCNT1 + 10;
            TIMSK1 |= (1 << OCIE1B);
        }
    }
}

void uart_send_string(const char *msg) {
    while (*msg) {
        uart_send(*msg++);
    }
}

uint8_t uart_available() {
    uint8_t count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        count = (rx_buffer.head - rx_buffer.tail + RX_BUFFER_SIZE) % RX_BUFFER_SIZE;
    }
    return count;
}

char uart_read() {
    if (rx_buffer.head == rx_buffer.tail) {
        return -1;
    }
    char b = rx_buffer.buffer[rx_buffer.tail];
    rx_buffer.tail = (rx_buffer.tail + 1) % RX_BUFFER_SIZE;
    return b;
}

bool uart_read_string(char *rx_data, uint8_t max_len) {
    if (!uart_available()) return false;
    uint8_t i = 0;
    while (uart_available() && i < (max_len - 1)) {
        rx_data[i++] = uart_read();
    }
    rx_data[i] = '\0';
    return true;
}


ISR(INT0_vect) {
    EIMSK &= ~(1 << INT0);

    rx_state = RX_DATA_BITS;
    rx_bit_index = 0;
    rx_byte = 0;

    OCR1A = TCNT1 + (timer_ticks_per_bit * 3 / 2);
    TIMSK1 |= (1 << OCIE1A); 
}

ISR(TIMER1_COMPA_vect) {
    OCR1A += timer_ticks_per_bit;

    switch (rx_state) {
        case RX_DATA_BITS:
            if (!(PIND & (1 << UART_RX_PIN))) {
                // Бит равен 0, ничего не делаем, т.к. rx_byte уже 0
            } else {
                // Бит равен 1
                rx_byte |= (1 << rx_bit_index);
            }
            rx_bit_index++;

            if (rx_bit_index >= 8) {
                rx_state = RX_STOP_BIT;
            }
            break;

        case RX_STOP_BIT:
            if (PIND & (1 << UART_RX_PIN)) {
                uint8_t next_head = (rx_buffer.head + 1) % RX_BUFFER_SIZE;
                if (next_head != rx_buffer.tail) {
                    rx_buffer.buffer[rx_buffer.head] = rx_byte;
                    rx_buffer.head = next_head;
                } 
            }
            rx_state = RX_IDLE;
            TIMSK1 &= ~(1 << OCIE1A); 
            EIFR |= (1 << INTF0);     
            EIMSK |= (1 << INT0);     
            break;

        default:
             rx_state = RX_IDLE; 
             TIMSK1 &= ~(1 << OCIE1A);
             EIMSK |= (1 << INT0);
             break;
    }
}

ISR(TIMER1_COMPB_vect) {
    OCR1B += timer_ticks_per_bit;
    
    switch (tx_state) {
        case TX_START_BIT:
            PORTD &= ~(1 << UART_TX_PIN);
            
            tx_byte = tx_buffer.buffer[tx_buffer.tail];
            tx_buffer.tail = (tx_buffer.tail + 1) % TX_BUFFER_SIZE;

            tx_bit_index = 0;
            tx_state = TX_DATA_BITS;
            break;

        case TX_DATA_BITS:
            if (tx_byte & (1 << tx_bit_index)) {
                PORTD |= (1 << UART_TX_PIN); // 1
            } else {
                PORTD &= ~(1 << UART_TX_PIN); // 0
            }
            tx_bit_index++;
            
            if (tx_bit_index >= 8) {
                tx_state = TX_STOP_BIT;
            }
            break;

        case TX_STOP_BIT:
            PORTD |= (1 << UART_TX_PIN);
            tx_state = TX_IDLE;
            break;

        case TX_IDLE:
            if (tx_buffer.head != tx_buffer.tail) {
                tx_state = TX_START_BIT;
            } else {
                TIMSK1 &= ~(1 << OCIE1B);
            }
            break;
    }
}


void setup() {
    uart_set_baudrate(9600);
    uart_send_string("Software UART Initialized.\nSend me something!\n");
}

void loop() {
    if (uart_available()) {
        char c = uart_read();
        uart_send('E');
        uart_send('c');
        uart_send('h');
        uart_send('o');
        uart_send(':');
        uart_send(' ');
        uart_send(c);
        uart_send('\n');
    }

    // Здесь можно выполнять другую работу, UART работает в фоне
}