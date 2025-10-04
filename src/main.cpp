#include <Arduino.h>
#define LED_COUNT 5

const unsigned int led_periods[LED_COUNT] = {1, 2, 3, 4, 5};

volatile unsigned int led_counters[LED_COUNT] = {0};

volatile bool readyToPrint = false;

void setup() {
  Serial.begin(9600);
  Serial.println("--- System Initialized (v3 - XOR Toggle Logic) ---");

  DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4);
  Serial.print("DDRB register set to: 0b");
  Serial.println(DDRB, BIN);

  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11) | (1 << CS10);
  OCR1A = 2499;
  TIMSK1 |= (1 << OCIE1A);

  Serial.println("Timer1 configured for 100Hz interrupts.");

  sei();
  Serial.println("Global interrupts enabled. Starting...");
  Serial.println("------------------------------------");
}

ISR(TIMER1_COMPA_vect) {
  for (int i = 0; i < LED_COUNT; i++) {
    led_counters[i]++;

    if (led_counters[i] >= led_periods[i]) {
      PORTB ^= (1 << i);

      led_counters[i] = 0;
    }
  }

  readyToPrint = true;
}

void loop() {
  if (readyToPrint) {
    readyToPrint = false;

    static int print_counter = 0;
    print_counter++;

    if (print_counter >= 50) { // Вывод каждые 500 мс
      print_counter = 0;

      Serial.print("Counters: [");
      for(int i = 0; i < LED_COUNT; i++) {
        Serial.print(led_counters[i]);
        if (i < LED_COUNT - 1) Serial.print(", ");
      }
      Serial.print("] \t");

      Serial.print("PORTB State: 0b");
      for (int i = 4; i >= 0; i--) {
        Serial.print(bitRead(PORTB, i));
      }
      Serial.println();
    }
  }
}