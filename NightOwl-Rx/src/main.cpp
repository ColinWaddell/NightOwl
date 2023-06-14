#include <Arduino.h>

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

/**********************************************************
 * Settings
 *********************************************************/
#define LIGHT_TIMEOUT 30

/**********************************************************
 * Power Management
 *********************************************************/
void blink_led() {
    static bool blink_high = false;

    digitalWrite(LED_BUILTIN, blink_high ? LOW : HIGH);
    blink_high ^= 0x01;
}

/**********************************************************
 * Radio Config
 *********************************************************/
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 915.0
#define RF95_TX_PWR 5 /* 5dBm to 23dBm */
#define RF95_NODE_ADDRESS 1

/* Singleton instance of the radio driver */
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, RF95_NODE_ADDRESS);

/* Radio Packet */
typedef struct __attribute__((__packed__)) _rf_packet {
    bool door_open;
} rf_packet;

void radio_init() {
    /* Radio Config */
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);

    /* Manual reset */
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    /* while (!rf95.init()) { */
    while (!manager.init()) {
        /* Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info */
        Serial.println("LoRa radio init failed");
        while (1)
            ;
    }

    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1)
            ;
    }
    Serial.print("Set Freq to: ");
    Serial.println(RF95_FREQ);

    rf95.setTxPower(RF95_TX_PWR, false);
    Serial.println(RF95_FREQ);
}

/**********************************************************
 * Relay Config
 *********************************************************/
#define RELAY_SET 11
#define RELAY_UNSET 10
#define RELAY_HOLD_MS 10
#define LAMP_FLASHES 2

void relay_open() {
    digitalWrite(RELAY_SET, HIGH);
    delay(RELAY_HOLD_MS);
    digitalWrite(RELAY_SET, LOW);
}

void relay_close() {
    digitalWrite(RELAY_UNSET, HIGH);
    delay(RELAY_HOLD_MS);
    digitalWrite(RELAY_UNSET, LOW);
}

void relay_init() {
    pinMode(RELAY_SET, OUTPUT);
    pinMode(RELAY_UNSET, OUTPUT);
    relay_close();
}

void flash_lamp() {
    uint8_t i = 0;

    for (i = 0; i < LAMP_FLASHES; i++) {
        relay_open();
        delay(500);
        relay_close();
        delay(500);
    }
}

/**********************************************************
 * Board setup
 *********************************************************/
void setup() {
    /* Debug output */
    Serial.begin(115200);

    /* Blinky light */
    pinMode(LED_BUILTIN, OUTPUT);

    radio_init();
    relay_init();

    /* Indicate we're running */
    flash_lamp();
}

/**********************************************************
 * Super-loop
 *********************************************************/
void loop() {
    rf_packet packet;
    uint8_t pkt_len = sizeof(rf_packet);
    uint8_t from;
    static uint16_t timeout_remaining = 0;

    /* Toggle light on each loop*/
    blink_led();

    if (manager.recvfromAck((uint8_t *)&packet, &pkt_len, &from)) {
        Serial.printf("Door changed state: ");

        if (packet.door_open) {
            Serial.printf("OPEN\n");
            relay_open();
            timeout_remaining = LIGHT_TIMEOUT;
        }
        else {
            Serial.printf("CLOSED\n");
            relay_close();
        }
    }

    if (timeout_remaining) {
        if (timeout_remaining == 1) {
            Serial.printf("TIMEOUT\n");
            relay_close();
        }
    }

    /* 1 second between loops */
    delay(1000);
}
