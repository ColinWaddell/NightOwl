#include <Arduino.h>

/**********************************************************
 * Power Management
 *********************************************************/
#include <RTCZero.h>

#define LOW_POWER_SLEEP 1
#define REQUIRE_USB 0
#define VBATPIN A7

RTCZero zerortc;

/* Set the wake-up period */
const uint8_t wakeup_seconds = 1;
const uint8_t wakeup_minutes = 0;
const uint8_t wakeup_hours = 0;

void resetAlarm(void) {
    zerortc.setTime(0, 0, 0);
    zerortc.setDate(1, 1, 1);

    zerortc.setAlarmTime(wakeup_hours, wakeup_minutes, wakeup_seconds);
    zerortc.enableAlarm(zerortc.MATCH_HHMMSS);
}

void blink_led() {
    static bool blink_high = false;

    digitalWrite(LED_BUILTIN, blink_high ? LOW : HIGH);
    blink_high ^= 0x01;
}

/**********************************************************
 * Radio Config
 *********************************************************/
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <RadioHead.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 915.0
#define RF95_TX_PWR 20 /* 5dBm to 23dBm */
#define RF95_NODE_ADDRESS 0

/* Singleton instance of the radio driver */
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, RF95_NODE_ADDRESS);

/* Radio Packet */
typedef struct __attribute__((__packed__)) _rf_packet {
    uint16_t packet_number;
    uint16_t luminance;
    uint16_t door_open_seconds;
    uint16_t battery_voltage;
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
}

/**********************************************************
 * Door Detection
 *********************************************************/
#define DOOR_PIN 12

volatile bool _door_open = false;

void door_changed_ISR() {
    noInterrupts();
    _door_open = digitalRead(DOOR_PIN);
    interrupts();
}

void door_init() {
    pinMode(DOOR_PIN, INPUT_PULLUP);
    _door_open = digitalRead(DOOR_PIN);
    attachInterrupt(digitalPinToInterrupt(DOOR_PIN), door_changed_ISR, CHANGE);
}

bool door_is_open() {
    return _door_open;
}

/**********************************************************
 * Luminance Config
 *********************************************************/
#define LUMINANCE_AIN 3

uint16_t luminance_read() {
    return 1023 - analogRead(LUMINANCE_AIN);
}

/**********************************************************
 * Board Setup
 *********************************************************/
void setup() {
    /* Debug output */
    Serial.begin(115200);

    /* Blinky light */
    pinMode(LED_BUILTIN, OUTPUT);

    radio_init();
    door_init();

    /* Power management */
#if LOW_POWER_SLEEP
    delay(500);
    zerortc.begin();
    resetAlarm();
#endif

#if REQUIRE_USB == 0
    USBDevice.detach();
#endif
}

/**********************************************************
 * Sleep thresholds
 *********************************************************/
#define TX_DOOR_STEADYSTATE_PERIOD 10 /* Seconds */

/* Transmit N second to signal change of state
 * Repeating transactions ensures it's received */
#define TX_DOOR_CHANGE_STATE_REPEATS 3 

/**********************************************************
 * Super-loop
 *********************************************************/
void loop() {
    static bool first_loop = true;
    static bool door_open_previously = false;
    static uint16_t packet_n = 0;
    static uint16_t door_open_seconds = 0;
    static uint16_t time_since_tx = 0;
    static uint8_t repeat_tx = 0;

    uint16_t luminance = 0;
    bool door_open = door_is_open();

    /* Toggle light on each loop*/
    blink_led();

    /* Acquire data */
    if (door_open) {
        /* Increment door seconds counter */
        door_open_seconds++;
        if (!door_open_seconds) {
            /* Integer overflow handling */
            door_open_seconds++;
        }
    }
    else {
        door_open_seconds = 0;
    }

    /* Send a packet once every TX_DOOR_STEADYSTATE_PERIOD
     * If a change of the door's state is noticed then 
     * to ensure that the Rx unit is updated we send updates
     * for the next TX_DOOR_CHANGE_STATE_REPEATS seconds */

    if (door_open_previously != door_open){
        repeat_tx = TX_DOOR_CHANGE_STATE_REPEATS;
    }

    if (
        first_loop || repeat_tx || time_since_tx >= TX_DOOR_STEADYSTATE_PERIOD 
    ) {
        /* Only read ADC if we're transmitting */
        luminance = luminance_read();

        /* Build packet */
        packet_n++;
        rf_packet packet = {
            .packet_number = packet_n,
            .luminance = luminance,
            .door_open_seconds = door_open_seconds,
            .battery_voltage = analogRead(VBATPIN)
        };

        if (manager.sendtoWait((uint8_t *)&packet, sizeof(packet), RH_BROADCAST_ADDRESS)) {
            /* If successfully sent then reset counter */
            /* Reset as 1 as this is the expected value on the next loop */
            time_since_tx = 1;

            /* If we're repeating transactions
             * then decrement the counter */
            if (repeat_tx){
                repeat_tx--;
            }
        }
        else {
            Serial.println("Broadcast failed");
        }

        Serial.printf(
            "[TX #%05d] Luminance = %04d, Door Open = %05d seconds\n",
            packet_n,
            luminance,
            door_open_seconds
        );
    }
    else {
        time_since_tx++;
    }

    /* Tidy up operations */
    rf95.sleep();
    first_loop = false;
    door_open_previously = door_open;

    /* Sleep till next loop */
#if LOW_POWER_SLEEP /* Put device to sleep */
    resetAlarm();
    Serial.end();
#if REQUIRE_USB
    USBDevice.detach();     /* Safely detach the USB prior to sleeping */
#endif
    zerortc.standbyMode();  /* Sleep until next alarm match */
#if REQUIRE_USB
    USBDevice.attach();     /* Re-attach the USB */
#endif

#else /* Regular delay */
    delay(1000);
#endif
}
