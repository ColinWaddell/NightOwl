#include <Arduino.h>

/**********************************************************
 * Power Management
 *********************************************************/
#include <RTCZero.h>

#define LOW_POWER_SLEEP 1
#define REQUIRE_USB 0
#define VBATPIN A7

RTCZero zerortc;

/* Set the wake-up period
 * If LOW_POWER_SLEEP is disabled then
 * only set a delay of a couple of seconds
 * */
#define WAKEUP_SECONDS 3
#define WAKEUP_MINUTES 0
#define WAKEUP_HOURS 0

#define WAKEUP_MILLI_SECONDS (                                                      \
    WAKEUP_SECONDS * 1000 + WAKEUP_MINUTES * 60 * 1000 + WAKEUP_HOURS * 3600 * 1000 \
)

void resetAlarm(void) {
    zerortc.setTime(0, 0, 0);
    zerortc.setDate(1, 1, 1);

    zerortc.setAlarmTime(WAKEUP_HOURS, WAKEUP_MINUTES, WAKEUP_SECONDS);
    zerortc.enableAlarm(zerortc.MATCH_HHMMSS);
}

void blink_led() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
    delay(10);
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
#define RF95_TX_PWR 5 /* 5dBm to 23dBm */
#define RF95_NODE_ADDRESS 0

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
 * Switching Thresholds
 *********************************************************/
#define LUMINANCE_THRESHOLD 20

/**********************************************************
 * Super-loop
 *********************************************************/
void loop() {
    static bool first_loop = true;
    static bool door_open_previously = false;
    bool perform_tx = false;
    bool door_is_open = false;

    blink_led();

    if (!first_loop) {
        /* Tx a packet if the door has changed state and
         * there is enough light
        */
        door_is_open = door_is_open();
        if (door_open_previously != door_is_open) {
            if (luminance_read() < LUMINANCE_THRESHOLD) {
                perform_tx = true;
            }
        }
    }
    else {
        /* Always Tx on boot */
        perform_tx = true;
    }

    /* Tx a packet if it's required */
    if (perform_tx) {
        rf_packet packet = {
            .door_open = door_is_open
        }

        if (manager.sendtoWait((uint8_t *)&packet, sizeof(packet), RH_BROADCAST_ADDRESS)) {
            blink_led();
        }
        else {
            Serial.println("Broadcast failed");
        }
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
    USBDevice.detach();    /* Safely detach the USB prior to sleeping */
#endif
    zerortc.standbyMode(); /* Sleep until next alarm match */
#if REQUIRE_USB
    USBDevice.attach();    /* Re-attach the USB */
#endif

#else /* Regular delay */
    delay(WAKEUP_MILLI_SECONDS);
#endif
}
