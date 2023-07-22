#include <Arduino.h>
#include <assert.h>

/**********************************************************
 * Watchdog Setup
 *********************************************************/
#include <Adafruit_SleepyDog.h>

#define WDT_MS 2000

/**********************************************************
 * General Config
 *********************************************************/
#define VERBOSE_DEBUG 0
#define LOW_POWER_SLEEP 1 /* USB and Serial disabled after initialisation when true */
#define STATUS_BLINKS 1   /* Different patterns to show status. When false BLINK_NOTHING_HAPPENED used */
#define VBATPIN A7
#define SLEEP_MS 3000

/**********************************************************
 * Status Blinks
 *********************************************************/
typedef enum {
    BLINK_FIRST_LOOP,
    BLINK_NOTHING_HAPPENED,
    BLINK_DOOR_CHANGED,
    BLINK_TX_SUCCESS,
    BLINK_TX_FAILURE,
    BLINK_CODES_ERROR /* Must be last entry */
} blink_code;

/* Table of blink status codes versus patterns*/
struct _blink_patterns {
    uint32_t duration;
    uint8_t blinks;
} const BLINKS[] = {
    [BLINK_FIRST_LOOP] = { .duration = 500, .blinks = 1 },
    [BLINK_NOTHING_HAPPENED] = { .duration = 10, .blinks = 1 },
    [BLINK_DOOR_CHANGED] = { .duration = 100, .blinks = 2 },
    [BLINK_TX_SUCCESS] = { .duration = 200, .blinks = 4 },
    [BLINK_TX_FAILURE] = { .duration = 200, .blinks = 8 },
    [BLINK_CODES_ERROR] = { .duration = 800, .blinks = 8 },
};

void blink_led(blink_code status) {
    if (status > BLINK_CODES_ERROR) {
        status = BLINK_CODES_ERROR;
    }

    /* Very easy to accidentally overrun
     * the WDT here so remember to give
     * it a kick. Check the two aren't
     * in conflict before blinking.
     */
    assert(BLINKS[status].duration < WDT_MS);

    for (uint8_t i = 0; i < BLINKS[status].blinks; i++) {
        bool last_loop = (i == (BLINKS[status].blinks - 1));

        digitalWrite(LED_BUILTIN, HIGH);
        delay(BLINKS[status].duration);
        Watchdog.reset();

        digitalWrite(LED_BUILTIN, LOW);
        if (!last_loop) {
            delay(BLINKS[status].duration);
            Watchdog.reset();
        }
    }
}

float battery_voltage() {
    float bat_v;

    bat_v = analogRead(VBATPIN);
    bat_v *= 2;    /* v-divider halfs voltage */
    bat_v *= 3.3;  /* Multiply by 3.3V, our reference voltage */
    bat_v /= 1024; /* convert to voltage */

    return bat_v;
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
#define RF95_TX_PWR 10 /* 5dBm to 23dBm */
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
        while (1) {
            /* nop */
        }
    }

    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1) {
            /* nop */
        }
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
    /* Enable Watchdog */
    (void)Watchdog.enable(WDT_MS);

    /* Debug output */
    Serial.begin(115200);

    /* Blinky light */
    pinMode(LED_BUILTIN, OUTPUT);

    radio_init();
    door_init();

    /* Power management */
#if LOW_POWER_SLEEP
    Serial.end();
    USBDevice.detach();
#endif
}

/**********************************************************
 * Switching Thresholds
 *********************************************************/
#define LUMINANCE_THRESHOLD 50

/**********************************************************
 * Super-loop
 *********************************************************/
void loop() {
    static bool first_loop = true;
    static bool door_open_previously = false;
    bool perform_tx = false;
    bool door_open = false;
    blink_code status = BLINK_NOTHING_HAPPENED;

    door_open = door_is_open();

    if (!first_loop) {
        /* Tx a packet if the door has changed state and
         * there is enough light
        */
        if (door_open_previously != door_open) {
            status = BLINK_DOOR_CHANGED;
            if (luminance_read() < LUMINANCE_THRESHOLD) {
                perform_tx = true;
            }
        }
    }
    else {
        /* Always Tx on boot */
        perform_tx = true;
        status = BLINK_FIRST_LOOP;
    }

    /* Tx a packet if it's required */
    if (perform_tx) {
        rf_packet packet = {
            .door_open = door_open
        };

#if 0
        /* Makes no difference. Leaving in here for
         * now in case I want to try a similar trick
         */

        /* Give the radio time to wake up */
        rf95.setModeIdle();
        delay(10);
#endif
        if (manager.sendtoWait((uint8_t *)&packet, sizeof(packet), RH_BROADCAST_ADDRESS)) {
            status = BLINK_TX_SUCCESS;
        }
        else {
            status = BLINK_TX_FAILURE;
#if VERBOSE_DEBUG
            Serial.println("Broadcast failed");
#endif
        }
    }

#if STATUS_BLINKS
    blink_led(status);
#else
    blink_led(BLINK_NOTHING_HAPPENED);
#endif

#if VERBOSE_DEBUG
    Serial.printf("Door: %d, Luminance: %ld, Battery: ", door_open, luminance_read());
    Serial.println(battery_voltage());
#endif

    /* Tidy up operations */
    rf95.sleep();
    first_loop = false;
    door_open_previously = door_open;

    /* Sleep till next loop */
#if LOW_POWER_SLEEP /* Put device to sleep */
    Watchdog.sleep(SLEEP_MS);
#else  /* Regular delay */
    delay(SLEEP_MS);
#endif /* LOW_POWER_SLEEP */

    /* WDT Kick */
    Watchdog.reset();
    Watchdog.enable(WDT_MS);  // todo: test if I need to re-enable here.
                              // I've a suspicion that using the sleep
                              // function will leave the WDT in the wrong
                              // setup for it's expected functionality
}
