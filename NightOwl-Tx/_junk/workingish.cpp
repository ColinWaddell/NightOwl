#include <Arduino.h>

/**********************************************************
 * Power Management
 *********************************************************/
#include <RTCZero.h>

#define LOW_POWER_SLEEP 0

RTCZero zerortc;

// Set how often alarm goes off here
const byte alarmSeconds = 1;
const byte alarmMinutes = 0;
const byte alarmHours = 0;

void rtc_alarm_ISR() {
}

void resetAlarm(void) {
    byte seconds = 0;
    byte minutes = 0;
    byte hours = 0;
    byte day = 1;
    byte month = 1;
    byte year = 1;

    zerortc.setTime(hours, minutes, seconds);
    zerortc.setDate(day, month, year);

    zerortc.setAlarmTime(alarmHours, alarmMinutes, alarmSeconds);
    zerortc.enableAlarm(zerortc.MATCH_HHMMSS);
}

void blink_led() {
    static bool blink_high = false;

    if (blink_high) {
        digitalWrite(LED_BUILTIN, LOW);
        blink_high = false;
    }
    else {
        digitalWrite(LED_BUILTIN, HIGH);
        blink_high = true;
    }
}

/**********************************************************
 * Radio Config
 *********************************************************/
#include <RHReliableDatagram.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 915.0
#define RF95_TX_PWR 5
#define RF95_NODE_ADDRESS 0

/* Singleton instance of the radio driver */
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, RF95_NODE_ADDRESS);

/* Radio Packet */
typedef struct __attribute__((__packed__)) _rf_packet {
    uint16_t packet_number;
    uint16_t luminance;
    uint16_t door_open_seconds;
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
    while (!Serial) {
        delay(1);
    }

    /* Blinky light */
    pinMode(LED_BUILTIN, OUTPUT);

    radio_init();
    door_init();

    /* Power management */
#if LOW_POWER_SLEEP
    delay(5000);
    zerortc.begin();
    resetAlarm();
    zerortc.attachInterrupt(rtc_alarm_ISR);
#endif
}

/**********************************************************
 * Sleep thresholds
 *********************************************************/
#define TX_DOOR_CLOSED_PERIOD 10
#define TX_DOOR_OPEN_MAXIMUM_PERIOD 60

/**********************************************************
 * Super-loop
 *********************************************************/
void loop() {
    static bool first_loop = true;
    static bool door_open_previously = false;
    static uint16_t packet_n = 0;
    static uint16_t door_open_seconds = 0;
    static uint16_t time_since_tx = 0;

    uint16_t luminance = 0;

    /* Toggle light on each loop*/
    toggle_led();

    /* Acquire data */
    if (door_is_open()) {
        /* Only read ADC if door is open */
        luminance = luminance_read();

        /* Increment door seconds counter */
        door_open_seconds++;
        if (!door_open_seconds) {
            /* Integer overflow protection */
            door_open_seconds++;
        }
    }
    else {
        door_open_seconds = 0;
        luminance = 0;
    }

    /* Send a packet every TX_DOOR_CLOSED_PERIOD seconds
     * or every loop until the door has been openned for
     * TX_DOOR_OPEN_MAXIMUM_PERIOD seconds. This is to
     * save power and not Tx every second if possible
     */
    if (
        first_loop || door_open_previously != door_is_open() || time_since_tx >= TX_DOOR_CLOSED_PERIOD || (door_open_seconds && door_open_seconds < TX_DOOR_OPEN_MAXIMUM_PERIOD)
    ) {
        /* Build packet */
        packet_n++;
        rf_packet packet = {
            .packet_number = packet_n,
            .luminance = luminance,
            .door_open_seconds = door_open_seconds
        };

        if (manager.sendtoWait((uint8_t *)&packet, sizeof(packet), RH_BROADCAST_ADDRESS)) {
            time_since_tx = 0;
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
    // rf95.sleep();
    first_loop = false;
    door_open_previously = door_is_open();

    /* Sleep till next loop */
#if LOW_POWER_SLEEP
    resetAlarm();
    Serial.end();
    USBDevice.detach();     // Safely detach the USB prior to sleeping
    zerortc.standbyMode();  // Sleep until next alarm match
    USBDevice.attach();     // Re-attach the USB, audible sound on windows machines
#else
    delay(1000);
#endif
}
