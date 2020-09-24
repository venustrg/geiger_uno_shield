/*
  Arduino Geiger counter 01.3 by toxcat // https://cxem.net/dozimetr/3-10.php
  Arduino 1.0.5
  ATmega328P 16MHz
  mod by venus@trg.ru - 2.3

  required libraries:
    hd44780 by Bill Perry (used v1.3.1)
    BMP085 unified by Adafruit (used v1.1.0)
    AHTX0 by Adafruit (used v2.0.0)

  indented with: indent -kr -nut -c 40 -cd 40 -l 120 geiger.ino
*/
#define APP_VERSION "2.3"

#include <hd44780.h>
#include <hd44780ioClass/hd44780_pinIO.h>       // Arduino pin i/o class header
#include <EEPROM.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//#define SERIAL_DEBUG

// rate measurement time
//#define GEIGER_TIME 75                 // 75 sec for SI29BG
//#define GEIGER_DIV  1                  // no div for SI29BG
//#define GEIGER_TIME 36                 // 36 sec for SBM-20
//#define GEIGER_DIV  1                  // no div for SBM-20
#define GEIGER_TIME 21                 // SBT-10A -- 2.17 / sec == 15 pulses for 6.9s ~~ 7s
#define GEIGER_DIV  3                  // SBT-10A -- TIME=28, DIV=4 or TIME=21, DIV=3

#define T2_FREQ  100000L               // timer2 (main) frequency

#define BOOST_FREQ  1000               // boost converter 10us pulses frequency

#define NUM_KEYS 5                     // number of keys

#define BOOST_DDR   DDRD
#define BOOST_PORT PORTD
#define BOOST        PD3

#define IN_DDR      DDRD
#define IN_PORT    PORTD
#define IN          PD2

#define BUZZ_DDR    DDRC
#define BUZZ_PORT  PORTC
#define BUZZ        PC1

#define LED_DDR     DDRB
#define LED_PORT   PORTB
#define LED          PB5

#define BUZZER_ON bitSet(BUZZ_PORT, BUZZ)
#define BUZZER_OFF bitClear(BUZZ_PORT, BUZZ)

#define BOOSTER_ON bitSet(BOOST_PORT, BOOST)
#define BOOSTER_OFF bitClear(BOOST_PORT, BOOST)

#define BEEP_FREQ 2000                 // alarm beep frequency (Hz)

enum scr_pages {
    SCR_RATE = 0,
    SCR_DOSE,
    SCR_SENSORS
};

enum menu_pages {
    SET_BUZZER_VOL = 0,
    SET_ALARM_VOL,
    SET_ALARM_LEVEL,
    SET_BRIGHTNESS
};

volatile uint8_t beep_gen = 0;         // beep now, 1 == on, 0 == off
volatile uint16_t beep_high;           // beep meander high level length in 10us units
volatile uint16_t beep_low;            // beep meander low level length in 10us units
volatile uint8_t tick_gen = 0;         // tick now, 1 == on, no need for 0

volatile uint16_t boost_pulses;        // pulses to go with boost

uint8_t brightness = 4;                // screen brightness 0..5

//LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
//const int rs = 8, en = 9, db4 = 4, db5 = 5, db6 = 6, db7 = 7, bl = 10, blLevel = LOW;
//hd44780_pinIO lcd(rs, en, db4, db5, db6, db7, bl, blLevel);
hd44780_pinIO lcd(8, 9, 4, 5, 6, 7, 10, LOW);

Adafruit_BME280 bme;                   // I2C

#define BME280_ADDR 0x76

#define SENS_NONE   0x00
#define SENS_BME280 0x01

#define SEALEVELPRESSURE_HPA (1013.25)

uint8_t sensors = SENS_NONE;
uint8_t keep = 0;
float prs, alt, hum, temp;

#define SENS_TIME 2

enum keypress {
    KEY_RIGHT = 1,
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_SELECT
};

uint16_t adc_key_val[5] = { 50, 200, 400, 600, 800 };   // ADC values for keys

volatile uint16_t geiger[GEIGER_TIME]; // per-sec pulse counters
volatile uint32_t total;               // total pulse counter

volatile uint32_t rate;                // current rate
volatile uint32_t rate_max;            // max rate
uint32_t dose;                         // calculated dose
uint8_t t_sec, t_min, t_hrs;           // clock

//volatile uint8_t scr_page = SCR_RATE;  // currently displayed page
volatile uint8_t scr_page = SCR_SENSORS;        // currently displayed page

uint8_t redraw = 1;                    // redraw screen
uint8_t alarm = 0;                     // alarm raised
uint8_t alarm_disable = 0;             // alarm prohibited
uint8_t alarm_wait = 0;                // alarm is awaiting for rate lowering
uint8_t buzz_disable = 0;              // buzz ticked prohibited

volatile uint8_t timer = 0;            // just for delay
volatile uint8_t timer_out = 0;        // timer exausted flag

uint8_t buzz_vol = 3;                  // buzz tick volume (1-5)
uint8_t beep_vol = 3;                  // alarm beep volume (1-5)
uint8_t alarm_level = 50;              // alarm rate level, uR/h (40..250 step 10)

uint8_t i;

#define SCR_WIDTH 16
char str_buf[SCR_WIDTH * 2];

// *INDENT-OFF*

// "alarm on" sign 1
uint8_t s0[8] = {
    0b00000,
    0b00100,
    0b01110,
    0b01110,
    0b01110,
    0b11111,
    0b00100,
    0b00000
};

// "alarm on" sign 2
uint8_t  s1[8] = {
    0b00000,
    0b00100,
    0b10010,
    0b01010,
    0b01010,
    0b10010,
    0b00100,
    0b00000
};

// "menu page" sign 
uint8_t  s2[8] = {
    0b00000,
    0b11111,
    0b00000,
    0b11111,
    0b00000,
    0b11111,
    0b00000,
    0b00000
};

// "awaiting" sign - "..."
uint8_t  s3[8] = {
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b10101
};

// "buzzer on" sign 1
uint8_t  s4[8] = {
    0b00000,
    0b00001,
    0b00011,
    0b01111,
    0b01111,
    0b00011,
    0b00001,
    0b00000
};

// "buzzer on" sign 2
uint8_t  s5[8] = {
    0b00000,
    0b00000,
    0b01110,
    0b01010,
    0b01010,
    0b11011,
    0b00000,
    0b00000
};

// *INDENT-ON*

void eeprom_read(void)
{
    if ((scr_page = EEPROM.read(0x00)) > SCR_SENSORS)
        scr_page = SCR_SENSORS;
    alarm_disable = EEPROM.read(0x01);
    buzz_disable = EEPROM.read(0x02);
    if (!(buzz_vol = EEPROM.read(0x03)) || buzz_vol > 5)
        buzz_vol = 5;
    if (!(beep_vol = EEPROM.read(0x04)) || beep_vol > 5)
        beep_vol = 5;
    if ((alarm_level = EEPROM.read(0x05)) < 40 || alarm_level > 250)
        alarm_level = 50;
    if ((brightness = EEPROM.read(0x06)) > 5)
        brightness = 4;
#ifdef SERIAL_DEBUG
    Serial.println("reading EEPROM");
#endif
}

void eeprom_update(void)
{
    EEPROM.update(0x00, scr_page);
    EEPROM.update(0x01, alarm_disable);
    EEPROM.update(0x02, buzz_disable);
    EEPROM.update(0x03, buzz_vol);
    EEPROM.update(0x04, beep_vol);
    EEPROM.update(0x05, alarm_level);
    EEPROM.update(0x06, brightness);
#ifdef SERIAL_DEBUG
    Serial.println("EEPROM updated");
#endif

}

// calculate timer parameters for given frequency and volume
void calc_beep()
{
    beep_high = beep_vol;
    beep_low = (T2_FREQ / BEEP_FREQ) - beep_high;
}

void set_brightness()
{
    // analogWrite(10, brightness * 50); // maybe +5
    // 0..5 ==> 255..5
    lcd.setBacklight(255 - brightness * 50);    // maybe +5
    delay(10);
#ifdef SERIAL_DEBUG
    Serial.println("brightness has been set");
#endif
}

void setup(void)
{

#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif

    bitSet(LED_DDR, LED);              // disable LED on PB5 (pin 13) - Arduino Uno
    bitClear(LED_PORT, LED);           // disable LED on PB5 (pin 13) - Arduino Uno

    bitSet(BOOST_DDR, BOOST);          // boost converter port - out
    BOOSTER_OFF;

    bitSet(BUZZ_DDR, BUZZ);            // buzzer port - out
    BUZZER_OFF;

    bitClear(IN_DDR, IN);              // input port - in
    bitSet(IN_PORT, IN);               // w/ pullup

    calc_beep();

    // timer2 - 100kHz (for 10us boost/sound pulses)
    TCNT2 = 0;
    TCCR2A = (1 << WGM21);             // CTC mode
    TCCR2B = (0 << CS22) | (1 << CS21) | (0 << CS20);   // div=8 -- 2MHz base
    OCR2A = F_CPU / 8 / T2_FREQ;       // calc OCR for 100kHz
    TIMSK2 |= (1 << OCIE2A);           // enable t2

    eeprom_read();

    lcd.begin(SCR_WIDTH, 2);
    set_brightness();
    lcd.setCursor(0, 0);
    lcd.write("GEiGER COUNtER");
    lcd.setCursor(0, 1);
    lcd.write(APP_VERSION " StARtiNG");

    // init BME280 sensor
    if (bme.begin(BME280_ADDR))
        sensors |= SENS_BME280;

    if (scr_page != SCR_SENSORS)
        boost_pulses = BOOST_FREQ;     // 1 sec boost
    delay(1000);                       // wait for boost
    lcd.clear();
    delay(10);
    lcd.createChar(0, s0);             // custom chars loading
    lcd.createChar(1, s1);
    lcd.createChar(2, s2);
    lcd.createChar(3, s3);
    lcd.createChar(4, s4);
    lcd.createChar(5, s5);

    // irq
    //EICRA = (1 << ISC01) | (0 << ISC00);        // irq0 - falling edge
    EICRA = (1 << ISC01) | (1 << ISC00);        // irq0 - rising edge
    EIMSK = (0 << INT1) | (1 << INT0); // enable irq0
}

enum boost_states {
    BOOST_IDLE = 0,
    BOOST_ON,
    BOOST_OFF,
    BOOST_PAUSE
};

enum beep_states {
    BEEP_IDLE = 0,
    BEEP_START,
    BEEP_ON,
    BEEP_OFF
};

uint8_t boost_idle = 0;

ISR(TIMER2_COMPA_vect)
{
    static uint8_t tick_running;
    static uint8_t boost_state = BOOST_IDLE;
    static uint8_t beep_state = BEEP_IDLE;
    static uint16_t boost_low;
    static uint32_t beep_running;
    static uint8_t i;
    static uint8_t cnt0 = 0;
    static uint8_t cnt1 = 0;
    static uint8_t cntms = 0;

    // boost processing
    switch (boost_state) {
    case BOOST_IDLE:
        if (scr_page != SCR_SENSORS && boost_pulses)
            boost_state = BOOST_ON;
        else
            break;
    case BOOST_ON:
        BOOSTER_ON;
        boost_state++;
        break;
    case BOOST_OFF:
        BOOSTER_OFF;
        boost_low = (T2_FREQ / BOOST_FREQ) - 1;
        boost_state++;
        break;
    case BOOST_PAUSE:
        if (!--boost_low) {
            boost_idle = 0;
            if (boost_pulses)
                boost_pulses--;
            boost_state = BOOST_IDLE;
        }
        break;
    }

    // beep processing
    if (beep_gen) {
        switch (beep_state) {
        case BEEP_IDLE:
            beep_state++;
        case BEEP_START:
            BUZZER_ON;
            beep_running = beep_high;
            beep_state++;
            break;
        case BEEP_ON:
            if (!--beep_running) {
                BUZZER_OFF;
                beep_running = beep_low;
                beep_state++;
            }
            break;
        case BEEP_OFF:
            if (!--beep_running)
                beep_state = BEEP_START;
            break;
        }
    } else if (beep_state != BEEP_IDLE) {
        BUZZER_OFF;
        beep_state = BEEP_IDLE;
    }

    // buzz ticker
    if (tick_gen) {
        BUZZER_ON;
        tick_running = buzz_vol;
        tick_gen = 0;
    } else if (tick_running && !--tick_running)
        BUZZER_OFF;

    if (++cntms == 100) {
        // 1kHz / 1ms here
        cntms = 0;

        if (++cnt0 == 10) {
            // 100Hz / 10ms here
            cnt0 = 0;

            // boost every 200 msec
            if (++boost_idle >= BOOST_FREQ / 50 && boost_pulses < BOOST_FREQ / 50)
                boost_pulses = BOOST_FREQ / 50; // 20 msec boost

            if (timer && !--timer)
                timer_out = 1;

            if (++cnt1 == 100) {
                // 1Hz / 1s here
                cnt1 = 0;

                for (i = 0, rate = 0; i < GEIGER_TIME; i++)
                    rate += geiger[i]; // count rate for GEIGER_TIME seconds

                rate /= GEIGER_DIV;

                if (rate > 999999)
                    rate = 999999;     // rate overflow

                if (rate > rate_max)
                    rate_max = rate;   // save max rate

                if (rate >= alarm_level)
                    alarm = 1;         // rate too high - alarm
                else {
                    alarm = 0;
                    if (alarm_wait) {
                        alarm_wait = 0;
                        alarm_disable = 0;
                    }
                }

                // next sec - shift geiger array, zero 1st
                for (i = GEIGER_TIME - 1; i > 0; i--)
                    geiger[i] = geiger[i - 1];
                geiger[0] = 0;

                // dose counting clock
                if (++t_sec > 59) {
                    t_sec = 0;
                    if (++t_min > 59) {
                        t_min = 0;
                        if (++t_hrs > 99) {
                            t_hrs = t_min = t_sec = 0;
                            total = 0;
                        }
                    }
                }
                redraw = 1;
            }
        }
    }
}

ISR(INT0_vect)                         // ext IRQ - count geiger pulses
{
    if (geiger[0] != 65535)
        geiger[0]++;                   // current sec counter

    // check for dose overflow (dose = total * GEIGER_TIME / GEIGER_DIV / 3600)
    if (++total > 999999UL * 3600 * GEIGER_DIV / GEIGER_TIME)
        total = 999999UL * 3600 * GEIGER_DIV / GEIGER_TIME;

    if (boost_pulses < BOOST_FREQ / 50)
        boost_pulses = BOOST_FREQ / 50; // 10 msec boost

    if (!buzz_disable && !tick_gen)
        tick_gen = 1;                  // make tick

}

// read and decode key pressed
uint8_t get_key(void)
{
    uint8_t key = 0;
    uint16_t adc_result = analogRead(0);

    for (uint8_t i = 0; i < NUM_KEYS; i++)
        if (adc_result < adc_key_val[i]) {
            key = i + 1;
            break;
        }
    return key;
}

uint8_t check_keys(void)
{
    uint8_t k = 0;
    static uint8_t old_key;

    uint8_t new_key = get_key();       // read key state
    if (new_key != old_key) {          // key state changed == key pressed
        delay(5);                      // protect from bouncing
        new_key = get_key();
        if (new_key != old_key) {
            old_key = new_key;
            k = new_key;
        }
    }
    return k;                          // 0 - not pressed, 1..5 - key
}

enum sound_states {
    SOUND_START = 0,
    SOUND_ON,
    SOUND_OFF
};

void alarm_warning(void)
{
    uint8_t sound_state = 0;
    uint32_t rad_alrm = 0;

    uint8_t old_buzz = buzz_disable;   // save ticker state
    buzz_disable = 1;                  // disable ticker

    lcd.clear();
    delay(10);

    lcd.setCursor(0, 1);
    lcd.write("WARNiNG");
    redraw = 1;
    sound_state = SOUND_START;

    while (1) {
        if (redraw) {
            redraw = 0;
            if (rate > rad_alrm)
                rad_alrm = rate;       // handle max rate
            snprintf(str_buf, SCR_WIDTH + 1, "%6lu uR/h", rad_alrm);
            lcd.setCursor(5, 0);
            lcd.write(str_buf);
        }

        switch (sound_state) {
        case SOUND_START:
            beep_gen = 1;
            timer = 35;                // tone length x 10ms
            timer_out = 0;
            sound_state = SOUND_ON;
            break;
        case SOUND_ON:
            if (timer_out) {
                beep_gen = 0;
                timer = 15;            // pause length x 10ms
                timer_out = 0;
                sound_state = SOUND_OFF;
            }
            break;
        case SOUND_OFF:
            if (timer_out)
                sound_state = SOUND_START;      // next tone
            break;
        }

        if (check_keys() == KEY_LEFT) { // stop alarm if LEFT pressed
            beep_gen = 0;
            lcd.setCursor(0, 1);
            lcd.write("AlARM DiSAblEd");
            for (timer_out = 0, timer = 150; !timer_out && check_keys() != KEY_LEFT;);
            lcd.clear();
            delay(10);
            buzz_disable = old_buzz;
            alarm_disable = 1;
            scr_page = SCR_RATE;
            redraw = 1;
            break;
        }
        delay(10);
    }
}

void menu(void)
{
    uint8_t menu_page = SET_BUZZER_VOL;

    lcd.clear();
    delay(10);
    redraw = 1;

    while (1) {
        if (alarm && alarm_disable == 0)
            alarm_warning();

        if (redraw) {                  // update screen
            redraw = 0;

            lcd.setCursor(0, 1);
            lcd.write(byte(2));        // "menu page" sign
            lcd.setCursor(2, 1);
            snprintf(str_buf, SCR_WIDTH + 1, "%01u", menu_page + 1);
            lcd.write(str_buf);
            switch (menu_page) {
            case SET_BUZZER_VOL:
                snprintf(str_buf, SCR_WIDTH + 1, "BUZZ VOLUME   %2u", buzz_vol);
                break;
            case SET_ALARM_VOL:
                snprintf(str_buf, SCR_WIDTH + 1, "ALARM VOLUME  %2u", beep_vol);
                break;
            case SET_ALARM_LEVEL:
                snprintf(str_buf, SCR_WIDTH + 1, "ALARM LEVEL  %3u", alarm_level);
                break;
            case SET_BRIGHTNESS:
                snprintf(str_buf, SCR_WIDTH + 1, "BRIGHTNESS    %2u", brightness);
                break;
            }
            lcd.setCursor(0, 0);
            lcd.write(str_buf);
        }
        // handle keypress
        switch (check_keys()) {
        case KEY_UP:
            switch (menu_page) {
            case SET_BUZZER_VOL:
                if (buzz_vol < 5)
                    buzz_vol++;
                break;
            case SET_ALARM_VOL:
                if (beep_vol < 5) {
                    beep_vol++;
                    calc_beep();
                }
                break;
            case SET_ALARM_LEVEL:
                if (alarm_level < 250)
                    alarm_level += 10;
                break;
            case SET_BRIGHTNESS:
                if (brightness < 5)
                    brightness++;
                set_brightness();
                break;
            }
            redraw = 1;
            break;
        case KEY_DOWN:
            switch (menu_page) {
            case SET_BUZZER_VOL:
                if (buzz_vol > 1)
                    buzz_vol--;
                break;
            case SET_ALARM_VOL:
                if (beep_vol > 1) {
                    beep_vol--;
                    calc_beep();
                }
                break;
            case SET_ALARM_LEVEL:
                if (alarm_level > 40)
                    alarm_level -= 10;
                break;
            case SET_BRIGHTNESS:
                if (brightness > 0)
                    brightness--;
                set_brightness();
                break;
            }
            redraw = 1;
            break;
        case KEY_LEFT:                // left key
        case KEY_RIGHT:
            redraw = 1;
            break;
        case KEY_SELECT:              // select key
            if (++menu_page > SET_BRIGHTNESS) {
                lcd.clear();
                delay(10);
                redraw = 1;
                return;
            }
            redraw = 1;
        default:
            break;
        }
        delay(10);
    }
}


void loop(void)
{
    uint8_t i;
    //  float val;
    static uint8_t eeup = 5;
    float temperature;
    if (alarm && alarm_disable == 0)
        alarm_warning();

    if (redraw) {                      // screen to be updated
        redraw = 0;
        // 1st line
        switch (scr_page) {
        case SCR_RATE:
            // rate, uR/h
            snprintf(str_buf, SCR_WIDTH + 1, "RAtE %6lu uR/h", rate);
            break;
        case SCR_DOSE:
            // dose, uR
            dose = total * GEIGER_TIME / GEIGER_DIV / 3600;
            snprintf(str_buf, SCR_WIDTH + 1, "D0SE   %6lu uR", dose);
            break;
        case SCR_SENSORS:
            if ((sensors & SENS_BME280)) {
                if (!keep) {
#ifdef SERIAL_DEBUG
                    Serial.println("running BME280..");
#endif
                    prs = bme.readPressure();
                    alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
                }
#ifdef SERIAL_DEBUG
                Serial.print("BME180 pressure: ");
                Serial.print(prs);
                Serial.println(" hPa");
                Serial.print("BME180 altiture: ");
                Serial.print(alt);
                Serial.println(" m");
#endif
                snprintf(str_buf, SCR_WIDTH + 1, "%4d mmHg %4d m", (int) (prs * 0.007501), (int) alt);
            } else
                snprintf(str_buf, SCR_WIDTH + 1, "b00St 0ff       ");
#ifdef SERIAL_DEBUG
            Serial.print("=1= ");
            Serial.println(str_buf);
#endif
            break;
        }
        lcd.setCursor(0, 0);
        lcd.write(str_buf);
        // 2nd line
        switch (scr_page) {
        case SCR_RATE:
            snprintf(str_buf, SCR_WIDTH + 1, "  %6lu", rate_max);
            break;                     // peak rate
        case SCR_DOSE:
            snprintf(str_buf, SCR_WIDTH + 1, "%02u:%02u:%02u", t_hrs, t_min, t_sec);
            break;
        case SCR_SENSORS:
            if ((sensors & SENS_BME280)) {
                if (!keep) {
#ifdef SERIAL_DEBUG
                    Serial.println("running AHT10..");
#endif
                    hum = bme.readHumidity();
                    temp = bme.readTemperature();
                }
                dtostrf(hum, 5, 1, str_buf);
                strcat(str_buf, "%rH");
                dtostrf(temp, 6, 1, str_buf + 8);
                strcat(str_buf, "\xdf");
                strcat(str_buf, "C");
#ifdef SERIAL_DEBUG
                Serial.print("AHT humidity: ");
                Serial.print(hum);
                Serial.println(" %%rH");
                Serial.print("AHT temp: ");
                Serial.print(temp);
                Serial.println(" C");
#endif
            } else
                snprintf(str_buf, SCR_WIDTH + 1, "no sensors");
#ifdef SERIAL_DEBUG
            Serial.print("=2= ");
            Serial.println(str_buf);
#endif
            break;
        }
        if (sensors && ++keep == SENS_TIME)
            keep = 0;
        if (scr_page != SCR_SENSORS) {
            lcd.setCursor(8, 1);
            lcd.write(str_buf);
            // alarm on/off sign
            memset(str_buf, ' ', 8);
            if (alarm_disable) {       // if alarm prohibited
                if (alarm_wait)        // white while rate lowered
                    str_buf[0] = str_buf[1] = '\x03';   // "awaiting" sign - "..."
                else
                    str_buf[0] = str_buf[1] = ' ';
            } else {
                str_buf[0] = '\x00';   // "alarm on" sign
                str_buf[1] = '\x01';   // "alarm on" sign
            }
            if (buzz_disable)
                str_buf[3] = str_buf[4] = ' ';
            else {
                str_buf[3] = '\x04';   // "buzzer on" sign
                str_buf[4] = '\x05';   // "buzzer on" sign
            }
            lcd.setCursor(0, 1);
            lcd.write(str_buf, 8);
        } else {
            lcd.setCursor(0, 1);
            lcd.write(str_buf);
        }
        // update EEPROM every 5 sec
        if (!--eeup) {
            eeup = 5;
            eeprom_update();
        }
    }
    // handle keypress
    switch (check_keys()) {
    case KEY_UP:
        if (++scr_page > SCR_SENSORS) {
            scr_page = SCR_RATE;
            // little boost up
            boost_pulses = BOOST_FREQ; // 1 sec boost
        }
        redraw = 1;
        break;
    case KEY_DOWN:
        switch (scr_page) {
        case SCR_RATE:
            // reset rate and max on DOWN
            for (i = 0; i < GEIGER_TIME; i++)
                geiger[i] = 0;
            rate = 0;
            rate_max = 0;
            break;
        case SCR_DOSE:
            // reset dose and time on DOWN
            total = 0;
            dose = 0;
            t_hrs = t_min = t_sec = 0;
            break;
        }
        redraw = 1;
        break;
    case KEY_LEFT:                    // left key
        if (alarm) {
            if (alarm_disable == 1)
                alarm_wait = !alarm_wait;
        } else
            alarm_disable = !alarm_disable;
        redraw = 1;
        break;
    case KEY_RIGHT:
        buzz_disable = !buzz_disable;
        redraw = 1;
        break;
    case KEY_SELECT:
        menu();
        break;
    }
    delay(10);
}
