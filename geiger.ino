// Arduino Geiger counter 01.3
// Arduino 1.0.5
// ATmega328P 16MHz
// mod by venus@trg.ru

#include "LiquidCrystal.h"

#include <avr/delay.h>

#define GEIGER_TIME 28                 //время измерения, для СИ29БГ 75 секунд (1..255)
// time for SBT-10A -- 2.17 / sec == 15 pulses for 6.9s ~~ 7s, so use TIME=28, DIV=4
#define GEIGER_DIV  4                  // divisor for measured values

//Timer1 25Hz для секунд
#define T1_FREQ     25                 // timer1 frequency (meta value) for main loop
#define T2_FREQ 100000L                // timer2 (main) frequency

#define BOOST_FREQ      10             // boost converter 10us pulses frequency
#define BOOST_PUMP_FREQ 1000           // initial boost frequency

#define NUM_KEYS 5                     //количество кнопок

#define BOOST_DDR  DDRD
#define BOOST_PORT PORTD
#define BOOST      PD3

#define IN_DDR  DDRD
#define IN_PORT PORTD
#define IN      PD2

#define BUZZ_DDR  DDRC
#define BUZZ_PORT PORTC
#define BUZZ      PC1

#define BEEP_FREQ 2000

volatile uint8_t beep_gen = 0;         // beep now, 1 == on, 0 == off
volatile uint32_t beep_high;           // beep meander high level length in 10us units
volatile uint32_t beep_low;            // beep meander low level length in 10us units
uint32_t beep_state = 0;
uint32_t beep_running;
volatile uint8_t tick_gen = 0;         // tick now, 1 == on, no need for 0
uint8_t tick_running;

// boost low level len in 10us units, initially for 1000Hz
volatile uint32_t boost_low = (T2_FREQ / BOOST_PUMP_FREQ) - 1;
volatile uint8_t boost_state = 1;
uint32_t boost_running;

uint8_t brightness = 4;                // screen brightness 1..5

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

uint16_t rad_buff[GEIGER_TIME];        //массив секундных замеров для расчета фона
uint16_t adc_key_val[5] = { 50, 200, 400, 600, 800 };   //значения АЦП для обработки кнопок

uint32_t rad_sum;                      //сумма импульсов за все время
uint32_t rad_back;                     //текущий фон
uint32_t rad_max;                      //максимум фона
uint32_t rad_dose;                     //доза

uint8_t time_sec;                      //секунды //счетчики времени
uint8_t time_min;                      //минуты
uint8_t time_hrs;                      //часы

uint8_t scr_mode;                      //режим

uint8_t scr = 0;                       //флаг обновления экрана
uint8_t alarm = 0;                     //флаг тревоги
uint8_t alarm_disable = 0;             //флаг запрета тревоги
uint8_t alarm_wait = 0;                //флаг ожидания выключения запрета
uint8_t buzz_disable = 0;              //флаг запрет треска пищалкой

uint8_t timer = 0;                     //for delay
uint8_t timer_out = 0;                 //flag

uint8_t buzz_vol = 5;                  //громкость треска(щелчков)  (1-5)
uint8_t beep_vol = 5;                  //громкость тревоги  (1-5)
uint8_t alarm_level = 50;              //уровень тревоги uR/h  (40..250 с шагом 10)

char str_buff[18];

// *INDENT-OFF*

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

uint8_t  s6[8] = {
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000
};

uint8_t  s7[8] = {
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000
};

// *INDENT-ON*

void calc_beep()
{
    beep_high = beep_vol;
    beep_low = (T2_FREQ / BEEP_FREQ) - beep_high;
}

void set_brightness()
{
    analogWrite(10, brightness * 50);  // maybe +5
}

//-------------------------------------------------------------------------------------------------
void setup(void)                       //инициализация
{

    bitSet(BOOST_DDR, BOOST);          // boost converter port - out
    bitClear(BOOST_PORT, BOOST);       // disable boost

    bitSet(BUZZ_DDR, BUZZ);            // buzzer port - out
    bitClear(BUZZ_PORT, BUZZ);

    bitClear(IN_DDR, IN);              // input port - in
    bitSet(IN_PORT, IN);               // w/ pullup

    set_brightness();

    // timer2 - 100kHz
    TCNT2 = 0;
    TCCR2A = (1 << WGM21);             // CTC mode
    TCCR2B = (0 << CS22) | (0 << CS21) | (1 << CS20);   // no divisor - full cpu speed
    OCR2A = F_CPU / T2_FREQ;           // 100kHz
    TIMSK2 |= (1 << OCIE2A);           // enable t2 for boost

    lcd.begin(16, 2);
    lcd.print("Geiger Counter");
    lcd.setCursor(0, 1);
    lcd.print("Wait a moment");

    calc_beep();
    _delay_ms(100);                    // wait for boost
    boost_low = (T2_FREQ / BOOST_FREQ) - 1;     // lower boost freq downto BOOST_FREQ

    lcd.clear();

    lcd.createChar(0, s0);             //загружаем символы в дисплей
    lcd.createChar(1, s1);
    lcd.createChar(2, s2);
    lcd.createChar(3, s3);
    lcd.createChar(4, s4);
    lcd.createChar(5, s5);
    lcd.createChar(6, s6);
    lcd.createChar(7, s7);

    // irq
    EICRA = (1 << ISC01) | (0 << ISC01);        // irq0 - falling edge
    EIMSK = (0 << INT1) | (1 << INT0); //enable irq0

}

//-------------------------------------------------------------------------------------------------
ISR(INT0_vect)                         //внешнее прерывание на пине INT0 - считаем импульсы от счетчика
{
    if (rad_buff[0] != 65535)
        rad_buff[0]++;                 //нулевой элемент массива - текущий секундный замер
    if (++rad_sum > 999999UL * 3600 * GEIGER_DIV / GEIGER_TIME)
        rad_sum = 999999UL * 3600 * GEIGER_DIV / GEIGER_TIME;   //сумма импульсов

    boost_state = 1;                   // force boost pulse

    if (!buzz_disable)
        tick_gen = 1;
}

// 100 kHz timer
ISR(TIMER2_COMPA_vect)
{

    switch (boost_state) {
    case 1:
        bitSet(BOOST_PORT, BOOST);
        boost_state++;
        break;
    case 2:
        bitClear(BOOST_PORT, BOOST);
        boost_state = 0;
        boost_running = boost_low;
        break;
    default:
        if (!--boost_running)
            boost_state = 1;
        break;
    }

    if (beep_gen) {
        switch (beep_state) {
        case 0:
            beep_state++;
        case 1:
            bitSet(BUZZ_PORT, BUZZ);
            beep_running = beep_high;
            beep_state++;
            break;
        case 2:
            if (!--beep_running) {
                bitClear(BUZZ_PORT, BUZZ);
                beep_running = beep_low;
                beep_state++;
            }
            break;
        default:
            if (!--beep_running)
                beep_state = 1;
            break;
        }
    } else if (beep_state) {
        bitClear(BUZZ_PORT, BUZZ);
        beep_state = 0;
    }

    if (tick_gen) {
        bitSet(BUZZ_PORT, BUZZ);
        tick_running = buzz_vol;
        tick_gen = 0;
    } else if (tick_running && !--tick_running)
        bitClear(BUZZ_PORT, BUZZ);

    static uint16_t cnt0 = 0;
    static uint8_t cnt1 = 0;

    // main cycle -- measurements

    if (++cnt0 >= T2_FREQ / T1_FREQ) {
        cnt0 = 0;

        // 1/25 sec block
        if (++cnt1 >= T1_FREQ)         //расчет показаний один раз в секунду
        {
            cnt1 = 0;

            uint32_t tmp_buff = 0;
            for (uint8_t i = 0; i < GEIGER_TIME; i++)
                tmp_buff += rad_buff[i];        //расчет фона мкР/ч

            tmp_buff /= GEIGER_DIV;

            if (tmp_buff > 999999)
                tmp_buff = 999999;     //переполнение

            rad_back = tmp_buff;

            if (rad_back > rad_max)
                rad_max = rad_back;    //фиксируем максимум фона

            if (rad_back >= alarm_level)
                alarm = 1;             //превышение фона
            else {
                alarm = 0;
                if (alarm_wait)
                    alarm_disable = 0;
                alarm_wait = 0;
            }

            for (uint8_t k = GEIGER_TIME - 1; k > 0; k--)
                rad_buff[k] = rad_buff[k - 1];  //перезапись массива
            rad_buff[0] = 0;           //сбрасываем счетчик импульсов

            rad_dose = (rad_sum * GEIGER_TIME / GEIGER_DIV / 3600);     //расчитаем дозу

            if (time_hrs < 99)         //если таймер не переполнен
            {
                if (++time_sec > 59)   //считаем секунды
                {
                    if (++time_min > 59)        //считаем минуты
                    {
                        if (++time_hrs > 99)
                            time_hrs = 99;      //часы
                        time_min = 0;
                    }
                    time_sec = 0;
                }
            }

            scr = 0;
        }

        if (timer)                     //таймер для разного
        {
            if (--timer == 0)
                timer_out = 1;
        }
    }
}

//-------------------------------------------------------------------------------------------------
uint8_t get_key(void)                  //получить номер нажатой кнопки из данных АЦП
{
    uint8_t key = 0;
    uint16_t adc_result = analogRead(0);

    for (uint8_t i = 0; i < NUM_KEYS; i++) {
        if (adc_result < adc_key_val[i]) {
            key = i + 1;
            break;
        }
    }
    return key;
}


//-------------------------------------------------------------------------------------------------
uint8_t check_keys(void)               //проверить клавиатуру
{
    uint8_t k = 0;
    static uint8_t old_key;

    uint8_t new_key = get_key();       //обновить состояние
    if (new_key != old_key)            //если состояние не равно старому - была нажата копка
    {
        _delay_ms(5);                  //защита от дребезга
        new_key = get_key();
        if (new_key != old_key) {
            old_key = new_key;
            k = new_key;
        }
    }
    return k;                          //вернем номер кнопки 1..5, 0-кнопка не нажата
}

//-------------------------------------------------------------------------------------------------
void alarm_warning(void)               //выводим предупреждение
{
    uint8_t n = 0;
    uint32_t rad_alrm = 0;

    uint8_t bd = buzz_disable;         //запомнить настройку звуковой индикации импульсов

    buzz_disable = 1;                  //запретить звуковую индикацию импульсов

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("WARNING");
    scr == 0;

    while (1) {
        if (scr == 0) {
            scr = 1;
            if (rad_back > rad_alrm)
                rad_alrm = rad_back;   //максимум
            sprintf(str_buff, "%6lu uR/h", rad_alrm);
            lcd.setCursor(5, 0);
            lcd.print(str_buff);
        }

        //==================================================================
        if (n == 0)                    //начало прерывистого звукового сигнала
        {
            n = 1;
            timer = 9;                 //длительность сигнала x40ms
            timer_out = 0;
            beep_gen = 1;
        }

        if (n == 1 && timer_out == 1)  //начало паузы между сигналами
        {
            n = 2;
            timer = 4;                 //длительность паузы x40ms
            timer_out = 0;
            beep_gen = 0;
        }

        if (n == 2 && timer_out == 1)
            n = 0;                     //запуск следующего цикла
        //==================================================================

        if (check_keys() == 4)         //если нажата кнопка left отключаем тревогу
        {
            beep_gen = 0;
            lcd.setCursor(0, 1);
            lcd.print("ALARM DISABLE");
            n = 0;
            timer = 35;                //длительность сообщения x40ms
            timer_out = 0;
            while (timer_out == 0)
                if (check_keys() == 4)
                    break;
            lcd.clear();
            buzz_disable = bd;
            alarm_disable = 1;
            scr_mode = 0;
            scr = 0;
            return;
        }
    }
}

//-------------------------------------------------------------------------------------------------
void menu(void)
{
    uint8_t n = 0;

    lcd.clear();
    scr = 0;

    while (1) {
        if (alarm && alarm_disable == 0)
            alarm_warning();

        if (scr == 0)                  //+++++++++++++++++++   вывод информации на экран  +++++++++++++++++++++++++
        {
            scr = 1;

            lcd.setCursor(0, 1);
            lcd.write(byte(2));
            lcd.setCursor(2, 1);
            sprintf(str_buff, "%01u", n + 1);
            lcd.print(str_buff);

            switch (n) {
            case 0:
                sprintf(str_buff, "BUZZ VOLUME   %2u", buzz_vol);
                break;
            case 1:
                sprintf(str_buff, "ALARM VOLUME  %2u", beep_vol);
                break;
            case 2:
                sprintf(str_buff, "ALARM LEVEL  %3u", alarm_level);
                break;
            case 3:
                sprintf(str_buff, "BRIGHTNESS    %2u", brightness);
                break;
            }
            lcd.setCursor(0, 0);
            lcd.print(str_buff);
        }

        switch (check_keys())          //+++++++++++++++++++++  опрос кнопок  +++++++++++++++++++++++++++
        {
        case 1:                       //right key
            scr = 0;
            break;
        case 2:                       //up key
            switch (n) {
            case 0:
                if (buzz_vol < 5)
                    buzz_vol++;
                break;
            case 1:
                if (beep_vol < 5) {
                    beep_vol++;
                    calc_beep();
                }
                break;
            case 2:
                if (alarm_level < 250)
                    alarm_level += 10;
                break;
            case 3:
                if (brightness < 5)
                    brightness++;
                set_brightness();
                break;
            }
            scr = 0;
            break;
        case 3:                       //down key
            switch (n) {
            case 0:
                if (buzz_vol > 1)
                    buzz_vol--;
                break;
            case 1:
                if (beep_vol > 1) {
                    beep_vol--;
                    calc_beep();
                }
                break;
            case 2:
                if (alarm_level > 40)
                    alarm_level -= 10;
                break;
            case 3:
                if (brightness > 1)
                    brightness--;
                set_brightness();
                break;
            }
            scr = 0;
            break;
        case 4:                       //left key
            scr = 0;
            break;
        case 5:                       //select key
            if (++n > 3) {
                n = 0;
                lcd.clear();
                scr = 0;
                return;
            }
            scr = 0;
        default:
            break;
        }
    }
}

//-------------------------------------------------------------------------------------------------
void loop(void)                        //главная
{

    if (alarm && alarm_disable == 0)
        alarm_warning();

    if (scr == 0)                      //+++++++++++++++++++   вывод информации на экран  +++++++++++++++++++++++++
    {
        scr = 1;                       //сброс флага

        switch (scr_mode) {
        case 0:
            sprintf(str_buff, "Rate %6lu uR/h", rad_back);
            break;                     //dose rate, uR/h
        case 1:
            sprintf(str_buff, "Dose   %6lu uR", rad_dose);
            break;                     //dose, uR
        }
        lcd.setCursor(0, 0);
        lcd.print(str_buff);

        switch (scr_mode) {
        case 0:
            sprintf(str_buff, "  %6lu", rad_max);
            break;                     //peak rate
        case 1:
            sprintf(str_buff, "%02u:%02u:%02u", time_hrs, time_min, time_sec);
            break;
        }
        lcd.setCursor(8, 1);
        lcd.print(str_buff);

        lcd.setCursor(0, 1);
        if (alarm_disable)             //если тревога запрещена
        {
            if (alarm_wait)            //если ждем понижения фона
            {
                lcd.write(byte(3));    //значок "ожидание"
                lcd.write(byte(3));
            } else
                lcd.print("  ");
        } else {
            lcd.write(byte(0));        //значок "вкл. тревожная сигнализация"
            lcd.write(byte(1));
        }

        lcd.setCursor(3, 1);
        if (buzz_disable)
            lcd.print("  ");
        else {
            lcd.write(byte(4));        //значок "вкл. звуковая индикация импульсов"
            lcd.write(byte(5));
        }
    }

    switch (check_keys())              //+++++++++++++++++++++  опрос кнопок  +++++++++++++++++++++++++++
    {
    case 1:                           //right key
        buzz_disable = !buzz_disable;
        scr = 0;
        break;
    case 2:                           //up key //выбор режима
        if (++scr_mode > 1)
            scr_mode = 0;
        scr = 0;
        break;
    case 3:                           //down key //сброс
        switch (scr_mode) {
        case 0:                       //сбрасываем фон и макс. фон
            for (uint8_t i = 0; i < GEIGER_TIME; i++)
                rad_buff[i] = 0;
            rad_back = 0;
            rad_max = 0;
            break;
        case 1:
            rad_sum = 0;
            rad_dose = 0;
            time_hrs = time_min = time_sec = 0;
            break;
        }
        scr = 0;
        break;
    case 4:                           //left key
        if (alarm) {
            if (alarm_disable == 1)
                alarm_wait = !alarm_wait;
        } else
            alarm_disable = !alarm_disable;
        scr = 0;
        break;
    case 5:                           //select key
        menu();
        break;
    }
}
