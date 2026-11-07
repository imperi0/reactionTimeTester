/*****************************************************************************
 * PROBLEM STATEMENT
 * Multi Mode Touchless Reaction Time Tester on the LPC1768 (ALS-SDA-ARMCTXM3-01).
 * The player reacts to a green / red cue LED by moving a hand in front of an
 * IR proximity sensor. Time is measured with a 1 ms hardware timer and shown
 * on the 16x2 LCD; a 4x4 keypad selects the mode.
 *   Mode 1 : basic reaction (green cue)
 *   Mode 2 : Go / No-Go (random green or red cue). After a successful No-Go
 *            (red held out, then red goes off) a new random cue starts by
 *            itself, with no key press, until a green reaction is timed.
 *   Mode 3 : three valid rounds, rounded average
 *
 * TARGET : Keil uVision4, 12 MHz crystal
 *
 * ========================= CONNECTIONS =========================
 * 1) LCD  (FRC cable CND -> CNAD)
 *      D4-D7 = P0.23-P0.26,  RS = P0.27,  EN = P0.28
 *      Jumpers JP16 and JP5 shorted; POT3 adjusts contrast.
 *
 * 2) KEYPAD (FRC cable CNC -> CNB3)
 *      CNC pins 1-4 = P0.15-P0.18 -> keypad COLUMNS 1-4 (inputs)
 *      CNC pins 5-8 = P0.19-P0.22 -> keypad ROWS    1-4 (outputs)
 *      Jumper JP4 (1,2) shorted.
 *
 * 3) EXTERNAL PARTS on the CNA header (NO cable plugged into CNA; push
 *    female jumper wires directly onto the CNA header pins)
 *      CNA pin 1  = P0.4  -> 470 ohm resistor -> GREEN LED anode (long leg)
 *      CNA pin 2  = P0.5  -> 470 ohm resistor -> RED   LED anode (long leg)
 *      CNA pin 3  = P0.6  <- IR sensor DOUT
 *      CNA pin 10 = GND   -> both LED cathodes (short leg) and IR sensor GND
 *      IR sensor VCC -> 3.3 V (preferred) or +5 V from the board supply
 *      IR sensor AOUT -> not connected
 *      (a red LED and a green LED each need their own resistor)
 *
 * The on-board LED block, SW2 and the 7-segment are NOT used in this build
 * (do not connect a cable to CNA or CNB).
 *
 * TIMERS
 *   Timer0 : free-running, 1 ms tick   (millis)
 *   Timer2 : free-running, 1 us tick   (LCD / keypad delays)
 *
 * KEYS (key code = row*4 + column, 0..15; edit the #defines if your labels differ)
 *   1,2,3 = select mode   9 = Start/Retry   B = Menu   0 = IR hardware test
 *****************************************************************************/
#include <LPC17xx.h>
#include <stdio.h>

/* ---------------- pin masks ---------------- */
#define GREEN_BIT  4                 /* GPIO0 P0.4 : green cue LED */
#define RED_BIT    5                 /* GPIO0 P0.5 : red   cue LED */
#define IR_BIT     6                 /* GPIO0 P0.6 : IR sensor DOUT */
#define GREEN_M    (1u << GREEN_BIT)
#define RED_M      (1u << RED_BIT)
#define COL_MASK   0x00078000u       /* GPIO0 P0.15-P0.18 : keypad columns */
#define ROW_MASK   0x00780000u       /* GPIO0 P0.19-P0.22 : keypad rows    */
#define LCD_D      0x07800000u       /* GPIO0 P0.23-P0.26 */
#define LCD_RS     0x08000000u       /* GPIO0 P0.27       */
#define LCD_EN     0x10000000u       /* GPIO0 P0.28       */

/* ---------------- keys ---------------- */
#define NOKEY       0xFF
#define KEY_TEST    0
#define KEY_MODE1   1
#define KEY_MODE2   2
#define KEY_MODE3   3
#define KEY_START   9
#define KEY_MENU    11               /* 'B' */

/* ---------------- timing (ms) ---------------- */
#define WAIT_MIN_MS    2000u
#define WAIT_SPAN_MS   3001u         /* random wait 2000..5000 ms */
#define GREEN_LIMIT_MS 5000u
#define NOGO_HOLD_MS   2000u

/* trial results */
#define R_OK          0
#define R_FALSE_START 1
#define R_TIMEOUT     2
#define R_WRONG       3              /* hit on RED cue */
#define R_NOGO_OK     4              /* correctly held on RED */

static const char key_char[16] = "0123456789ABCDEF";
static unsigned int best[4] = {0, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
static unsigned int rnd_seed = 1;
static unsigned char ir_idle = 1;    /* DOUT level with NO hand (auto-detected) */

/* ======================== timers & delays ======================== */
static void timers_init(void)
{
    unsigned int pclk;
    LPC_SC->PCONP |= (1u<<1) | (1u<<22);                /* TIM0, TIM2 on */
    LPC_SC->PCLKSEL0 &= ~(3u<<2);                       /* TIM0 = CCLK/4 */
    LPC_SC->PCLKSEL1 &= ~(3u<<12);                      /* TIM2 = CCLK/4 */
    pclk = SystemCoreClock / 4;

    LPC_TIM0->PR  = pclk/1000 - 1;                      /* 1 ms tick */
    LPC_TIM0->TCR = 2;  LPC_TIM0->TCR = 1;

    LPC_TIM2->PR  = pclk/1000000 - 1;                   /* 1 us tick */
    LPC_TIM2->TCR = 2;  LPC_TIM2->TCR = 1;
}

static unsigned int millis(void) { return LPC_TIM0->TC; }

static void delay_us(unsigned int us)
{
    unsigned int s = LPC_TIM2->TC;
    while ((LPC_TIM2->TC - s) < us);
}

static void delay_ms(unsigned int ms)
{
    unsigned int s = LPC_TIM0->TC;
    while ((LPC_TIM0->TC - s) < ms);
}

static unsigned int rnd(void)
{
    rnd_seed = rnd_seed * 1103515245u + 12345u;
    return (rnd_seed >> 16) & 0x7FFF;
}

/* ======================== cue LEDs (external, active high) ======================== */
static void leds_off(void)  { LPC_GPIO0->FIOCLR = GREEN_M | RED_M; }
static void led_green(void) { LPC_GPIO0->FIOCLR = RED_M;   LPC_GPIO0->FIOSET = GREEN_M; }
static void led_red(void)   { LPC_GPIO0->FIOCLR = GREEN_M; LPC_GPIO0->FIOSET = RED_M; }

static void leds_flash(unsigned int mask, unsigned char n)
{
    unsigned char i;
    for (i = 0; i < n; i++) {
        leds_off();  delay_ms(150);
        LPC_GPIO0->FIOSET = mask;  delay_ms(150);
    }
    leds_off();
}

/* ======================== LCD (4-bit, write only) ======================== */
static void lcd_nibble(unsigned char nib, unsigned char rs)
{
    LPC_GPIO0->FIOCLR = LCD_D | LCD_RS | LCD_EN;
    LPC_GPIO0->FIOSET = ((unsigned int)(nib & 0x0F)) << 23;
    if (rs) LPC_GPIO0->FIOSET = LCD_RS;
    delay_us(2);
    LPC_GPIO0->FIOSET = LCD_EN;
    delay_us(2);
    LPC_GPIO0->FIOCLR = LCD_EN;
    delay_us(2);
}

static void lcd_byte(unsigned char b, unsigned char rs)
{
    lcd_nibble(b >> 4, rs);
    lcd_nibble(b & 0x0F, rs);
    delay_us(50);
}

static void lcd_cmd(unsigned char c)
{
    lcd_byte(c, 0);
    if (c <= 0x03) delay_us(2000);                  /* clear / home */
}

static void lcd_init(void)
{
    delay_ms(20);
    lcd_nibble(0x3, 0); delay_ms(5);
    lcd_nibble(0x3, 0); delay_us(200);
    lcd_nibble(0x3, 0); delay_us(200);
    lcd_nibble(0x2, 0); delay_us(200);              /* 4-bit mode */
    lcd_cmd(0x28);                                  /* 4-bit, 2 lines, 5x8 */
    lcd_cmd(0x0C);                                  /* display on, cursor off */
    lcd_cmd(0x06);                                  /* entry mode: increment */
    lcd_cmd(0x01);                                  /* clear */
}

static void lcd_line(unsigned char row, const char *s)   /* pads to 16 chars */
{
    unsigned char i = 0;
    lcd_cmd(row ? 0xC0 : 0x80);
    while (i < 16) {
        lcd_byte(*s ? (unsigned char)*s++ : ' ', 1);
        i++;
    }
}

/* ======================== keypad ======================== */
static unsigned char scan_raw(void)                 /* any key; NOKEY if none */
{
    unsigned char r, c;
    unsigned int cols;
    for (r = 0; r < 4; r++) {
        LPC_GPIO0->FIOCLR = ROW_MASK;
        LPC_GPIO0->FIOSET = 1u << (19 + r);
        delay_us(20);
        cols = (LPC_GPIO0->FIOPIN >> 15) & 0x0F;
        if (cols) {
            LPC_GPIO0->FIOCLR = ROW_MASK;
            for (c = 0; c < 4; c++)
                if (cols & (1u << c)) return (unsigned char)(r * 4 + c);
        }
    }
    LPC_GPIO0->FIOCLR = ROW_MASK;
    return NOKEY;
}

static unsigned char key_poll(void)                 /* debounced, once per press */
{
    static unsigned char last = NOKEY;
    unsigned char k = scan_raw();
    if (k != NOKEY && last == NOKEY) {
        delay_ms(20);
        if (scan_raw() == k) { last = k; return k; }
    } else if (k == NOKEY && last != NOKEY) {
        delay_ms(20);
        if (scan_raw() == NOKEY) last = NOKEY;
    }
    return NOKEY;
}

/* ======================== IR sensor (P0.6) ======================== */
static unsigned char sensor_raw(void)
{
    return (unsigned char)(((LPC_GPIO0->FIOPIN >> IR_BIT) & 1u) != ir_idle);
}

/* hand detected only if 3 samples ~30 us apart agree (rejects glitches) */
static unsigned char sensor_detected(void)
{
    if (!sensor_raw()) return 0;
    delay_us(30);
    if (!sensor_raw()) return 0;
    delay_us(30);
    return sensor_raw();
}

/* ======================== reaction logic ======================== */
/* green = 1 : GREEN cue, react to it.   green = 0 : RED cue, do NOT react.
   Timing starts at the moment the cue LED comes on. */
static unsigned char run_trial(unsigned char green, unsigned int *ms)
{
    unsigned int wait = WAIT_MIN_MS + rnd() % WAIT_SPAN_MS;
    unsigned int t = millis();

    while ((millis() - t) < wait)                   /* random wait, LEDs off */
        if (sensor_detected()) return R_FALSE_START;

    if (green) led_green(); else led_red();         /* cue ON ...            */
    t = millis();                                   /* ... timing starts now */

    if (green) {
        while ((millis() - t) < GREEN_LIMIT_MS)
            if (sensor_detected()) { *ms = millis() - t; leds_off(); return R_OK; }
        leds_off();
        return R_TIMEOUT;
    } else {
        while ((millis() - t) < NOGO_HOLD_MS)
            if (sensor_detected()) { leds_off(); return R_WRONG; }
        leds_off();                                 /* red goes off */
        return R_NOGO_OK;
    }
}

static void show_best(unsigned char mode)
{
    char buf[20];
    if (best[mode] == 0xFFFFFFFFu) sprintf(buf, "Best: ---");
    else sprintf(buf, "Best: %u ms", best[mode]);
    lcd_line(1, buf);
}

/* shows the message for a failed trial; returns 1 if the trial was invalid */
static unsigned char report_invalid(unsigned char res)
{
    if (res == R_FALSE_START) lcd_line(1, "FALSE START!");
    else if (res == R_TIMEOUT) lcd_line(1, "TIMEOUT");
    else if (res == R_WRONG)   lcd_line(1, "WRONG: HIT RED");
    else return 0;
    leds_flash(GREEN_M | RED_M, 3);
    return 1;
}

static void run_mode(unsigned char mode)
{
    unsigned char res, round;
    unsigned int ms, sum;
    char buf[20];

    if (sensor_detected()) {                        /* round starts only if clear */
        lcd_line(1, "Hand away first");
        delay_ms(1200);
        return;
    }
    rnd_seed ^= (LPC_TIM2->TC << 4) ^ millis();     /* seed from timing of press */
    sprintf(buf, "Mode %u: ready", mode);
    lcd_line(0, buf);
    lcd_line(1, mode == 2 ? "GRN=go RED=stop" : "Wait for LED...");

    if (mode == 1) {
        res = run_trial(1, &ms);
        if (res == R_OK) {
            sprintf(buf, "Time: %u ms", ms); lcd_line(0, buf);
            if (ms < best[1]) { best[1] = ms; lcd_line(1, "NEW BEST!"); }
            else show_best(1);
            leds_flash(GREEN_M, 2);
        } else report_invalid(res);
    }
    else if (mode == 2) {                           /* Go / No-Go, auto-repeats */
        unsigned char green;
        do {
            green = rnd() & 1;                      /* random green / red cue */
            res = run_trial(green, &ms);
            if (res == R_NOGO_OK) {                 /* red held out: go again */
                lcd_line(0, "No-Go OK!");
                lcd_line(1, "Next cue coming");
                delay_ms(1000);
                lcd_line(0, "Mode 2: ready");
                lcd_line(1, "GRN=go RED=stop");
            }
        } while (res == R_NOGO_OK);                 /* no key press needed */

        if (res == R_OK) {
            sprintf(buf, "Time: %u ms", ms); lcd_line(0, buf);
            if (ms < best[2]) { best[2] = ms; lcd_line(1, "NEW BEST!"); }
            else show_best(2);
            leds_flash(GREEN_M, 2);
        } else report_invalid(res);
    }
    else {                                          /* mode 3: three rounds */
        sum = 0;
        for (round = 1; round <= 3; round++) {
            sprintf(buf, "Round %u/3", round);
            lcd_line(0, buf);
            do {
                if (sensor_detected()) { lcd_line(1, "Hand away first"); delay_ms(1000); }
                lcd_line(1, "Wait for LED...");
                res = run_trial(1, &ms);
                if (report_invalid(res)) {          /* invalid -> retry round */
                    delay_ms(1200);
                    lcd_line(1, "Retry round...");
                    delay_ms(800);
                }
            } while (res != R_OK);
            sprintf(buf, "Round %u: %u ms", round, ms);
            lcd_line(1, buf);
            sum += ms;
            delay_ms(1500);
        }
        ms = (sum + 1) / 3;                         /* rounded average */
        sprintf(buf, "Avg: %u ms", ms);
        lcd_line(0, buf);
        if (ms < best[3]) { best[3] = ms; lcd_line(1, "NEW BEST AVG!"); }
        else show_best(3);
        leds_flash(GREEN_M, 2);
    }
}

/* hardware test: live IR state on LCD, green LED = hand, red LED = clear */
static void hw_test(void)
{
    unsigned char on;
    lcd_line(0, "IR test: any key");
    while (key_poll() == NOKEY) {
        on = sensor_detected();
        lcd_line(1, on ? "IR: DETECTED" : "IR: clear");
        if (on) led_green(); else led_red();
        delay_ms(100);
    }
    leds_off();
}

/* ======================== main ======================== */
int main(void)
{
    unsigned char k, mode = 0, pending = NOKEY;     /* mode 0 = at menu */
    char buf[20];

    SystemInit();
    SystemCoreClockUpdate();

    /* all used pins as GPIO */
    LPC_PINCON->PINSEL0 &= ~(0x00FFFF00u | 0xC0000000u);  /* P0.4-11, P0.15  */
    LPC_PINCON->PINSEL1 &= ~0x03FFFFFFu;                  /* P0.16-P0.28     */
    /* keypad columns: pull-down so they read low when no key is pressed */
    LPC_PINCON->PINMODE0 |= 0xC0000000u;                  /* P0.15 */
    LPC_PINCON->PINMODE1 |= 0x0000003Fu;                  /* P0.16-P0.18 */

    LPC_GPIO0->FIODIR |= GREEN_M | RED_M | ROW_MASK | LCD_D | LCD_RS | LCD_EN;
    LPC_GPIO0->FIODIR &= ~(COL_MASK | (1u << IR_BIT));    /* inputs: cols, IR */
    leds_off();

    timers_init();
    lcd_init();

    /* power-on self test: both LEDs light in turn, then IR idle calibration */
    lcd_line(0, "Reaction Tester");
    lcd_line(1, "Hand away...");
    led_green(); delay_ms(500);
    led_red();   delay_ms(500);
    leds_off();
    delay_ms(500);
    ir_idle = (unsigned char)((LPC_GPIO0->FIOPIN >> IR_BIT) & 1u);

    while (1) {
        if (mode == 0) {                            /* ---- MENU ---- */
            lcd_line(0, "Select mode 1-3");
            lcd_line(1, "0=IR test");
            while (mode == 0) {
                k = key_poll();
                if (k == KEY_MODE1 || k == KEY_MODE2 || k == KEY_MODE3) mode = k;
                else if (k == KEY_TEST) {
                    hw_test();
                    lcd_line(0, "Select mode 1-3");
                    lcd_line(1, "0=IR test");
                }
                else if (k != NOKEY) {              /* keypad test */
                    sprintf(buf, "Key code: %c", key_char[k]);
                    lcd_line(1, buf);
                }
            }
        }
        if (pending == NOKEY) {                     /* ---- READY ---- */
            sprintf(buf, "Mode %u  9=Start", mode);
            lcd_line(0, buf);
            show_best(mode);
            k = NOKEY;
            while (k == NOKEY) k = key_poll();
        } else { k = pending; pending = NOKEY; }

        if (k == KEY_START) {                       /* ---- RUN ---- */
            run_mode(mode);                         /* result stays on LCD */
            while (pending == NOKEY) pending = key_poll();  /* next key decides */
        }
        else if (k == KEY_MENU) mode = 0;
        else if (k == KEY_MODE1 || k == KEY_MODE2 || k == KEY_MODE3) mode = k;
        else {
            sprintf(buf, "Key code: %c", key_char[k]);
            lcd_line(1, buf);
            delay_ms(800);
        }
    }
}