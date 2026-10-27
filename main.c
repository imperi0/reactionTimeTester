/*****************************************************************************
 * PROBLEM STATEMENT
 * Bench-test build of the "Multi Mode Touchless Reaction Time Tester".
 * No external LED and no IR sensor are used yet:
 *   - one KEYPAD KEY (SENSOR_KEY) stands in for the IR sensor "hand detected"
 *   - the LCD text "GREEN"/"RED" stands in for the green/red cue LEDs
 * Verifies: matrix keypad, 16x2 LCD, 4-digit multiplexed 7-segment,
 * 1 ms hardware timer, and the three test modes + best-score logic.
 *
 * TARGET : ALS-SDA-ARMCTXM3-01 (LPC1768), Keil uVision4, 12 MHz crystal
 *
 * PIN MAP
 *   7-seg data  a..g,dp : P0.4  - P0.11  (CNA  -> CNA2)
 *   7-seg digit select  : P1.23 - P1.26  (CNB  -> CNB2)  4-bit code 0..3 into
 *                         74LS145; code 0xF = all digits off
 *   Keypad columns (in) : P0.15 - P0.18  (CNC pins 1-4 -> CNB3 pins 1-4)
 *   Keypad rows   (out) : P0.19 - P0.22  (CNC pins 5-8 -> CNB3 pins 5-8)
 *   LCD D4-D7           : P0.23 - P0.26  (CND  -> CNAD)
 *   LCD RS / EN         : P0.27 / P0.28
 *
 * TIMERS
 *   Timer0 : free-running, 1 ms tick   (millis)
 *   Timer1 : MR0 interrupt every 2 ms  (7-seg digit refresh)
 *   Timer2 : free-running, 1 us tick   (LCD / keypad delays)
 *
 * KEYS (key code = row*4 + column, 0..15; edit the #defines if your keypad
 *       labels differ. Press any other key at the menu to see its code.)
 *   1,2,3 = select mode   A = Start/Retry   B = Menu   F = "sensor"
 *****************************************************************************/
#include <LPC17xx.h>
#include <stdio.h>

/* ---------------- pin masks ---------------- */
#define SEG_MASK   0x00000FF0u   /* GPIO0 P0.4-P0.11  */
#define COL_MASK   0x00078000u   /* GPIO0 P0.15-P0.18 */
#define ROW_MASK   0x00780000u   /* GPIO0 P0.19-P0.22 */
#define LCD_D      0x07800000u   /* GPIO0 P0.23-P0.26 */
#define LCD_RS     0x08000000u   /* GPIO0 P0.27       */
#define LCD_EN     0x10000000u   /* GPIO0 P0.28       */
#define SEL_MASK   0x07800000u   /* GPIO1 P1.23-P1.26 */

/* ---------------- keys ---------------- */
#define NOKEY       0xFF
#define KEY_MODE1   1
#define KEY_MODE2   2
#define KEY_MODE3   3
#define KEY_START   10           /* 'A' */
#define KEY_MENU    11           /* 'B' */
#define SENSOR_KEY  15           /* 'F' : stands in for the IR sensor */

/* ---------------- timing (ms) ---------------- */
#define WAIT_MIN_MS    2000u
#define WAIT_SPAN_MS   3001u     /* random wait 2000..5000 ms */
#define GREEN_LIMIT_MS 5000u
#define NOGO_HOLD_MS   2000u

/* ---------------- display codes ---------------- */
#define DASH   10                /* shows '-'     */
#define BLANK  0xFF              /* digit off     */

/* trial results */
#define R_OK          0
#define R_FALSE_START 1
#define R_TIMEOUT     2
#define R_WRONG       3          /* hit on RED cue */
#define R_NOGO_OK     4          /* correctly held on RED */

/* 0-9 and '-' ; bit order h g f e d c b a (common cathode, active high) */
static const unsigned char seg_table[11] =
    {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x40};
static const char key_char[16] = "0123456789ABCDEF";

volatile unsigned char disp_digit[4] = {BLANK, BLANK, BLANK, BLANK};
/* disp_digit[0] = units (selected by code 0 -> U8), [3] = thousands (U11).
   If your board shows the number mirrored, swap the order in the ISR. */

static unsigned int best[4] = {0, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
static unsigned int rnd_seed = 1;

/* ======================== timers & delays ======================== */
static void timers_init(void)
{
    unsigned int pclk;
    LPC_SC->PCONP |= (1u<<1) | (1u<<2) | (1u<<22);      /* TIM0,TIM1,TIM2 on */
    LPC_SC->PCLKSEL0 &= ~((3u<<2) | (3u<<4));           /* TIM0,TIM1 = CCLK/4 */
    LPC_SC->PCLKSEL1 &= ~(3u<<12);                      /* TIM2 = CCLK/4      */
    pclk = SystemCoreClock / 4;

    LPC_TIM0->PR  = pclk/1000 - 1;                      /* 1 ms tick */
    LPC_TIM0->TCR = 2;  LPC_TIM0->TCR = 1;

    LPC_TIM2->PR  = pclk/1000000 - 1;                   /* 1 us tick */
    LPC_TIM2->TCR = 2;  LPC_TIM2->TCR = 1;

    LPC_TIM1->PR  = 0;                                  /* 2 ms refresh */
    LPC_TIM1->MR0 = pclk/500 - 1;
    LPC_TIM1->MCR = 3;                                  /* int + reset on MR0 */
    LPC_TIM1->TCR = 2;  LPC_TIM1->TCR = 1;
    NVIC_EnableIRQ(TIMER1_IRQn);
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

/* ======================== 7-segment ======================== */
void TIMER1_IRQHandler(void)            /* every 2 ms: show next digit */
{
    static unsigned char d = 0;
    LPC_TIM1->IR = 1;                               /* clear MR0 flag */

    LPC_GPIO1->FIOSET = SEL_MASK;                   /* code 0xF: all off  */
    LPC_GPIO0->FIOCLR = SEG_MASK;
    if (disp_digit[d] <= DASH)
        LPC_GPIO0->FIOSET = ((unsigned int)seg_table[disp_digit[d]]) << 4;
    LPC_GPIO1->FIOCLR = SEL_MASK;
    LPC_GPIO1->FIOSET = ((unsigned int)d) << 23;    /* enable digit d     */
    d = (d + 1) & 3;
}

static void show_blank(void)
{
    unsigned char i;
    for (i = 0; i < 4; i++) disp_digit[i] = BLANK;
}

static void show_dashes(void)
{
    unsigned char i;
    for (i = 0; i < 4; i++) disp_digit[i] = DASH;
}

static void show_number(unsigned int n)             /* leading zeros blanked */
{
    unsigned char i;
    if (n > 9999) n = 9999;
    for (i = 0; i < 4; i++) {
        if (n == 0 && i > 0) disp_digit[i] = BLANK;
        else { disp_digit[i] = n % 10; n /= 10; }
    }
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

/* "IR sensor": level of ONE keypad key. Replace this body with a read of the
   sensor GPIO pin when the real sensor is wired in. */
static unsigned char sensor_detected(void)
{
    unsigned char r = SENSOR_KEY >> 2, c = SENSOR_KEY & 3, hit;
    LPC_GPIO0->FIOCLR = ROW_MASK;
    LPC_GPIO0->FIOSET = 1u << (19 + r);
    delay_us(5);
    hit = (LPC_GPIO0->FIOPIN >> (15 + c)) & 1;
    LPC_GPIO0->FIOCLR = ROW_MASK;
    return hit;
}

/* ======================== reaction logic ======================== */
static unsigned char run_trial(unsigned char green, unsigned int *ms)
{
    unsigned int wait = WAIT_MIN_MS + rnd() % WAIT_SPAN_MS;
    unsigned int t = millis();

    while ((millis() - t) < wait)                   /* random wait */
        if (sensor_detected()) return R_FALSE_START;

    if (green) lcd_line(1, "GREEN: HIT NOW!");     /* stands in for green LED */
    else       lcd_line(1, "RED: DON'T HIT!");     /* stands in for red LED   */

    t = millis();                                   /* timing starts at cue    */
    if (green) {
        while ((millis() - t) < GREEN_LIMIT_MS)
            if (sensor_detected()) { *ms = millis() - t; return R_OK; }
        return R_TIMEOUT;
    } else {
        while ((millis() - t) < NOGO_HOLD_MS)
            if (sensor_detected()) return R_WRONG;
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

/* prints the message for a failed trial; returns 1 if the trial was invalid */
static unsigned char report_invalid(unsigned char res)
{
    if (res == R_FALSE_START)      { lcd_line(1, "FALSE START!");  show_dashes(); return 1; }
    if (res == R_TIMEOUT)          { lcd_line(1, "TIMEOUT");       show_dashes(); return 1; }
    if (res == R_WRONG)            { lcd_line(1, "WRONG: HIT RED"); show_dashes(); return 1; }
    return 0;
}

static void run_mode(unsigned char mode)
{
    unsigned char res, round;
    unsigned int ms, sum;
    char buf[20];

    if (sensor_detected()) {                        /* round starts only if clear */
        lcd_line(1, "Release F first");
        delay_ms(1200);
        return;
    }
    rnd_seed ^= (LPC_TIM2->TC << 4) ^ millis();     /* seed from timing of press */
    show_blank();
    sprintf(buf, "Mode %u: wait cue", mode);
    lcd_line(0, buf);
    lcd_line(1, "Hand clear...");

    if (mode == 1) {
        res = run_trial(1, &ms);
        if (res == R_OK) {
            show_number(ms);
            sprintf(buf, "Time: %u ms", ms); lcd_line(0, buf);
            if (ms < best[1]) { best[1] = ms; }
            show_best(1);
        } else report_invalid(res);
    }
    else if (mode == 2) {
        unsigned char green = rnd() & 1;            /* random green / red */
        res = run_trial(green, &ms);
        if (res == R_OK) {
            show_number(ms);
            sprintf(buf, "Time: %u ms", ms); lcd_line(0, buf);
            if (ms < best[2]) { best[2] = ms; }
            show_best(2);
        } else if (res == R_NOGO_OK) {
            show_blank();
            lcd_line(0, "No-Go trial");
            lcd_line(1, "SUCCESS (no time)");
        } else report_invalid(res);
    }
    else {                                          /* mode 3: three rounds */
        sum = 0;
        for (round = 1; round <= 3; round++) {
            sprintf(buf, "Round %u/3", round);
            lcd_line(0, buf);
            lcd_line(1, "Hand clear...");
            show_blank();
            do {
                if (sensor_detected()) { lcd_line(1, "Release F first"); delay_ms(1000); }
                res = run_trial(1, &ms);
                if (report_invalid(res)) {          /* invalid -> retry round */
                    delay_ms(1500);
                    lcd_line(1, "Retry round...");
                    delay_ms(800);
                }
            } while (res != R_OK);
            show_number(ms);
            sprintf(buf, "Round %u: %u ms", round, ms);
            lcd_line(1, buf);
            sum += ms;
            delay_ms(1500);
        }
        ms = (sum + 1) / 3;                         /* rounded average */
        show_number(ms);
        sprintf(buf, "Avg: %u ms", ms);
        lcd_line(0, buf);
        if (ms < best[3]) best[3] = ms;
        show_best(3);
    }
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
    LPC_PINCON->PINSEL3 &= ~0x003FC000u;                  /* P1.23-P1.26     */
    /* keypad columns: pull-down so they read low when no key is pressed */
    LPC_PINCON->PINMODE0 |= 0xC0000000u;                  /* P0.15 */
    LPC_PINCON->PINMODE1 |= 0x0000003Fu;                  /* P0.16-P0.18 */

    LPC_GPIO0->FIODIR |= SEG_MASK | ROW_MASK | LCD_D | LCD_RS | LCD_EN;
    LPC_GPIO0->FIODIR &= ~COL_MASK;
    LPC_GPIO1->FIODIR |= SEL_MASK;

    timers_init();                                  /* display refresh starts */
    lcd_init();

    /* power-on self test: 7-seg shows 1234, LCD shows banner */
    lcd_line(0, "Reaction Tester");
    lcd_line(1, "Self test 1234");
    disp_digit[0] = 4; disp_digit[1] = 3; disp_digit[2] = 2; disp_digit[3] = 1;
    delay_ms(1500);
    show_blank();

    while (1) {
        if (mode == 0) {                            /* ---- MENU ---- */
            lcd_line(0, "Select mode 1-3");
            lcd_line(1, "(other key=test)");
            while (mode == 0) {
                k = key_poll();
                if (k == KEY_MODE1 || k == KEY_MODE2 || k == KEY_MODE3) mode = k;
                else if (k != NOKEY) {              /* keypad test */
                    sprintf(buf, "Key code: %c", key_char[k]);
                    lcd_line(1, buf);
                    show_number(k);
                }
            }
        }
        if (pending == NOKEY) {                     /* ---- READY ---- */
            sprintf(buf, "Mode %u  A=Start", mode);
            lcd_line(0, buf);
            show_best(mode);
            show_blank();
            k = NOKEY;
            while (k == NOKEY) k = key_poll();
        } else { k = pending; pending = NOKEY; }

        if (k == KEY_START) {                       /* ---- RUN ---- */
            run_mode(mode);                         /* result stays on display */
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
