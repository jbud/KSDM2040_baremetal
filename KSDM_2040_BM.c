#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#define KS_RP2040 127
#define KS_AVR    63
#define KSDM_SP   0x115
#define KSDM_3    0x100

#define KSDM_VARIANT KSDM_SP
//#define KSDM_VARIANT KSDM_3

#define KSDM_RP2040_REVISION 5

#define TOGGLE_WAIT_TIME 500
#define STARTUP_WAIT_TIME 4000

#define OFFSET 0
#define TRACT_LONG 1
#define TRACT_SHORT 0

typedef unsigned char kbyte;
typedef unsigned long uli;

const kbyte SMART      = 0x01;
const kbyte ECO        = 0x02;
const kbyte COMFORT    = 0x03;  
const kbyte SPORT      = 0x04;
const kbyte CUSTOM     = 0x05;

#if KSDM_VARIANT == KSDM_SP
  const kbyte TSTATEON  = 0; // Traction on, default
  const kbyte TSTATEOF  = 1; // Traction off, stability on
  const kbyte TSTATETS  = 2; // Traction and Stability off

  const kbyte TRACMSK   = 0x18;
#endif
const kbyte SETUP      = 0x80;
const kbyte AHOLD      = 0x10;
const kbyte ISG        = 0x20;
const kbyte MODE       = 0x07;

#if KSDM_VARIANT == KSDM_SP
    const kbyte KSDEFAULT   = 0xA3; // comfort, isg on, traction on.
#else
    const kbyte KSDEFAULT   = 0xA3; // comfort, isg on, autohold off.
#endif

const uint DMR_IN   = 0;
const uint DML_IN   = 1;
const uint DMR_OUT  = 2;
const uint DML_OUT  = 3;
const uint ISG_IN   = 4;
const uint ISG_OUT  = 5;
const uint AH_IN    = 6;
const uint AH_OUT   = 7;

kbyte currentMode;
bool isSetup, isIsg;
#if KSDM_VARIANT == KSDM_SP
  kbyte tract;
#else
  bool isAhold;
#endif

#if KSDM_VARIANT == KSDM_SP
  uli t_tract;
  uli pt_tract;

  bool b_tract_timer = false;
  bool b_theld = false;
#endif

bool dm_i = true;
bool ahold_i = true;
bool isg_i = true;

bool g_changed = false;

#define FLASH_TARGET_OFFSET (256 * 1024)

const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
/**
 * pack_byte packs global variables for KSDM into a single kbyte
 * 
 * @returns kbyte ready for flash/eeprom storage
 */
kbyte pack_byte()
{
  #if KSDM_VARIANT == KSDM_SP
    return SETUP | currentMode | (isIsg << 5) | (tract << 3); // setup is implied
  #else
    return SETUP | currentMode | (isIsg << 5) | (isAhold << 4); // setup is implied
  #endif
}

/**
 * unpack_byte unpacks a byte from flash/eeprom storage into global variables for KSDM
 * 
 * @param b the kbyte to unpack onto global variables
 */
void unpack_byte(kbyte b)
{                                    
  isSetup = (b & SETUP) >> 7;        
#if KSDM_VARIANT == KSDM_SP
  tract = (b & TRACMSK) >> 3;
#else
  isAhold = (b & AHOLD) >> 4;
#endif
  isIsg = (b & ISG) >> 5;
  currentMode = b & MODE;
}

/**
 * flash_read will grab the first byte from the flash pointer and return it as kbyte
 * 
 * @returns kbyte data the first byte from the flash page
*/
kbyte flash_read()
{
    kbyte data;
    data = flash_target_contents[0];
    return data;
}
/**
 * flash_write will write 1 byte of data to the beginning of the flash page, then verify it
 * 
 * @param data the byte of data to write to the first address in the flash page
 * @returns 1 if verified, 0 if failed.
*/
int flash_write(kbyte data)
{
    uint8_t temp[FLASH_PAGE_SIZE];
    temp[0] = data;
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, temp, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    return flash_read() == data;
}

/**
 * traction_press pulses AH_OUT pin high and low with a delay determined by input.
 * 
 * @param longPress evaluated as boolean, will delay TOGGLE_WAIT_TIME if 0 or 3500ms if 1
 */
void traction_press(int longPress)
{
    int d = TOGGLE_WAIT_TIME;
    if (longPress)
    {
        d = 3500;
    }
#if KSDM_RP2040_REVISION < 6
    gpio_put(AH_OUT, 0);
    sleep_ms(d);
    gpio_put(AH_OUT, 1);
#else
    gpio_put(AH_OUT, 1);
    sleep_ms(d);
    gpio_put(AH_OUT, 0);
#endif
}
/**
 * clock_wise simulates a number of clockwise turns of drive mode switch by pulsing DMR_OUT pin
 * 
 * @param num Number of pulses
 */
void clock_wise(int num)
{
  for(int i=0;i<num;i++)
  {
    gpio_put(DMR_OUT, 1);
    sleep_ms(TOGGLE_WAIT_TIME);
    gpio_put(DMR_OUT, 0);
    sleep_ms(TOGGLE_WAIT_TIME);
  }
}

/**
 * counter_clock_wise simulates a number of counter-clockwise turns of drive mode switch by pulsing DML_OUT pin
 * 
 * @param num Number of pulses
 */
void counter_clock_wise(int num)
{
  for(int i=0;i<num;i++)
  {
    gpio_put(DML_OUT, 1);
    sleep_ms(TOGGLE_WAIT_TIME);
    gpio_put(DML_OUT, 0);
    sleep_ms(TOGGLE_WAIT_TIME);
  }
}

/**
 * millis is a wrapper for RP2040 milliseconds since boot
 * 
 * @returns milliseconds since bootup.
*/
uli millis()
{
    return to_ms_since_boot(get_absolute_time());
}

int main()
{
    stdio_init_all();

    gpio_init_mask(0xFF); // all 8 pins
    gpio_set_dir_in_masked(0x53); // just input pins 
    gpio_set_dir_out_masked(0xAC); // just output pins

    gpio_pull_up(DMR_IN);
    gpio_pull_up(DML_IN);
    gpio_pull_up(ISG_IN);
    gpio_pull_down(AH_IN);

#if KSDM_RP2040_REVISION < 6
    sleep_ms(TOGGLE_WAIT_TIME);
    gpio_put(AH_OUT, 1);
    sleep_ms(TOGGLE_WAIT_TIME);
    gpio_put(AH_OUT, 0);
    sleep_ms(TOGGLE_WAIT_TIME);
    gpio_put(AH_OUT, 1);
#endif

    kbyte data = flash_read();
    if (data > 0xB5 || data < 0x81)
    {
        flash_write(KSDEFAULT);
        unpack_byte(KSDEFAULT);
    }
    else
    {
        unpack_byte(data);
    }

    sleep_ms(STARTUP_WAIT_TIME);

    printf("Hello ksdm!");

#if KSDM_VARIANT == KSDM_SP
    switch(tract)
    {
        case TSTATEOF:
            if (currentMode == SPORT)
            {
                traction_press(TRACT_SHORT);
            }
            break;
        case TSTATETS:
            if (currentMode == SPORT)
            {
                traction_press(TRACT_LONG);
            }
            break;
        case TSTATEON:
        default:
            break;
    }
#else
    if (isAhold)
    {
        gpio_put(AH_OUT, 1);
        sleep_ms(TOGGLE_WAIT_TIME);
        gpio_put(AH_OUT, 0);
    }
#endif
    if (!isIsg)
    {
        gpio_put(ISG_OUT, 1);
        sleep_ms(TOGGLE_WAIT_TIME);
        gpio_put(ISG_OUT, 0);
    }
    switch(currentMode)
    {
        case SPORT:
            clock_wise(1 - OFFSET);
            break;
        case CUSTOM:
            clock_wise(2 - OFFSET);
            break;
        case SMART:
            counter_clock_wise(2 + OFFSET);
            break;
        case COMFORT:
            counter_clock_wise(OFFSET);
            break;
        case ECO:
            counter_clock_wise(OFFSET);
            break;
        default:
            break;
    }
    
    
    while(true)
    {
        if (uart_is_readable(uart0) >= 2)
        {
            kbyte ch[2];
            ch[0] = uart_getc(uart0);
            ch[1] = uart_getc(uart0);
            
            while(uart_is_readable(uart0))
            {
                char dispose = uart_getc(uart0); // dispose of remainder
            } 
            
            if (strcmp(ch, "id") == 0)
            {
#if KSDM_VARIANT == KSDM_SP
                uart_puts(uart0, "ksdm3-rp2040-sportplus");
#else
                uart_puts(uart0, "ksdm3-rp2040-3");
#endif
            }
        }
        if (gpio_get(DMR_IN) && gpio_get(DML_IN) && !dm_i)
        {
            dm_i = true;
        }
        if (gpio_get(ISG_IN) && !isg_i)
        {
            isg_i = true;
        }
#if KSDM_VARIANT == KSDM_SP
        if (gpio_get(AH_IN) && b_theld)
        {
            ahold_i = true;
            b_theld = false;
            t_tract = millis();
            uli diff = t_tract - pt_tract;

            if (diff < 2500)
            {
                if (tract == TSTATEON)
                {
                    tract = TSTATEOF;
                }
                else
                {
                    tract = TSTATEON;
                }
            }
            else if (tract != TSTATETS)
            {
                tract = TSTATETS;
            }
            else
            {
                tract = TSTATEOF;
            }
            if (diff >= 9000)
            {
                tract = TSTATEON; // Reset traction setting if out of sync. hold for 9sec, let go and restart the vehicle.
            }

            g_changed = true;
        }
#else
        if (gpio_get(AH_IN) && !ahold_i)
        {
            ahold_i = true;
        }
        if (gpio_get(AH_IN) && ahold_i)
        { 
            isAhold = !isAhold;
            g_changed = true;
            ahold_i = false;
        }
#endif
        if (gpio_get(DMR_IN) && dm_i)
        {
            if (currentMode >= CUSTOM)
            {
                currentMode = CUSTOM; 
            }
            else
            {
                currentMode++;
            }
#if KSDM_VARIANT == KSDM_SP
            if (currentMode == SPORT)
            {
                switch(tract)
                {
                    case TSTATEOF:
                        traction_press(TRACT_SHORT);
                        break;
                    case TSTATETS:
                        traction_press(TRACT_LONG);
                        break;
                    case TSTATEON:
                    default:
                    break;
                }
            } 
            else if (tract == TSTATETS || tract == TSTATEOF)
            {
                traction_press(TRACT_SHORT);
            }
#endif
            g_changed = true;
            dm_i = false;
        }
        if (gpio_get(DML_IN) && dm_i)
        {
            if (currentMode <= SMART)
            {
                currentMode = SMART; 
            }
            else
            {
                currentMode--;
            }
#if KSDM_VARIANT == KSDM_SP
            if (currentMode == SPORT)
            {
                switch(tract)
                {
                    case TSTATEOF:
                        traction_press(TRACT_SHORT);
                        break;
                    case TSTATETS:
                        traction_press(TRACT_LONG);
                        break;
                    case TSTATEON:
                    default:
                        break;
                }
            } 
            else if (tract == TSTATETS || tract == TSTATEOF)
            {
                traction_press(TRACT_SHORT);
            }
#endif
            g_changed = true;
            dm_i = false;
        }
        if (gpio_get(ISG_IN) && isg_i)
        {
            isIsg = !isIsg;
            g_changed = true;
            isg_i = false;
        }
#if KSDM_VARIANT == KSDM_SP
        if (gpio_get(AH_IN) && ahold_i)
        {
            if (!b_theld)
            {
                pt_tract = millis();
            }
            b_theld = true;
        }
#endif
        if (g_changed)
        {
            kbyte b = pack_byte();
            flash_write(b);
            g_changed = false;
        }
    }
    return 0;
}