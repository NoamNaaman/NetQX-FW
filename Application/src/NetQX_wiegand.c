#include "main.h"
#include "stm32f0xx_hal.h"
#include "setup.h"

u8 zeros[MAX_DOORS], ones[MAX_DOORS];
u32 raw_wiegand;

extern u32 dip_switches;

//--------------------------------------------------------------
// wiegand reader functions
//--------------------------------------------------------------
void wiegand_d1(u32 reader)
  {

  if ((reader_bitmask[reader] >>= 1) == 0)
    {
    reader_bitmask[reader] = 0x80;
    WGND_kepress[reader] = *reader_byteptr[reader];
    reader_byteptr[reader]++;
    }
  Timer_1mS_Count[reader] = 0; // start timeout counter from this bit
  BitCnt[reader]++;
  }

void wiegand_d0(u32 reader)
  {
  // data input is zero. simply move to next bit
  *reader_byteptr[reader] |= reader_bitmask[reader];

  if ((reader_bitmask[reader] >>= 1) == 0)
    {
    reader_bitmask[reader] = 0x80;
    WGND_kepress[reader] = *reader_byteptr[reader];
    reader_byteptr[reader]++;
    }
  Timer_1mS_Count[reader] = 0; // start timeout counter from this bit
  BitCnt[reader]++;
  }



void WGND_init(u8 reader)
  {
  reader_bitmask[reader] = 0x80;
  BitCnt[reader]= 0;
  WGND_kepress[reader] = 0xFF;
  reader_data[reader][8] = 0;
  reader_data[reader][7] = 0;
  reader_data[reader][6] = 0;
  reader_data[reader][5] = 0;
  reader_data[reader][4] = 0;
  reader_data[reader][3] = 0;
  reader_data[reader][2] = 0;
  reader_data[reader][1] = 0;
  reader_data[reader][0] = 0;
  reader_byteptr[reader] = &reader_data[reader][0];
  zeros[reader] = 0;
  ones[reader] = 0;
  }


u32 extract_wiegand_key(u32 first, u32 digits, u8 *data)
  {
//  u16 idx;
  u32 number = 0L, reader_mode;
  u8 digit, mask;
  first -= 1;
  number = 0;
  reader_mode = dip_switches & 0x80;
  raw_wiegand = make32(data[3], data[2], data[1], data[0]);
  if (reader_mode == 0x80)
    {
    number = make32(0,0, data[2], data[3]); // HID Jerusalem municipality?
    return number;
    }
  
  while (first >= 8) // skip to first byte of data
    {
    first -= 8;
    data++;
    }
  mask = 1 << (7 - (first & 7));
  digit = *data++;
//  idx = 0;
  while (digits)
    {
      {
      number <<= 1;
      if (digit & mask)
        number |= 1;
      mask >>= 1;
      if (!mask)
        {
        mask = 0x80;
        digit = *data++;
        }
      digits--;
      }
    }
  return number;
  }


void process_pin(u32 reader, u32 tag)
  {
  new_key[reader] = tag;
  led_period[reader] = 20;
  led_onoff(reader, 1);
  UserKeyReady[reader] = KEY_SEND_TIMES;
  KeySource[reader] = 0;
  init_pin_only_wait(reader);
  WGND_init(reader);
  }

u8 WGND_wait(u32 reader)
  {
  u32 digit;//, bytes;
  u16 pincode;
  u8 kswitch[4];
  u32 tag;
  if (!BitCnt[reader])
    {
    Timer_1mS_Count[reader] = 0;
    }
  else if (Timer_1mS_Count[reader] > 20)
    {
    if (BitCnt[reader] < 26)
      {
      if ((pin_only_mode(reader) || wait_wiegand_digit[reader]) && BitCnt[reader] == 4)
        {
        digit = reader_data[reader][0];
        reader_data[reader][0] = 0;
        digit >>= 4;
        BitCnt[reader] = 0;
        if (digit < 10)
          {
          digit <<= (8 - ++collected_digits[reader]) * 4;
          collected_pin[reader] |= digit;
          }
        else
          {
          while (collected_digits[reader] < 9)
            {
            digit = (u32)15 << (8 - ++collected_digits[reader]) * 4;
            collected_pin[reader] |= digit;
            }
          digit = collected_pin[reader];
          if (pin_only_mode(reader))
            {
            process_pin(reader, digit);
            return 1;
            }
          pincode = CalculateCRC((u8 *)&digit, 4);
          if (pincode == pin_key_rec[reader].PIN)
            {
            process_key(reader, pin_key_rec[reader].Code, 1);
            }
          else
            {
            generate_event(reader, pin_key_rec[reader].ID, pin_key_rec[reader].Code, EVT_wrong_PIN_code);
            }
          clear_pin_wait(reader);
          }
        }
      
      WGND_init(reader);
      return 0;
      }
    else if (wait_wiegand_digit[reader])
      {
      clear_pin_wait(reader);
      }
//    new_code:
//    bytes = (BitCnt[reader]+ 7) / 8;
    tag = extract_wiegand_key(doors[reader].First_digit, doors[reader].Number_of_digits, &reader_data[reader][0]);
    if (mul_t_lock)
      {
      kswitch[0] = make8(tag, 2);
      kswitch[1] = make8(tag, 1);
      kswitch[2] = make8(tag, 0);
      kswitch[3] = make8(tag, 3);
      tag = make32(kswitch[3],kswitch[2],kswitch[1],kswitch[0]);
      }
    new_key[reader] = tag;
    led_period[reader] = 20;
    led_onoff(reader, 1);
    
     UserKeyReady[reader] = KEY_SEND_TIMES;
    KeySource[reader] = 0;
    WGND_init(reader);
    return 1;
    }
  return 0;
  }
