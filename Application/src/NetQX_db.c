#include "main.h"
#include "stm32f0xx_hal.h"
#include "setup.h"

void InitWDG(void);
void DB_load_key(KEY_RECORD *key, u32 index);

//-----------------------------------------------------------------------------
                                          //NetMX changes needed
void DB_store_key(KEY_RECORD *key, u32 index)
  {
  u32 addr;
  static KEY_RECORD tkey;
  last_function = 101;
//  index--;
  addr = ADDR_KEYS + index * sizeof(KEY_RECORD);
  write_ext_eeprom(addr, (u8 *)key, sizeof(KEY_RECORD));
  delay_ms(5);
  DB_load_key(&tkey, index);
  __NOP();
  }

//-----------------------------------------------------------------------------
                                          //NetMX changes needed
void DB_load_key_code(u32 *key_code, u32 index)
  {
  u32 addr = 0;
  last_function = 102;
//  index--;
  addr = ADDR_KEYS + index * sizeof(KEY_RECORD);
  read_ext_eeprom(EE_LKC, addr, (u8 *)key_code, 4);
  }

//-----------------------------------------------------------------------------
                                          //NetMX changes needed
void DB_clear_key_code(u32 index)
  {
  u32 addr;
  u32 key_code;
  last_function = 103;
  key_code = 0xFFFFFFFFL;
//  index--;
  addr = ADDR_KEYS + index * sizeof(KEY_RECORD);
  write_ext_eeprom(addr, (u8 *)&key_code, 4);
  }

//-----------------------------------------------------------------------------
                                          //NetMX changes needed
void DB_load_key(KEY_RECORD *key, u32 index)
  {
  u32 addr;
  last_function = 104;
//  index--;
  addr = ADDR_KEYS + index * sizeof(KEY_RECORD);
  read_ext_eeprom(EE_LKR, addr, (u8 *)key, sizeof(KEY_RECORD));
  }

//-----------------------------------------------------------------------------
                                          //NetMX changes needed
void DB_delete_key(u32 index)
  {
  KEY_RECORD key;
  last_function = 105;
  key.Code = 0xFFFFFFFFL;
  key.al[0] = 0xFF;
  DB_store_key(&key, index);
  }

//-----------------------------------------------------------------------------
u32 DB_hash(u32 Key)                            //NetMX changes needed
  {
  u32 x1, x2;
  union  {
         u32 k;
         u8 c[4];
         } u;
  last_function = 106;
  u.k = Key;
  x1 = CalculateCRC(&u.c[0], 4);
//  x1 &=  0xFFF;
//  while (x1 >= MAX_KEYS)
//    x1 -= MAX_KEYS;
  x2 = x1 % MAX_KEY_RECORDS;
  return x2;
  }

//-----------------------------------------------------------------------------
u32 DB_rehash(u32 *Key)                         //NetMX changes needed
  {
  u32 x1;
  u32 kcode;
  last_function = 107;
  kcode = *Key;
//  rotate_right(&kcode, 4);
  kcode = (kcode >> 1) | (kcode << 31);
  *Key = kcode;
  x1 = DB_hash(kcode);
  return x1;
  }

//-----------------------------------------------------------------------------
u16 DB_search_empty_slot(u32 pcode) 
  {
  u32 index = 0;
  u16  loop;
  u32 xcode, Code;
//  KEY_RECORD k;
  last_function = 108;
  xcode = pcode;
  restart_wdt();
  for (loop = 1; loop <= 31; loop++)
    {
    index = DB_rehash(&xcode);
    if (!index)
      {
      continue;
      }
    DB_load_key_code(&Code, index);
    if (Code == 0xFFFFFFFFL)// || k.create_seq != global_setup.create_seq)
      {
      return index;
      }
    }
  restart_wdt();
  for (loop = 1; loop <= 96; loop++)
    {
    DB_load_key_code(&Code, ++index);
    if (Code == 0xFFFFFFFFL)// || k.create_seq != global_setup.create_seq)
      {
      return index;
      }
    }
  return 0;
  }

//-----------------------------------------------------------------------------

u32 DB_search_key(u32 pcode, KEY_RECORD *key) 
  {
  u32  index, Code;
  u16  loop;
  u32 xcode, nokey = 0;
  KEY_RECORD k;
  last_function = 109;
  xcode = pcode;
  index = 0;
  restart_wdt();
  for (loop = 1; loop <= 31; loop++)
    {
    index = DB_rehash(&xcode);
//    DB_load_key_code(&k.code, index);
    DB_load_key_code(&Code, index);
    if (Code == pcode)
      {
      DB_load_key(&k, index);
      memcpy(key, &k, sizeof(k));
      return index;
      }
    }
  restart_wdt();
  for (loop = 1; loop <= 96; loop++)
    {
    DB_load_key_code(&Code, ++index);
    if (Code == pcode)
      {
      DB_load_key(key, index);
      return index;
      }
    }
  return nokey;
  }

//-----------------------------------------------------------------------------

u16 DB_add_key(KEY_RECORD *key)    //NetMX changes needed
  {
  u32 index;
  last_function = 110;
  index = DB_search_empty_slot(key->Code); // look for free slot
  if (index)
    {
    DB_store_key(key, index);
    }
  return index;
  }

//-----------------------------------------------------------------------------
void DB_store_access_level(u16 index, AL_RECORD *al)
  {
  last_function = 112;
  index--;
  write_ext_eeprom((u32)index * sizeof(AL_RECORD) + ADDR_AL, (u8 *)al, sizeof(AL_RECORD));
  }

//-----------------------------------------------------------------------------

void DB_load_access_level(u16 index, AL_RECORD *al)
  {
  last_function = 113;
  index--;
  read_ext_eeprom(EE_LAL, (u32)index * sizeof(AL_RECORD) + ADDR_AL, (u8 *)al, sizeof(AL_RECORD));
  }

//-----------------------------------------------------------------------------
void DB_store_weekzone(u16 index, WZ_RECORD *tz)
  {
  last_function = 114;
  index--;
  write_ext_eeprom((u32)index * sizeof(WZ_RECORD) + ADDR_TZ, (u8 *)tz, sizeof(WZ_RECORD));
  }

//-----------------------------------------------------------------------------

void DB_load_weekzone(u16 index, WZ_RECORD *tz)
  {
  last_function = 115;
  index--;
  read_ext_eeprom(EE_LWZ, (u32)index * sizeof(WZ_RECORD) + ADDR_TZ, (u8 *)tz, sizeof(WZ_RECORD));
  }

//-----------------------------------------------------------------------------
void DB_store_dayzone(u16 index, DZ_RECORD *dz)
  {
  last_function = 116;
  index--;
  write_ext_eeprom((u32)index * sizeof(DZ_RECORD) + ADDR_DZ, (u8 *)dz, sizeof(DZ_RECORD));
  }

//-----------------------------------------------------------------------------

void DB_load_dayzone(u16 index, DZ_RECORD *dz)
  {
  last_function = 117;
  index--;
  read_ext_eeprom(EE_LDZ, (u32)index * sizeof(DZ_RECORD) + ADDR_DZ, (u8 *)dz, sizeof(DZ_RECORD));
  }

//-----------------------------------------------------------------------------
void DB_store_holidays(u16 month, u8 *holidays)
  {
  u32 holiday_flags, mask;
  u16 idx;
  last_function = 118;
  holiday_flags = 0;
  for (idx = 0, mask = 1L; idx < 31; idx++, mask <<= 1)
    {
    if (*holidays++)
      {
      holiday_flags |= mask;
      }
    }
  write_ext_eeprom((u32)month * 4 + ADDR_HOLIDAY, (u8 *)&holiday_flags, 4);
  }

//-----------------------------------------------------------------------------
void DB_load_holidays(u16 month, u8 *holidays)
  {
  u32 holiday_flags;
  u16 idx;
  last_function = 119;
  read_ext_eeprom(EE_LMO, (u32)month * 4 + ADDR_HOLIDAY, (u8 *)&holiday_flags, 4);
  if (holiday_flags == 0xFFFFFFFFL)
    holiday_flags = 0;
  for (idx = 0; idx < 31; idx++, holiday_flags >>= 1)
    if (holiday_flags & 1L)
      holidays[idx] = 1;
    else
      holidays[idx] = 0;
  }

//-----------------------------------------------------------------------------
void DB_erase_Al_wk_dy(void)
  {
  u16 index, blink = 0;
  WZ_RECORD w;
  DZ_RECORD d;
  AL_RECORD a;
  last_function = 120;
  memset(&w, 0xFF, sizeof(w));
  memset(&d, 0xFF, sizeof(d));
  memset(&a, 0xFF, sizeof(a));
  for (index = 1; index < 256; index++)
    {
    restart_wdt();
    DB_store_access_level(index, &a);
    DB_store_dayzone(index, &d);
    DB_store_weekzone(index, &w);
    if (++blink > 20)
      {
      blink = 0;
      output_toggle(LED_RED);
      output_toggle(LED_GREEN);
      restart_wdt();
      }
    }
  }


//-----------------------------------------------------------------------------
void set_apb_counter(u32 index, u16 dir)    //NetMX changes needed
  {
  u16 field;
  u8 temp, mask;
  u32 addr;
  u16 direction;
  last_function = 121;
  index--;
  addr = index / 4; // each record has 2 bits
  field = index & 3;
  mask  = 3 << (field * 2);
  dir &= 3;
  direction = dir;
  direction <<= (field * 2);
  temp = APB_table[addr];
  temp &= ~mask;
  temp |= direction;
  APB_table[addr] = temp;
  }

//-----------------------------------------------------------------------------
u8 get_apb_counter(u32 index)              //NetMX changes needed
  {
  u16 field, temp;
  u32 addr;
  last_function = 122;
  index--;
  addr = index / 4; // each record has 2 bits
  field = index & 3;
  temp = APB_table[addr];
  temp >>= (field * 2);
  temp &= 3;
  return temp;
  }


//-----------------------------------------------------------------------------
u32 count_keys_in_DB(void)
  {
  u32 count, key_code, led_toggle = 0;
  last_function = 123;
  keys_in_eeprom = 0;
//  KEY_RECORD key_rec;
  output_low(LED_RED);
  output_low(LED_GREEN);
  for (count = 0; count < MAX_KEY_RECORDS; count++)
    {
    if (++led_toggle > 20)
      {
      led_toggle = 0;
      output_toggle(LED_RED);
      output_toggle(LED_GREEN);
      InitWDG();
      }
//    DB_load_key(&key_rec, count);
    
    DB_load_key_code(&key_code, count);
//    if (key_rec.Code && key_rec.Code != 0xFFFFFFFFL)
    if (key_code && key_code != 0xFFFFFFFF)
      {
      keys_in_eeprom++;
      }
    restart_wdt();
    }
  return keys_in_eeprom;
  }

//-----------------------------------------------------------------------------
void erase_keys_in_DB(void)
  {
  u32 count, key_code, led_toggle = 0;
  keys_in_eeprom = 0;
//  KEY_RECORD key_rec;
  last_function = 124;
  output_low(LED_RED);
  output_high(LED_GREEN);
  for (count = 0; count < MAX_KEY_RECORDS; count++)
    {
    if (++led_toggle > 20)
      {
      led_toggle = 0;
      output_toggle(LED_RED);
      output_toggle(LED_GREEN);
      InitWDG();
      }
//    DB_load_key(&key_rec, count);
//    if (key_rec.Code && key_rec.Code != 0xFFFFFFFFL)
    DB_load_key_code(&key_code, count);
    if (key_code && key_code != 0xFFFFFFFF)
      {
//      DB_delete_key(count);
      DB_clear_key_code(count);
      restart_wdt();
      }
    }
  keys_in_eeprom = 0;
  }



