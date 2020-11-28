//////------------------------------------
////// Manage settings exchanged in BLE and stored in EEPOM

#include "Arduino.h"

#ifndef _Settings_h

#define _Settings_h

class Settings
{
public:
  Settings();

  void displaySettings1();
  void displaySettings2();
  void displaySettings3();
  void displaySettings();
  void restoreSettings();
  void saveSettings();

// The actions I ca do...
typedef enum ButtonAction
{
  ACTION_None = 0,
  ACTION_Mode_Z_on_off,
  ACTION_Anti_theft_manual_lock_on,
  ACTION_Nitro_boost_cont,
  ACTION_Startup_speed_limitation_on_off,
  ACTION_Aux_on_off, 
  ACTION_Nitro_boost_on_off,
} ButtonActions;

#pragma pack(push, 1)
  struct field_s1
  {
    int8_t Beacon_range;
    uint8_t Mode_Z_Power_limitation;
    uint8_t Mode_Z_Eco_mode;
    uint8_t Mode_Z_Acceleration;
    uint8_t Electric_brake_progressive_mode;
    uint8_t Electric_brake_min_value;
    uint8_t Electric_brake_max_value;
    uint16_t Electric_brake_time_between_mode_shift;
    uint8_t Electric_brake_disabled_condition;
    uint8_t Electric_brake_disabled_percent_limit;
    uint8_t Current_loop_mode;
    uint8_t Current_loop_max_current;
    uint8_t Speed_loop_mode;
    uint8_t Speed_limiter_at_startup;
    uint8_t Wheel_size;
    uint8_t Motor_pole_number;
    uint8_t Bluetooth_lock_mode;
    int8_t LCD_Speed_adjustement;
    int8_t Speed_limiter_max_speed;

  } __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
  struct field_s2
  {
    char Beacon_Mac_Address[17];

  } __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
  struct field_s3
  {
    uint8_t Button_1_short_press_action;
    uint8_t Button_1_long_press_action;
    uint8_t Button_2_short_press_action;
    uint8_t Button_2_long_press_action;
    uint8_t Button_long_press_duration;
    uint16_t Battery_min_voltage;
    uint16_t Battery_max_voltage;
    uint8_t Battery_saving_medium_voltage;
    uint8_t Battery_saving_strong_voltage;
    uint32_t Bluetooth_pin_code;

  } __attribute__((packed));
#pragma pack(pop)

  union settings_bt1
  {
    struct field_s1 fields;
    unsigned char buffer[sizeof(struct field_s1)];
  };

  union settings_bt2
  {
    struct field_s2 fields;
    unsigned char buffer[sizeof(struct field_s2)];
  };

  union settings_bt3
  {
    struct field_s3 fields;
    unsigned char buffer[sizeof(struct field_s3)];
  };

  union settings_bt1 settings1;
  union settings_bt2 settings2;
  union settings_bt3 settings3;

  field_s1 getS1F();
  field_s2 getS2F();
  field_s3 getS3F();

  unsigned char* getS1B();
  unsigned char* getS2B();
  unsigned char* getS3B();
};

#endif