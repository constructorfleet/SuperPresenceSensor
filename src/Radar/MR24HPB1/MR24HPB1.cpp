#include <functional>
#include "Arduino.h"
#include "MR24HPB1.h"
#include "MR24HPB1_def.h"

namespace MR24HPB1
{

  MR24HPB1::MR24HPB1(HardwareSerial &serialPort, uint8_t presencePin, uint8_t motionPin) : serial(serialPort)
  {
    serial = serialPort;
    serial.begin(9600);
    presence_pin = presencePin;
    motion_pin = motionPin;
    pinMode(presence_pin, INPUT);
    pinMode(motion_pin, INPUT);
  }

  void MR24HPB1::register_on_unoccupied(std::function<void(void)> callback)
  {
    _on_unoccupied = callback;
  }
  void MR24HPB1::register_on_occupied(std::function<void(void)> callback)
  {
    _on_occupied = callback;
  }
  void MR24HPB1::register_on_stationary(std::function<void(void)> callback)
  {
    _on_stationary = callback;
  }
  void MR24HPB1::register_on_movement(std::function<void(void)> callback)
  {
    _on_movement = callback;
  }
  void MR24HPB1::register_on_away_state(std::function<void(uint8_t)> callback)
  {
    _on_away_state = callback;
  }
  void MR24HPB1::register_on_environmental_state(std::function<void(uint8_t)> callback)
  {
    _on_environmental_state = callback;
  }
  void MR24HPB1::register_on_motor_signs(std::function<void(float)> callback)
  {
    _on_motor_signs = callback;
  }

  uint8_t MR24HPB1::getMotionStatus()
  {
    return motion;
  }
  uint8_t MR24HPB1::getThreshold()
  {
    uint8_t data[1];
    sendMsg(READ, SYSTEM_PARAM, THRESHOLD_GEAR, data, 0);
    yield(100);
    return threshold;
  }
  uint8_t MR24HPB1::getSceneSetting()
  {
    uint8_t data[1];
    sendMsg(READ, SYSTEM_PARAM, SCENE_SETTING, data, 0);
    yield(100);
    return scene_setting;
  }
  uint8_t MR24HPB1::getMotorSigns()
  {
    uint8_t data[1];
    sendMsg(READ, RADAR_INFO, MOTOR_SIGNS, data, 0);
    yield(100);
    return motor_signs;
  }
  uint8_t MR24HPB1::getEnvironmentalState()
  {
    uint8_t data[1];
    sendMsg(READ, RADAR_INFO, ENVIRONMENTAL_STATUS, data, 0);
    yield(100);
    return environmental_state;
  }
  boolean MR24HPB1::getPresence()
  {
    return presence;
  }
  away_state_t MR24HPB1::getAwayState()
  {
    return (away_state_t)away_state;
  }
  uint8_t MR24HPB1::getUpdatedMemberType()
  {
    uint8_t member = updated_member;
    updated_member = 0xFF;
    return member;
  }

  int MR24HPB1::setThreshold(uint8_t gear)
  {
    if (gear <= 0 || gear > 10)
      return -1;
    uint8_t data[1] = {gear};
    long start = millis();
    long timeout = 10;
    while (threshold != gear)
    {
      if (millis() - start > timeout)
        return -2; // timeout
      sendMsg(WRITE, SYSTEM_PARAM, THRESHOLD_GEAR, data, 1);
      yield(50);
    }
    return 0;
  }
  int MR24HPB1::setSceneSetting(scene_setting_t scene)
  {
    uint8_t data[1] = {(uint8_t)scene};
    long start = millis();
    long timeout = 10;
    while (scene_setting != (uint8_t)scene)
    {
      if (millis() - start > timeout)
        return -2; // timeout
      sendMsg(WRITE, SYSTEM_PARAM, (addr_cmd2_t)0x10, data, 1);
      yield(50);
    }
    return 0;
  }
  void MR24HPB1::Reboot()
  {
    uint8_t data[1];
    sendMsg(WRITE, OTHER, REBOOT, data, 0);
  }
  void MR24HPB1::getPinValues()
  {
    boolean curr_p = digitalRead(presence_pin);
    boolean curr_m = digitalRead(motion_pin);

    if (presence != curr_p)
    { // edge detection
      presence = curr_p;
      std::function<void(void)> cb = (curr_p) ? _on_occupied : _on_unoccupied; // decide which callback to call
      if (cb != NULL)
        cb(); // call callback if registered
    }

    if (motion != curr_m)
    {
      motion = curr_m;
      std::function<void(void)> cb = (curr_m) ? _on_movement : _on_stationary;
      if (cb != NULL)
        cb(); // call callback if registered
    }
  }

  // Helpers
  uint16_t MR24HPB1::CRC16(uint8_t *Frame, uint8_t Len)
  {
    unsigned char luc_CRCHi = 0xFF;
    unsigned char luc_CRCLo = 0xFF;
    int li_Index = 0;
    while (Len--)
    {
      li_Index = luc_CRCLo ^ *(Frame++);
      luc_CRCLo = (unsigned char)(luc_CRCHi ^ cuc_CRCHi[li_Index]);
      luc_CRCHi = cuc_CRCLo[li_Index];
    }
    return (uint16_t)(luc_CRCLo << 8 | luc_CRCHi);
  }
  uint16_t MR24HPB1::getMsg_length()
  {
    uint16_t size = msg[1];
    size = size << 8 | msg[0];
    msg_size = size;
    return size;
  }
  uint16_t MR24HPB1::getData_length()
  {
    return getMsg_length() - 7;
  }
  boolean MR24HPB1::verifyMsg(uint8_t *msg, uint8_t length)
  {

    uint16_t crc16_msg = msg[length - 2];
    crc16_msg = crc16_msg << 8 | msg[length - 1];

    uint8_t frame[length + 1];
    frame[0] = 0x55;
    for (int i = 1; i <= length; i++)
    {
      frame[i] = msg[i - 1];
    }
    uint16_t crc = CRC16(frame, length - 1);

    return crc == crc16_msg;
  }
  void MR24HPB1::sendMsg(function_cmd_t fc, addr_cmd1_t cmd1, addr_cmd2_t cmd2, uint8_t *data, uint8_t data_length)
  {
    uint8_t frame_length = data_length + 8;
    uint8_t frame[frame_length];
    frame[0] = HEADER;
    frame[1] = lowByte(frame_length - 1); // length
    frame[2] = highByte(frame_length - 1);
    frame[3] = (uint8_t)fc;   // function code
    frame[4] = (uint8_t)cmd1; // addres code 1
    frame[5] = (uint8_t)cmd2; // address code 2
    for (uint8_t i = 0; i < data_length; i++)
    { // data if any
      frame[i + 6] = data[i];
    }
    uint16_t crc = CRC16(frame, 6 + data_length);
    frame[data_length + 6] = highByte(crc);
    frame[data_length + 7] = lowByte(crc);
    serial.write(frame, frame_length);
#ifdef DEBUG
    Serial.print("Send: ");
    for (int i = 0; i < frame_length; i++)
    {
      Serial.print(" 0x");
      Serial.print(frame[i], HEX);
    }
    Serial.println();
#endif
  }
  void MR24HPB1::betterRecieveMsg()
  {
    while (serial.available() > 0)
    {
      if (serial.read() == HEADER)
      {
        uint8_t dataLength = serial.read() + (serial.read() << 8);
        DataFrameBuilder dataFrame;
        for (int i = 0; i < dataLength - 2; i++)
        {
          dataFrame.bytes[i] = serial.read();
        }
      }
    }
  }
  void MR24HPB1::recieveMsg()
  {
    static boolean recieving = false;
    static uint8_t index = 0;
    static uint8_t data_len = 12;
    uint8_t r_byte; // recieved byte
    // Serial.println(serial.available());
    while (serial.available() > 0 && newData == false)
    {
      r_byte = serial.read();
      if (recieving)
      { // recieved Header
        if (data_len > index)
        { // length in range
          msg[index] = r_byte;
          if (index == 0)
          {
            data_len = msg[0]; // Get length frame
            msg_size = data_len;
          }
          index++;
        }
        else
        {
          recieving = false;
          index = 0;
          newData = true;
        }
      }
      else if (r_byte == HEADER)
      {                   // detected Header byte
        recieving = true; // start reading
      }
    }
  }
  void MR24HPB1::parseMsg()
  {
    // function_cmd_t FunctionCommand = (function_cmd_t) msg[2];
    addr_cmd1_t ADDRESS1 = (addr_cmd1_t)msg[3];
    addr_cmd2_t ADDRESS2 = (addr_cmd2_t)msg[4];
    std::function<void(void)> cb;
    switch (ADDRESS1)
    { // function Codes
    case MODULE_INFO:
    {
      // Not implemented (yet)
      break;
    }
    case RADAR_INFO:
    {
      switch (ADDRESS2)
      {
      case ENVIRONMENTAL_STATUS:
      {
        updated_member = ENVIRONMENTAL_STATUS;
        std::function<void(uint8_t)> _cb = _on_environmental_state;
        auto prev = environmental_state;
        auto new_ = prev;
        if (msg[5] == 0x00 && msg[6] == 0xFF && msg[7] == 0xFF)
        {
          new_ = (uint8_t)UNOCCUPIED;
          presence = false;
          motion = false;
        }
        else if (msg[5] == 0x01 && msg[6] == 0x00 && msg[7] == 0xFF)
        {
          new_ = (uint8_t)STATIONARY;
          presence = true;
          motion = false;
        }
        else if (msg[5] == 0x01 && msg[6] == 0x01 && msg[7] == 0x01)
        {
          new_ = (uint8_t)EXERCISING;
          presence = true;
          motion = true;
        }
        else
          _cb = NULL;

        if (new_ != prev)
        {
          environmental_state = new_;
          if (_cb != NULL)
            _cb((uint8_t)environmental_state);
        }

        break;
      } // end environmental state
      case MOTOR_SIGNS:
      {
        updated_member = MOTOR_SIGNS;
        std::function<void(float)> _cb = _on_motor_signs;
        FB signs;

        signs.B[0] = msg[5];
        signs.B[1] = msg[6];
        signs.B[2] = msg[7];
        signs.B[3] = msg[8];
        if (signs.F >= motor_signs + 1 || signs.F <= motor_signs - 1)
        {
          motor_signs = signs.F;
          if (_cb != NULL)
            _cb(signs.F);
        }

#ifdef DEBUG
        for (int i = 0; i < msg_size; i++)
        {
          Serial.print(" ");
          Serial.print(msg[i], HEX);
        }
        Serial.print(" f: ");
        Serial.println(signs.F);
#endif
        break;
      } // end Motor signs
      case APPROACHING_AWAY_STATE:
      {
        std::function<void(uint8_t)> _cb = _on_away_state;
        updated_member = APPROACHING_AWAY_STATE;
        if (msg[5] == 0x01 && msg[6] == 0x01)
        {
          switch (msg[7])
          {
          case 0x01:
            away_state = (uint8_t)NONE;
            break;
          case 0x02:
            away_state = (uint8_t)CLOSE_TO;
            break;
          case 0x03:
            away_state = (uint8_t)STAY_AWAY;
            break;
          default:
            _cb = NULL;
          }
          if (_cb != NULL)
            _cb(away_state); // call callback if registered
        }

        break;
      } // end of away state

      } // end of Radar Info
      break;
    } // end Radar Info
    case SYSTEM_PARAM:
    {
      switch (ADDRESS2)
      {
      case THRESHOLD_GEAR:
      {
        updated_member = THRESHOLD_GEAR;
        threshold = (msg[5] >= 0 && msg[5] <= 10) ? msg[5] : threshold;
        break;
      } // end Threshold
      case SCENE_SETTING:
      {
        scene_setting = (msg[5] > 0 && msg[5] <= 6) ? msg[5] : scene_setting;
        break;
      } // end scene setting
      } // end of address code 2
      break;
    } // end system Parameters
    case OTHER:
    {
      switch (ADDRESS2)
      {
      case HEARTBEAT:
      {
        updated_member = ENVIRONMENTAL_STATUS;
        std::function<void(uint8_t)> _cb = _on_environmental_state;
        if (msg[5] == 0x00 && msg[6] == 0xFF && msg[7] == 0xFF)
        {
          environmental_state = (uint8_t)UNOCCUPIED;
        }
        else if (msg[5] == 0x01 && msg[6] == 0x00 && msg[7] == 0xFF)
        {
          environmental_state = (uint8_t)STATIONARY;
        }
        else if (msg[5] == 0x01 && msg[6] == 0x01 && msg[7] == 0x01)
        {
          environmental_state = (uint8_t)EXERCISING;
        }
        else
          _cb = NULL;

        if (_cb != NULL)
          _cb((uint8_t)environmental_state);
        break;
      } // end of heartbeat
      case ABNORMAL_RESET:
      {
        if (msg[5] == 0x0F)
          abnormalResets++;
        break;
      }
      } // end of address code 2
      break;
    } // end of other funct
    default:
      for (int i = 0; i < msg_size; i++)
      {
        Serial.print(" 0x");
        Serial.print(msg[i], HEX);
      }
      Serial.println();
      break;
    } // end of function codes
  }
  void MR24HPB1::yield(long Delay)
  {
    long start = millis();
    while (millis() - start < Delay)
    {
      refresh();
      delay(2);
    }
  }
  int MR24HPB1::begin(uint8_t threshold, scene_setting_t scene)
  {

    setThreshold(threshold);
    yield(100);
    setSceneSetting(scene);
    yield(100);
    getEnvironmentalState();
    yield(100);
    getMotorSigns();
    return 0;
  }
  int MR24HPB1::begin()
  {

    getThreshold();
    yield(100);
    getSceneSetting();
    yield(100);
    getEnvironmentalState();
    yield(100);
    getMotorSigns();

    return 0;
  }
  void MR24HPB1::refresh()
  {
    getPinValues(); // Get current state for fast reaction
    recieveMsg();   // Get serial data into array
    bool is_valid = verifyMsg(msg, msg_size);
    if (is_valid && newData)
    {
      parseMsg();
    }
#ifdef DEBUG
    if (newData)
    {
      Serial.print("\n############################################### \n\n");
      for (int i = 0; i < msg_size; i++)
      {
        Serial.print(" 0x");
        Serial.print(msg[i], HEX);
      }
      Serial.println();

      Serial.print("away_state: . . . . : ");
      Serial.println(away_state);
      Serial.print("threshold: - - - - -: ");
      Serial.println(threshold);
      Serial.print("scene_setting: . . .: ");
      Serial.println(scene_setting);
      Serial.print("environmental_state : ");
      Serial.println(environmental_state);
      Serial.print("motor_signs: . . . .: ");
      Serial.println(motor_signs, 4);
      Serial.print("presence: - - - - - : ");
      Serial.println(presence);
      Serial.print("motion: . . . . . . : ");
      Serial.println(motion);
      Serial.print("newData: - -  - - - : ");
      Serial.println(newData);
      Serial.print("abnormalResets: . . : ");
      Serial.println(abnormalResets);
    }
#endif
    newData = false; // mark data as read
  }

};