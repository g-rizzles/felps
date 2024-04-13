#pragma once
#ifndef FELPS_MOCK_SERIAL_HANDLER_H_
#define FELPS_MOCK_SERIAL_HANDLER_H_

#include <string>

/** @brief Mock hardware serial class. */
class HardwareSerial {
  public:
    HardwareSerial() {;}
    ~HardwareSerial() {;}
    size_t available() {
      return this->buffer.length();
    }
    char read() {
      char t = this->buffer[0];
      this->buffer.erase(0, 1);
      return t;
    }
    void load_buffer(std::string& str) {
      this->buffer = str;
    }

  private:
    std::string buffer;
};

#endif // FELPS_MOCK_SERIAL_HANDLER_H_