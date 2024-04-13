#pragma once
#ifndef SERIAL_HANDLER_H_
#define SERIAL_HANDLER_H_

#include <cstring>

#ifndef FELPS_TEST_BUILD
#include <Arduino.h>
#endif

#include "src/os/err.h"

#define SERIAL_HANDLER_BUF_LENGTH (64)

typedef int (*serial_handler_cmd_f)(char * buf, uint8_t len,
                                    const char delim, void * sysargs);

typedef struct {
  const char * cmd_word;
  serial_handler_cmd_f handler;
  void * sysargs;
} serial_handler_cmd_map_t;

typedef struct {
  uint8_t num_cmds;
  serial_handler_cmd_map_t cmds[];
} serial_handler_cmd_dictionary_t;

/**
 * @brief Manages a serial device and attempts to call mapped commands when
 * possible. Serial handlers expect ASCII encoded strings! EOLs and DELIMs
 * may not be processed correctly if sending non-ascii strings.
 */
class SerialHandler {
  public:
    SerialHandler(HardwareSerial * serial, serial_handler_cmd_dictionary_t * cmd_dict,
                  char eol, char delim)
                 : serial(serial), cmd_dict(cmd_dict), eol(eol), delim(delim) {
      this->buf_fill = 0;
      memset(this->buf, 0x00, SERIAL_HANDLER_BUF_LENGTH * sizeof(char));
    }
    ~SerialHandler() {;}

    /**
     * @brief Empties the serial handler's buffer.
     * @return the number of bytes discarded or < 0 for an error code.
     */
    int flush_buf() {
      int rc = this->buf_fill;
      this->buf_fill = 0;
      memset(this->buf, 0x00, SERIAL_HANDLER_BUF_LENGTH * sizeof(char));
      return rc;
    }

    /**
     * @brief Checks the provided serial device for any characters. Places as
     * many as possible into the serial handler's buffer.
     * @param[in]   ready pointer to store whether a command has been detected
     * @return the number of characters read, or <0 for an error.
     * @details
     * If the buffer is full and no EOL is in the buffer, discard bytes until an
     * EOL is found. Then flush the buffer. If buffer is full and an EOL was found,
     * stop reading bytes. Then flush the buffer. If requested, report found EOL
     * chars by marking ready true if one is found.
     */
    int process_serial(bool * ready) {
      int rc = -FELPS_ERR_UNKNOWN;
      int chars_read = 0;
      char b = 0;
      bool found_eol = false;
      // Process bytes until out of bytes to process
      while (this->serial->available() > 0) {
        if (this->buf_fill < SERIAL_HANDLER_BUF_LENGTH) {
          // Get bytes and append to buffer. If EOL is found, mark it
          b = this->serial->read();
          chars_read++;
          if (b == this->eol) {
            found_eol = true;
          }
          this->buf[buf_fill] = b;
          this->buf_fill++;
          rc = chars_read;
        } else {
          // Buffer full!
          if (found_eol) {
            // Found an EOL and then filled the buffer, stop reading and save bytes
            // in serial object buffer for next process_serial call
            rc = chars_read;
            break;
          } else {
            // Did not find EOL, discard until we do or are out of bytes. Report
            // whether we have a bad buffer or dropped a message. Caller response
            // behavior may be different. Buffer will be flushed in both cases.
            rc = -FELPS_ERR_BAD_BUF;
            b = this->serial->read();
            if (b == this->eol) {
              rc = -FELPS_ERR_DROPPED_MSG;
              break;
            }
          }
        }
      }
      if ((rc == -FELPS_ERR_BAD_BUF) ||
          (rc == -FELPS_ERR_DROPPED_MSG)) {
        this->flush_buf();
      }
      if (ready != NULL) {
        *ready = found_eol;
      }

      #ifdef FELPS_TEST_BUILD
      printf("----------------------\n");
      printf("this->buf_fill: %d\n", this->buf_fill);
      printf("%s\n", this->buf);
      #endif // FELPS_TEST_BUILD

      return rc;
    }

    /**
     * @brief Attempt to process data in the buf by matching a cmd and an EOL.
     * The serial_handler_cmd_f is called with appropriate arguments if a valid
     * command is found.
     * @return the return code of the handler called, or <0 if no handler
     * @details
     * If a full command is found, the full command is removed from the buffer
     * after processing, even if there was no matching map in the dict. Any left
     * over bytes in the buffer are shifted to align with the 0 index of the buf.
     */
    int process_cmd() {
      int rc = -FELPS_ERR_UNKNOWN;
      int8_t eol_ind = this->find_eol_ind();
      int8_t cmd_end_ind = this->find_delim_ind(0);
      int8_t rem_bytes = this->buf_fill - eol_ind - 1;

      #ifdef FELPS_TEST_BUILD
      printf("----------------------\n");
      printf("eol_ind: %d\n", eol_ind);
      printf("cmd_end_ind: %d\n", cmd_end_ind);
      printf("rem_bytes: %d\n", rem_bytes);
      printf("this->buf_fill: %d\n", this->buf_fill);
      printf("%s\n", this->buf);
      #endif // FELPS_TEST_BUILD

      if ((rem_bytes < 0)) {
        rc = -FELPS_ERR_UNKNOWN;
        return rc;
      }
      if ((eol_ind < 0) || (cmd_end_ind < 0)) {
        rc = -FELPS_ERR_NOT_FOUND;
        return rc;
      }
      serial_handler_cmd_map_t * cmd_map = this->find_cmd_in_dict(this->buf, cmd_end_ind);
      if (cmd_map == NULL) {
        rc = -FELPS_ERR_NOT_IMPLEMENTED;
      } else {
        rc = (*(cmd_map->handler))(&this->buf[cmd_end_ind], eol_ind - cmd_end_ind,
                                   this->delim, cmd_map->sysargs);
      }
      // Clean the buffer of the last command and move any other bytes over
      if (rem_bytes > 0) {
        memmove(&this->buf, &this->buf[eol_ind + 1], rem_bytes);
      }
      memset(&this->buf[rem_bytes], 0x00, SERIAL_HANDLER_BUF_LENGTH - rem_bytes);
      this->buf_fill = rem_bytes;
      return rc;
    }

  private:
    HardwareSerial * serial;
    serial_handler_cmd_dictionary_t * cmd_dict;
    char eol;
    char delim;
    uint8_t buf_fill;
    char buf[SERIAL_HANDLER_BUF_LENGTH + 1];

    /**
     * @brief Find the index in the serial handler buffer of the EOL char.
     * @return the index, or -1 if the character could not be found.
     */
    int8_t find_eol_ind() {
      for (uint8_t i = 0; i < this->buf_fill; ++i) {
        if (this->buf[i] == this->eol) {
          return i;
        }
      }
      return -FELPS_ERR_NOT_FOUND;
    }

    /**
     * @brief Find the next delimeter from the provided start index. Return
     * the first EOL if one is found first (no arg cmd).
     * @param[in]   start the index to start from
     * @return int8_t the index, or -1 if the character could not be found.
     */
    int8_t find_delim_ind(uint8_t start) {
      if (start > this->buf_fill) {
        return -FELPS_ERR_BAD_PARAM;
      }
      for (uint8_t i = start; i < this->buf_fill; ++i) {
        if ((this->buf[i] == this->delim) || (this->buf[i] == this->eol)) {
          return i;
        }
      }
      return -FELPS_ERR_NOT_FOUND;
    }

    /**
     * @brief Attempts to find a cmd_map for a given cmd.
     * @param[in]   cmd_word cmd to search for
     * @param[in]   len the length of the cmd_word
     * @return pointer to cmd_map, or NULL if command not found
     */
    serial_handler_cmd_map_t * find_cmd_in_dict(const char * cmd_word, uint8_t len) {
      serial_handler_cmd_map_t * cmd_map;
      uint8_t p = 0;

      for (uint8_t i = 0; i < this->cmd_dict->num_cmds; ++i) {
        cmd_map = &this->cmd_dict->cmds[i];
        // Discard if word in map is not same length as cmd_word param
        if (strlen(cmd_map->cmd_word) != len) {
          continue;
        }
        for (p = 0; p < len; ++p) {
          if (cmd_word[p] != cmd_map->cmd_word[p]) {
            break;
          }
        }
        // Reached end of the loop, cmds match!
        if (p == len) {
          return cmd_map;
        }
      }
      return NULL;
    }
};

#endif // SERIAL_HANDLER_H_