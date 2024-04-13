#include <string>

#include "mock_serial_handler.hpp"
#include "src/os/err.h"
#include "src/os/serial_handler.hpp"

#include "tests.h"

#define TEST_EOL '\n'
#define TEST_DELIM ' '

typedef struct {
  int a;
  int b;
} test_args_t;

static test_args_t test_args = {
  .a = 0,
  .b = 0,
};

static std::string bad_call = "test_cmd_1\n";
static std::string bad_call2 = "test_cmd_1";
static std::string too_long_call = "aRbXCjgv66hkozXNggwgAkeLpyfREqPw9QeXaj4GKSONpAwxnXi0lJKfKXjaTmCp0";
static std::string too_long_weol_call = "aRbXCjgv66hkozXNggwgAkeLpyfREqPw9QeXaj4GKSONpAwxnXi0lJKfKXjaTmCHx\n";
static std::string ping_call = "ping\n";
static std::string ping_with_exta_call = "ping\notherstuff";
static std::string test_cmd_2_call = "test_cmd_2 arg1 arg2\n";
static std::string test_cmd_2_args = "arg1 arg2";
static std::string test_with_sysargs_call = "with_sysargs\n";

#define PING_SUCCEEDED 1
#define TEST_CMD_2_SUCCEEDED 2
#define WITH_SYSARGS_SUCCEEDED 3

static int succeeded = 0;
static int test_a = 10;
static int test_b = 20;

static int ping_handler(char * buf, uint8_t len, const char delim,
                        void * sysargs) {
  succeeded = PING_SUCCEEDED;
  return FELPS_ERR_NONE;
}

static int test_cmd_2_handler(char * buf, uint8_t len, const char delim,
                              void * sysargs) {
  std::string args = "";
  // len - 1 because len includes the EOL
  for (int i = 0; i < len-1; ++i) args += buf[i];
  if (args.compare(test_cmd_2_args) != 0) {
    succeeded = TEST_CMD_2_SUCCEEDED;
    return FELPS_ERR_NONE;
  } else {
    return -FELPS_ERR_BAD_PARAM;
  }
}

static int with_args_handler(char * buf, uint8_t len, const char delim,
                             void * sysargs) {
  test_args_t * args = (test_args_t *)sysargs;
  args->a = test_a;
  args->b = test_b;
  succeeded = WITH_SYSARGS_SUCCEEDED;
  return FELPS_ERR_NONE;
}

static serial_handler_cmd_dictionary_t test_dict = {
  .num_cmds = 3,
  .cmds  {
    { "ping",         &ping_handler,        NULL        },
    { "test_cmd_2",   &test_cmd_2_handler,  NULL        },
    { "with_sysargs", &with_args_handler,   &test_args  },
  },
};

int test_serial_handler() {
  int rc = 0;
  bool ready = false;
  HardwareSerial ser = HardwareSerial();
  SerialHandler handler = SerialHandler(&ser, &test_dict, TEST_EOL, TEST_DELIM);

  // Try a command that doesn't work
  printf("---------------------------------------------\n");
  printf("%s\n", bad_call.c_str());
  ready = false;
  succeeded = 0;
  ser.load_buffer(bad_call);
  rc = handler.process_serial(&ready);
  if (rc != bad_call.length()) {
    printf("did not process entire string! returned %d, but expected %ld\n",
           rc, bad_call.length());
    return -0x001;
  }
  if (!ready) {
    printf("did not find a possible command, but one was expected\n");
    return -0x002;
  }
  rc = handler.process_cmd();
  if (rc != -FELPS_ERR_NOT_IMPLEMENTED) {
    printf("did not fail to find command, expected rc %d, but returned %d\n",
           -FELPS_ERR_NOT_IMPLEMENTED, rc);
    return -0x003;
  }

  // Try an incomplete command
  printf("---------------------------------------------\n");
  printf("%s\n", bad_call2.c_str());
  ready = false;
  succeeded = 0;
  ser.load_buffer(bad_call2);
  rc = handler.process_serial(&ready);
  if (rc != bad_call2.length()) {
    printf("did not process entire string! returned %d, but expected %ld\n",
           rc, bad_call2.length());
    return -0x101;
  }
  if (ready) {
    printf("found a possible command, but one was not expected\n");
    return -0x102;
  }
  rc = handler.flush_buf();
  if (rc != bad_call2.length()) {
    printf("did not flush buffer! expected to flush %ld but flushed %d\n",
           bad_call2.length(), rc);
    return -0x103;
  }

  // Try a command that is too long with no EOL
  printf("---------------------------------------------\n");
  printf("%s\n", too_long_call.c_str());
  ready = false;
  succeeded = 0;
  ser.load_buffer(too_long_call);
  rc = handler.process_serial(&ready);
  if (rc != -FELPS_ERR_BAD_BUF) {
    printf("did not report error! returned %d, but expected %d\n",
           rc, -FELPS_ERR_BAD_BUF);
    return -0x201;
  }
  if (ready) {
    printf("found a possible command, but one was not expected\n");
    return -0x202;
  }
  rc = handler.flush_buf();
  //if (rc != SERIAL_HANDLER_BUF_LENGTH) {
  if (rc != 0) {
    printf("did not flush buffer! expected to flush %d but flushed %d\n",
           0, rc);
           //SERIAL_HANDLER_BUF_LENGTH, rc);
    return -0x203;
  }

  // Try a command that is too long with an EOL
  printf("---------------------------------------------\n");
  printf("%s\n", too_long_call.c_str());
  ready = false;
  succeeded = 0;
  ser.load_buffer(too_long_weol_call);
  rc = handler.process_serial(&ready);
  if (rc != -FELPS_ERR_DROPPED_MSG) {
    printf("did not report error! returned %d, but expected %d\n",
           rc, -FELPS_ERR_DROPPED_MSG);
    return -0x301;
  }
  if (ready) {
    printf("found a possible command, but one was not expected\n");
    return -0x302;
  }
  rc = handler.flush_buf();
  if (rc != 0) {
    printf("flushed buffer! expected to flush %d but flushed %d\n",
           0, rc);
    return -0x303;
  }

  // Try a valid command
  printf("---------------------------------------------\n");
  printf("%s\n", ping_call.c_str());
  ready = false;
  succeeded = 0;
  ser.load_buffer(ping_call);
  rc = handler.process_serial(&ready);
  if (rc != ping_call.length()) {
    printf("did not process entire string! returned %d, but expected %d\n",
           rc, FELPS_ERR_NONE);
    return -0x401;
  }
  if (!ready) {
    printf("did not find a possible command, but one was expected\n");
    return -0x402;
  }
  rc = handler.process_cmd();
  if (rc != FELPS_ERR_NONE) {
    printf("failed to execute command! returned %d, but expected %d\n",
           rc, FELPS_ERR_NONE);
    return -0x403;
  }
  if (succeeded != PING_SUCCEEDED) {
    printf("command execution did not succeed! found %d, but expected %d!\n",
           succeeded, PING_SUCCEEDED);
    return -0x404;
  }

  // Try a different valid command
printf("---------------------------------------------\n");
  printf("%s\n", test_cmd_2_call.c_str());
  ready = false;
  succeeded = 0;
  ser.load_buffer(test_cmd_2_call);
  rc = handler.process_serial(&ready);
  if (rc != test_cmd_2_call.length()) {
    printf("did not process entire string! returned %d, but expected %d\n",
           rc, FELPS_ERR_NONE);
    return -0x501;
  }
  if (!ready) {
    printf("did not find a possible command, but one was expected\n");
    return -0x502;
  }
  rc = handler.process_cmd();
  if (rc != FELPS_ERR_NONE) {
    printf("failed to execute command! returned %d, but expected %d\n",
           rc, FELPS_ERR_NONE);
    return -0x503;
  }
  if (succeeded != TEST_CMD_2_SUCCEEDED) {
    printf("command execution did not succeed! found %d, but expected %d!\n",
           succeeded, TEST_CMD_2_SUCCEEDED);
    return -0x504;
  }

  // Try a valid command with extra stuff after
  printf("---------------------------------------------\n");
  printf("%s\n", ping_with_exta_call.c_str());
  ready = false;
  succeeded = 0;
  ser.load_buffer(ping_with_exta_call);
  rc = handler.process_serial(&ready);
  if (rc != ping_with_exta_call.length()) {
    printf("did not process entire string! returned %d, but expected %d\n",
           rc, FELPS_ERR_NONE);
    return -0x601;
  }
  if (!ready) {
    printf("did not find a possible command, but one was expected\n");
    return -0x602;
  }
  rc = handler.process_cmd();
  if (rc != FELPS_ERR_NONE) {
    printf("failed to execute command! returned %d, but expected %d\n",
           rc, FELPS_ERR_NONE);
    return -0x603;
  }
  if (succeeded != PING_SUCCEEDED) {
    printf("command execution did not succeed! found %d, but expected %d!\n",
           succeeded, PING_SUCCEEDED);
    return -0x604;
  }
  std::string newline = "\n";
  ser.load_buffer(newline);
  rc = handler.process_serial(&ready);
  if (rc != newline.length()) {
    printf("did not process entire string! returned %d, but expected %d\n",
           rc, FELPS_ERR_NONE);
    return -0x605;
  }
  if (!ready) {
    printf("did not find a possible command, but one was expected\n");
    return -0x606;
  }
  rc = handler.process_cmd();
  if (rc != -FELPS_ERR_NOT_IMPLEMENTED) {
    printf("executed command! returned %d, but expected %d\n",
           rc, -FELPS_ERR_NOT_IMPLEMENTED);
    return -0x607;
  }

  // Try a valid command with sysargs
  printf("---------------------------------------------\n");
  printf("%s\n", test_with_sysargs_call.c_str());
  ready = false;
  succeeded = 0;
  test_args.a = 0;
  test_args.b = 0;
  ser.load_buffer(test_with_sysargs_call);
  rc = handler.process_serial(&ready);
  if (rc != test_with_sysargs_call.length()) {
    printf("did not process entire string! returned %d, but expected %d\n",
           rc, FELPS_ERR_NONE);
    return -0x701;
  }
  if (!ready) {
    printf("did not find a possible command, but one was expected\n");
    return -0x702;
  }
  rc = handler.process_cmd();
  if (rc != FELPS_ERR_NONE) {
    printf("failed to execute command! returned %d, but expected %d\n",
           rc, FELPS_ERR_NONE);
    return -0x703;
  }
  if (succeeded != WITH_SYSARGS_SUCCEEDED) {
    printf("command execution did not succeed! found %d, but expected %d!\n",
           succeeded, WITH_SYSARGS_SUCCEEDED);
    return -0x704;
  }
  if ((test_args.a != test_a) || (test_args.b != test_b)) {
    printf("failed to set test args! found (%d, %d), but expected (%d, %d)\n",
           test_args.a, test_args.b, test_a, test_b);
  }

  return 0;
}