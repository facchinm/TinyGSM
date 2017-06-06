/**
 * @file       TinyGsmClientSIM800.h
 * @author     Volodymyr Shymanskyy
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
 * @date       Nov 2016
 */

#ifndef TinyGsmClientU201_h
#define TinyGsmClientU201_h

//#define TINY_GSM_DEBUG Serial
//#define TINY_GSM_USE_HEX

#if !defined(TINY_GSM_RX_BUFFER)
  #define TINY_GSM_RX_BUFFER 64
#endif

#include <TinyGsmCommon.h>

#define GSM_NL "\r\n"
static const char GSM_OK[] TINY_GSM_PROGMEM = "OK" GSM_NL;
static const char GSM_ERROR[] TINY_GSM_PROGMEM = "ERROR" GSM_NL;

enum SimStatus {
  SIM_ERROR = 0,
  SIM_READY = 1,
  SIM_LOCKED = 2,
};

enum RegStatus {
  REG_UNREGISTERED = 0,
  REG_SEARCHING    = 2,
  REG_DENIED       = 3,
  REG_OK_HOME      = 1,
  REG_OK_ROAMING   = 5,
  REG_UNKNOWN      = 4,
};


class TinyGsm
{

public:
  TinyGsm(__attribute__((unused)) Stream& stream = (Stream&)Serial)
#ifdef ARDUINO_GSM_COMPATIBILITY_WRAPPER
    : stream(GSM_DEFAULT_STREAM)
#else
    : stream(stream)
#endif
  {
  }

public:

class GsmClient : public Client
{
  friend class TinyGsm;
  typedef TinyGsmFifo<uint8_t, TINY_GSM_RX_BUFFER> RxFifo;

public:
  GsmClient() {}

  GsmClient(TinyGsm& modem, uint8_t mux = 1) {
    init(&modem, mux);
  }

  bool init(TinyGsm* modem, uint8_t mux = 1) {
    this->at = modem;
    this->mux = mux;
    sock_available = 0;
    sock_connected = false;

    return true;
  }

public:
  virtual int connect(const char *host, uint16_t port) {
    TINY_GSM_YIELD();
    rx.clear();
    sock_connected = at->modemConnect(host, port, &mux);
    at->sockets[mux] = this;
    return sock_connected;
  }

  virtual int connect(IPAddress ip, uint16_t port) {
    String host; host.reserve(16);
    host += ip[0];
    host += ".";
    host += ip[1];
    host += ".";
    host += ip[2];
    host += ".";
    host += ip[3];
    return connect(host.c_str(), port);
  }

  virtual void stop() {
    TINY_GSM_YIELD();
    at->sendAT(GF("+USOCL="), mux);
    sock_connected = false;
    at->waitResponse();
  }

  virtual size_t write(const uint8_t *buf, size_t size) {
    TINY_GSM_YIELD();
    at->maintain();
    return at->modemSend(buf, size, mux);
  }

  virtual size_t write(uint8_t c) {
    return write(&c, 1);
  }

  virtual int available() {
    TINY_GSM_YIELD();
    if (!rx.size()) {
      at->maintain();
    }
    //DebugSerial.println("available: " + String(rx.size()));
    //DebugSerial.println("available: " + sock_available);
    return rx.size() + sock_available;
  }

  virtual int read(uint8_t *buf, size_t size) {
    TINY_GSM_YIELD();
    at->maintain();
    size_t cnt = 0;
    while (cnt < size) {
      size_t chunk = TinyGsmMin(size-cnt, rx.size());
      if (chunk > 0) {
        rx.get(buf, chunk);
        buf += chunk;
        cnt += chunk;
        continue;
      }
      // TODO: Read directly into user buffer?
      at->maintain();
      if (sock_available > 0) {
        at->modemRead(rx.free(), mux);
      } else {
        break;
      }
    }
    return cnt;
  }

  virtual int read() {
    uint8_t c;
    if (read(&c, 1) == 1) {
      return c;
    }
    return -1;
  }

  virtual int peek() { return -1; } //TODO
  virtual void flush() { at->stream.flush(); }

  virtual uint8_t connected() {
    if (available()) {
      return true;
    }
    return sock_connected;
  }
  virtual operator bool() { return connected(); }
private:
  TinyGsm*      at;
  uint8_t       mux;
  uint16_t      sock_available;
  bool          sock_connected;
  RxFifo        rx;
};

class GsmSSLClient : public GsmClient
{
  friend class TinyGsm;
  typedef TinyGsmFifo<uint8_t, TINY_GSM_RX_BUFFER> RxFifo;

public:
  GsmSSLClient() {}

  GsmSSLClient(TinyGsm& modem, uint8_t mux = 1) {
    init(&modem, mux);
  }

  virtual int connect(const char *host, uint16_t port) {
    TINY_GSM_YIELD();
    rx.clear();
    sock_connected = at->modemSSLConnect(host, port, &mux);
    at->sockets[mux] = this;
    return sock_connected;
  }

  virtual int connect(IPAddress ip, uint16_t port) {
    String host; host.reserve(16);
    host += ip[0];
    host += ".";
    host += ip[1];
    host += ".";
    host += ip[2];
    host += ".";
    host += ip[3];
    return connect(host.c_str(), port);
  }
};

public:

  /*
   * Basic functions
   */
  bool begin(const char* pin = NULL) {
    return init(pin);
  }

  bool init(const char* pin = NULL) {
    if (!autoBaud()) {
      return false;
    }
    sendAT(GF("E0"));  // Echo Off
    if (waitResponse() != 1) {
      return false;
    }
    int ret = getSimStatus();
    if (ret != SIM_READY && pin != NULL && strlen(pin) > 0) {
      simUnlock(pin);
    }
    return (getSimStatus() == SIM_READY);
  }

  void setBaud(unsigned long baud) {
    // set baud rate to 921600
    sendAT(GF("+IPR="), baud);
  }

  bool autoBaud(unsigned long timeout = 10000L) {
    for (unsigned long start = millis(); millis() - start < timeout; ) {
      sendAT(GF(""));
      if (waitResponse(200) == 1) {
          delay(100);
          return true;
      }
      delay(100);
#ifdef ARDUINO_GSM_COMPATIBILITY_WRAPPER
      changeBaudRate();
#endif
    }
    return false;
  }

  void maintain() {
    while (stream.available()) {
      waitResponse(100);
    }
  }

  bool factoryDefault() {
    sendAT(GF("+UFACTORY=0,1"));  // Factory + Reset + Echo Off
    waitResponse();
    sendAT(GF("+CFUN=16"));   // Auto-baud
    return waitResponse() == 1;
  }

  /*
   * Power functions
   */

  bool restart() {
    if (!autoBaud()) {
      return false;
    }
    sendAT(GF("+CFUN=16"));
    if (waitResponse(10000L) != 1) {
      return false;
    }
    delay(3000);
    return init();
  }

  /*
   * SIM card & Networ Operator functions
   */

  bool simUnlock(const char *pin) {
    sendAT(GF("+CPIN=\""), pin, GF("\""));
    return waitResponse() == 1;
  }

  String getSimCCID() {
    sendAT(GF("+CCID"));
    if (waitResponse(GF(GSM_NL "+CCID:")) != 1) {
      return "";
    }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  String getIMEI() {
    sendAT(GF("+CGSN"));
    if (waitResponse(GF(GSM_NL)) != 1) {
      return "";
    }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  int getSignalQuality() {
    sendAT(GF("+CSQ"));
    if (waitResponse(GF(GSM_NL "+CSQ:")) != 1) {
      return 99;
    }
    int res = stream.readStringUntil(',').toInt();
    waitResponse();
    return res;
  }

  String getGsmLocation() {
    sendAT(GF("+ULOC=2,3,0,120,1"));
    if (waitResponse(GF(GSM_NL "+UULOC:")) != 1) {
      return "";
    }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  bool setGsmBusy(bool busy = true) {
    sendAT(GF("+GSMBUSY="), busy ? 1 : 0);
    return waitResponse() == 1;
  }

  bool callAnswer() {
    sendAT(GF("A"));
    return waitResponse() == 1;
  }

  bool callNumber(const String& number) {
    sendAT(GF("D"), number);
    return waitResponse() == 1;
  }

  bool callHangup(const String& number) {
    sendAT(GF("H"), number);
    return waitResponse() == 1;
  }

  bool sendSMS(const String& number, const String& text) {
    sendAT(GF("+CMGF=1"));
    waitResponse();
    sendAT(GF("+CMGS=\""), number, GF("\""));
    if (waitResponse(GF(">")) != 1) {
      return false;
    }
    stream.print(text);
    stream.write((char)0x1A);
    return waitResponse(60000L) == 1;
  }

  bool sendSMS_UTF16(const String& number, const void* text, size_t len) {
    sendAT(GF("+CMGF=1"));
    waitResponse();
    sendAT(GF("+CSCS=\"HEX\""));
    waitResponse();
    sendAT(GF("+CSMP=17,167,0,8"));
    waitResponse();

    sendAT(GF("+CMGS=\""), number, GF("\""));
    if (waitResponse(GF(">")) != 1) {
      return false;
    }

    uint16_t* t = (uint16_t*)text;
    for (size_t i=0; i<len; i++) {
      uint8_t c = t[i] >> 8;
      if (c < 0x10) { stream.print('0'); }
      stream.print(c, HEX);
      c = t[i] & 0xFF;
      if (c < 0x10) { stream.print('0'); }
      stream.print(c, HEX);
    }
    stream.write((char)0x1A);
    return waitResponse(60000L) == 1;
  }

  SimStatus getSimStatus(unsigned long timeout = 10000L) {
    for (unsigned long start = millis(); millis() - start < timeout; ) {
      sendAT(GF("+CPIN?"));
      if (waitResponse(GF(GSM_NL "+CPIN:")) != 1) {
        delay(1000);
        continue;
      }
      int status = waitResponse(GF("READY"), GF("SIM PIN"), GF("SIM PUK"), GF("NOT INSERTED"));
      waitResponse();
      switch (status) {
      case 2:
      case 3:  return SIM_LOCKED;
      case 1:  return SIM_READY;
      default: return SIM_ERROR;
      }
    }
    return SIM_ERROR;
  }

  RegStatus getRegistrationStatus() {
    sendAT(GF("+CGREG?"));
    if (waitResponse(GF(GSM_NL "+CGREG:")) != 1) {
      return REG_UNKNOWN;
    }
    streamSkipUntil(','); // Skip format (0)
    int status = stream.readStringUntil('\n').toInt();
    waitResponse();
    return (RegStatus)status;
  }

  String getOperator() {
    sendAT(GF("+COPS?"));
    if (waitResponse(GF(GSM_NL "+COPS:")) != 1) {
      return "";
    }
    streamSkipUntil('"'); // Skip mode and format
    String res = stream.readStringUntil('"');
    waitResponse();
    return res;
  }

  bool waitForNetwork(unsigned long timeout = 60000L) {
    for (unsigned long start = millis(); millis() - start < timeout; ) {
      RegStatus s = getRegistrationStatus();
      if (s == REG_OK_HOME || s == REG_OK_ROAMING) {
        return true;
      }
      delay(1000);
    }
    return false;
  }

  /*
   * GPRS functions
   */
  bool gprsConnect(const char* apn, const char* user, const char* pwd) {
    gprsDisconnect();

    sendAT(GF("+CGATT=1"));
    waitResponse(5000L);

    sendAT(GF("+UPSD=0,1,\""), apn, '"');
    waitResponse();

    if (strlen(user) > 0) {
      sendAT(GF("+UPSD=0,2,\""), user, '"');
      waitResponse();
    }
    if (strlen(pwd) > 0) {
      sendAT(GF("+UPSD=0,3,\""), pwd, '"');
      waitResponse();
    }

    sendAT(GF("+UPSD=0,7,\"0.0.0.0\"")); //dynamic IP
    waitResponse();

    sendAT(GF("+UPSDA=0,3"));
    waitResponse(6000L);

    // Open a GPRS context
    sendAT(GF("+UPSND=0,8"));
    if (waitResponse(GF(",8,1")) != 1) {
      return false;
    }
    return true;
  }

  bool gprsDisconnect() {
    sendAT(GF("+UPSDA=0,4"));
    waitResponse(60000L);
    sendAT(GF("+CGATT=0"));
    return waitResponse(60000L) == 1;
  }

  /*
   * Phone Call functions
   */

  /*
   * Messaging functions
   */

  void sendUSSD() {
  }

  /*
   * Location functions
   */
  void getLocation() {
  }

  /*
   * Battery functions
   */
  // Use: float vBatt = modem.getBattVoltage() / 1000.0;
  uint16_t getBattVoltage() {
    sendAT(GF("+CIND"));
    if (waitResponse(GF(GSM_NL "+CIND:")) != 1) {
      return 0;
    }

    uint16_t res = stream.readStringUntil(',').toInt();
    waitResponse();
    return res;
  }

private:
  int modemConnect(const char* host, uint16_t port, uint8_t* mux) {

    // create TCP socket
    sendAT(GF("+USOCR=6"));
    if (waitResponse(GF(GSM_NL "+USOCR:")) != 1) {
      return -1;
    }
    *mux = stream.readStringUntil('\n').toInt();
    waitResponse();

    sendAT(GF("+USOCO="), *mux, ",\"", host, "\",", port);
    int rsp = waitResponse(75000L,
                  GF("OK" GSM_NL),
                  GF("ERROR" GSM_NL));

    return (1 == rsp);
  }

  int modemSSLConnect(const char* host, uint16_t port, uint8_t* mux) {

    // create TCP socket
    sendAT(GF("+USOCR=6"));
    if (waitResponse(GF(GSM_NL "+USOCR:")) != 1) {
      return -1;
    }
    *mux = stream.readStringUntil('\n').toInt();
    waitResponse();

    sendAT(GF("+USOSEC="), *mux, ",1");
    waitResponse();

    sendAT(GF("+USOCO="), *mux, ",\"", host, "\",", port);
    int rsp = waitResponse(75000L,
                  GF("OK" GSM_NL),
                  GF("ERROR" GSM_NL));

    return (1 == rsp);
  }

  int modemSend(const void* buff, size_t len, uint8_t mux) {
    sendAT(GF("+USOWR="), mux, ',', len);
    if (waitResponse(GF("@")) != 1) {
      return -1;
    }
    // at least 50mSeconds delay, see AT manual section 25.10.4
    delay(50);
    stream.write((uint8_t*)buff, len);
    if (waitResponse(GF(GSM_NL "+USOWR:")) != 1) {
      return -1;
    }
    streamSkipUntil(','); // Skip mux
    return stream.readStringUntil('\n').toInt();
  }

  size_t modemRead(size_t size, uint8_t mux) {
    sendAT(GF("+USORD="), mux, ',', size);
    if (waitResponse(GF(GSM_NL "+USORD:")) != 1) {
      return 0;
    }
    streamSkipUntil(','); // Skip mux
    size_t len = stream.readStringUntil(',').toInt();
    streamSkipUntil('\"');
    //sockets[mux]->sock_available = stream.readStringUntil('\n').toInt();

    for (size_t i=0; i<len; i++) {
#ifdef TINY_GSM_USE_HEX
      while (stream.available() < 2) {}
      char buf[4] = { 0, };
      buf[0] = stream.read();
      buf[1] = stream.read();
      char c = strtol(buf, NULL, 16);
#else
      while (!stream.available()) {}
      char c = stream.read();
#endif
      sockets[mux]->rx.put(c);
    }
    streamSkipUntil('\"');
    waitResponse();
    return len;
  }

  size_t modemGetAvailable(uint8_t mux) {
    sendAT(GF("+USORD="), mux, ',', 0);
    size_t result = 0;
    if (waitResponse(1000L, GF(GSM_NL "+USORD:")) == 1) {
      streamSkipUntil(','); // Skip mux
      result = stream.readStringUntil('\n').toInt();
      waitResponse();
    }
    if (!result) {
      sockets[mux]->sock_connected = modemGetConnected(mux);
    }
    return result;
  }

  bool modemGetConnected(uint8_t mux) {
    sendAT(GF("+USOCTL="), mux, ",10");
    if (waitResponse(GF(GSM_NL "+USOCTL:")) == 1) {
      streamSkipUntil(','); // Skip mux
      streamSkipUntil(','); // Skip type
      size_t result = stream.readStringUntil('\n').toInt();
      return result != 0;
    }
    return false;
  }

  /* Utilities */
  template<typename T>
  void streamWrite(T last) {
    stream.print(last);
  }

  template<typename T, typename... Args>
  void streamWrite(T head, Args... tail) {
    stream.print(head);
    streamWrite(tail...);
  }

  int streamRead() { return stream.read(); }

  bool streamSkipUntil(char c) { //TODO: timeout
    while (true) {
      while (!stream.available()) {}
      if (stream.read() == c)
        return true;
    }
    return false;
  }

  template<typename... Args>
  void sendAT(Args... cmd) {
    streamWrite("AT", cmd..., GSM_NL);
    stream.flush();
    TINY_GSM_YIELD();
    //DBG("### AT:", cmd...);
  }

  // TODO: Optimize this!
  uint8_t waitResponse(uint32_t timeout, String& data,
                       GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                       GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
  {
    /*String r1s(r1); r1s.trim();
    String r2s(r2); r2s.trim();
    String r3s(r3); r3s.trim();
    String r4s(r4); r4s.trim();
    String r5s(r5); r5s.trim();
    DBG("### ..:", r1s, ",", r2s, ",", r3s, ",", r4s, ",", r5s);*/
    data.reserve(64);
    bool gotData = false;
    int mux = -1;
    int index = 0;
    unsigned long startMillis = millis();
    do {
      TINY_GSM_YIELD();
      while (stream.available() > 0) {
        int a = streamRead();
        if (a < 0) continue;
        data += (char)a;
        if (r1 && data.endsWith(r1)) {
          index = 1;
          goto finish;
        } else if (r2 && data.endsWith(r2)) {
          index = 2;
          goto finish;
        } else if (r3 && data.endsWith(r3)) {
          index = 3;
          goto finish;
        } else if (r4 && data.endsWith(r4)) {
          index = 4;
          goto finish;
        } else if (r5 && data.endsWith(r5)) {
          index = 5;
          goto finish;
        } else if (data.endsWith("+UUSORD:")) {
          mux = stream.readStringUntil(',').toInt();
          gotData = true;
          data = "";
        } else if (data.endsWith("+UUSOCL:")) {
          mux = stream.readStringUntil('\n').toInt();
          gotData = false;
          if (sockets[mux] != NULL) {
            sockets[mux]->sock_connected = false;
            sockets[mux]->sock_available = 0;
          }
        }
      }
    } while (millis() - startMillis < timeout);
finish:
    if (!index) {
      data.trim();
      if (data.length()) {
        DBG("### Unhandled:", data);
      }
      data = "";
    }
    if (gotData) {
      sockets[mux]->sock_available = modemGetAvailable(mux);
    }
    return index;
  }

  uint8_t waitResponse(uint32_t timeout,
                       GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                       GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
  {
    String data;
    return waitResponse(timeout, data, r1, r2, r3, r4, r5);
  }

  uint8_t waitResponse(GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                       GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
  {
    return waitResponse(1000, r1, r2, r3, r4, r5);
  }

private:
  Stream&       stream;
  GsmClient*    sockets[5];
};

typedef TinyGsm::GsmClient TinyGsmClient;
typedef TinyGsm::GsmSSLClient TinyGsmSSLClient;

#ifdef ARDUINO_GSM_COMPATIBILITY_WRAPPER

typedef TinyGsmClient GSMClient;
typedef TinyGsmSSLClient GSMSSLClient;
typedef TinyGsm GSM;

#define GPRS      "#Please use only one GSM object"
#define GSM_SMS   "#Please use only one GSM object"

#endif

#endif
