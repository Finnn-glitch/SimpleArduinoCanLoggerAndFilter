#include <SPI.h>
#include "mcp_can.h"

const uint8_t CAN_CS_PIN  = 8;
const uint8_t CAN_INT_PIN = 2;

MCP_CAN CAN(CAN_CS_PIN);

const byte CAN_SPEED = CAN_500KBPS;
const byte OSC       = MCP_8MHZ;

// REDUCED:  64 IDs (saves memory)
const size_t MAX_IDS = 64;
uint32_t seenIds[MAX_IDS];
uint16_t seenCounts[MAX_IDS];
uint16_t changeCounts[MAX_IDS];  // NEW: Track number of changes
size_t idsUsed = 0;

// Store last seen data for change detection
byte lastData[MAX_IDS][8];
byte lastDataLen[MAX_IDS];

// REDUCED: 16 filter IDs (saves 64 bytes)
const size_t MAX_FILTER_IDS = 16;
unsigned long filterIds[MAX_FILTER_IDS];
size_t filterCount = 0;
bool filterEnabled = false;
bool filterReverse = false;

// REDUCED: 48 bytes linebuf (saves 16 bytes)
char lineBuf[48];
size_t lineLen = 0;

bool showTxOutput = true;
bool showChangeOnly = false;
uint8_t frameTypeFilter = 0;

// ========== Send Task Structure ==========
struct SendTask {
  unsigned long id;
  unsigned long interval_ms;
  unsigned long last_send_ms;
  byte data[8];
  byte len;
  bool active;
  bool extended;
};

const size_t MAX_SEND_TASKS = 4;
SendTask sendTasks[MAX_SEND_TASKS];
bool canSendMode = false;

// ========== Functions ==========

inline uint32_t packId(unsigned long id, bool extended) {
  return (extended ? 0x80000000UL :   0) | (id & 0x1FFFFFFFUL);
}
inline bool isExtendedPacked(uint32_t packed) { return (packed & 0x80000000UL) != 0; }
inline unsigned long unpackId(uint32_t packed) { return packed & 0x1FFFFFFFUL; }

int16_t findSeenIndex(uint32_t packed) {
  for (size_t i = 0; i < idsUsed; i++) {
    if (seenIds[i] == packed) return (int16_t)i;
  }
  return -1;
}

// UPDATED: Store OLD data before updating, return change status
// Returns:   0=new ID, 1=no change, 2=data changed
uint8_t checkAndUpdateData(uint32_t packed, byte* newData, byte newLen, int16_t &idx, byte* oldData, byte &oldLen) {
  idx = findSeenIndex(packed);
  
  if (idx >= 0) {
    if (seenCounts[idx] != 0xFFFF) seenCounts[idx]++;
    
    // Store old data
    oldLen = lastDataLen[idx];
    for (byte i = 0; i < oldLen; i++) {
      oldData[i] = lastData[idx][i];
    }
    
    // Check if data changed
    bool changed = false;
    if (lastDataLen[idx] != newLen) {
      changed = true;
    } else {
      for (byte i = 0; i < newLen; i++) {
        if (lastData[idx][i] != newData[i]) {
          changed = true;
          break;
        }
      }
    }
    
    // NEW:  Increment change counter if data changed
    if (changed && changeCounts[idx] != 0xFFFF) {
      changeCounts[idx]++;
    }
    
    // Update stored data
    lastDataLen[idx] = newLen;
    for (byte i = 0; i < newLen; i++) {
      lastData[idx][i] = newData[i];
    }
    
    return changed ? 2 : 1;
  }
  
  // New ID
  if (idsUsed < MAX_IDS) {
    idx = idsUsed;
    seenIds[idsUsed] = packed;
    seenCounts[idsUsed] = 1;
    changeCounts[idsUsed] = 0;  // NEW: Initialize change count to 0
    lastDataLen[idsUsed] = newLen;
    for (byte i = 0; i < newLen; i++) {
      lastData[idsUsed][i] = newData[i];
    }
    idsUsed++;
    return 0; // New ID
  }
  
  return 1; // Array full
}

void printSummary() {
  Serial.print(F("\nSum: "));
  Serial.print(idsUsed);
  Serial.println(F(" IDs"));
  for (size_t i = 0; i < idsUsed; i++) {
    bool ext = isExtendedPacked(seenIds[i]);
    unsigned long id = unpackId(seenIds[i]);
    Serial.print(F("  0x"));
    Serial.print(id, HEX);
    if (ext) Serial.print(F("(E)"));
    Serial.print(F(" n: "));
    Serial.print(seenCounts[i]);
    Serial.print(F(" chg:"));  // NEW: Show change count
    Serial.print(changeCounts[i]);
    Serial.print(F(" ["));
    for (byte j = 0; j < lastDataLen[i]; j++) {
      if (lastData[i][j] < 16) Serial.print('0');
      Serial.print(lastData[i][j], HEX);
      if (j < lastDataLen[i] - 1) Serial.print(' ');
    }
    Serial.println(F("]"));
  }
  Serial.println();
}

bool isIdInList(unsigned long id) {
  for (size_t i = 0; i < filterCount; i++) {
    if (filterIds[i] == id) return true;
  }
  return false;
}

bool shouldDisplayId(unsigned long id) {
  if (!  filterEnabled) return true;
  if (filterCount == 0) return false;
  bool inList = isIdInList(id);
  return filterReverse ? !inList : inList;
}

bool shouldDisplayFrameType(bool extended) {
  if (frameTypeFilter == 0) return true;
  if (frameTypeFilter == 1) return extended;
  if (frameTypeFilter == 2) return !extended;
  return true;
}

bool addFilterId(unsigned long id) {
  for (size_t i = 0; i < filterCount; i++) {
    if (filterIds[i] == id) return false;
  }
  if (filterCount >= MAX_FILTER_IDS) return false;
  filterIds[filterCount++] = id;
  return true;
}

void clearFilter() {
  filterCount = 0;
}

void listFilter() {
  Serial.print(F("Filt "));
  Serial.print(filterEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" "));
  Serial.print(filterReverse ? F("REV") : F("NOR"));
  Serial.print(F(" ("));
  Serial.print(filterCount);
  Serial.println(F(")"));
  for (size_t i = 0; i < filterCount; i++) {
    Serial.print(F("  0x"));
    Serial.println(filterIds[i], HEX);
  }
  if (filterCount == 0 && filterEnabled) Serial.println(F("  (no output)"));
  
  Serial.print(F("Frame: "));
  if (frameTypeFilter == 0) Serial.println(F("BOTH"));
  else if (frameTypeFilter == 1) Serial.println(F("EXT"));
  else Serial.println(F("NORM"));
  
  Serial.print(F("Change: "));
  Serial.println(showChangeOnly ? F("ON") : F("OFF"));
}

static void trimInPlace(char* s) {
  while (*s == ' ' || *s == '\t') { memmove(s, s+1, strlen(s)); }
  size_t n = strlen(s);
  while (n && (s[n-1] == ' ' || s[n-1] == '\t')) { s[--n] = 0; }
}

static char* nextToken(char* &p) {
  while (*p == ' ' || *p == '\t') p++;
  if (*p == 0) return nullptr;
  char* start = p;
  while (*p && *p != ' ' && *p != '\t') p++;
  if (*p) { *p = 0; p++; }
  return start;
}

bool parseHexToken(const char* tok, unsigned long &out) {
  if (!  tok || ! *tok) return false;
  if ((tok[0] == '0') && (tok[1] == 'x' || tok[1] == 'X')) tok += 2;
  char* endp = nullptr;
  unsigned long val = strtoul(tok, &endp, 16);
  if (endp == tok) return false;
  out = val;
  return true;
}

bool parseDecToken(const char* tok, unsigned long &out) {
  if (! tok || !*tok) return false;
  char* endp = nullptr;
  unsigned long val = strtoul(tok, &endp, 10);
  if (endp == tok) return false;
  out = val;
  return true;
}

// ========== Send Task Functions ==========

void initSendTasks() {
  for (size_t i = 0; i < MAX_SEND_TASKS; i++) {
    sendTasks[i].active = false;
  }
}

void switchToSendMode() {
  if (!  canSendMode) {
    CAN.setMode(MCP_NORMAL);
    canSendMode = true;
    Serial.println(F("Mode: NORM"));
  }
}

void switchToListenMode() {
  if (canSendMode) {
    CAN.setMode(MCP_LISTENONLY);
    canSendMode = false;
    Serial. println(F("Mode: LIST"));
  }
}

int8_t findFreeSendTaskSlot() {
  for (size_t i = 0; i < MAX_SEND_TASKS; i++) {
    if (!sendTasks[i].active) return (int8_t)i;
  }
  return -1;
}

int8_t findTaskByIdIdx(unsigned long id) {
  for (size_t i = 0; i < MAX_SEND_TASKS; i++) {
    if (sendTasks[i].active && sendTasks[i].id == id) {
      return (int8_t)i;
    }
  }
  return -1;
}

bool addSendTask(unsigned long id, bool extended, unsigned long interval_ms, byte* data, byte len) {
  int8_t existingSlot = findTaskByIdIdx(id);
  int8_t slot;
  
  if (existingSlot >= 0) {
    slot = existingSlot;
    Serial.print(F("Upd #"));
    Serial.print(slot);
    Serial.print(F(" 0x"));
    Serial.println(id, HEX);
  } else {
    slot = findFreeSendTaskSlot();
    if (slot < 0) {
      Serial.println(F("Full"));
      return false;
    }
    Serial.print(F("Task #"));
    Serial.print(slot);
    Serial.print(F(" 0x"));
    Serial.print(id, HEX);
  }
  
  if (len > 8) len = 8;
  
  sendTasks[slot].id = id;
  sendTasks[slot].extended = extended;
  sendTasks[slot].interval_ms = interval_ms;
  sendTasks[slot].len = len;
  for (byte i = 0; i < len; i++) {
    sendTasks[slot].data[i] = data[i];
  }
  sendTasks[slot].  last_send_ms = millis();
  sendTasks[slot]. active = true;
  
  switchToSendMode();
  
  if (extended) Serial.print(F("(E)"));
  Serial.print(F(" @"));
  Serial.print(interval_ms);
  Serial.println(F("ms"));
  
  return true;
}

void stopSendTask(size_t index) {
  if (index >= MAX_SEND_TASKS || ! sendTasks[index].active) {
    Serial.println(F("Invalid"));
    return;
  }
  sendTasks[index].active = false;
  Serial.print(F("Stop #"));
  Serial.println(index);
  
  bool anyActive = false;
  for (size_t i = 0; i < MAX_SEND_TASKS; i++) {
    if (sendTasks[i].active) {
      anyActive = true;
      break;
    }
  }
  if (! anyActive) switchToListenMode();
}

void stopAllSendTasks() {
  for (size_t i = 0; i < MAX_SEND_TASKS; i++) {
    sendTasks[i].active = false;
  }
  Serial.println(F("Stop all"));
  switchToListenMode();
}

void listSendTasks() {
  Serial.println(F("\n=== Tasks ==="));
  bool anyActive = false;
  for (size_t i = 0; i < MAX_SEND_TASKS; i++) {
    if (sendTasks[i].  active) {
      anyActive = true;
      Serial.print(F("#"));
      Serial.print(i);
      Serial.print(F(" 0x"));
      Serial.print(sendTasks[i].id, HEX);
      if (sendTasks[i].extended) Serial.print(F("(E)"));
      Serial.print(F(" @"));
      Serial.print(sendTasks[i].interval_ms);
      Serial.print(F("ms ["));
      for (byte j = 0; j < sendTasks[i].len; j++) {
        if (sendTasks[i].data[j] < 16) Serial.print('0');
        Serial.print(sendTasks[i].data[j], HEX);
        if (j < sendTasks[i].  len - 1) Serial.print(' ');
      }
      Serial.println(F("]"));
    }
  }
  if (!anyActive) Serial.println(F("<none>"));
  Serial.print(F("TX: "));
  Serial.println(showTxOutput ? F("ON") : F("OFF"));
  Serial.println();
}

void processSendTasks() {
  unsigned long now = millis();
  for (size_t i = 0; i < MAX_SEND_TASKS; i++) {
    if (sendTasks[i].  active) {
      if (now - sendTasks[i].last_send_ms >= sendTasks[i].interval_ms) {
        byte sndStat = sendTasks[i].extended ? 
          CAN.sendMsgBuf(sendTasks[i].  id, 1, sendTasks[i].len, sendTasks[i].data) :
          CAN.sendMsgBuf(sendTasks[i]. id, 0, sendTasks[i].len, sendTasks[i].data);
        
        if (showTxOutput) {
          if (sndStat == CAN_OK) {
            Serial.print(F("[TX#"));
            Serial.print(i);
            Serial.print(F("] 0x"));
            Serial.println(sendTasks[i].id, HEX);
          } else {
            Serial.print(F("[ERR#"));
            Serial.print(i);
            Serial.print(F(":  "));
            Serial.print(sndStat);
            Serial.  println(F("]"));
          }
        }
        
        sendTasks[i].last_send_ms = now;
      }
    }
  }
}

bool sendSinglePacket(unsigned long id, bool extended, byte* data, byte len) {
  if (! canSendMode) switchToSendMode();
  
  byte sndStat = extended ? 
    CAN.sendMsgBuf(id, 1, len, data) :
    CAN.sendMsgBuf(id, 0, len, data);
  
  if (showTxOutput) {
    if (sndStat == CAN_OK) {
      Serial.print(F("[TX] 0x"));
      Serial.print(id, HEX);
      if (extended) Serial.print(F("(E)"));
      Serial.print(F(" ["));
      for (byte i = 0; i < len; i++) {
        if (data[i] < 16) Serial.print('0');
        Serial.print(data[i], HEX);
        if (i < len - 1) Serial.print(' ');
      }
      Serial.println(F("]"));
      return true;
    } else {
      Serial.print(F("[TX ERR:  "));
      Serial.print(sndStat);
      Serial.println(F("]"));
      return false;
    }
  }
  
  return (sndStat == CAN_OK);
}

// ========== Help ==========

void printHelp() {
  Serial.println(F("\n=== Log ==="));
  Serial.println(F("s  summary"));
  Serial.println(F("c  clear"));
  Serial.println(F("fe/fd  filt on/off"));
  Serial.println(F("fr  toggle rev"));
  Serial.println(F("fl  list filt"));
  Serial.println(F("fx  clear filt"));
  Serial.println(F("f/a <id>  set/add"));
  Serial.println(F("fb/fext/fnorm"));
  Serial.println(F("ce/cd  change on/off"));
  
  Serial.println(F("\n=== Send ==="));
  Serial.println(F("w <id> <data>"));
  Serial.println(F("t <id> <ms> <data>"));
  Serial.println(F("tl/ts/tx"));
  Serial.println(F("txe/txd"));
  Serial.println(F("mn/ml"));
}

// ========== Command Handler ==========

void handleCommandLine(char* line) {
  trimInPlace(line);
  if (line[0] == 0) return;

  Serial.print(F("> "));
  Serial.println(line);

  char* p = line;
  char* cmd = nextToken(p);

  if (strcmp(cmd, "s") == 0 || strcmp(cmd, "S") == 0) {
    printSummary();
    return;
  }
  if (strcmp(cmd, "c") == 0 || strcmp(cmd, "C") == 0) {
    idsUsed = 0;
    Serial.println(F("Cleared"));
    return;
  }

  if (strcasecmp(cmd, "ce") == 0) {
    showChangeOnly = true;
    Serial.println(F("Change ON"));
    return;
  }
  if (strcasecmp(cmd, "cd") == 0) {
    showChangeOnly = false;
    Serial.println(F("Change OFF"));
    return;
  }

  if (strcasecmp(cmd, "fb") == 0) {
    frameTypeFilter = 0;
    Serial.println(F("Frame: BOTH"));
    return;
  }
  if (strcasecmp(cmd, "fext") == 0) {
    frameTypeFilter = 1;
    Serial.println(F("Frame: EXT"));
    return;
  }
  if (strcasecmp(cmd, "fnorm") == 0) {
    frameTypeFilter = 2;
    Serial.println(F("Frame: NORM"));
    return;
  }

  if (strcasecmp(cmd, "fe") == 0) {
    filterEnabled = true;
    Serial.println(F("Filter ON"));
    listFilter();
    return;
  }
  if (strcasecmp(cmd, "fd") == 0) {
    filterEnabled = false;
    Serial.println(F("Filter OFF"));
    return;
  }
  
  if (strcasecmp(cmd, "fr") == 0) {
    filterReverse = !  filterReverse;
    Serial.  print(F("Mode: "));
    Serial.println(filterReverse ? F("REV") : F("NORM"));
    listFilter();
    return;
  }
  
  if (strcasecmp(cmd, "fl") == 0) {
    listFilter();
    return;
  }
  if (strcasecmp(cmd, "fx") == 0) {
    clearFilter();
    Serial.println(F("Filt clear"));
    return;
  }

  if (strcasecmp(cmd, "f") == 0) {
    char* tok = nextToken(p);
    unsigned long id;
    if (parseHexToken(tok, id)) {
      clearFilter();
      addFilterId(id);
      filterEnabled = true;
      Serial.  print(F("Filt:   0x"));
      Serial.println(id, HEX);
      return;
    }
    Serial.println(F("Bad ID"));
    return;
  }

  if (strcasecmp(cmd, "a") == 0) {
    char* tok = nextToken(p);
    unsigned long id;
    if (parseHexToken(tok, id)) {
      if (addFilterId(id)) {
        Serial.print(F("Add 0x"));
        Serial.println(id, HEX);
      } else {
        Serial. println(F("Fail"));
      }
      filterEnabled = true;
      return;
    }
    Serial.println(F("Bad ID"));
    return;
  }

  if (strcasecmp(cmd, "mn") == 0) {
    switchToSendMode();
    return;
  }
  if (strcasecmp(cmd, "ml") == 0) {
    stopAllSendTasks();
    return;
  }

  if (strcasecmp(cmd, "txe") == 0) {
    showTxOutput = true;
    Serial.println(F("TX ON"));
    return;
  }
  if (strcasecmp(cmd, "txd") == 0) {
    showTxOutput = false;
    Serial.println(F("TX OFF"));
    return;
  }

  if (strcasecmp(cmd, "w") == 0) {
    char* idTok = nextToken(p);
    unsigned long id;
    if (!  parseHexToken(idTok, id)) {
      Serial.println(F("Bad ID"));
      return;
    }
    
    byte data[8];
    byte len = 0;
    char* dataTok;
    while ((dataTok = nextToken(p)) != nullptr && len < 8) {
      unsigned long byteVal;
      if (parseHexToken(dataTok, byteVal)) {
        data[len++] = (byte)byteVal;
      }
    }
    
    if (len == 0) {
      Serial.println(F("No data"));
      return;
    }
    
    sendSinglePacket(id, false, data, len);
    return;
  }

  if (strcasecmp(cmd, "t") == 0) {
    char* idTok = nextToken(p);
    char* intervalTok = nextToken(p);
    
    unsigned long id, interval;
    if (! parseHexToken(idTok, id) || !parseDecToken(intervalTok, interval)) {
      Serial.println(F("Bad args"));
      return;
    }
    
    byte data[8];
    byte len = 0;
    char* dataTok;
    while ((dataTok = nextToken(p)) != nullptr && len < 8) {
      unsigned long byteVal;
      if (parseHexToken(dataTok, byteVal)) {
        data[len++] = (byte)byteVal;
      }
    }
    
    if (len == 0) {
      Serial.println(F("No data"));
      return;
    }
    
    addSendTask(id, false, interval, data, len);
    return;
  }

  if (strcasecmp(cmd, "tl") == 0) {
    listSendTasks();
    return;
  }

  if (strcasecmp(cmd, "ts") == 0) {
    char* indexTok = nextToken(p);
    unsigned long index;
    if (!parseDecToken(indexTok, index)) {
      Serial.println(F("Bad idx"));
      return;
    }
    stopSendTask((size_t)index);
    return;
  }

  if (strcasecmp(cmd, "tx") == 0) {
    stopAllSendTasks();
    return;
  }

  printHelp();
}

// ========== Setup ==========

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(CAN_INT_PIN, INPUT);
  initSendTasks();

  if (CAN.  begin(MCP_ANY, CAN_SPEED, OSC) != CAN_OK) {
    Serial.println(F("CAN fail"));
    while (1) { delay(1000); }
  }

  CAN.setMode(MCP_LISTENONLY);
  Serial.println(F("CAN Ready"));
  printHelp();
  listFilter();
}

// ========== Loop ==========

void loop() {
  // Serial commands
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        handleCommandLine(lineBuf);
        lineLen = 0;
      }
    } else if (ch == '\n') {
      lineBuf[lineLen] = 0;
      handleCommandLine(lineBuf);
      lineLen = 0;
    } else {
      if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = ch;
      } else {
        lineBuf[lineLen] = 0;
        handleCommandLine(lineBuf);
        lineLen = 0;
      }
    }
  }

  // Process send tasks
  processSendTasks();

  // Receive CAN
  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long id = 0;
    byte ext = 0;
    byte len = 0;
    byte buf[8];
    if (CAN.readMsgBuf(&id, &ext, &len, buf) != CAN_OK) continue;
    
    bool extended = (ext != 0);
    uint32_t packed = packId(id, extended);
    
    // Check and update data, store old data
    byte oldData[8];
    byte oldLen;
    int16_t idx;
    uint8_t changeStatus = checkAndUpdateData(packed, buf, len, idx, oldData, oldLen);
    
    // Check if should display based on filters
    bool passesFilters = shouldDisplayId(id) && shouldDisplayFrameType(extended);
    
    // If change detection is ON, only show if data changed
    if (showChangeOnly && changeStatus != 2) {
      continue;  // Skip if not changed (status 0=new, 1=no change, 2=changed)
    }
    
    if (passesFilters) {
      // Show change indicator and old->new
      if (showChangeOnly && changeStatus == 2) {
        Serial.print(F("[CHG] 0x"));
        Serial.print(id, HEX);
        if (extended) Serial.print(F("(E)"));
        Serial.print(F(" ["));
        
        // Show OLD data
        for (byte i = 0; i < oldLen; i++) {
          if (oldData[i] < 16) Serial.print('0');
          Serial.print(oldData[i], HEX);
          if (i < oldLen - 1) Serial.print(' ');
        }
        
        Serial.print(F("]->["));
        
        // Show NEW data
        for (byte i = 0; i < len; i++) {
          if (buf[i] < 16) Serial.print('0');
          Serial.print(buf[i], HEX);
          if (i < len - 1) Serial.print(' ');
        }
        Serial.println(F("]"));
      } else {
        // Normal display
        Serial.print(millis());
        Serial.print(F("ms 0x"));
        Serial.print(id, HEX);
        if (extended) Serial.print(F("(E)"));
        Serial.print(F(" ["));
        for (byte i = 0; i < len; i++) {
          if (buf[i] < 16) Serial.print('0');
          Serial.print(buf[i], HEX);
          if (i < len - 1) Serial.print(' ');
        }
        Serial.println(F("]"));
      }
    }
  }
}
