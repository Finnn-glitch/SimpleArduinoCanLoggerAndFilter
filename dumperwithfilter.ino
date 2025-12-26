#include <SPI.h>
#include "mcp_can.h"

const uint8_t CAN_CS_PIN  = 8;
const uint8_t CAN_INT_PIN = 2;

MCP_CAN CAN(CAN_CS_PIN);

const byte CAN_SPEED = CAN_500KBPS;
const byte OSC       = MCP_8MHZ;

const size_t MAX_IDS = 256;
uint32_t seenIds[MAX_IDS];
uint16_t seenCounts[MAX_IDS];
size_t idsUsed = 0;

const size_t MAX_FILTER_IDS = 16;
unsigned long filterIds[MAX_FILTER_IDS];
size_t filterCount = 0;
bool filterEnabled = false;   // set true to enable filtering at startup

char lineBuf[64];
size_t lineLen = 0;

inline uint32_t packId(unsigned long id, bool extended) {
  return (extended ? 0x80000000UL : 0) | (id & 0x1FFFFFFFUL);
}
inline bool isExtendedPacked(uint32_t packed) { return (packed & 0x80000000UL) != 0; }
inline unsigned long unpackId(uint32_t packed) { return packed & 0x1FFFFFFFUL; }

int16_t findSeenIndex(uint32_t packed) {
  for (size_t i = 0; i < idsUsed; i++) {
    if (seenIds[i] == packed) return (int16_t)i;
  }
  return -1;
}

bool rememberId(uint32_t packed) {
  int16_t idx = findSeenIndex(packed);
  if (idx >= 0) {
    if (seenCounts[idx] != 0xFFFF) seenCounts[idx]++;
    return false;
  }
  if (idsUsed < MAX_IDS) {
    seenIds[idsUsed] = packed;
    seenCounts[idsUsed] = 1;
    idsUsed++;
  }
  return true;
}

void printSummary() {
  Serial.print(F("\nSummary: "));
  Serial.print(idsUsed);
  Serial.println(F(" unique IDs seen."));
  for (size_t i = 0; i < idsUsed; i++) {
    bool ext = isExtendedPacked(seenIds[i]);
    unsigned long id = unpackId(seenIds[i]);
    Serial.print(F("  ID:0x"));
    Serial.print(id, HEX);
    if (ext) Serial.print(F(" (ext)"));
    Serial.print(F(" count:"));
    Serial.println(seenCounts[i]);
  }
  Serial.println();
}

bool idInFilter(unsigned long id) {
  if (!filterEnabled || filterCount == 0) return true;
  for (size_t i = 0; i < filterCount; i++) {
    if (filterIds[i] == id) return true;
  }
  return false;
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
  Serial.print(F("Filter "));
  Serial.print(filterEnabled ? F("ENABLED") : F("DISABLED"));
  Serial.print(F(". IDs ("));
  Serial.print(filterCount);
  Serial.println(F("):"));
  for (size_t i = 0; i < filterCount; i++) {
    Serial.print(F("  0x"));
    Serial.println(filterIds[i], HEX);
  }
  if (filterCount == 0) Serial.println(F("  <none>"));
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
  if (!tok || !*tok) return false;
  // optional 0x/0X
  if ((tok[0] == '0') && (tok[1] == 'x' || tok[1] == 'X')) tok += 2;
  char* endp = nullptr;
  unsigned long val = strtoul(tok, &endp, 16);
  if (endp == tok) return false;
  out = val;
  return true;
}

void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  s            -> summary of all IDs seen"));
  Serial.println(F("  c            -> clear remembered IDs"));
  Serial.println(F("  fe           -> enable show-only filter"));
  Serial.println(F("  fd           -> disable show-only filter"));
  Serial.println(F("  fl           -> list filter IDs"));
  Serial.println(F("  fx           -> clear filter IDs"));
  Serial.println(F("  f <hex>      -> set single filter to <hex> (replaces list)"));
  Serial.println(F("  a <hex>      -> add <hex> to filter list"));
  Serial.println(F("Examples: f 100 (only show 0x100), a 203 (also show 0x203), fe"));
}

void handleCommandLine(char* line) {
  trimInPlace(line);
  if (line[0] == 0) return;

  Serial.print(F("> "));
  Serial.println(line);

  char* p = line;
  char* cmd = nextToken(p);

  // Single-letter quick commands
  if (strcmp(cmd, "s") == 0 || strcmp(cmd, "S") == 0) {
    printSummary();
    return;
  }
  if (strcmp(cmd, "c") == 0 || strcmp(cmd, "C") == 0) {
    idsUsed = 0;
    Serial.println(F("Cleared remembered IDs."));
    return;
  }

  if (strcasecmp(cmd, "fe") == 0) {
    filterEnabled = true;
    Serial.println(F("Filter ENABLED."));
    listFilter();
    return;
  }
  if (strcasecmp(cmd, "fd") == 0) {
    filterEnabled = false;
    Serial.println(F("Filter DISABLED. All IDs will be shown."));
    return;
  }
  if (strcasecmp(cmd, "fl") == 0) {
    listFilter();
    return;
  }
  if (strcasecmp(cmd, "fx") == 0) {
    clearFilter();
    Serial.println(F("Filter list cleared."));
    return;
  }

  // Commands with arguments
  if (strcasecmp(cmd, "f") == 0) {
    // set single filter ID
    char* tok = nextToken(p);
    unsigned long id;
    if (parseHexToken(tok, id)) {
      clearFilter();
      addFilterId(id);
      filterEnabled = true;
      Serial.print(F("Filter set to only show ID 0x"));
      Serial.println(id, HEX);
      return;
    }
    Serial.println(F("Invalid hex after 'f'. Example: f 100"));
    return;
  }

  if (strcasecmp(cmd, "a") == 0) {
    char* tok = nextToken(p);
    unsigned long id;
    if (parseHexToken(tok, id)) {
      if (addFilterId(id)) {
        Serial.print(F("Added 0x"));
        Serial.print(id, HEX);
        Serial.println(F(" to filter list."));
      } else {
        Serial.println(F("Failed to add (duplicate or list full)."));
      }
      filterEnabled = true;
      return;
    }
    Serial.println(F("Invalid hex after 'a'. Example: a 203"));
    return;
  }
  printHelp();
}

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(CAN_INT_PIN, INPUT);

  if (CAN.begin(MCP_ANY, CAN_SPEED, OSC) != CAN_OK) {
    Serial.println(F("CAN init failed. Check wiring, bitrate, and 8 MHz oscillator."));
    while (1) { delay(1000); }
  }

  CAN.setMode(MCP_LISTENONLY); // sniffer mode
  Serial.println(F("CAN listen-only started."));
  printHelp();
  listFilter();
}

void loop() {
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

  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long id = 0;
    byte ext = 0;
    byte len = 0;
    byte buf[8];
    if (CAN.readMsgBuf(&id, &ext, &len, buf) != CAN_OK) {
      continue;
    }
    bool extended = (ext != 0);

    uint32_t packed = packId(id, extended);
    bool isNew = rememberId(packed);

    if (!idInFilter(id)) {
      continue;
    }

    // Dump the frame
    Serial.print(millis());
    Serial.print(F("ms "));
    Serial.print(F("ID:0x"));
    Serial.print(id, HEX);
    if (extended) Serial.print(F(" (ext)"));
    Serial.print(F(" DLC:"));
    Serial.print(len);
    Serial.print(F(" DATA:"));
    for (byte i = 0; i < len; i++) {
      if (buf[i] < 16) Serial.print('0');
      Serial.print(buf[i], HEX);
      Serial.print(' ');
    }
    Serial.println();

    // Optional: notify when an ID is seen for the first time
    if (isNew) {
      //Serial.print(F("  New ID observed -> 0x"));
      //Serial.print(id, HEX);
      //if (extended) Serial.print(F(" (ext)"));
      //Serial.print(F(" | total unique: "));
      //Serial.println(idsUsed);
    }
  }
}
