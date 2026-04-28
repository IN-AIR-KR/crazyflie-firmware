#ifndef IDS_H
#define IDS_H

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"

static inline bool appIsValidNodeId(const uint8_t id)
{
  return ((id == NODE_ID_D1) || (id == NODE_ID_D2) || (id == NODE_ID_D3));
}

static inline uint8_t appNodeIndexFromId(const uint8_t id)
{
  if (id == NODE_ID_D1) { return 0u; }
  if (id == NODE_ID_D2) { return 1u; }
  return 2u;
}

static inline uint8_t appNodeIdFromIndex(const uint8_t idx)
{
  if (idx == 0u) { return NODE_ID_D1; }
  if (idx == 1u) { return NODE_ID_D2; }
  return NODE_ID_D3;
}

static inline uint8_t appAgentIdFromRadioLow(const uint8_t low)
{
  if (low == NODE_ID_D1) { return 1u; }
  if (low == NODE_ID_D2) { return 2u; }
  return 3u;
}

static inline const char* appNodeName(const uint8_t id)
{
  if (id == NODE_ID_D1) { return "D1"; }
  if (id == NODE_ID_D2) { return "D2"; }
  if (id == NODE_ID_D3) { return "D3"; }
  return "DX";
}

#endif
