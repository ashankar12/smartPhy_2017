#include <Arduino.h>
#include <stdint.h>

// Enumerate events
#define EVENT_NULL  (0x00000000UL)
#define EVENT_01    (0x00000001UL)
#define EVENT_02    (0x00000002UL)
#define EVENT_03    (0x00000004UL)
#define EVENT_04    (0x00000008UL)
#define EVENT_05    (0x00000010UL)
#define EVENT_06    (0x00000020UL)
#define EVENT_07    (0x00000040UL)
#define EVENT_08    (0x00000080UL)

// Preserve memory for now
/*
#define EVENT_09    (0x00000100UL)
#define EVENT_10    (0x00000200UL)
#define EVENT_11    (0x00000400UL)
#define EVENT_12    (0x00000800UL)
#define EVENT_13    (0x00001000UL)
#define EVENT_14    (0x00002000UL)
#define EVENT_15    (0x00004000UL)
#define EVENT_16    (0x00008000UL)
#define EVENT_17    (0x00010000UL)
#define EVENT_18    (0x00020000UL)
#define EVENT_19    (0x00040000UL)
#define EVENT_20    (0x00080000UL)
#define EVENT_21    (0x00100000UL)
#define EVENT_22    (0x00200000UL)
#define EVENT_23    (0x00400000UL)
#define EVENT_24    (0x00800000UL)
#define EVENT_25    (0x01000000UL)
#define EVENT_26    (0x02000000UL)
#define EVENT_27    (0x04000000UL)
#define EVENT_28    (0x08000000UL)
#define EVENT_29    (0x10000000UL)
#define EVENT_30    (0x20000000UL)
#define EVENT_31    (0x40000000UL)
#define EVENT_32    (0x80000000UL)
*/
