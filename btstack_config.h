#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// Enable BTstack features
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

// BTstack Configuration
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_PAYLOAD_SIZE (1691 + 4)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4

// Enable Classic Bluetooth (RFCOMM/SPP)
// Guard these to avoid "redefined" warnings from CMake
#ifndef ENABLE_CLASSIC
#define ENABLE_CLASSIC
#endif

#define ENABLE_L2CAP
#define ENABLE_RFCOMM
#define ENABLE_SDP

// Memory Configuration
#define MAX_NR_HCI_CONNECTIONS 1
#define MAX_NR_L2CAP_SERVICES  2
#define MAX_NR_L2CAP_CHANNELS  2
#define MAX_NR_RFCOMM_MULTIPLEXERS 1
#define MAX_NR_RFCOMM_SERVICES 1
#define MAX_NR_RFCOMM_CHANNELS 1
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES 2
#define MAX_NR_BNEP_SERVICES 0
#define MAX_NR_BNEP_CHANNELS 0
#define MAX_NR_HFP_CONNECTIONS 0
#define MAX_NR_WHITELIST_ENTRIES 1
#define MAX_NR_SM_LOOKUP_ENTRIES 3
#define MAX_NR_SERVICE_RECORD_ITEMS 2

// --- FIXED: Missing NVM Definition ---
// Defines how many paired devices (Link Keys) to store in Flash memory.
#define NVM_NUM_LINK_KEYS 16

#endif // BTSTACK_CONFIG_H