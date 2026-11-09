/*****************************************************************************
 * | File        :   espnow_link.h
 * | Function    :   Thin wrapper around ESP-NOW used as the wireless
 * |                 transport between the LCD controller and the relay
 * |                 module. See README.md for why ESP-NOW was chosen instead
 * |                 of Bluetooth / Wi-Fi Direct / a hidden Wi-Fi network.
 ******************************************************************************/

#ifndef __ESPNOW_LINK_H
#define __ESPNOW_LINK_H

#include <stdint.h>
#include "remote_protocol.h"

namespace remote_link {

constexpr uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/** @brief Called whenever a Message is received from any peer (including broadcast). */
using MessageHandler = void (*)(const uint8_t mac[6], const Message &msg, int8_t rssi);

/**
 * @brief Put Wi-Fi in STA mode (required by ESP-NOW, no AP/router involved)
 *        and start ESP-NOW with encryption derived from `pairing_key`.
 *
 * @param pairing_key Shared passphrase. BOTH firmwares (LCD and relay) must
 *        use the exact same string - it is expanded into the ESP-NOW PMK/LMK.
 * @param wifi_channel Fixed 2.4GHz channel (1-13) used by both devices.
 * @param handler Callback invoked from the ESP-NOW receive context for every
 *        valid Message received.
 */
bool begin(const char *pairing_key, uint8_t wifi_channel, MessageHandler handler);

/** @brief Add (or refresh) an encrypted peer so send() can unicast to it. */
bool addPeer(const uint8_t mac[6]);

/** @brief Remove a previously added peer. */
void removePeer(const uint8_t mac[6]);

/** @brief Send a message to a specific, already-paired peer (encrypted). */
bool send(const uint8_t mac[6], const Message &msg);

/** @brief Send a message to the broadcast address (unencrypted, discovery only). */
bool sendBroadcast(const Message &msg);

/** @brief This device's own Wi-Fi STA MAC address. */
void getOwnMac(uint8_t mac_out[6]);

/** @brief Format a MAC address as "AA:BB:CC:DD:EE:FF" (buf must be >= 18 bytes). */
void macToStr(const uint8_t mac[6], char *buf);

/** @brief Compare two MAC addresses. */
bool macEquals(const uint8_t a[6], const uint8_t b[6]);

} // namespace remote_link

#endif // __ESPNOW_LINK_H
