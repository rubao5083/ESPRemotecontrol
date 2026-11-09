/*****************************************************************************
 * | File        :   espnow_link.cpp
 * | Function    :   ESP-NOW transport implementation (see espnow_link.h).
 ******************************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>
#include <stdio.h>
#include "espnow_link.h"

namespace remote_link {

namespace {

MessageHandler s_handler = nullptr;
uint8_t s_pmk[16];
uint8_t s_lmk[16];

// Deterministic (not cryptographically strong) expansion of the shared
// pairing passphrase into a 16-byte key. Both firmwares must use the same
// `pairing_key` string so they derive the same key and can talk to each other.
void deriveKey(const char *pass, uint8_t out[16])
{
    size_t len = strlen(pass);
    for (int i = 0; i < 16; i++) {
        uint8_t v = (uint8_t)(i * 31 + 7);
        if (len > 0) {
            v ^= (uint8_t)pass[i % len];
        }
        out[i] = v;
    }
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (!s_handler || !info || !data) {
        return;
    }
    if (len != sizeof(Message)) {
        return; // ignore malformed or foreign ESP-NOW traffic
    }
    Message msg;
    memcpy(&msg, data, sizeof(Message));
    if (msg.magic != kMagic || msg.version != kProtocolVersion) {
        return;
    }
    int8_t rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
    s_handler(info->src_addr, msg, rssi);
}

} // namespace

bool begin(const char *pairing_key, uint8_t wifi_channel, MessageHandler handler)
{
    s_handler = handler;
    deriveKey(pairing_key, s_pmk);
    memcpy(s_lmk, s_pmk, sizeof(s_lmk));

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        return false;
    }
    esp_now_register_recv_cb(onDataRecv);
    esp_now_set_pmk(s_pmk);

    // The broadcast address must be registered as a (unencrypted) peer
    // before sendBroadcast() can be used - needed for discovery/pairing.
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, kBroadcastMac, 6);
    peer.channel = wifi_channel;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;
    if (!esp_now_is_peer_exist(peer.peer_addr)) {
        esp_now_add_peer(&peer);
    }

    return true;
}

bool addPeer(const uint8_t mac[6])
{
    if (esp_now_is_peer_exist(mac)) {
        esp_now_del_peer(mac); // refresh in case it was registered differently before
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    memcpy(peer.lmk, s_lmk, sizeof(s_lmk));
    peer.channel = 0; // keep using the current Wi-Fi channel
    peer.encrypt = true;
    peer.ifidx = WIFI_IF_STA;
    return esp_now_add_peer(&peer) == ESP_OK;
}

void removePeer(const uint8_t mac[6])
{
    if (esp_now_is_peer_exist(mac)) {
        esp_now_del_peer(mac);
    }
}

bool send(const uint8_t mac[6], const Message &msg)
{
    return esp_now_send(mac, reinterpret_cast<const uint8_t *>(&msg), sizeof(Message)) == ESP_OK;
}

bool sendBroadcast(const Message &msg)
{
    return esp_now_send(kBroadcastMac, reinterpret_cast<const uint8_t *>(&msg), sizeof(Message)) == ESP_OK;
}

void getOwnMac(uint8_t mac_out[6])
{
    WiFi.macAddress(mac_out);
}

void macToStr(const uint8_t mac[6], char *buf)
{
    sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool macEquals(const uint8_t a[6], const uint8_t b[6])
{
    return memcmp(a, b, 6) == 0;
}

} // namespace remote_link
