/*****************************************************************************
 * | File        :   remote_protocol.h
 * | Function    :   Wire protocol shared between the LCD controller
 * |                 (src/lcd_main) and the relay module (src/relay_main),
 * |                 transported over ESP-NOW (see espnow_link.h).
 ******************************************************************************/

#ifndef __REMOTE_PROTOCOL_H
#define __REMOTE_PROTOCOL_H

#include <stdint.h>

namespace remote_link {

constexpr uint8_t kMagic = 0xA5;
// v5: adiciona ao StatusReport a inclinacao (roll/pitch) do MPU6050 montado
// na placa de reles e o estado do sensor de nivel de agua sem contato
// XKC-Y25-V (reservatorio) - ver lib/imu_mpu6050 e src/relay_main/main.cpp.
// v4: adiciona PIDs extras ao ObdData (carga do motor, MAP, nivel de
// combustivel) e os MsgType::DtcControl/DtcData para leitura/limpeza de
// codigos de falha (DTCs) - ver CLAUDE.md regra 1 (mudou o layout de
// Message, incrementa a versao; mensagens de versoes diferentes sao
// ignoradas em espnow_link.cpp::onDataRecv).
constexpr uint8_t kProtocolVersion = 5;
// Reduzido de 6 para 4: a placa de reles atual so usa 4 dos seus 6 canais, e
// a futura placa de maior potencia (ESP32_Relay_30A_X4, ver CLAUDE.md regra 7)
// tem exatamente 4 reles.
constexpr uint8_t kRelayChannels = 4;

// Numero maximo de DTCs (codigos de falha) que cabem numa unica mensagem
// MsgType::DtcData. Mais que suficiente para o uso real (a grande maioria
// dos veiculos reporta poucos codigos de cada vez); se um dia precisar de
// mais, so aumentar aqui (ainda sobra bastante margem no limite de 200
// bytes do Message - ver static_assert abaixo).
constexpr uint8_t kMaxDtcEntries = 10;

enum class MsgType : uint8_t {
    Hello        = 0x00, // relay -> broadcast: "I'm here, not paired yet"
    HelloAck     = 0x01, // controller -> relay (unicast): "I'll control you"
    SetRelay     = 0x10, // controller -> relay: set a single channel
    SetAllRelays = 0x11, // controller -> relay: set all channels (bitmask)
    ObdControl   = 0x12, // controller -> relay: liga/desliga a conexao com o OBD-II
    DtcControl   = 0x13, // controller -> relay: pede leitura ou limpeza dos DTCs
    StatusReport = 0x20, // relay -> controller: channel states + telemetry
    Heartbeat    = 0x21, // either direction: keep-alive
    ObdData      = 0x22, // relay -> controller: dados lidos do adaptador OBD-II (ELM327)
    DtcData      = 0x23, // relay -> controller: resultado da ultima leitura/limpeza de DTCs
};

#pragma pack(push, 1)
struct Message {
    uint8_t magic = kMagic;
    uint8_t version = kProtocolVersion;
    MsgType type = MsgType::Heartbeat;
    union Payload {
        struct { uint8_t unused; } hello;
        struct { uint8_t unused; } hello_ack;
        struct { uint8_t channel; uint8_t state; } set_relay;
        struct { uint8_t mask; } set_all_relays;
        // tilt_valid: 0 se o MPU6050 nao foi detectado na placa de reles
        //   (ver imu::begin() em relay_main/main.cpp) - roll/pitch ficam 0.
        // water_level_valid: sempre 1 hoje (a leitura e um digitalRead()
        //   simples, sem deteccao de presenca do sensor XKC-Y25-V).
        // water_level_low: 1 = nivel do reservatorio ABAIXO do ponto onde o
        //   sensor foi colado (alarme), 0 = nivel OK naquele ponto.
        struct {
            uint8_t relay_mask; uint32_t uptime_s; int8_t rssi;
            uint8_t tilt_valid; float roll_deg; float pitch_deg;
            uint8_t water_level_valid; uint8_t water_level_low;
        } status;
        struct { uint32_t uptime_s; } heartbeat;
        // Cada campo *_valid indica se o respectivo dado foi lido com
        // sucesso nesta rodada (adaptador conectado e PID respondeu).
        // - engine_temp_c: PID 0105, graus Celsius.
        // - rpm: PID 010C, RPM do motor.
        // - speed_kmh: PID 010D, km/h.
        // - voltage_v: PID 0142, tensao do modulo de controle (bateria).
        // - fuel_rate_lph: estimativa calculada a partir do MAF (PID 0110),
        //   NAO e um PID padrao - ver comentario em relay_main/main.cpp.
        // - engine_load_pct: PID 0104, carga calculada do motor (%).
        // - map_kpa: PID 010B, pressao absoluta do coletor de admissao (kPa).
        // - fuel_level_pct: PID 012F, nivel do tanque de combustivel (%).
        //   Os 3 acima foram confirmados suportados no veiculo de teste -
        //   ver tabela.md.
        // Nao ha PID padrao Modo 01 para temperatura de cambio (especifico
        // de cada fabricante) - por isso o protocolo nao carrega esse dado.
        struct {
            uint8_t connected;
            uint8_t engine_temp_valid;  float engine_temp_c;
            uint8_t rpm_valid;          uint16_t rpm;
            uint8_t speed_valid;        uint8_t speed_kmh;
            uint8_t voltage_valid;      float voltage_v;
            uint8_t fuel_rate_valid;    float fuel_rate_lph;
            uint8_t engine_load_valid;  uint8_t engine_load_pct;
            uint8_t map_valid;          uint8_t map_kpa;
            uint8_t fuel_level_valid;   uint8_t fuel_level_pct;
        } obd_data;
        // connect = 1: tenta conectar/manter conectado no ELM327.
        // connect = 0: desconecta e para de tentar reconectar.
        struct { uint8_t connect; } obd_control;
        // action = 0: pede uma leitura dos DTCs (Modo 03 armazenados + Modo
        //             07 pendentes), resposta chega via MsgType::DtcData.
        // action = 1: pede limpeza dos DTCs (Modo 04) - o relay re-le os
        //             codigos logo em seguida para confirmar (mesma
        //             resposta MsgType::DtcData, idealmente com contagem 0).
        struct { uint8_t action; } dtc_control;
        // Resultado da ultima leitura/limpeza pedida via MsgType::DtcControl.
        // mil_on: status da luz de check engine (bit7 do PID 0101, byte A).
        // dtc_count: quantas entradas de `dtcs` estao preenchidas (<= kMaxDtcEntries).
        // dtcs[i].code: codigo bruto de 2 bytes como retornado pela ECU -
        //   use formatDtcCode() abaixo para decodificar em texto (ex. "P0301").
        // dtcs[i].pending: 0 = armazenado (Modo 03), 1 = pendente (Modo 07).
        struct {
            uint8_t mil_on;
            uint8_t dtc_count;
            struct { uint16_t code; uint8_t pending; } dtcs[kMaxDtcEntries];
        } dtc_data;
        Payload() { heartbeat.uptime_s = 0; }
    } payload;
};
#pragma pack(pop)

static_assert(sizeof(Message) <= 200, "Message must fit comfortably in one ESP-NOW frame (max 250 bytes)");

/**
 * @brief Decode a raw 2-byte DTC code (as returned by Mode 03/07) into its
 *        standard text form (e.g. "P0301"), per SAE J2012/ISO 15031-6:
 *        byte A bits 7-6 = category (00=P,01=C,10=B,11=U), bits 5-4 = 1st
 *        digit (0-3), bits 3-0 = 2nd digit (hex); byte B bits 7-4 = 3rd
 *        digit (hex), bits 3-0 = 4th digit (hex).
 *
 * @param out Buffer of at least 6 bytes ("Xdddd\0").
 */
inline void formatDtcCode(uint16_t raw, char *out)
{
    static const char kCategory[4] = {'P', 'C', 'B', 'U'};
    static const char kHexDigits[16] = {
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    uint8_t byte_a = (uint8_t)(raw >> 8);
    uint8_t byte_b = (uint8_t)(raw & 0xFF);
    out[0] = kCategory[(byte_a >> 6) & 0x03];
    out[1] = (char)('0' + ((byte_a >> 4) & 0x03));
    out[2] = kHexDigits[byte_a & 0x0F];
    out[3] = kHexDigits[(byte_b >> 4) & 0x0F];
    out[4] = kHexDigits[byte_b & 0x0F];
    out[5] = '\0';
}

} // namespace remote_link

#endif // __REMOTE_PROTOCOL_H
