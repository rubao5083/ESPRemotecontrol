/*****************************************************************************
 * | File        :   main.cpp  (relay_main)
 * | Function    :   Firmware for the RELAY MODULE ("ESP32_Relay_30A_X4",
 * |                 ESP32-WROOM-32E - classic ESP32, not S3).
 * |
 * | Receives relay commands from the LCD controller over ESP-NOW, drives the
 * | 4 relay channels, and reports status back periodically.
 * |
 * | Tambem conecta via Bluetooth Classico (SPP) num adaptador ELM327 e envia
 * | dados do OBD-II (temperatura do motor, RPM, velocidade, tensao da
 * | bateria, consumo de combustivel estimado, carga do motor, MAP e nivel
 * | de combustivel) para a tela via ESP-NOW (MsgType::ObdData) - a tela
 * | (ESP32-S3) nao tem Bluetooth Classico, so este modulo consegue falar
 * | com o adaptador (ver README.md "Roadmap"/secao OBD-II). A consulta roda
 * | numa FreeRTOS task separada (obdTask) para NAO bloquear loop() - o
 * | failsafe de seguranca (desligar reles se perder o link com a tela) nao
 * | pode ficar refem do tempo de conexao/resposta do Bluetooth Classico. A
 * | tela pode ligar/desligar essa conexao via MsgType::ObdControl (botao
 * | "Conectar"/"Desconectar" na 3a tela, ver src/lcd_main/main.cpp).
 * |
 * | A mesma obdTask tambem atende pedidos sob demanda de leitura/limpeza de
 * | DTCs (codigos de falha, Modo 03/07/04) vindos da 4a tela
 * | (MsgType::DtcControl), respondendo com MsgType::DtcData.
 ******************************************************************************/

#include <Arduino.h>
#include <string.h>
#include <string>
#include <vector>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <BluetoothSerial.h>
#include "espnow_link.h"
#include "mpu6050.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth classico nao habilitado neste build (CONFIG_BT_ENABLED/CONFIG_BLUEDROID_ENABLED)
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Perfil SPP (Bluetooth Classico serial) nao habilitado - so disponivel no ESP32 classico, nao no S3.
#endif

using namespace remote_link;

// ======================= Configuration ====================================

// Shared secret used to derive the ESP-NOW encryption key.
// MUST be identical to the one used in src/lcd_main/main.cpp.
static const char *PAIRING_KEY = "ALTERE-ESTA-CHAVE-0001";

// Fixed 2.4GHz Wi-Fi channel used by both devices for ESP-NOW (1-13).
static const uint8_t WIFI_CHANNEL = 6;

// GPIOs dos 4 canais de rele da placa atual "ESP32_Relay_30A_X4"
// (ESP32-WROOM-32E, ESP32 classico - NAO e S3). A pinagem generica da
// documentacao da comunidade ESPHome (GPIO32/33/25/26) NAO bateu com esta
// unidade (revisao de PCB "2025.0506") - nenhum rele acionou com ela.
// Pinout real confirmado por varredura empirica pino-a-pino (ver
// docs/HISTORICO-DEPURACAO.md): CH1=GPIO12, CH2=GPIO13, CH3=GPIO14,
// CH4=GPIO15. Placa anterior (Waveshare ESP32-S3-Relay-6CH) usava
// GPIO 1, 2, 41, 42.
static const uint8_t RELAY_PINS[kRelayChannels] = {12, 13, 14, 15};

// Some relay boards drive the opto-isolator input active LOW. If the relays
// behave inverted (ON when they should be OFF), flip this to false.
static const bool RELAY_ACTIVE_HIGH = true;

static const uint32_t LINK_TIMEOUT_MS   = 10000; // no msg from controller -> failsafe OFF
static const uint32_t HELLO_INTERVAL_MS = 1000;
static const uint32_t STATUS_INTERVAL_MS = 1000;

// Pinos I2C para o modulo MPU6050 (acelerometro+giroscopio 6 eixos),
// montado fisicamente sobre esta placa. Movidos de GPIO21/22 (pinos I2C
// padrao) para GPIO17 (SDA) / GPIO18 (SCL) durante a depuracao de leituras
// instaveis do sensor (ver docs/HISTORICO-DEPURACAO.md). Como o I2C e
// feito por GPIO comum/Wire com pinos configuraveis, qualquer GPIO livre
// com saida serve - 17 e 18 nao sao usados por relés, sensor de nivel nem
// pelos pinos de boot/flash do ESP32-WROOM-32E.
static const uint8_t MPU6050_SDA_PIN = 17;
static const uint8_t MPU6050_SCL_PIN = 18;
static const uint32_t IMU_READ_INTERVAL_MS = 50; // ~20Hz, ver imu_mpu6050::read()

// Sensor de nivel de agua sem contato (XKC-Y25-V), colado na parede externa
// do reservatorio na altura minima aceitavel. ALIMENTACAO: o datasheet
// exige minimo 5V (NAO ligar no 3.3V do ESP32) - use o proprio conector de
// 5V dedicado desta placa. A saida do sensor acompanha a tensao de
// alimentacao (~5V quando ativa), que ULTRAPASSA o limite absoluto de
// 3.6V de uma GPIO do ESP32 - e OBRIGATORIO um divisor resistivo entre o
// OUT do sensor e este pino (ex.: 10k do OUT ate aqui + 20k daqui ate o
// GND, resultando em ~3.3V). Pino atual: GPIO34 ("G34" no header da placa -
// CONFIRME no esquematico/serigrafia antes de ligar). Historico: GPIO27 foi
// descartado apos um acidente de fiacao (ligado ao fio do sensor antes do GND)
// e depois GPIO26, trocado por ser ADC2 (inutilizavel com Wi-Fi/ESP-NOW ativo).
static const uint8_t WATER_LEVEL_PIN = 34;
// Leitura ANALOGICA (em vez de digital): com o sensor sem liquido o sinal
// oscilava entre OK/nao-OK por causa de tensao residual. GPIO34 e ADC1 (somente
// entrada) - o ADC2 (ex.: GPIO26, usado antes) NAO funciona com Wi-Fi/ESP-NOW
// ativo. Limiar de 1.5V NO PINO (ja depois do divisor resistivo) com histerese
// de +-WATER_LEVEL_HYST_MV: so muda de estado ao passar de um dos extremos,
// ignorando a faixa intermediaria. GPIO34 nao tem pull-up/pull-down interno.
static const int WATER_LEVEL_THRESHOLD_MV = 1500;
static const int WATER_LEVEL_HYST_MV = 200;
static const uint8_t WATER_LEVEL_SAMPLES = 8; // media para filtrar ruido do ADC
// Fio preto (MODE) ligado no 3.3V (nao flutuando nem aterrado - desvio do
// datasheet, mas testado e confirmado em bancada). Saida (NPN open-collector
// com pull-up interno) CONFIRMADA por medicao real: ~3.3V (HIGH) quando NAO
// ha liquido no ponto do sensor (transistor interno cortado, OUT puxado
// para VCC pelo pull-up interno do sensor); ~0V (LOW, na pratica 4-6mV de
// tensao residual de saturacao do transistor - normal, bem abaixo do
// limiar logico BAIXO do ESP32) quando HA liquido (transistor interno
// satura, puxando OUT para perto do GND). Logica INVERTIDA em relacao ao
// que o comentario anterior assumia (sem teste em bancada) - NAO inverter
// de volta sem antes remedir o fio amarelo (OUT) nos dois estados.
static const bool WATER_LEVEL_ACTIVE_HIGH = false;

// Endereco MAC do adaptador ELM327 Bluetooth Classico usado neste projeto,
// descoberto e confirmado com o ambiente de diagnostico `obd2_scan_classic`
// (ver README.md - "OBD-II"). Troque este valor se usar outro adaptador
// (regrave `-e obd2_scan_classic` e rode "scan" para descobrir o MAC dele).
static const char *OBD_ELM327_MAC = "00:0D:18:3A:67:89";

static const uint32_t OBD_POLL_INTERVAL_MS = 2000; // intervalo entre consultas de PID
static const uint32_t OBD_RECONNECT_DELAY_MS = 15000; // espera antes de tentar reconectar
static const uint32_t OBD_SEND_INTERVAL_MS = 2000; // intervalo de envio do MsgType::ObdData
static const uint32_t OBD_DISABLED_POLL_MS = 500; // intervalo de checagem enquanto desabilitado

// Constantes usadas para ESTIMAR o consumo instantaneo de combustivel a
// partir do MAF (PID 0110, g/s de ar admitido) - nao existe um PID padrao
// Modo 01 que devolva litros/hora diretamente para a maioria dos veiculos.
// Formula classica (mesma usada por scangauges/apps tipo Torque):
//   L/h = (MAF_g_s * 3600) / (AFR * densidade_combustivel_g_por_L)
// Valores assumidos para DIESEL (ajuste se o veiculo for a gasolina):
// AFR estequiometrico ~14.5:1, densidade do diesel ~832 g/L. Isso e uma
// ESTIMATIVA - varia com qualidade/temperatura do combustivel e a mistura
// real (lambda) do motor, que o PID 0110 sozinho nao informa.
static const float OBD_FUEL_AFR = 14.5f;
static const float OBD_FUEL_DENSITY_G_PER_L = 832.0f;

// ======================= State =============================================

static uint8_t s_controller_mac[6] = {0};
static bool s_paired = false;
static uint8_t s_relay_mask = 0;
static volatile uint32_t s_last_controller_msg_ms = 0; // escrito por onMessage() (outra task)
static uint32_t s_last_hello_ms = 0;
static uint32_t s_last_status_ms = 0;
static int8_t s_last_rssi = 0;

// Estado do MPU6050 (inclinacao da placa de reles) e do sensor de nivel de
// agua - lidos periodicamente em loop() e enviados dentro de sendStatus().
static bool s_mpu_available = false;
static imu::Attitude s_last_attitude;
static uint32_t s_last_imu_read_ms = 0;

// ======================= OBD-II (ELM327 via Bluetooth Classico) =============

BluetoothSerial SerialBT;

// Estado compartilhado entre obdTask() (FreeRTOS, le/escreve no ELM327) e
// loop() (le para montar o MsgType::ObdData) - protegido por s_obd_mutex
// porque as duas tasks rodam em contextos/cores diferentes.
static SemaphoreHandle_t s_obd_mutex = nullptr;
static bool s_obd_enabled = true; // controlado por MsgType::ObdControl (botao na tela)
static bool s_obd_connected = false;
static bool s_obd_engine_temp_valid = false;
static float s_obd_engine_temp_c = 0.0f;
static bool s_obd_rpm_valid = false;
static uint16_t s_obd_rpm = 0;
static bool s_obd_speed_valid = false;
static uint8_t s_obd_speed_kmh = 0;
static bool s_obd_voltage_valid = false;
static float s_obd_voltage_v = 0.0f;
static bool s_obd_fuel_rate_valid = false;
static float s_obd_fuel_rate_lph = 0.0f;
static bool s_obd_engine_load_valid = false;
static uint8_t s_obd_engine_load_pct = 0;
static bool s_obd_map_valid = false;
static uint8_t s_obd_map_kpa = 0;
static bool s_obd_fuel_level_valid = false;
static uint8_t s_obd_fuel_level_pct = 0;
static uint32_t s_last_obd_send_ms = 0;

// Pedido de leitura/limpeza de DTCs vindo da tela (MsgType::DtcControl),
// consumido por obdTask() no proximo ciclo: 0 = nenhum, 1 = ler, 2 = limpar.
// Mesmo mutex do resto do estado OBD (onMessage() roda num contexto
// diferente de obdTask() - ver espnow_link.h).
static uint8_t s_dtc_action_request = 0;

// ======================= Relay control ======================================

static void applyRelay(uint8_t channel, bool on)
{
    if (channel >= kRelayChannels) {
        return;
    }
    if (on) {
        s_relay_mask |= (uint8_t)(1 << channel);
    } else {
        s_relay_mask &= (uint8_t)~(1 << channel);
    }
    digitalWrite(RELAY_PINS[channel], (on == RELAY_ACTIVE_HIGH) ? HIGH : LOW);
}

static void applyAll(uint8_t mask)
{
    for (uint8_t ch = 0; ch < kRelayChannels; ch++) {
        applyRelay(ch, ((mask >> ch) & 0x01) != 0);
    }
}

static void allRelaysOff()
{
    applyAll(0);
}

// ======================= OBD-II (ELM327 via Bluetooth Classico) =============

// Envia um comando AT/ELM327 (acrescenta o CR de fim de linha, convencao do
// protocolo) e aguarda a resposta ate o prompt ">" ou o timeout. A resposta
// crua (sem o ">") fica em `response_out`.
static bool obdSendCommand(const char *cmd, std::string &response_out, uint32_t timeout_ms)
{
    response_out.clear();
    std::string line = std::string(cmd) + "\r";
    SerialBT.write(reinterpret_cast<const uint8_t *>(line.data()), line.size());

    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        while (SerialBT.available() > 0) {
            char c = static_cast<char>(SerialBT.read());
            if (c == '>') {
                return true;
            }
            response_out += c;
        }
        delay(5);
    }
    return false; // timeout sem receber o prompt ">"
}

// Tokeniza uma resposta do ELM327 por espaco/quebra de linha (em vez de so
// filtrar digitos hex), para nao se confundir com prefixos como
// "SEARCHING..." que o adaptador pode intercalar antes da resposta real.
// Compartilhado por parseObdBytes() (Modo 01) e parseDtcCodes() (Modo 03/07).
static void tokenizeObdResponse(const std::string &resp, std::vector<std::string> &tokens_out)
{
    tokens_out.clear();
    std::string cur;
    for (char c : resp) {
        if (isspace(static_cast<unsigned char>(c))) {
            if (!cur.empty()) {
                tokens_out.push_back(cur);
                cur.clear();
            }
        } else {
            cur += c;
        }
    }
    if (!cur.empty()) {
        tokens_out.push_back(cur);
    }
}

// Extrai N bytes de dados de uma resposta Modo 01 (ex.: "41 <PID> <b0> <b1>
// ...").
static bool parseObdBytes(const std::string &resp, uint8_t expected_pid, uint8_t *bytes_out, size_t count)
{
    char pid_hex[3];
    snprintf(pid_hex, sizeof(pid_hex), "%02X", expected_pid);

    std::vector<std::string> tokens;
    tokenizeObdResponse(resp, tokens);

    for (size_t i = 0; i + 1 + count <= tokens.size(); i++) {
        if (tokens[i] == "41" && tokens[i + 1] == pid_hex) {
            for (size_t b = 0; b < count; b++) {
                bytes_out[b] = static_cast<uint8_t>(strtoul(tokens[i + 2 + b].c_str(), nullptr, 16));
            }
            return true;
        }
    }
    return false; // resposta de erro (NO DATA, ?, barramento ocupado, etc.)
}

// Extrai os codigos de falha (DTCs) de uma resposta Modo 03 (header "43",
// armazenados) ou Modo 07 (header "47", pendentes): pares de bytes
// consecutivos apos o header, ate `max_count` codigos. Um par "00 00" e
// tratado como preenchimento/fim da lista (convencao comum do ELM327) e
// interrompe a extracao. Se duas UCEs responderem (ver tabela.md - comum em
// veiculos multiplexados), usa so a primeira resposta, mesma convencao
// adotada em parseObdBytes()/obdPollPid() para o Modo 01.
static uint8_t parseDtcCodes(const std::string &resp, const char *header_hex, uint16_t *codes_out, uint8_t max_count)
{
    std::vector<std::string> tokens;
    tokenizeObdResponse(resp, tokens);

    for (size_t i = 0; i < tokens.size(); i++) {
        if (tokens[i] != header_hex) {
            continue;
        }
        uint8_t count = 0;
        size_t j = i + 1;
        while (count < max_count && j + 1 < tokens.size()) {
            uint8_t b0 = static_cast<uint8_t>(strtoul(tokens[j].c_str(), nullptr, 16));
            uint8_t b1 = static_cast<uint8_t>(strtoul(tokens[j + 1].c_str(), nullptr, 16));
            j += 2;
            uint16_t code = static_cast<uint16_t>((b0 << 8) | b1);
            if (code == 0) {
                break; // "00 00" = preenchimento, fim da lista real de codigos
            }
            codes_out[count++] = code;
        }
        return count;
    }
    return 0; // header nao encontrado (NO DATA = nenhum codigo, ou erro)
}

static bool obdIsEnabled()
{
    xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
    bool enabled = s_obd_enabled;
    xSemaphoreGive(s_obd_mutex);
    return enabled;
}

static void obdSetConnected(bool connected)
{
    xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
    s_obd_connected = connected;
    if (!connected) {
        s_obd_engine_temp_valid = false;
        s_obd_rpm_valid = false;
        s_obd_speed_valid = false;
        s_obd_voltage_valid = false;
        s_obd_fuel_rate_valid = false;
        s_obd_engine_load_valid = false;
        s_obd_map_valid = false;
        s_obd_fuel_level_valid = false;
    }
    xSemaphoreGive(s_obd_mutex);
}

static void obdSetEngineTemp(bool valid, float temp_c)
{
    xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
    s_obd_engine_temp_valid = valid;
    s_obd_engine_temp_c = temp_c;
    xSemaphoreGive(s_obd_mutex);
}

static void obdSetRpm(bool valid, uint16_t rpm)
{
    xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
    s_obd_rpm_valid = valid;
    s_obd_rpm = rpm;
    xSemaphoreGive(s_obd_mutex);
}

static void obdSetSpeed(bool valid, uint8_t speed_kmh)
{
    xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
    s_obd_speed_valid = valid;
    s_obd_speed_kmh = speed_kmh;
    xSemaphoreGive(s_obd_mutex);
}

static void obdSetVoltage(bool valid, float voltage_v)
{
    xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
    s_obd_voltage_valid = valid;
    s_obd_voltage_v = voltage_v;
    xSemaphoreGive(s_obd_mutex);
}

static void obdSetFuelRate(bool valid, float fuel_rate_lph)
{
    xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
    s_obd_fuel_rate_valid = valid;
    s_obd_fuel_rate_lph = fuel_rate_lph;
    xSemaphoreGive(s_obd_mutex);
}

static void obdSetEngineLoad(bool valid, uint8_t pct)
{
    xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
    s_obd_engine_load_valid = valid;
    s_obd_engine_load_pct = pct;
    xSemaphoreGive(s_obd_mutex);
}

static void obdSetMap(bool valid, uint8_t kpa)
{
    xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
    s_obd_map_valid = valid;
    s_obd_map_kpa = kpa;
    xSemaphoreGive(s_obd_mutex);
}

static void obdSetFuelLevel(bool valid, uint8_t pct)
{
    xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
    s_obd_fuel_level_valid = valid;
    s_obd_fuel_level_pct = pct;
    xSemaphoreGive(s_obd_mutex);
}

// Consulta um PID Modo 01 e atualiza o estado compartilhado correspondente
// via `setter`. Retorna false se a conexao caiu (usado para o chamador
// decidir se deve abortar o ciclo e tentar reconectar).
template <typename Setter>
static bool obdPollPid(const char *pid_cmd, uint8_t pid, uint8_t *bytes, size_t count, Setter setter)
{
    std::string resp;
    bool got_reply = obdSendCommand(pid_cmd, resp, 1500);
    if (!got_reply || !SerialBT.connected()) {
        return false;
    }
    if (parseObdBytes(resp, pid, bytes, count)) {
        setter(bytes);
    } else {
        setter(nullptr);
    }
    return true;
}

// Task dedicada (nao bloqueia loop()/o failsafe do rele) que mantem a
// conexao Bluetooth Classico com o ELM327 e consulta os PIDs priorizados
// (ver README.md - mesmo conjunto de dados mostrado pelos HUDs OBD-II
// comerciais): temperatura do motor, RPM, velocidade, tensao da bateria, e
// MAF (usado so para estimar o consumo de combustivel).
static void obdTask(void *)
{
    for (;;) {
        if (!obdIsEnabled()) {
            if (SerialBT.connected()) {
                SerialBT.disconnect();
            }
            obdSetConnected(false);
            vTaskDelay(pdMS_TO_TICKS(OBD_DISABLED_POLL_MS));
            continue;
        }

        if (!SerialBT.connected()) {
            obdSetConnected(false);
            Serial.println("[OBD] Conectando ao ELM327...");
            bool ok = SerialBT.connect(BTAddress(String(OBD_ELM327_MAC)));
            if (!ok) {
                Serial.println("[OBD] Falha ao conectar ao ELM327 - tentando de novo em breve.");
                vTaskDelay(pdMS_TO_TICKS(OBD_RECONNECT_DELAY_MS));
                continue;
            }

            std::string resp;
            obdSendCommand("ATZ", resp, 3000);   // reset
            obdSendCommand("ATE0", resp, 1000);  // desliga eco
            obdSendCommand("ATL0", resp, 1000);  // desliga linefeed extra
            obdSendCommand("ATSP0", resp, 1000); // protocolo automatico
            obdSetConnected(true);
            Serial.println("[OBD] Conectado ao ELM327.");
        }

        bool link_ok = true;
        uint8_t b[2];

        link_ok = link_ok && obdPollPid("0105", 0x05, b, 1, [](const uint8_t *v) {
            obdSetEngineTemp(v != nullptr, v ? static_cast<float>(v[0]) - 40.0f : 0.0f); // formula padrao SAE
        });

        link_ok = link_ok && obdPollPid("010C", 0x0C, b, 2, [](const uint8_t *v) {
            obdSetRpm(v != nullptr, v ? static_cast<uint16_t>((v[0] * 256 + v[1]) / 4) : 0); // formula padrao SAE
        });

        link_ok = link_ok && obdPollPid("010D", 0x0D, b, 1, [](const uint8_t *v) {
            obdSetSpeed(v != nullptr, v ? v[0] : 0); // formula padrao SAE (valor direto)
        });

        link_ok = link_ok && obdPollPid("0142", 0x42, b, 2, [](const uint8_t *v) {
            obdSetVoltage(v != nullptr, v ? static_cast<float>(v[0] * 256 + v[1]) / 1000.0f : 0.0f); // formula padrao SAE
        });

        // MAF (g/s) - usado so para estimar o consumo, nao enviado sozinho.
        link_ok = link_ok && obdPollPid("0110", 0x10, b, 2, [](const uint8_t *v) {
            if (v == nullptr) {
                obdSetFuelRate(false, 0.0f);
                return;
            }
            float maf_g_s = static_cast<float>(v[0] * 256 + v[1]) / 100.0f; // formula padrao SAE
            float fuel_rate_lph = (maf_g_s * 3600.0f) / (OBD_FUEL_AFR * OBD_FUEL_DENSITY_G_PER_L);
            obdSetFuelRate(true, fuel_rate_lph);
        });

        // Carga do motor (%), MAP - pressao absoluta do coletor (kPa) e
        // nivel do tanque (%) - confirmados suportados no veiculo de teste,
        // ver tabela.md. Formulas padrao SAE (raw*100/255 para percentuais,
        // MAP e o valor bruto direto em kPa).
        link_ok = link_ok && obdPollPid("0104", 0x04, b, 1, [](const uint8_t *v) {
            obdSetEngineLoad(v != nullptr, v ? static_cast<uint8_t>((v[0] * 100) / 255) : 0);
        });

        link_ok = link_ok && obdPollPid("010B", 0x0B, b, 1, [](const uint8_t *v) {
            obdSetMap(v != nullptr, v ? v[0] : 0);
        });

        link_ok = link_ok && obdPollPid("012F", 0x2F, b, 1, [](const uint8_t *v) {
            obdSetFuelLevel(v != nullptr, v ? static_cast<uint8_t>((v[0] * 100) / 255) : 0);
        });

        if (!link_ok || !SerialBT.connected()) {
            Serial.println("[OBD] Link com o ELM327 perdido.");
            SerialBT.disconnect();
            obdSetConnected(false);
            continue;
        }

        // Leitura/limpeza de DTCs sob demanda (botoes da 4a tela) - nao faz
        // parte do ciclo normal de telemetria porque e mais lento (Modo
        // 03/07 podem vir em varios quadros) e so precisa rodar quando o
        // usuario pedir. O pedido e consumido aqui (zerado) mesmo se o
        // adaptador nao responder, para nao ficar repetindo a tentativa a
        // cada ciclo de 2s.
        uint8_t dtc_action = 0;
        xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
        dtc_action = s_dtc_action_request;
        s_dtc_action_request = 0;
        xSemaphoreGive(s_obd_mutex);

        if (dtc_action != 0) {
            std::string resp;
            if (dtc_action == 2) {
                Serial.println("[OBD] Limpando DTCs (Modo 04)...");
                obdSendCommand("04", resp, 2000);
                delay(200); // da tempo da ECU processar antes de reler
            }

            bool mil_on = false;
            uint8_t b4[4];
            if (obdSendCommand("0101", resp, 1500) && parseObdBytes(resp, 0x01, b4, 4)) {
                mil_on = (b4[0] & 0x80) != 0; // bit7 = MIL ligada/desligada
            }

            uint16_t stored[kMaxDtcEntries];
            uint8_t stored_count = obdSendCommand("03", resp, 2000)
                                       ? parseDtcCodes(resp, "43", stored, kMaxDtcEntries)
                                       : 0;

            uint16_t pending[kMaxDtcEntries];
            uint8_t pending_count = 0;
            if (stored_count < kMaxDtcEntries) {
                pending_count = obdSendCommand("07", resp, 2000)
                                    ? parseDtcCodes(resp, "47", pending, (uint8_t)(kMaxDtcEntries - stored_count))
                                    : 0;
            }

            Message dtc_msg;
            dtc_msg.type = MsgType::DtcData;
            dtc_msg.payload.dtc_data.mil_on = mil_on ? 1 : 0;
            uint8_t total = 0;
            for (uint8_t i = 0; i < stored_count; i++) {
                dtc_msg.payload.dtc_data.dtcs[total].code = stored[i];
                dtc_msg.payload.dtc_data.dtcs[total].pending = 0;
                total++;
            }
            for (uint8_t i = 0; i < pending_count; i++) {
                dtc_msg.payload.dtc_data.dtcs[total].code = pending[i];
                dtc_msg.payload.dtc_data.dtcs[total].pending = 1;
                total++;
            }
            dtc_msg.payload.dtc_data.dtc_count = total;
            Serial.printf("[OBD] DTCs: MIL=%s, %u armazenado(s), %u pendente(s).\n",
                          mil_on ? "ligada" : "desligada", stored_count, pending_count);

            if (s_paired) {
                send(s_controller_mac, dtc_msg);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(OBD_POLL_INTERVAL_MS));
    }
}

// ======================= ESP-NOW ============================================

// Le o sensor de nivel (analogico + histerese) e devolve true se ha liquido
// no ponto do sensor ("OK"). Mantem o ultimo estado enquanto a tensao estiver
// na faixa intermediaria entre os dois limiares.
static bool readWaterLevelOk()
{
    static bool initialized = false;
    static bool level_high = false;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < WATER_LEVEL_SAMPLES; i++) {
        sum += analogReadMilliVolts(WATER_LEVEL_PIN);
    }
    int mv = (int)(sum / WATER_LEVEL_SAMPLES);

    if (!initialized) {
        level_high = mv >= WATER_LEVEL_THRESHOLD_MV;
        initialized = true;
    } else if (mv >= WATER_LEVEL_THRESHOLD_MV + WATER_LEVEL_HYST_MV) {
        level_high = true;
    } else if (mv <= WATER_LEVEL_THRESHOLD_MV - WATER_LEVEL_HYST_MV) {
        level_high = false;
    }

    return level_high == WATER_LEVEL_ACTIVE_HIGH;
}

static void sendStatus()
{
    Message msg;
    msg.type = MsgType::StatusReport;
    msg.payload.status.relay_mask = s_relay_mask;
    msg.payload.status.uptime_s = millis() / 1000;
    msg.payload.status.rssi = s_last_rssi;

    msg.payload.status.tilt_valid = s_last_attitude.valid ? 1 : 0;
    msg.payload.status.roll_deg = s_last_attitude.roll_deg;
    msg.payload.status.pitch_deg = s_last_attitude.pitch_deg;

    bool water_ok = readWaterLevelOk();
    msg.payload.status.water_level_valid = 1;
    msg.payload.status.water_level_low = water_ok ? 0 : 1;

    send(s_controller_mac, msg);
}

static void sendObdData()
{
    bool connected, engine_temp_valid, rpm_valid, speed_valid, voltage_valid, fuel_rate_valid;
    bool engine_load_valid, map_valid, fuel_level_valid;
    float engine_temp_c, voltage_v, fuel_rate_lph;
    uint16_t rpm;
    uint8_t speed_kmh, engine_load_pct, map_kpa, fuel_level_pct;
    xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
    connected = s_obd_connected;
    engine_temp_valid = s_obd_engine_temp_valid;
    engine_temp_c = s_obd_engine_temp_c;
    rpm_valid = s_obd_rpm_valid;
    rpm = s_obd_rpm;
    speed_valid = s_obd_speed_valid;
    speed_kmh = s_obd_speed_kmh;
    voltage_valid = s_obd_voltage_valid;
    voltage_v = s_obd_voltage_v;
    fuel_rate_valid = s_obd_fuel_rate_valid;
    fuel_rate_lph = s_obd_fuel_rate_lph;
    engine_load_valid = s_obd_engine_load_valid;
    engine_load_pct = s_obd_engine_load_pct;
    map_valid = s_obd_map_valid;
    map_kpa = s_obd_map_kpa;
    fuel_level_valid = s_obd_fuel_level_valid;
    fuel_level_pct = s_obd_fuel_level_pct;
    xSemaphoreGive(s_obd_mutex);

    Message msg;
    msg.type = MsgType::ObdData;
    msg.payload.obd_data.connected = connected ? 1 : 0;
    msg.payload.obd_data.engine_temp_valid = engine_temp_valid ? 1 : 0;
    msg.payload.obd_data.engine_temp_c = engine_temp_c;
    msg.payload.obd_data.rpm_valid = rpm_valid ? 1 : 0;
    msg.payload.obd_data.rpm = rpm;
    msg.payload.obd_data.speed_valid = speed_valid ? 1 : 0;
    msg.payload.obd_data.speed_kmh = speed_kmh;
    msg.payload.obd_data.voltage_valid = voltage_valid ? 1 : 0;
    msg.payload.obd_data.voltage_v = voltage_v;
    msg.payload.obd_data.fuel_rate_valid = fuel_rate_valid ? 1 : 0;
    msg.payload.obd_data.fuel_rate_lph = fuel_rate_lph;
    msg.payload.obd_data.engine_load_valid = engine_load_valid ? 1 : 0;
    msg.payload.obd_data.engine_load_pct = engine_load_pct;
    msg.payload.obd_data.map_valid = map_valid ? 1 : 0;
    msg.payload.obd_data.map_kpa = map_kpa;
    msg.payload.obd_data.fuel_level_valid = fuel_level_valid ? 1 : 0;
    msg.payload.obd_data.fuel_level_pct = fuel_level_pct;
    send(s_controller_mac, msg);
}

static void onMessage(const uint8_t mac[6], const Message &msg, int8_t rssi)
{
    switch (msg.type) {
    case MsgType::HelloAck:
        if (!s_paired) {
            memcpy(s_controller_mac, mac, 6);
            addPeer(s_controller_mac);
            s_paired = true;
            Serial.println("[RELAY] Pareado com o controlador.");
        }
        s_last_controller_msg_ms = millis();
        s_last_rssi = rssi;
        sendStatus();
        break;

    case MsgType::SetRelay:
        if (!s_paired || !macEquals(mac, s_controller_mac)) {
            break;
        }
        applyRelay(msg.payload.set_relay.channel, msg.payload.set_relay.state != 0);
        Serial.printf("[RELAY] Canal %u -> %s\n", msg.payload.set_relay.channel,
                      msg.payload.set_relay.state != 0 ? "LIGADO" : "DESLIGADO");
        s_last_controller_msg_ms = millis();
        s_last_rssi = rssi;
        sendStatus();
        break;

    case MsgType::SetAllRelays:
        if (!s_paired || !macEquals(mac, s_controller_mac)) {
            break;
        }
        applyAll(msg.payload.set_all_relays.mask);
        s_last_controller_msg_ms = millis();
        s_last_rssi = rssi;
        sendStatus();
        break;

    case MsgType::Heartbeat:
        if (!s_paired || !macEquals(mac, s_controller_mac)) {
            break;
        }
        s_last_controller_msg_ms = millis();
        s_last_rssi = rssi;
        break;

    case MsgType::ObdControl:
        if (!s_paired || !macEquals(mac, s_controller_mac)) {
            break;
        }
        {
            bool enable = msg.payload.obd_control.connect != 0;
            xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
            s_obd_enabled = enable;
            xSemaphoreGive(s_obd_mutex);
            Serial.printf("[OBD] %s via comando da tela.\n", enable ? "Habilitado" : "Desabilitado");
        }
        s_last_controller_msg_ms = millis();
        s_last_rssi = rssi;
        sendObdData(); // devolve o status atualizado imediatamente (nao espera o proximo ciclo)
        break;

    case MsgType::DtcControl:
        if (!s_paired || !macEquals(mac, s_controller_mac)) {
            break;
        }
        {
            uint8_t action = (msg.payload.dtc_control.action == 1) ? 2 : 1; // 1=ler, 2=limpar (ver obdTask())
            xSemaphoreTake(s_obd_mutex, portMAX_DELAY);
            s_dtc_action_request = action;
            xSemaphoreGive(s_obd_mutex);
            Serial.printf("[OBD] Pedido de %s DTCs recebido da tela.\n", action == 2 ? "limpar" : "ler");
        }
        s_last_controller_msg_ms = millis();
        s_last_rssi = rssi;
        // A resposta (MsgType::DtcData) sai de obdTask() quando o pedido
        // terminar - a leitura/limpeza e lenta demais (varios comandos
        // AT/PID sequenciais) para fazer aqui dentro do callback do ESP-NOW.
        break;

    default:
        break;
    }
}

// ======================= Arduino entry points ===============================

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESPRemotecontrol - Modulo de Reles ===");

    for (uint8_t ch = 0; ch < kRelayChannels; ch++) {
        pinMode(RELAY_PINS[ch], OUTPUT);
    }
    allRelaysOff();

    s_mpu_available = imu::begin(MPU6050_SDA_PIN, MPU6050_SCL_PIN);
    Serial.println(s_mpu_available
        ? "[RELAY] MPU6050 detectado."
        : "[RELAY] MPU6050 NAO detectado (verifique fiacao/pinagem I2C).");
    pinMode(WATER_LEVEL_PIN, INPUT);
    analogSetPinAttenuation(WATER_LEVEL_PIN, ADC_11db); // faixa ~0-3.1V

    s_obd_mutex = xSemaphoreCreateMutex();
    SerialBT.begin("ESPRemote_Relay", true); // true = modo master, para conectar no ELM327
    xTaskCreate(obdTask, "obdTask", 4096, nullptr, 1, nullptr);

    if (!begin(PAIRING_KEY, WIFI_CHANNEL, onMessage)) {
        Serial.println("ERRO: falha ao iniciar ESP-NOW!");
    }

    uint8_t mac[6];
    getOwnMac(mac);
    char macstr[18];
    macToStr(mac, macstr);
    Serial.printf("MAC deste modulo: %s (canal %u)\n", macstr, WIFI_CHANNEL);
}

void loop()
{
    uint32_t now = millis();

    if (!s_paired && (now - s_last_hello_ms >= HELLO_INTERVAL_MS)) {
        s_last_hello_ms = now;
        Message hello;
        hello.type = MsgType::Hello;
        sendBroadcast(hello);
    }

    if (s_mpu_available && (now - s_last_imu_read_ms >= IMU_READ_INTERVAL_MS)) {
        s_last_imu_read_ms = now;
        s_last_attitude = imu::read();
    }

    if (s_paired && (now - s_last_status_ms >= STATUS_INTERVAL_MS)) {
        s_last_status_ms = now;
        sendStatus();
    }

    if (s_paired && (now - s_last_obd_send_ms >= OBD_SEND_INTERVAL_MS)) {
        s_last_obd_send_ms = now;
        sendObdData();
    }

    // Diferenca COM SINAL de proposito: s_last_controller_msg_ms e atualizado
    // por onMessage() (outra task) com um millis() possivelmente MAIS NOVO
    // que o "now" capturado no inicio deste loop. Com uint32_t, essa
    // subtracao daria ~4 bilhoes e dispararia o failsafe sem motivo (reles
    // desligando e pareamento caindo logo apos cada comando recebido).
    if (s_paired && (int32_t)(now - s_last_controller_msg_ms) > (int32_t)LINK_TIMEOUT_MS) {
        Serial.println("[RELAY] Link perdido - desligando reles por seguranca (failsafe).");
        allRelaysOff();
        s_paired = false;
    }

    delay(20);
}
