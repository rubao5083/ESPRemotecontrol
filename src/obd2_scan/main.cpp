/*****************************************************************************
 * | File        :   main.cpp  (obd2_scan)
 * | Function    :   Firmware TEMPORARIO de diagnostico BLE - conecta num
 * |                 adaptador OBD-II BLE (candidato atual: scanner "V800" /
 * |                 app INNOOBD+) e despeja no Serial todos os servicos e
 * |                 caracteristicas GATT reais do dispositivo, alem de
 * |                 permitir escrever comandos AT/ELM327 manualmente e ver
 * |                 as notificacoes de resposta. O protocolo GATT do V800
 * |                 nao e documentado publicamente (ao contrario do vLinker
 * |                 MC+ ja cotado no Roadmap do README.md); este firmware
 * |                 existe para descobrir o protocolo real em bancada antes
 * |                 de escrever o cliente OBD-II definitivo.
 * |
 * | Uso:
 * |   1. Grave este ambiente (-e obd2_scan) na placa da TELA (ESP32-S3) e
 * |      abra o monitor serial (115200 baud).
 * |   2. Ligue o adaptador no conector OBD-II do carro (alimentado pelo
 * |      proprio chicote, nao precisa do motor ligado).
 * |   3. O firmware ja inicia um scan BLE automaticamente. Anote o indice
 * |      do dispositivo que aparecer com nome "V800" (ou similar).
 * |   4. Digite "connect <indice>" e tecle enter.
 * |   5. Apos conectar, a lista numerada de caracteristicas GATT e impressa.
 * |      Caracteristicas com notificacao/indicacao sao inscritas automatica-
 * |      mente - qualquer dado recebido aparece no Serial em hex + ASCII.
 * |   6. Digite "use <indice>" para escolher a caracteristica de escrita,
 * |      depois "write ATZ" (ou "hex 41 54 5A 0D") para testar comandos.
 * |
 * | Comandos: scan | list | connect <n> | use <n> | write <texto> |
 * |           hex <bytes hex> | read | disconnect | help
 ******************************************************************************/

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string>
#include <vector>

namespace {

constexpr uint32_t kDefaultScanMs = 8000;

struct DiscoveredDevice {
    NimBLEAddress address;
    std::string name;
    int rssi;
};

struct CharEntry {
    NimBLERemoteCharacteristic *chr;
    std::string service_uuid;
};

std::vector<DiscoveredDevice> s_devices;
std::vector<CharEntry> s_chars;
NimBLEClient *s_client = nullptr;
int s_active_char = -1;

void printHelp()
{
    Serial.println("\nComandos disponiveis:");
    Serial.println("  scan                 - inicia um novo scan BLE (8s)");
    Serial.println("  list                 - reimprime os dispositivos ja encontrados");
    Serial.println("  connect <indice>     - conecta no dispositivo <indice> da lista");
    Serial.println("  use <indice>         - escolhe a caracteristica <indice> para escrita");
    Serial.println("  write <texto>        - escreve <texto> + CR na caracteristica ativa");
    Serial.println("  hex <AA BB CC ...>   - escreve bytes crus (hex) na caracteristica ativa");
    Serial.println("  read                 - le o valor atual da caracteristica ativa (se suportado)");
    Serial.println("  disconnect           - desconecta do dispositivo atual");
    Serial.println("  help                 - mostra esta lista\n");
}

void printHexAndAscii(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.print(" | ");
    for (size_t i = 0; i < len; i++) {
        char c = static_cast<char>(data[i]);
        Serial.print((c >= 32 && c < 127) ? c : '.');
    }
    Serial.println();
}

std::string charFlags(NimBLERemoteCharacteristic *chr)
{
    std::string flags;
    if (chr->canRead()) flags += "R ";
    if (chr->canWrite()) flags += "W ";
    if (chr->canWriteNoResponse()) flags += "WNR ";
    if (chr->canNotify()) flags += "N ";
    if (chr->canIndicate()) flags += "I ";
    if (flags.empty()) flags = "-";
    return flags;
}

void onNotify(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify)
{
    Serial.printf("<< %s svc=%s char=%s (%u bytes): ",
                   isNotify ? "NOTIFY" : "INDICATE",
                   chr->getRemoteService()->getUUID().toString().c_str(),
                   chr->getUUID().toString().c_str(),
                   static_cast<unsigned>(len));
    printHexAndAscii(data, len);
}

class ClientCallbacksImpl : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient *pClient) override
    {
        Serial.println(">>> Conectado. Descobrindo servicos/caracteristicas...");
    }

    void onDisconnect(NimBLEClient *pClient, int reason) override
    {
        Serial.printf(">>> Desconectado (motivo=%d).\n", reason);
        s_chars.clear();
        s_active_char = -1;
    }
};

class ScanCallbacksImpl : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override
    {
        for (const auto &d : s_devices) {
            if (d.address == dev->getAddress()) {
                return; // ja listado (varios pacotes de advertising do mesmo device)
            }
        }
        DiscoveredDevice entry;
        entry.address = dev->getAddress();
        entry.name = dev->haveName() ? dev->getName() : std::string("(sem nome)");
        entry.rssi = dev->getRSSI();
        s_devices.push_back(entry);
        Serial.printf("[%2u] %-24s %s  RSSI=%d\n",
                      static_cast<unsigned>(s_devices.size() - 1),
                      entry.name.c_str(), entry.address.toString().c_str(), entry.rssi);
    }

    void onScanEnd(const NimBLEScanResults &results, int reason) override
    {
        Serial.printf(">>> Scan finalizado: %u dispositivo(s). Use 'connect <indice>'.\n",
                      static_cast<unsigned>(s_devices.size()));
    }
};

ClientCallbacksImpl s_client_callbacks;
ScanCallbacksImpl s_scan_callbacks;

void doScan(uint32_t duration_ms)
{
    s_devices.clear();
    Serial.printf("\n=== Escaneando BLE por %lums ===\n", static_cast<unsigned long>(duration_ms));
    NimBLEDevice::getScan()->start(duration_ms, false, true);
}

void doDiscover()
{
    s_chars.clear();
    s_active_char = -1;

    const auto &services = s_client->getServices(true);
    Serial.printf("\n=== %u servico(s) encontrado(s) ===\n", static_cast<unsigned>(services.size()));
    for (auto *svc : services) {
        Serial.printf("Service %s\n", svc->getUUID().toString().c_str());
        for (auto *chr : svc->getCharacteristics(true)) {
            size_t idx = s_chars.size();
            s_chars.push_back({chr, svc->getUUID().toString()});
            Serial.printf("  [%2u] char=%s props=%s\n",
                          static_cast<unsigned>(idx), chr->getUUID().toString().c_str(),
                          charFlags(chr).c_str());

            if (chr->canNotify() || chr->canIndicate()) {
                bool ok = chr->subscribe(chr->canNotify(), onNotify);
                Serial.printf("        -> inscrito para %s: %s\n",
                              chr->canNotify() ? "notify" : "indicate", ok ? "OK" : "FALHOU");
            }
        }
    }
    Serial.println("=== Fim da lista. Use 'use <indice>' para escrever. ===\n");
}

bool doConnect(int idx)
{
    if (idx < 0 || static_cast<size_t>(idx) >= s_devices.size()) {
        Serial.println(">>> Indice invalido. Use 'list' para ver os dispositivos.");
        return false;
    }

    if (s_client && s_client->isConnected()) {
        s_client->disconnect();
    }
    if (!s_client) {
        s_client = NimBLEDevice::createClient();
        s_client->setClientCallbacks(&s_client_callbacks, false);
        s_client->setConnectTimeout(8 * 1000);
    }

    const DiscoveredDevice &target = s_devices[idx];
    Serial.printf(">>> Conectando em %s (%s)...\n", target.name.c_str(), target.address.toString().c_str());
    if (!s_client->connect(target.address)) {
        Serial.println(">>> Falha ao conectar.");
        return false;
    }

    doDiscover();
    return true;
}

void doDisconnect()
{
    if (s_client && s_client->isConnected()) {
        s_client->disconnect();
    }
}

bool parseHexBytes(const std::string &text, std::vector<uint8_t> &out)
{
    std::string digits;
    for (char c : text) {
        if (isxdigit(static_cast<unsigned char>(c))) {
            digits += c;
        }
    }
    if (digits.empty() || (digits.size() % 2) != 0) {
        return false;
    }
    out.clear();
    for (size_t i = 0; i < digits.size(); i += 2) {
        out.push_back(static_cast<uint8_t>(strtoul(digits.substr(i, 2).c_str(), nullptr, 16)));
    }
    return true;
}

bool writeToActive(const uint8_t *data, size_t len)
{
    if (s_active_char < 0 || static_cast<size_t>(s_active_char) >= s_chars.size()) {
        Serial.println(">>> Nenhuma caracteristica ativa. Use 'use <indice>' primeiro.");
        return false;
    }
    NimBLERemoteCharacteristic *chr = s_chars[s_active_char].chr;
    bool with_response = chr->canWrite();
    bool ok = chr->writeValue(data, len, with_response);
    Serial.printf(">> escrito (%u bytes, %s): ", static_cast<unsigned>(len),
                  with_response ? "with response" : "no response");
    printHexAndAscii(data, len);
    if (!ok) {
        Serial.println(">>> Falha ao escrever.");
    }
    return ok;
}

void handleLine(const std::string &line)
{
    if (line.empty()) {
        return;
    }

    size_t sp = line.find(' ');
    std::string cmd = (sp == std::string::npos) ? line : line.substr(0, sp);
    std::string args = (sp == std::string::npos) ? std::string() : line.substr(sp + 1);

    if (cmd == "scan") {
        doScan(kDefaultScanMs);
    } else if (cmd == "list") {
        for (size_t i = 0; i < s_devices.size(); i++) {
            Serial.printf("[%2u] %-24s %s  RSSI=%d\n", static_cast<unsigned>(i),
                          s_devices[i].name.c_str(), s_devices[i].address.toString().c_str(),
                          s_devices[i].rssi);
        }
    } else if (cmd == "connect") {
        doConnect(atoi(args.c_str()));
    } else if (cmd == "use") {
        int idx = atoi(args.c_str());
        if (idx >= 0 && static_cast<size_t>(idx) < s_chars.size()) {
            s_active_char = idx;
            Serial.printf(">>> Caracteristica ativa: [%d] %s\n", idx,
                          s_chars[idx].chr->getUUID().toString().c_str());
        } else {
            Serial.println(">>> Indice invalido. Conecte primeiro e veja a lista impressa.");
        }
    } else if (cmd == "write") {
        std::string payload = args + "\r"; // convencao AT/ELM327: comandos terminam em CR
        writeToActive(reinterpret_cast<const uint8_t *>(payload.data()), payload.size());
    } else if (cmd == "hex") {
        std::vector<uint8_t> bytes;
        if (parseHexBytes(args, bytes)) {
            writeToActive(bytes.data(), bytes.size());
        } else {
            Serial.println(">>> Hex invalido (use pares de digitos, ex: 41 54 5A 0D).");
        }
    } else if (cmd == "read") {
        if (s_active_char < 0 || static_cast<size_t>(s_active_char) >= s_chars.size()) {
            Serial.println(">>> Nenhuma caracteristica ativa.");
        } else {
            NimBLERemoteCharacteristic *chr = s_chars[s_active_char].chr;
            if (!chr->canRead()) {
                Serial.println(">>> Caracteristica ativa nao suporta leitura.");
            } else {
                auto value = chr->readValue();
                Serial.print("<< leitura: ");
                printHexAndAscii(reinterpret_cast<const uint8_t *>(value.data()), value.size());
            }
        }
    } else if (cmd == "disconnect") {
        doDisconnect();
    } else if (cmd == "help") {
        printHelp();
    } else {
        Serial.printf(">>> Comando desconhecido: '%s'. Digite 'help'.\n", cmd.c_str());
    }
}

} // namespace

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== obd2_scan - diagnostico BLE para adaptador OBD-II ===");
    Serial.println("Candidato atual em investigacao: scanner V800 / app INNOOBD+.");

    NimBLEDevice::init("obd2_scan");
    NimBLEDevice::setPower(9); // +9dbm, maximiza alcance ate o conector OBD-II do carro

    NimBLEScan *pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(&s_scan_callbacks, false);
    pScan->setInterval(100);
    pScan->setWindow(100);
    pScan->setActiveScan(true); // pede o scan response (geralmente contem o nome do device)

    printHelp();
    doScan(kDefaultScanMs);
}

void loop()
{
    static std::string line_buf;

    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        if (c == '\n') {
            // remove possivel '\r' residual do fim de linha
            while (!line_buf.empty() && (line_buf.back() == '\r')) {
                line_buf.pop_back();
            }
            handleLine(line_buf);
            line_buf.clear();
        } else {
            line_buf += c;
        }
    }
}
