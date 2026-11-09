/*****************************************************************************
 * | File        :   main.cpp  (obd2_scan_classic)
 * | Function    :   Firmware TEMPORARIO de diagnostico Bluetooth CLASSICO
 * |                 (SPP) - conecta num adaptador ELM327 Bluetooth Classico
 * |                 (o tipo mais comum e barato) e abre uma ponte serial
 * |                 transparente: tudo que for digitado no monitor serial e
 * |                 enviado ao adaptador (com CR no final, convencao
 * |                 AT/ELM327), e tudo que o adaptador responder aparece no
 * |                 monitor. So roda na placa de RELES (ESP32-WROOM-32E
 * |                 classico) - a tela (ESP32-S3) NAO tem Bluetooth Classico,
 * |                 so BLE (ver README.md).
 * |
 * | Uso: grave este ambiente (-e obd2_scan_classic) na placa de RELES, ligue
 * | o ELM327 no OBD-II do carro e abra o monitor serial (115200 baud):
 * |   1. Digite "scan" para descobrir o adaptador (10s).
 * |   2. Digite "connect <indice>" (ou "connect AA:BB:CC:DD:EE:FF").
 * |   3. Depois de conectado, digite comandos AT diretamente (ex: ATZ) -
 * |      cada linha digitada e enviada com CR, sem precisar de outro prefixo.
 * |   4. "disconnect" encerra a conexao.
 ******************************************************************************/

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <string>
#include <vector>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth classico nao habilitado neste build (CONFIG_BT_ENABLED/CONFIG_BLUEDROID_ENABLED)
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Perfil SPP (Bluetooth Classico serial) nao habilitado - so disponivel no ESP32 classico, nao no S3.
#endif

namespace {

// PIN mais comum entre adaptadores ELM327 Bluetooth Classico baratos.
// So e usado se o adaptador pedir pareamento legado (a maioria usa SSP
// "Just Works" e nem chega a usar isto).
constexpr const char *kFallbackPin = "1234";

BluetoothSerial SerialBT;

struct FoundDevice {
    BTAddress address;
    std::string name;
    int8_t rssi;
    bool has_rssi;
};

std::vector<FoundDevice> s_devices;
bool s_connected = false;

void printHelp()
{
    Serial.println("\nComandos disponiveis:");
    Serial.println("  scan              - descobre dispositivos Bluetooth Classico por perto (10s)");
    Serial.println("  list              - reimprime os dispositivos ja encontrados");
    Serial.println("  connect <indice>  - conecta no dispositivo <indice> da lista");
    Serial.println("  connect <MAC>     - conecta direto por endereco (AA:BB:CC:DD:EE:FF)");
    Serial.println("  disconnect        - encerra a conexao atual");
    Serial.println("  <qualquer outro texto> - enviado direto ao adaptador + CR (ex: ATZ)");
    Serial.println("  help              - mostra esta lista\n");
}

void onDeviceFound(BTAdvertisedDevice *dev)
{
    BTAddress addr = dev->getAddress();
    for (auto &d : s_devices) { // BTAddress::equals() nao e const nesta versao da lib
        if (d.address.equals(addr)) {
            return; // ja listado
        }
    }
    FoundDevice entry;
    entry.address = addr;
    entry.name = dev->haveName() ? dev->getName() : std::string("(sem nome)");
    entry.has_rssi = dev->haveRSSI();
    entry.rssi = dev->haveRSSI() ? dev->getRSSI() : 0;
    s_devices.push_back(entry);

    Serial.printf("[%2u] %-24s %s", static_cast<unsigned>(s_devices.size() - 1),
                  entry.name.c_str(), entry.address.toString().c_str());
    if (entry.has_rssi) {
        Serial.printf("  RSSI=%d", entry.rssi);
    }
    Serial.println();
}

void doScan()
{
    s_devices.clear();
    Serial.println("\n=== Escaneando Bluetooth Classico por 10s ===");
    if (!SerialBT.discoverAsync(onDeviceFound)) {
        Serial.println(">>> Falha ao iniciar o scan (talvez ja conectado?).");
        return;
    }
    delay(10000);
    SerialBT.discoverAsyncStop();
    Serial.printf(">>> Scan finalizado: %u dispositivo(s). Use 'connect <indice>'.\n",
                  static_cast<unsigned>(s_devices.size()));
}

bool doConnect(const BTAddress &address)
{
    if (s_connected) {
        SerialBT.disconnect();
        s_connected = false;
    }
    Serial.printf(">>> Conectando em %s...\n", address.toString().c_str());
    bool ok = SerialBT.connect(address);
    if (ok) {
        Serial.println(">>> Conectado! Digite comandos AT diretamente (ex: ATZ).");
        s_connected = true;
    } else {
        Serial.println(">>> Falha ao conectar.");
    }
    return ok;
}

void doDisconnect()
{
    SerialBT.disconnect();
    s_connected = false;
    Serial.println(">>> Desconectado.");
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
        doScan();
    } else if (cmd == "list") {
        for (size_t i = 0; i < s_devices.size(); i++) {
            Serial.printf("[%2u] %-24s %s\n", static_cast<unsigned>(i),
                          s_devices[i].name.c_str(), s_devices[i].address.toString().c_str());
        }
    } else if (cmd == "connect") {
        if (args.empty()) {
            Serial.println(">>> Uso: connect <indice> ou connect <MAC>");
        } else if (args.find(':') != std::string::npos) {
            doConnect(BTAddress(String(args.c_str())));
        } else {
            int idx = atoi(args.c_str());
            if (idx >= 0 && static_cast<size_t>(idx) < s_devices.size()) {
                doConnect(s_devices[idx].address);
            } else {
                Serial.println(">>> Indice invalido. Use 'list' para ver os dispositivos.");
            }
        }
    } else if (cmd == "disconnect") {
        doDisconnect();
    } else if (cmd == "help") {
        printHelp();
    } else {
        if (!s_connected) {
            Serial.println(">>> Nao conectado. Use 'connect <indice>' primeiro, ou 'help'.");
            return;
        }
        std::string payload = line + "\r";
        SerialBT.write(reinterpret_cast<const uint8_t *>(payload.data()), payload.size());
        Serial.printf(">> enviado: %s\\r\n", line.c_str());
    }
}

} // namespace

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== obd2_scan_classic - diagnostico Bluetooth Classico (SPP) para ELM327 ===");

    SerialBT.begin("ESPRemote_OBD_scan", true); // true = modo master, para poder conectar a um ELM327
    SerialBT.setPin(kFallbackPin, strlen(kFallbackPin));

    printHelp();
    doScan();
}

void loop()
{
    static std::string line_buf;

    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        if (c == '\n') {
            while (!line_buf.empty() && (line_buf.back() == '\r')) {
                line_buf.pop_back();
            }
            handleLine(line_buf);
            line_buf.clear();
        } else {
            line_buf += c;
        }
    }

    // Ponte transparente: qualquer byte recebido do adaptador aparece no Serial.
    while (SerialBT.available() > 0) {
        Serial.write(SerialBT.read());
    }
}
