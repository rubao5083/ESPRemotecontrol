# CLAUDE.md

Guia para assistentes de IA (Claude Code, Copilot, etc.) trabalhando neste
repositório. Leia antes de propor ou aplicar mudanças.

## O que é este projeto

`ESPRemotecontrol` é um sistema de **controle remoto veicular** com dois
firmwares ESP32-S3 que se comunicam por **ESP-NOW** (não Bluetooth, não
Wi-Fi/AP — veja justificativa em [README.md](./README.md)):

1. **Tela** (`env:lcd_display`, `src/lcd_main/`) — Waveshare ESP32-S3 Touch
   LCD 4.3", roda a UI touchscreen e atua como controlador.
2. **Módulo de relés** (`env:relay_module`, `src/relay_main/`) — hoje um
   Waveshare ESP32-S3-Relay-6CH, no futuro uma placa de relés de maior
   potência (também ESP32); aciona as luzes auxiliares do carro.

Os dois firmwares vivem no **mesmo repositório PlatformIO**, selecionados por
`-e lcd_display` ou `-e relay_module`. Não são projetos separados.

## Regras de projeto (obrigatórias)

1. **Protocolo compartilhado é contrato entre os dois firmwares.**
   Qualquer mudança em [lib/remote_link/remote_protocol.h](./lib/remote_link/remote_protocol.h)
   deve ser aplicada/validada nos DOIS `main.cpp` (`lcd_main` e `relay_main`).
   Ao mudar o layout de `Message`, incremente `kProtocolVersion` e trate
   versões antigas/ignore mensagens com versão diferente (já feito em
   `espnow_link.cpp::onDataRecv`).

2. **`PAIRING_KEY` e `WIFI_CHANNEL` devem ser idênticos** nos dois
   `main.cpp`. Se mudar um, mude o outro. Documentar a mudança no README.

3. **Segurança/failsafe não é opcional.** O módulo de relés DEVE desligar
   todos os canais se perder o link com o controlador por `LINK_TIMEOUT_MS`
   (hoje 10s). Não remova essa lógica nem a torne configurável para "manter
   ligado" sem discutir explicitamente a implicação de segurança (luzes
   presas ligadas no carro).

4. **Não reintroduzir as telas/estilos do projeto IQ50LCD** (`screen_calibracao`,
   `styles.cpp`, `peso_manager`, `ads1263`, `calibration`). Este projeto é
   independente e não tem relação funcional com a balança/ADS1263 do
   IQ50LCD — apenas reaproveita os *drivers de hardware do display*
   (`lvgl_port`, `rgb_lcd_port`, `touch`, `ch422g`, `i2c`), pois é a mesma
   placa física.

5. **QMI8658 só fornece inclinação (roll/pitch), nunca heading/bússola.**
   Não implemente um "heading" a partir apenas do giroscópio/acelerômetro
   (deriva sem correção magnética = não é uma bússola de verdade). Um
   magnetômetro externo é pré-requisito para a bússola — ver roadmap no
   README.

6. **OBD-II ainda não implementado.** Antes de implementar, é preciso decidir
   o tipo de adaptador ELM327 (Wi-Fi ou BLE — **não** Bluetooth Classic/SPP,
   que o ESP32-S3 não suporta) e como ele coexiste com o ESP-NOW (mesmo canal
   Wi-Fi). Não assuma Bluetooth Classic em nenhum código novo.

7. **Pinos de relé (`RELAY_PINS` em `src/relay_main/main.cpp`) são
   específicos da placa atual** (Waveshare ESP32-S3-Relay-6CH:
   GPIO 1, 2, 41, 42, 45, 46). Ao trocar de placa de relés, atualizar esta
   lista e o `board =` do ambiente `relay_module` no `platformio.ini`
   (hoje usa a definição genérica `esp32-s3-devkitc-1` como placeholder).

8. **`lib_ldf_mode = deep+` quebra a resolução de `Network.h` com o core
   arduino-esp32 3.x** (bug conhecido do PlatformIO). Use `deep` (sem `+`)
   no ambiente `lcd_display`, e `chain`/`chain+` no `relay_module`. Não
   reverta para `deep+` sem testar o build.

## Convenções de código

- C++ com Arduino framework, ESP-IDF 5.x por baixo (arduino-esp32 core 3.x).
- Namespaces para módulos novos (`imu::`, `remote_link::`) em vez de
  prefixos tipo `IMU_`/`REMOTE_`.
- Comentários em português (consistente com o restante do projeto); nomes de
  símbolos/funções em inglês.
- Sem alocação dinâmica desnecessária em loops de alta frequência (callback
  ESP-NOW, loop do IMU); `Message` é POD de tamanho fixo (`sizeof <= 200`
  garantido por `static_assert`).
- Pinos, chaves e timeouts ficam como `static const` no topo de cada
  `main.cpp` — não espalhar "números mágicos" pelo código.

## Build / validação

```bash
# Windows (PowerShell), caminho do PlatformIO CLI se não estiver no PATH:
# $env:USERPROFILE\.platformio\penv\Scripts\platformio.exe

platformio run -e lcd_display      # compila o firmware da tela
platformio run -e relay_module     # compila o firmware do módulo de relés
```

Sempre compile **os dois ambientes** depois de mexer em `lib/remote_link/`,
`lib/imu_qmi8658/` ou no `platformio.ini`, já que são compartilhados.

Não há testes automatizados de hardware (não há CI com placas reais). A
validação de pareamento ESP-NOW, failsafe e leitura do IMU precisa ser feita
em bancada com as duas placas.

## Onde procurar antes de perguntar

- Decisão de protocolo de comunicação (ESP-NOW vs BLE/Wi-Fi Direct/Wi-Fi
  oculto) e seu racional: [README.md](./README.md).
- Formato das mensagens: [lib/remote_link/remote_protocol.h](./lib/remote_link/remote_protocol.h).
- Transporte ESP-NOW (pareamento, criptografia, broadcast): [lib/remote_link/espnow_link.cpp](./lib/remote_link/espnow_link.cpp).
- Registradores/escala do IMU: [lib/imu_qmi8658/qmi8658.cpp](./lib/imu_qmi8658/qmi8658.cpp).
- Pinagem do display/touch: [include/ESP_Panel_Board_Custom.h](./include/ESP_Panel_Board_Custom.h).
