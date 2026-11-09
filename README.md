# ESPRemotecontrol

Controle remoto veicular baseado em dois módulos ESP32 comunicando-se via
**ESP-NOW**: uma tela touchscreen (Waveshare ESP32-S3 Touch LCD 4.3") que
comanda remotamente um módulo de relés (Waveshare ESP32-S3-Relay-6CH, a ser
futuramente substituído por uma placa de relés de maior potência).

## Hardware

| Papel | Placa atual | MCU | Observações |
|---|---|---|---|
| Tela / Controlador | Waveshare ESP32-S3 Touch LCD 4.3" | ESP32-S3 | Display RGB 800x480, touch GT911; IMU QMI8658 **não presente nesta unidade** (ver seção IMU abaixo) |
| Módulo de relés | "ESP32_Relay_30A_X4" | ESP32-WROOM-32E | 4 canais 30A; sem porta USB (grava-se via header serial TTL, ver abaixo); placa anterior era a Waveshare ESP32-S3-Relay-6CH |

A tela usa ESP32-S3, que **não possui Bluetooth Classic** (só BLE). O módulo
de relés já migrou para o ESP32-WROOM-32E, um ESP32 clássico com Bluetooth
Classic de verdade (ver Roadmap).

## Arquitetura / Protocolo de comunicação

Este projeto contém **dois firmwares** no mesmo repositório PlatformIO,
selecionados por ambiente (veja [platformio.ini](./platformio.ini)):

```
platformio run -e lcd_display     # firmware da tela (src/lcd_main)
platformio run -e relay_module    # firmware do módulo de relés (src/relay_main)
```

### Por que ESP-NOW (e não BLE / Wi-Fi Direct / rede Wi-Fi oculta)?

| Opção | Avaliação |
|---|---|
| **ESP-NOW** (escolhido) | Protocolo nativo Espressif, ponto-a-ponto, sem necessidade de roteador/AP. Baixíssima latência (~ms), sem handshake de conexão/pareamento BLE, reconecta sozinho após perda de sinal (útil em ambiente automotivo com vibração/ruído). Suporta criptografia AES-CCM por peer. Alcance tipicamente maior que BLE em campo aberto. |
| Bluetooth Classic (SPP) | **Indisponível** - ESP32-S3 não tem BT Classic, só BLE. |
| BLE | Funciona (ambos os módulos são S3), mas exige gerenciamento de conexão/reconexão, descoberta de serviços (GATT) e tem mais overhead para um caso de uso simples de comandos on/off. Pior opção que ESP-NOW aqui. |
| Wi-Fi Direct (P2P) | Suporte nativo no ESP-IDF é parcial/pouco documentado; a Espressif recomenda ESP-NOW para este tipo de controle dispositivo-a-dispositivo. |
| Rede Wi-Fi oculta (SSID oculto) + AP/estação | Exigiria manter um dos módulos como Access Point e lidar com conexão/IP/sockets - mais complexo, mais consumo, e sem ganho real para comandos simples. ESP-NOW já é "invisível" no sentido de não expor nenhum SSID. |

**Decisão:** ESP-NOW criptografado (chave derivada de uma senha compartilhada
em `PAIRING_KEY`, igual nos dois firmwares) em um canal Wi-Fi fixo (`WIFI_CHANNEL`).

### Fluxo de pareamento (automático, sem precisar configurar MAC manualmente)

1. O módulo de relés, enquanto não pareado, transmite `Hello` em broadcast a cada 1s.
2. A tela escuta broadcasts; ao receber o primeiro `Hello`, registra o MAC do
   relé como peer criptografado e responde `HelloAck` **também em broadcast**
   (não criptografado). Isso é proposital: o relé só registra o MAC da tela
   como peer criptografado ao processar esse `HelloAck` — se a tela enviasse
   o ACK unicast já criptografado, o relé ainda não teria a chave e o pacote
   seria descartado silenciosamente pela pilha ESP-NOW (bug já corrigido, ver
   [docs/HISTORICO-DEPURACAO.md](./docs/HISTORICO-DEPURACAO.md)).
3. A partir daí: a tela envia `SetRelay` / `SetAllRelays` / `Heartbeat`
   (unicast, criptografado); o relé responde com `StatusReport` (estado dos 6
   canais + uptime + RSSI) a cada 1s, também unicast criptografado.
4. **Failsafe:** se o relé não receber nada do controlador por 10s, ele
   **desliga todos os relés** e volta a transmitir `Hello` (evita luzes presas
   ligadas se o link cair). A tela, se não receber `StatusReport` por 8s,
   marca a conexão como "offline" na UI e volta a aceitar um novo `Hello`.

Protocolo definido em [lib/remote_link/remote_protocol.h](./lib/remote_link/remote_protocol.h),
transporte em [lib/remote_link/espnow_link.h](./lib/remote_link/espnow_link.h) /
[espnow_link.cpp](./lib/remote_link/espnow_link.cpp).

## Firmware da tela (`src/lcd_main/main.cpp`)

- Inicializa o painel RGB, backlight, touch GT911 e LVGL (reaproveitando os
  drivers de hardware do projeto IQ50LCD: `rgb_lcd_port`, `lvgl_port`, `touch`,
  `ch422g`, `i2c` — **sem** reaproveitar as telas/estilos do projeto antigo).
- UI com **quatro telas**, navegadas por gesto de deslizar (swipe) o dedo:
  - **Tela principal**: 4 botões (`AUX 1`..`AUX 4`) para os canais de relé,
    indicador de conexão com o módulo de relés, e inclinação resumida.
  - **Tela OBD-II** (deslizar para a esquerda): 4 medidores circulares -
    inclinação lateral/frontal (dados reais do IMU onboard) e temperatura de
    motor (dado real via ELM327, ver seção OBD-II abaixo)/câmbio (sem PID
    padrão - ver Roadmap). Deslizar para a direita volta à tela principal.
  - **Tela OBD-II - Informações gerais** (deslizar para a esquerda de novo,
    a partir da tela OBD-II): os parâmetros mais comuns em HUDs OBD-II
    comerciais - velocidade, RPM, temperatura do motor, tensão da bateria e
    consumo de combustível estimado - mais carga do motor, MAP (pressão do
    coletor de admissão) e nível do tanque de combustível numa 2ª coluna
    (PIDs extras confirmados suportados no veículo de teste, ver
    [tabela.md](./tabela.md)) - além de um botão **Conectar/Desconectar**
    que liga/desliga a conexão Bluetooth Clássico da placa de relés com o
    ELM327 (`MsgType::ObdControl`). Deslizar para a direita volta à tela
    OBD-II (medidores); deslizar para a esquerda vai para a tela de DTCs.
  - **Tela de DTCs - códigos de falha** (deslizar para a esquerda de novo,
    a partir da tela de informações gerais): status do MIL (luz de check
    engine), lista rolável dos códigos armazenados (Modo 03)/pendentes
    (Modo 07) decodificados no formato padrão (ex. `P0301`), e botões
    **Ler DTCs**/**Limpar códigos** (Modo 04) que pedem a operação à placa
    de relés via `MsgType::DtcControl` - resposta via `MsgType::DtcData`.
    Leitura **sob demanda** (não fica repetindo sozinha - é mais lenta que
    o ciclo normal de telemetria). Deslizar para a direita volta à tela de
    informações gerais.
  - **Indicador de bateria** (ícone no canto superior direito, presente nas
    4 telas): percentual de carga da LiPo ligada no conector da própria
    placa (não confundir com a "tensão da bateria" do veículo, acima, que
    vem do OBD-II) - ver seção dedicada abaixo.
- Lê o IMU **QMI8658** (acelerômetro + giroscópio onboard) a cada 200ms via
  filtro complementar para obter ângulos de inclinação suavizados,
  alimentando tanto o rótulo da tela principal quanto os medidores da tela
  OBD-II.

## Firmware do módulo de relés (`src/relay_main/main.cpp`)

- `kRelayChannels` (em [lib/remote_link/remote_protocol.h](./lib/remote_link/remote_protocol.h),
  compartilhado pelos dois firmwares) é **4** - reduzido de 6, pois o uso real
  não precisa de mais que isso.
- **Migrado** para a placa "ESP32_Relay_30A_X4" (ESP32-WROOM-32E, **não** é
  S3 - tem Bluetooth Clássico de verdade). Pinout confirmado pela comunidade
  ESPHome: `GPIO32/33/25/26` para os 4 relés (`RELAY_PINS` em
  `src/relay_main/main.cpp`), `GPIO23` para o LED de status (ainda não usado
  pelo firmware). `board = esp32dev` no ambiente `relay_module` do
  `platformio.ini` (troca de chip em relação à Waveshare ESP32-S3-Relay-6CH
  anterior, que usava `GPIO 1, 2, 41, 42` - ver regra 7 do
  [CLAUDE.md](./CLAUDE.md)). A flag comum `ARDUINO_USB_CDC_ON_BOOT=1` (pensada
  para o USB nativo do S3 da tela) é sobrescrita para `0` só no ambiente
  `relay_module`, já que o ESP32-WROOM-32E clássico não tem USB nativo e
  `Serial` precisa mapear para a UART0 de sempre.
- **Gravação sem porta USB**: a placa expõe um header serial de 6 pinos
  (`5V, TX0, RX0, GND, IO0, GND`) e precisa de um adaptador USB-serial TTL
  externo. Veja a seção "Gravando o módulo de relés" abaixo.
- `RELAY_ACTIVE_HIGH`: assumido nível lógico ativo-alto para acionar os relés;
  **verifique com um teste real** e inverta se os relés ligarem/desligarem trocado.
- Falhas de segurança: desliga todos os relés se perder o link com o controlador.

### Gravando o módulo de relés (sem porta USB)

A placa não tem conversor USB-serial embutido; grava-se pelo header de 6 pinos
acima do módulo ESP32-WROOM-32E (`5V, TX0, RX0, GND, IO0, GND`) com um
adaptador USB-TTL externo (ex.: CP2102/CH340/FTDI, ou o "ZXacc-esp08-v20"
usado neste projeto, que tem pinos `5V0, 3V3, RXD, TXD, EN, BOOT, GND`). A
placa também tem um **botão físico `RST`** (perto dos conectores dos relés,
rotulado `S2` na serigrafia) ligado ao reset do ESP32 - use-o em vez de
tentar levar o `EN` do adaptador até a placa (esse header não o expõe).

| Placa (header) | Adaptador | Observação |
|---|---|---|
| `5V`  | `5V0` | Alimenta a placa pelo header durante a gravação |
| `GND` | `GND` | Use qualquer um dos dois `GND` do header |
| `TX0` | `RXD` | Cruzado: TX do ESP32 -> RX do adaptador |
| `RX0` | `TXD` | Cruzado: RX do ESP32 -> TX do adaptador |
| `IO0` | — (jumper manual para `GND`) | Ver passo a passo abaixo |
| — | `EN`, `BOOT` do adaptador | Não usar / deixar desconectados |

Procedimento (mais confiável que depender do `EN`/`BOOT` do adaptador, que
não têm como chegar ao reset do ESP32 por este header):

1. Ligue `5V0`→`5V`, `GND`→`GND`, `RXD`→`TX0`, `TXD`→`RX0`.
2. Com um fio avulso, curto-circuite `IO0` no outro `GND` do mesmo header
   (mantém o GPIO0 em nível baixo).
3. Conecte o adaptador no USB do PC (a placa liga, alimentada pelo `5V`).
4. Aperte e solte o botão `RST` da placa **com o jumper de `IO0` ainda em
   `GND`** - isso reinicia o ESP32 já em modo bootloader (download).
5. Rode o upload apontando a porta COM do adaptador:
   ```bash
   platformio run -e relay_module --target upload --upload-port COMx
   ```
6. Depois de gravar com sucesso, **remova o jumper `IO0`-`GND`** e aperte
   `RST` de novo (ou religue a alimentação) para rodar o firmware normal.

Se o `esptool` não conseguir conectar, repita o passo 4 (jumper em `GND`,
aperte `RST`) enquanto o upload tenta reconectar - a janela de conexão do
`esptool` costuma tentar por alguns segundos antes de desistir.

> **Atenção à corrente**: o rádio Wi-Fi do ESP32 puxa picos de ~400mA durante
> transmissão (ESP-NOW). Adaptadores USB-serial baratos alimentados só pela
> porta USB do PC podem não sustentar isso e causar reset/brownout. Para
> gravar é geralmente suficiente, mas para testes com o ESP-NOW ativo prefira
> alimentar a placa pela sua entrada de força normal (5-30VDC) e ligar só
> `GND`/`TX0`/`RX0`/`IO0` ao adaptador (sem o `5V`), com os GNDs em comum.

## IMU onboard (inclinação) — `lib/imu_qmi8658/`

O driver em [lib/imu_qmi8658/qmi8658.cpp](./lib/imu_qmi8658/qmi8658.cpp)
fornece `roll`/`pitch` (inclinação) via filtro complementar **se** um
QMI8658 responder no barramento I2C compartilhado com o touch/expansor
(SDA=GPIO8, SCL=GPIO9).

> **Confirmado: a unidade física usada neste projeto NÃO tem o QMI8658
> montado**, apesar da especificação genérica de marketing do produto
> mencionar esse sensor. Confirmado por uma varredura completa do barramento
> I2C (só CH422G, GT911 e um chip de RTC respondem; nada nos endereços
> `0x6A`/`0x6B` do QMI8658) e pela ausência de qualquer menção ao QMI8658 no
> esquemático oficial e no pacote de exemplos oficial da Waveshare para este
> modelo exato — ver [docs/HISTORICO-DEPURACAO.md](./docs/HISTORICO-DEPURACAO.md).
> O driver continua no projeto (`imu::begin()` retorna `false` de forma
> limpa, sem travar nada) para o caso de uma unidade futura vir com o sensor
> populado, ou para quando um módulo QMI8658 externo for ligado ao mesmo
> barramento I2C.

> **Bússola (heading magnético): NÃO implementada.** O QMI8658 não tem
> magnetômetro. Para uma bússola de verdade é necessário um sensor externo
> (ex.: QMC5883L) no mesmo barramento I2C — hardware ainda não instalado.
> A estrutura `imu::Attitude` já tem os campos `heading_deg`/`has_heading`
> prontos para quando esse sensor for adicionado.

## Roadmap / não implementado ainda

- **Indicador de bateria da placa (não do veículo): IMPLEMENTADO em
  software, AGUARDANDO FIAÇÃO FÍSICA** - ícone + percentual no canto
  superior direito das 4 telas, lendo um **INA219 EXTERNO** via o terminal
  I2C da placa (`lib/battery_ina219/`). Por quê um sensor externo:
  - A Waveshare ESP32-S3-Touch-LCD-4.3B usa o conector MX1.25 (J5 no
    esquemático) + o chip gerenciador de carga **CS8501** para a bateria
    LiPo single-cell, com uma chave **SW1** ("Battery Connection Switch",
    item 10 da doc oficial) em série entre `J5` e a rede `VBAT` interna da
    placa - o próprio esquemático alerta para desligar essa chave ao
    debugar via USB. Conferido no esquemático oficial: o CS8501 só eleva
    a tensão da bateria para alimentar o sistema (5V) - **não existe
    nenhum traço que leve a tensão da bateria até um GPIO do ESP32-S3**.
    E mais: o pino que parecia "livre" nas specs genéricas (GPIO6) na
    verdade está ligado ao `RTC_INT` (interrupção do RTC PCF85063 onboard)
    segundo a tabela de mapeamento de pinos do esquemático oficial
    (folha "ESP32-S3-WROOM-1" do PDF) - ou seja, nem soldando direto no
    módulo haveria um pino realmente livre e seguro para ADC.
  - Solução adotada: um módulo **INA219** (monitor de tensão/corrente I2C,
    endereço `0x40` por padrão - não colide com touch/CH422G/IMU/RTC)
    ligado em série no fio `VBAT`, preferencialmente emendado nos
    capacitores `C50`/`C51` do esquemático da bateria - fisicamente perto
    do `U4`/CS8501, canto superior esquerdo da placa (ou seja, DEPOIS da
    chave `SW1`, para a leitura refletir a rede `VBAT` interna
    independente da posição dessa chave) - e nos pinos `SDA`/`SCL`/`VOUT`/
    `GND` do terminal parafuso grande serigrafado **"PORT1"** (mesmo
    barramento SDA=GPIO8/SCL=GPIO9 do touch/IMU; `VOUT` sai em 3.3V por
    padrão, selecionável para 5V pelo jumper `H8` perto do USB).
  - O firmware converte a tensão lida (`bus_voltage + queda no shunt`) em
    percentual usando uma curva de descarga típica de LiPo 1S (ver
    `kDischargeCurve` em [lib/battery_ina219/battery.cpp](./lib/battery_ina219/battery.cpp))
    - é uma ESTIMATIVA por tensão, não um fuel-gauge de precisão
    (coulomb counting) como um MAX17048.
  - **Enquanto o INA219 não for fisicamente ligado**, `battery::begin()`
    falha de forma limpa e a UI mostra "sem sensor" em cinza (mesmo padrão
    de degradação do IMU QMI8658 - ver seção abaixo).
  - Valores ajustáveis no topo de `battery.cpp` caso o módulo físico use
    outro endereço I2C ou outro resistor shunt: `kAddress`,
    `kShuntResistanceOhm`, `kMaxExpectedCurrentA`.

- **OBD-II: IMPLEMENTADO (02/11/2026, estendido em mesma data)** - leitura
  real do carro via um adaptador **ELM327 genérico (Bluetooth Clássico/
  SPP)**, cobrindo os parâmetros mais comuns em HUDs OBD-II comerciais:
  velocidade, RPM, temperatura do motor, tensão da bateria e consumo de
  combustível estimado. Arquitetura (decidida e confirmada em bancada com
  o carro real):
  - Como o ELM327 usado é **Bluetooth Clássico**, só a placa de relés
    (ESP32-WROOM-32E **clássico**) consegue falar com ele - a tela
    (ESP32-**S3**) não tem Bluetooth Clássico, só BLE (limitação de
    hardware do chip). Por isso a leitura roda na placa de relés:
    - `src/relay_main/main.cpp`: conecta no ELM327 via `BluetoothSerial`
      (MAC fixo em `OBD_ELM327_MAC`, descoberto com `obd2_scan_classic` -
      troque esse valor se usar outro adaptador), numa **FreeRTOS task
      separada** (`obdTask`) para não bloquear `loop()` - o failsafe de
      segurança (desligar relés se perder o link com a tela) não pode
      ficar refém do tempo de conexão/resposta do Bluetooth Clássico (ver
      regra 3 do [CLAUDE.md](./CLAUDE.md)). A cada ~2s consulta os PIDs
      Modo 01 `0105` (temperatura do motor), `010C` (RPM), `010D`
      (velocidade), `0142` (tensão do módulo de controle), `0110` (MAF -
      usado só para estimar o consumo), `0104` (carga do motor), `010B`
      (MAP - pressão absoluta do coletor de admissão) e `012F` (nível do
      tanque de combustível), e envia tudo para a tela via ESP-NOW
      (`MsgType::ObdData`).
    - **Consumo de combustível é uma ESTIMATIVA**, calculada a partir do
      MAF (fluxo de ar) com a fórmula clássica usada por scangauges/apps
      tipo Torque: `L/h = (MAF_g/s * 3600) / (AFR * densidade_combustível)`,
      assumindo AFR 14.5:1 e densidade do diesel 832 g/L (`OBD_FUEL_AFR` /
      `OBD_FUEL_DENSITY_G_PER_L` em `src/relay_main/main.cpp`) - não existe
      um PID Modo 01 padrão que devolva litros/hora diretamente para a
      maioria dos veículos. Varia com qualidade/temperatura do combustível
      e a mistura real (lambda), que o MAF sozinho não informa.
    - A tela pode **ligar/desligar** essa conexão (`MsgType::ObdControl`,
      botão "Conectar"/"Desconectar" na 3ª tela, ver abaixo) - útil para
      liberar o ELM327 para outro dispositivo (ex.: celular/app de
      diagnóstico) sem precisar desligar a placa de relés.
    - [lib/remote_link/remote_protocol.h](./lib/remote_link/remote_protocol.h):
      `MsgType::ObdData` estendido com RPM/velocidade/tensão/consumo
      estimado/carga do motor/MAP/nível de combustível, e os `MsgType::
      ObdControl`/`MsgType::DtcControl`/`MsgType::DtcData` (`kProtocolVersion`
      incrementado para 4 - regra 1 do CLAUDE.md; **os dois firmwares
      precisam estar na mesma versão do protocolo ou param de se
      enxergar** - foi exatamente isso que aconteceu na primeira tentativa
      desta sessão: a placa de relés atualizada parou de parear com a
      tela ainda no firmware antigo).
    - `src/lcd_main/main.cpp`: recebe `MsgType::ObdData` e atualiza o
      medidor "Temp. motor" na tela OBD-II, além da 3ª tela ("OBD-II -
      Informações gerais") com velocidade/RPM/temperatura/tensão/consumo/
      carga do motor/MAP/nível de combustível em texto e o botão Conectar/
      Desconectar.
    - `board_build.partitions = huge_app.csv` no ambiente `relay_module`
      (`platformio.ini`): a tabela de partições padrão do `esp32dev`
      (~1.31MB de app) não cabe mais o firmware desde que ESP-NOW/WiFi +
      `BluetoothSerial` (Classico) passaram a coexistir ali.
  - **Confirmado funcionando com o carro real**: `010C` (RPM) e `0105`
    (temperatura do motor) bateram com o relatório do app INNOOBD+ tirado
    minutos antes do teste de bancada (ver histórico de investigação
    abaixo), e depois disso a tela mostrou a temperatura real do motor
    variando ao vivo (77°C -> 68°C) no medidor "Temp. motor". Levantamento
    completo de todos os PIDs suportados pelo veículo (com fórmulas e
    unidades) em [tabela.md](./tabela.md).
  - **Não implementado**: temperatura de câmbio (sem PID Modo 01 padrão -
    é específico de cada fabricante).
  - **DTCs (códigos de falha): IMPLEMENTADO (03/11/2026)** - leitura sob
    demanda dos códigos armazenados (Modo `03`) e pendentes (Modo `07`), e
    limpeza (Modo `04`), pela 4ª tela ("DTCs - Códigos de falha", botões
    **Ler DTCs**/**Limpar códigos**):
    - `src/relay_main/main.cpp`: `parseDtcCodes()` extrai os pares de bytes
      da resposta Modo 03/07 (header `43`/`47`); o status do MIL vem do
      bit 7 do 1º byte do PID `0101`. É sob demanda (não entra no ciclo
      normal de ~2s) porque é mais lento - pode levar vários segundos.
      Pedido vem via `MsgType::DtcControl` (`action`: 0=ler, 1=limpar -
      limpar re-lê os códigos logo em seguida para confirmar) e a
      resposta sai via `MsgType::DtcData` (até `kMaxDtcEntries` = 10
      códigos brutos de 2 bytes + flag armazenado/pendente).
    - [lib/remote_link/remote_protocol.h](./lib/remote_link/remote_protocol.h):
      `formatDtcCode()` decodifica o código bruto para o formato padrão
      SAE J2012/ISO 15031-6 (ex. `P0301`) - usado só do lado da tela.
    - **Não inclui descrições textuais dos códigos** (ex. "P0301 = falha
      de ignição no cilindro 1") - ao contrário de projetos como o AURA
      (~190 descrições embutidas), aqui só mostra o código bruto decodificado;
      é uma extensão futura possível se precisar.
  - Outras opções de adaptador consideradas e descartadas (ver histórico de
    investigação abaixo):
    - **V800 / app INNOOBD+**: testado e **não é compatível** com AT/ELM327
      em texto puro (ver detalhes abaixo) - exigiria engenharia reversa do
      protocolo binário proprietário. Descartado em favor do ELM327 Clássico
      já confirmado.
    - **Vgate vLinker MC+**: nunca testado em mãos (só confirmado por
      terceiros online); não foi mais necessário dado que o ELM327 Clássico
      já funciona.
  - **BLE, não Wi-Fi** (motivo histórico da escolha original de BLE para a
    tela, antes do ELM327 Clássico ser testado): um adaptador ELM327 Wi-Fi
    forçaria o canal do rádio da tela para o canal do próprio Access Point
    do adaptador, quebrando o canal fixo do ESP-NOW com o módulo de relés.
  - <details>
    <summary>Histórico da investigação do adaptador V800 / INNOOBD+ (descartado)</summary>

    Scanner **"V800"** (app **INNOOBD+**, "AI-Powered Diagnostic Scanner"
    para ENG/AT/ABS/SRS). Confirmado **Bluetooth 5.1 BLE** (compatível com o
    ESP32-S3 da tela). **Testado em bancada em 02/11/2026** com o ambiente
    `obd2_scan` (ESP32 clássico da placa de relés, via NimBLE) e confirmado
    de forma independente com `bleak` direto do PC (Bluetooth do Windows) -
    os dois bateram no mesmo resultado:
    - GATT real do dispositivo: serviços padrão Generic Access (`0x1800`),
      Generic Attribute (`0x1801`), Tx Power (`0x1804`), Battery (`0x180f`),
      e um serviço customizado **`0xfff0`** com `0xfff1` (read+notify) e
      `0xfff2` (write/write-without-response) - o padrão clássico de "UART
      transparente sobre BLE" usado por vários clones ELM327 baratos.
    - **Resultado negativo confirmado**: escrever comandos AT em texto puro
      (`ATZ\r`, `ATZ\r\n`, `AT\r`, com e sem resposta ATT) em `0xfff2` **não
      produz nenhuma notificação** em `0xfff1`. Ou seja, ao contrário do
      ELM327 Clássico, o V800 **não** expõe um túnel AT/ELM327 transparente -
      usa um protocolo binário/proprietário próprio (esperado, dado o
      vínculo do produto com o app INNOOBD+ e os recursos de IA/nuvem).
    - Se um dia for necessário reaproveitar o V800 (ex.: pelos recursos de
      diagnóstico extra ABS/SRS que o ELM327 genérico não tem), o próximo
      passo seria capturar o tráfego BLE real do app INNOOBD+ com um Android
      (Opções do desenvolvedor -> "Ativar registro de rastreamento HCI
      Bluetooth", fazer uma varredura no app, extrair `btsnoop_hcci.log` via
      `adb bugreport` e abrir no Wireshark) para descobrir o framing real.
    </details>
  - **Ambientes de diagnóstico temporários** (remover depois que o cliente
    OBD-II definitivo estiver implementado em `src/relay_main/`):
    - **`obd2_scan_classic`** (`src/obd2_scan_classic/main.cpp`, biblioteca
      `BluetoothSerial` embutida no core) - o que resolveu o protocolo.
      Ponte serial transparente Bluetooth Clássico/SPP: `scan` descobre
      dispositivos por perto, `connect <indice>` conecta, e qualquer outra
      linha digitada é enviada direto ao adaptador com CR no final (convenção
      AT/ELM327). Só roda na placa de RELÉS (único chip dos dois com
      Bluetooth Clássico de verdade); **sobrescreve o firmware
      `relay_module`** - regrave `-e relay_module` depois do teste.
      ```bash
      platformio run -e obd2_scan_classic --target upload --upload-port COMx
      platformio device monitor -e obd2_scan_classic --port COMx
      ```
      Comandos: `scan`, `list`, `connect <indice|MAC>`, `disconnect`,
      `help`, e qualquer outro texto é tratado como comando AT.
    - **`obd2_scan`** / **`obd2_scan_relayboard`** (`src/obd2_scan/main.cpp`,
      biblioteca `h2zero/NimBLE-Arduino`) - usado para a investigação BLE
      (V800), descrita no histórico acima. Escaneia BLE e, ao conectar num
      adaptador, despeja todos os serviços/características GATT no Serial.
      `obd2_scan` grava na placa da TELA; `obd2_scan_relayboard` grava na
      placa de RELÉS (também sobrescreve `relay_module`).
      ```bash
      platformio run -e obd2_scan --target upload
      platformio device monitor -e obd2_scan
      # ou, na placa de reles:
      platformio run -e obd2_scan_relayboard --target upload --upload-port COMx
      platformio device monitor -e obd2_scan_relayboard --port COMx
      ```
      Comandos: `scan`, `list`, `connect <indice>`, `use <indice>`,
      `write <texto>`, `hex <bytes>`, `read`, `disconnect`, `help`.
    - **Atenção** (vale para os três ambientes): os índices da lista
      `scan`/`list` são válidos só para aquela sessão de scan - um novo
      `scan` (ou reboot) reordena tudo.
- **Bússola magnética**: requer magnetômetro externo (ver acima).
- Renomear os rótulos `AUX 1..4` para as funções reais das luzes auxiliares.

- **Câmera auxiliar (assoalho/trilha): PLANEJADO, aguardando o módulo físico
  em mãos** - ainda SEM nenhuma linha de código escrita, só o plano abaixo
  (decidido em sessão, 03/11/2026). Objetivo: um módulo **ESP32-S3-CAM**
  independente (próprio chip/câmera/Wi-Fi - não compartilha GPIO com a
  tela), montado no assoalho/cambagem do carro, com 4 modos escolhíveis
  pelo painel: **Standby** (sem câmera ligada, só aguardando comando),
  **Timelapse** (foto a cada X segundos), **Gravação de vídeo** e
  **Streaming**, tudo salvando no microSD do próprio módulo.
  - **Decisões já fechadas**:
    - Armazenamento: microSD no módulo da câmera (não envia para
      PC/rede).
    - Controle: integrado numa **5ª tela** no painel (`lcd_main`), não
      página web separada.
    - Preview no painel: snapshot atualizando a cada 1-2s (não vídeo
      fluido - isso fica para abrir o IP da câmera direto no navegador do
      celular, se quiser ver em tempo real de verdade).
  - **Arquitetura técnica proposta** (rede híbrida):
    - A câmera entra na rede **ESP-NOW já existente** como um 3º
      dispositivo pareado (MAC separado do módulo de relés, handshake
      dedicado `MsgType::CameraHello`/`CameraHelloAck` - **não** reaproveita
      o handshake do relé, para não arriscar regressão no que já funciona)
      - usado só para comando/status (`MsgType::CameraControl` /
      `MsgType::CameraStatus`), mensagens pequenas que cabem no limite do
      ESP-NOW (~250 bytes/pacote).
    - Como ESP-NOW não serve para transportar imagem/vídeo, a câmera
      também sobe um **Wi-Fi Access Point próprio no MESMO canal fixo 6**
      do ESP-NOW (`WIFI_CHANNEL`), servindo HTTP (`/snapshot`, `/stream`
      estilo `CameraWebServer` oficial da lib `esp32-camera`). A tela vira
      cliente Wi-Fi STA desse AP **simultaneamente** ao ESP-NOW com o relé
      (coexistência STA+ESP-NOW no mesmo canal - padrão documentado, mas é
      o ponto mais arriscado tecnicamente, vale validar isolado primeiro).
    - **Desacoplamento de energia**: o link ESP-NOW fica sempre ativo
      (permite "acordar" a câmera à distância); o sensor de imagem e o AP
      Wi-Fi só ligam fora do modo Standby.
    - Vídeo = **MJPEG em container AVI** (o ESP32-S3 não tem encoder
      H.264/H.265 em hardware) - sem áudio, framerate baixo (~5-15fps),
      arquivos grandes rápido. Cartão microSD Class 10/UHS-1 recomendado
      para não travar a escrita.
    - Preview no painel exige habilitar um decodificador JPEG no LVGL
      (`LV_USE_SJPG` em `lv_conf.h`, hoje desligado).
  - **Itens em aberto** (revisitar quando o módulo físico chegar):
    - Pinagem exata da câmera/SD (`camera_pins.h`) - módulos "ESP32-S3-CAM"
      genéricos variam entre fabricantes; precisa validar fisicamente.
    - Validar na prática a coexistência Wi-Fi STA + ESP-NOW no mesmo canal.
  - **Ordem de implementação sugerida** (fases testáveis separadamente):
    1. Firmware standalone da câmera (máquina de estados + SD + AP/HTTP) -
       testável sozinho com só um navegador, sem depender da tela.
    2. Extensão do protocolo compartilhado + cliente ESP-NOW na câmera
       (pareamento, comando, status) - testável por log serial.
    3. Tela: pareamento com a câmera + nova 5ª tela (controles/status, sem
       imagem ainda).
    4. Tela: snapshot/preview JPEG (parte mais nova tecnicamente - a mais
       arriscada, deixada por último de propósito).

## Build

```bash
# Tela
platformio run -e lcd_display
platformio run -e lcd_display --target upload
platformio device monitor -e lcd_display

# Módulo de relés
platformio run -e relay_module
platformio run -e relay_module --target upload
platformio device monitor -e relay_module
```

## Configuração que DEVE ser igual nos dois firmwares

- `PAIRING_KEY` (chave de pareamento/criptografia ESP-NOW)
- `WIFI_CHANNEL` (canal Wi-Fi fixo, 1-13)

Ambas as constantes estão no topo de cada `main.cpp` (`src/lcd_main/main.cpp`
e `src/relay_main/main.cpp`). **Altere o valor padrão de `PAIRING_KEY` antes
de usar em produção.**

## Estrutura

```
├── platformio.ini              # 2 ambientes: lcd_display / relay_module
├── lv_conf.h                   # Configuração LVGL (usada só por lcd_display)
├── boards/WAVESHARE-ESP-LCD.json
├── include/ESP_Panel_Board_Custom.h   # Pinagem do painel + touch habilitado
├── lib/
│   ├── i2c/, ch422g/, rgb_lcd_port/, touch/, lvgl_port/   # Drivers de hardware (copiados do IQ50LCD)
│   ├── imu_qmi8658/            # Driver do IMU onboard (inclinação)
│   ├── battery_ina219/         # Driver do INA219 externo (indicador de bateria da placa)
│   └── remote_link/            # Protocolo + transporte ESP-NOW (compartilhado pelos 2 firmwares)
└── src/
    ├── lcd_main/main.cpp       # Firmware da tela
    └── relay_main/main.cpp     # Firmware do módulo de relés
```

## Histórico de depuração

A primeira gravação em hardware revelou três bugs reais (crash no boot do
display, touch sem resposta, e relés não acionando apesar do "pareado" na
tela) — causas-raiz, correções e lições aprendidas estão documentadas em
detalhe em [docs/HISTORICO-DEPURACAO.md](./docs/HISTORICO-DEPURACAO.md).
