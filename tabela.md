# Levantamento de PIDs OBD-II (Modo 01) — GM Trailblazer 2017 Diesel

Varredura real feita em bancada em **02/11/2026**, conectando um adaptador
**ELM327 Bluetooth Clássico** (confirmado funcionando — ver
[README.md](./README.md), seção "Roadmap"/"OBD-II") no conector OBD-II do
veículo, usando o firmware de diagnóstico
[src/obd2_scan_classic/main.cpp](./src/obd2_scan_classic/main.cpp) gravado
na placa de relés (ESP32 clássico, único dos dois chips do projeto com
Bluetooth Clássico de verdade).

- **Veículo**: Chevrolet Trailblazer 2017, motor diesel.
- **VIN**: `9BG156MK0HC442213` (WMI `9BG` = General Motors do Brasil).
- **Protocolo detectado**: `ATSP0` (automático) — ECU respondeu via CAN,
  **duas unidades de controle (UCEs) diferentes** responderam a várias
  consultas de PID (ex.: motor + um segundo módulo, possivelmente câmbio).
  Quando isso aconteceu, a tabela abaixo traz o valor da **primeira**
  resposta (ECU principal/motor).
- **⚠️ Estado do veículo durante a leitura**: `010C` (RPM) e `010D`
  (velocidade) retornaram **0**, ou seja, **o motor estava desligado com a
  chave ligada** (key-on-engine-off) — não em marcha lenta nem em
  movimento. Isso explica valores como MAF = 0 g/s e pressão do rail de
  combustível baixa (bomba de alta pressão do common rail não estava ativa).
- **Como foram descobertos os PIDs suportados**: consultas aos PIDs de
  bitmask `0100`, `0120`, `0140`, `0160`, `0180`, `01A0`, `01C0` (cada um
  informa quais dos 32 PIDs seguintes o veículo suporta). `01A0` e `01C0`
  retornaram `NO DATA` — ou seja, **nenhum PID acima de `0x80` é suportado**.
- **Fórmulas**: conferidas contra a tabela padrão SAE J1979
  ([Wikipedia - OBD-II PIDs](https://en.wikipedia.org/wiki/OBD-II_PIDs)).
  Várias foram **cross-validadas com sucesso** contra o relatório do app
  INNOOBD+ tirado minutos antes (mesmo veículo) — marcadas com ✅ na coluna
  Observação.

## Tabela de parâmetros com retorno válido

| PID (hex) | Parâmetro | Bytes brutos (resposta) | Valor decodificado | Unidade | Observação |
|---|---|---|---|---|---|
| `0101` | Status de monitoramento desde que os DTCs foram apagados | `00 04 00 00` | MIL: **desligado**; DTCs armazenados: **0** | (bit-encoded) | Confirma o `varredura OBD` do app (MIL OFF, 0 códigos) |
| `0102` | DTC que causou o freeze frame | `00 00` | `P0000` (nenhum) | (código) | Sem freeze frame armazenado |
| `0104` | Carga calculada do motor | `6D` | 109 × 100/255 = **42.75** | % | |
| `0105` | Temperatura do líquido de arrefecimento do motor | `68` | 104 − 40 = **64** | °C | |
| `010B` | Pressão absoluta do coletor de admissão (MAP) | `4E` | **78** (valor direto) | kPa | Próximo da pressão atmosférica — motor desligado |
| `010C` | Rotação do motor (RPM) | `00 00` | **0** | rpm | Motor desligado (key-on-engine-off) |
| `010D` | Velocidade do veículo | `00` | **0** | km/h | Veículo parado |
| `010F` | Temperatura do ar de admissão | `5C` | 92 − 40 = **52** | °C | Ar ainda quente (calor residual do motor) |
| `0110` | Vazão do sensor de fluxo de massa de ar (MAF) | `00 00` | (0×256+0)/100 = **0.00** | g/s | Sem fluxo de ar — motor desligado |
| `011C` | Padrão OBD ao qual o veículo é compatível | `06` | **6 = EOBD** (padrão europeu) | (enumerado) | |
| `011F` | Tempo desde a partida do motor | `00 00` | **0** | s | Motor desligado |
| `0121` | Distância percorrida com a luz MIL acesa | `00 00` | **0** | km | Consistente com MIL desligada |
| `0123` | Pressão manométrica do rail de combustível (diesel) | `00 10` | 10×(256×0+16) = **160** | kPa | Baixa — bomba de alta pressão do common rail inativa (motor desligado) |
| `012C` | EGR comandada | `00` | 0×100/255 = **0** | % | Sem comando de EGR (motor desligado) |
| `012D` | Erro de EGR | `80` | (128×100/128) − 100 = **0** | % | Valor neutro/sem erro ativo |
| `012F` | Nível do tanque de combustível | `39` | 57×100/255 = **22.35** | % | ✅ Igual ao relatório INNOOBD+ (22.35%) |
| `0130` | Nº de aquecimentos (warm-ups) desde que os códigos foram apagados | `19` | **25** (valor direto) | contagem | ✅ Igual ao relatório INNOOBD+ (25) |
| `0131` | Distância percorrida desde que os códigos foram apagados | `04 1D` | 256×4+29 = **1053** | km | ✅ Igual ao relatório INNOOBD+ (1053 km) |
| `0133` | Pressão barométrica | `5C` | **92** (valor direto) | kPa | ✅ Igual ao relatório INNOOBD+ (92.0 kPa) |
| `0142` | Tensão do módulo de controle (ECU) | `2F A3` | (0x2FA3=12195)/1000 = **12.195** | V | Tensão de bateria em repouso (motor desligado; o relatório anterior, com o motor ligado, mostrou 14.26V — alternador carregando) |
| `0168` | Temperatura do sensor de ar de admissão (sensor 1 e 2) | `03 5C 63 ...` | Sensor 1: 92−40=**52**; Sensor 2: 99−40=**59** | °C | Byte de suporte `03` indica só 2 sensores válidos; bytes extras na resposta (fora do padrão de 3 bytes da SAE J1979) foram ignorados — ver nota¹ |
| `017F` | Tempo de funcionamento do motor (acumulado) | `03 01 03 F7 F0 00 3C 79 31 00 00 00 00` | Total: 0x0103F7F0 = 17.037.296s ≈ **4732.6 h**; Em marcha lenta: 0x003C7931 = 3.963.185s ≈ **1100.9 h** | h (segundos convertidos) | ✅ Muito próximo do relatório INNOOBD+ (4731h34min / 1099h52min) — pequena diferença é só o tempo real passado entre as duas leituras |

## PIDs suportados mas sem decodificação confiável

| PID (hex) | Parâmetro (nome SAE) | Bytes brutos (resposta) | Motivo de não decodificar |
|---|---|---|---|
| `016A` | Controle de vazão de ar de admissão comandado (diesel) e posição relativa da vazão de ar de admissão | `03 E6 B6 00 00` | A Wikipedia/SAE J1979 não publica a fórmula exata de divisão dos 5 bytes para este PID (diferente da maioria dos outros, documentados com fórmula). Fontes de terceiros divergem entre si — preferi não arriscar um valor errado. Dado bruto preservado para decodificação futura se uma fonte confiável (ex.: documentação de serviço GM) for encontrada. |

## Notas

1. **PID `0168`**: o padrão SAE J1979 define esta PID com apenas 3 bytes de
   dado (`A`=bitmask de suporte, `B`=sensor 1, `C`=sensor 2). A ECU deste
   veículo respondeu com mais bytes que o padrão (resposta de 7 bytes de
   dado em múltiplos quadros CAN) — provavelmente uma extensão do
   fabricante para suportar até 6 posições de sensor de temperatura do ar
   de admissão (comum em diesel com intercooler + pós-turbo). Como o byte
   de suporte (`03` = binário `00000011`) só marca os sensores 1 e 2 como
   presentes, os bytes extras (todos `28` = 40 → 0°C) foram tratados como
   não utilizados/preenchimento, não como leituras reais.
2. **Duas UCEs respondendo**: PIDs `0100`, `0120`, `0140` retornaram **duas
   respostas diferentes** no formato `41 xx ... 41 xx ...` — indicando que
   mais de um módulo de controle (ex.: motor + câmbio/TCM) está no mesmo
   barramento CAN e responde à mesma consulta broadcast. Isso é normal em
   veículos modernos multiplexados; a tabela usa a primeira resposta
   (presumivelmente o módulo do motor/PCM).
3. **PIDs consultados sem suporte confirmado pelo veículo** (não incluídos
   na tabela, pois o veículo não os declarou suportados nos bitmasks
   `0100`-`01C0`): nenhum PID acima de `0x80` (todo o bloco `01A0`-`01FF`
   retornou `NO DATA`). Dentro de `0x01`-`0x80`, PIDs comumente esperados
   em diesel mas **não suportados** por este veículo: `0106`-`0109`
   (ajustes de mistura combustível/ar — esperado, diesel não usa trim de
   O2 da mesma forma que motores a gasolina), `010E` (avanço de ignição —
   não aplicável a diesel), `0113`-`011B` (sensores de O2 — idem),
   `015E` (taxa de consumo de combustível), `0169`/`016B` (EGR
   atual/temperatura de EGR).
4. **Tempo de funcionamento (`017F`) convertido em h:min:s** para facilitar
   leitura:
   - Total: 4732.6 h ≈ **4732h 36min**
   - Em marcha lenta: 1100.9 h ≈ **1100h 53min**

## Como reproduzir esta varredura

Firmware usado: [src/obd2_scan_classic/main.cpp](./src/obd2_scan_classic/main.cpp)
(ambiente `obd2_scan_classic` no [platformio.ini](./platformio.ini)), que
abre uma ponte serial transparente Bluetooth Clássico/SPP — qualquer linha
digitada no monitor serial é enviada direto ao ELM327 com `\r` no final.

```bash
platformio run -e obd2_scan_classic --target upload --upload-port COMx
platformio device monitor -e obd2_scan_classic --port COMx
```

No monitor: `connect 00:0D:18:3A:67:89` (MAC do adaptador — ver
`OBD_ELM327_MAC` em [src/relay_main/main.cpp](./src/relay_main/main.cpp)),
depois `ATZ`, `ATE0`, `ATL0`, `ATH0`, `ATSP0`, depois qualquer PID
(ex.: `010C`).

> **Atenção**: este ambiente sobrescreve o firmware `relay_module` na placa
> de relés. Regrave `-e relay_module` depois de terminar os testes para
> devolver a placa à função normal (relés + temperatura do motor na tela).
