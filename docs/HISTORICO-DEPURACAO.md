# Histórico de depuração — gravação inicial e bugs encontrados (out/2026)

Este documento registra os problemas reais enfrentados ao gravar e colocar em
funcionamento os dois firmwares deste projeto pela primeira vez em hardware,
as causas-raiz encontradas, as correções aplicadas e — principalmente — **onde
mais se perdeu tempo**, para evitar repetir os mesmos caminhos em sessões
futuras de depuração. Os dois problemas mais graves e que mais consumiram
tempo foram na **inicialização do display** (crash/boot-loop) e na
**inicialização do touch** (não respondia); por isso estão detalhados em
seções próprias abaixo, com sintoma, hipóteses descartadas, causa raiz real e
correção aplicada. Três problemas adicionais, de natureza diferente, foram
encontrados e corrigidos depois: relés não acionando (protocolo ESP-NOW), o
sensor IMU "não detectado" (investigado e confirmado como hardware ausente,
não bug) e um clique fantasma ao navegar entre telas por gesto (LVGL).

Resumo rápido do resultado final: os dois firmwares foram gravados, a tela
parou de travar em loop de reset, o touch passou a responder, os relés
passaram a acionar fisicamente ao tocar nos botões da tela, e a navegação por
gesto entre as duas telas da UI funciona sem acionar relés sozinha.

---

## 1. Gravação inicial (sem bugs)

- Módulo de relés (`env:relay_module`): compilou e gravou **sem problemas**
  logo na primeira tentativa.
- Tela (`env:lcd_display`): compilou e gravou sem erros, mas o firmware
  **reiniciava em loop** assim que ligava (ver seção 2 abaixo).

### Problema recorrente de ambiente: porta COM muda a cada reset

Os dois ESP32-S3 usam **USB-CDC nativo** (não um conversor serial dedicado).
Isso faz a porta COM **reenumerar a cada reset/boot** (ex.: COM3 → COM9 →
COM10...). O reset automático via RTS/DTR do esptool também falha com
frequência ("No serial data received" / porta ocupada). **Fluxo que funcionou
de forma confiável:**

1. Sempre rodar `platformio device list` imediatamente antes de qualquer
   upload, nunca reutilizar uma porta "lembrada" de uma gravação anterior.
2. Se o reset automático falhar, pedir para o usuário colocar a placa em modo
   download manualmente (segurar BOOT, pulsar RESET, soltar BOOT).

### Problema de ambiente: PowerShell trava com a barra de progresso do esptool

Os caracteres Unicode da barra de progresso do esptool quebram o console
PowerShell padrão (encoding cp1252). Corrigido aplicando, antes de cada
build/upload:

```powershell
chcp 65001 | Out-Null
$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
```

---

## 2. Problema de inicialização do DISPLAY — tela reiniciando em loop (crash no boot)

**Sintoma:** a tela reiniciava sozinha repetidamente, nunca chegava a mostrar
a UI. Acontecia mesmo compilando e gravando sem nenhum erro.

### Hipóteses descartadas (nesta ordem, cada uma consumiu tempo real de teste)

1. **Brownout por pico de corrente do rádio Wi-Fi** → desabilitar Wi-Fi
   inteiramente não resolveu o crash.
2. **Timing do painel RGB em `include/ESP_Panel_Board_Custom.h`** → esse
   arquivo **não é lido por nenhum código** (achado só depois de um grep em
   todo o projeto); os timings reais estão hardcoded em
   [lib/rgb_lcd_port/rgb_lcd_port.cpp](../lib/rgb_lcd_port/rgb_lcd_port.cpp).
   O arquivo foi mantido no repositório apenas como documentação de
   referência do painel, com um comentário explicando que é código morto.
3. **`lib_ldf_mode = deep+` no `platformio.ini`** → trava a resolução de
   dependências do PlatformIO indefinidamente neste projeto (mesmo num build
   mínimo sem nenhum uso de `Network.h`), confirmando/estendendo o problema já
   documentado no [CLAUDE.md](../CLAUDE.md). Revertido para `deep`, o que
   resolveu o travamento do *build*, mas **não** o crash em tempo de execução.
4. **Cache de build (`.pio`) corrompido** → build limpo do zero, sem efeito.
5. **Hardware ou toolchain com defeito** → descartado comparando byte-a-byte
   com o projeto irmão `IQ50LCD` (mesma placa física, mas com touch sempre
   desabilitado lá), que roda perfeitamente estável.

### Investigação que encontrou a causa raiz

Foi criado um firmware mínimo de teste (só painel RGB + LVGL, sem touch, sem
IMU, sem Wi-Fi) usando a mesma sequência de boot do `IQ50LCD`
(`delay(2000)` fixo): **estável**. Em seguida, foi inserido um `delay(400)`
entre a inicialização do painel RGB e a chamada de `lvgl_port_init()` — **sem
nenhum código de touch envolvido** — e o crash reapareceu. Isso provou que o
bug era uma **condição de corrida (race condition)** ligada a *timing*, não
ao touch em si (o touch só havia sido o "gatilho" porque sua rotina de reset
leva ~100-400ms).

**Causa raiz:** a função `waveshare_esp32_s3_rgb_lcd_init()` registrava o
callback de vsync do painel (`esp_lcd_rgb_panel_register_event_callbacks`,
evento "bounce frame finish") **imediatamente após** `esp_lcd_panel_init()` —
ou seja, **antes** de `lvgl_port_init()` preparar as estruturas internas de
buffer do LVGL. O painel RGB começa a disparar esse callback assim que é
inicializado; qualquer atraso antes de `lvgl_port_init()` rodar permitia que
o callback executasse contra estado do LVGL ainda não inicializado,
corrompendo memória e derrubando a placa.

Essa suspeita foi confirmada comparando com o exemplo oficial
Waveshare/Espressif (`08_lvgl_Porting/main/waveshare_rgb_lcd_port.c`): lá, o
registro desse callback é feito **depois** de `lvgl_port_init()` — exatamente
a ordem que faltava neste projeto.

### Correção aplicada

- [lib/rgb_lcd_port/rgb_lcd_port.cpp](../lib/rgb_lcd_port/rgb_lcd_port.cpp) /
  [.h](../lib/rgb_lcd_port/rgb_lcd_port.h): o registro do callback de vsync
  foi retirado de dentro de `waveshare_esp32_s3_rgb_lcd_init()` e movido para
  uma nova função exportada, `waveshare_rgb_lcd_register_vsync_callback()`.
- [src/lcd_main/main.cpp](../src/lcd_main/main.cpp): ordem de inicialização
  corrigida para:
  `waveshare_esp32_s3_rgb_lcd_init()` → `touch_gt911_init()` →
  `lvgl_port_init()` → **só então** `waveshare_rgb_lcd_register_vsync_callback()`.

> Esse mesmo risco existe, em teoria, no projeto `IQ50LCD` se algum dia ele
> adicionar um atraso entre o init do painel e o `lvgl_port_init()` (hoje não
> tem esse problema porque nunca há atraso nenhum ali). **Não foi corrigido
> lá** — está fora do escopo deste projeto (ver regra 4 do
> [CLAUDE.md](../CLAUDE.md)) — mas vale revisar se o IQ50LCD mudar esse trecho.

---

## 3. Problema de inicialização do TOUCH — não respondia (sem travar mais)

**Sintoma:** depois de corrigido o crash do display (seção 2), a tela ficava
estável e abria normalmente, mas tocar nos botões na tela **não mudava
nada** (nenhuma mudança de cor, nenhuma reação).

### Onde mais se perdeu tempo nesta etapa

- **`ESP_LOGE`/`ESP_LOGx` não aparecia na saída serial capturada**, mesmo com
  `CORE_DEBUG_LEVEL=1` configurado. Isso escondeu o erro real por um bom
  tempo — várias tentativas de correção foram feitas "às cegas", sem nunca
  ver a mensagem de erro de verdade:
  - sequência de reset do expansor CH422G reescrita byte-a-byte igual à
    referência oficial (via `DEV_I2C_Write_nByte`);
  - troca de `esp_rom_delay_us()` por `vTaskDelay()` nos atrasos do reset;
  - pull-up habilitado no GPIO4 (pino de interrupção do touch);
  - `gpio_reset_pin()` no pino de interrupção antes de configurá-lo;
  - usuário recolocou fisicamente o cabo flex (FPC) do touch.

  Nenhuma dessas mudanças resolveu o problema sozinha (algumas eram boas
  práticas e foram mantidas, mas não eram a causa raiz).

  **Lição principal desta sessão: neste projeto/ambiente, `ESP_LOGx` não é
  confiável para diagnóstico — use sempre `Serial.printf`/`Serial.println`
  diretamente.**

- Um `return NULL;` de diagnóstico (de uma tentativa de teste bem anterior)
  tinha ficado **esquecido no meio** de `touch_gt911_init()`, fazendo a
  inicialização real do touch ser pulada silenciosamente em todo "teste
  final" seguinte, até ser notado e removido.

### Causa raiz real

Só depois de trocar todos os pontos de verificação para `Serial.printf`
diretamente (em vez de `ESP_LOGx`) é que apareceu o erro real:

```
esp_lcd_new_panel_io_i2c -> ESP_ERR_INVALID_ARG
```

Consultando o código-fonte do próprio ESP-IDF v5.5 no GitHub
(`components/esp_lcd/i2c/esp_lcd_panel_io_i2c_v1.c`), ficou claro que a API
**legada** de I2C (`esp_lcd_new_panel_io_i2c_v1` — usada aqui porque o
barramento I2C deste projeto é configurado com a API clássica
`i2c_driver_install()`, e não a nova `i2c_master_bus_handle_t`) **valida que o
campo `scl_speed_hz` precisa ser 0** e rejeita qualquer outro valor com essa
mensagem de erro ("scl_speed_hz is not need to set in legacy i2c_lcd
driver").

Uma "correção" aplicada **bem no início da sessão**, baseada numa sugestão
genérica de busca na internet sobre um problema *diferente* (relacionado à
API **nova** `i2c_master`, não à **legada** usada aqui), tinha configurado
`.scl_speed_hz = 400000` em `ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG()`. Essa foi a
causa real do touch nunca funcionar — e ficou mascarada justamente porque o
erro era logado via `ESP_LOGE`, que não aparecia na nossa captura serial.

### Correção aplicada

- [lib/touch/gt911.h](../lib/touch/gt911.h): removido `.scl_speed_hz = 400000`
  da macro `ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG()` (voltou a 0/omitido, que é o
  valor exigido pela API legada), com comentário explicando por que precisa
  ficar assim e não deve ser "corrigido" de volta por uma sugestão genérica.
- [lib/touch/gt911.cpp](../lib/touch/gt911.cpp): mantidas como melhoria,
  mesmo não sendo a causa do bug (confirmadas corretas pela referência
  oficial da Waveshare):
  - sequência de reset do CH422G via `DEV_I2C_Write_nByte` em vez da função
    com cache que tinha um bug de estado;
  - `vTaskDelay` em vez de `esp_rom_delay_us` (não bloqueia outras tasks);
  - `gpio_reset_pin()` no pino de interrupção antes de configurá-lo;
  - pull-up habilitado no pino de interrupção (INT);
  - tratamento de erro não-fatal (`if (ret != ESP_OK) return NULL;` em vez de
    `ESP_ERROR_CHECK`, que travava a placa inteira se o GT911 não respondesse
    a tempo, em vez de simplesmente desabilitar o touch e seguir sem ele).

**Confirmado pelo usuário:** touch respondendo normalmente, botões mudando de
cor ao toque.

---

## 4. Problema à parte — touch OK, mas relés não acionavam fisicamente

Depois dos dois problemas de display/touch acima resolvidos, surgiu um
terceiro problema, de natureza completamente diferente (protocolo de rede, não
display/touch): a tela mostrava "Pareado com o módulo de relés" e o touch
funcionava, mas tocar nos botões **não acionava nenhum relé físico**.

### Causa raiz (resumo)

A mensagem `"[LCD] Pareado com o modulo de reles."` só prova que **a tela**
recebeu o `Hello` do relé — não prova que o relé recebeu/decodificou a
resposta da tela. Monitorando o log serial do **módulo de relés** durante um
boot completo, a mensagem `"[RELAY] Pareado com o controlador."` **nunca
apareceu**: havia um impasse ("deadlock") na troca de chaves de criptografia
do ESP-NOW — a tela respondia ao `Hello` do relé com um `HelloAck` **unicast
já criptografado**, mas o relé só registra a chave de criptografia da tela ao
*processar* esse mesmo `HelloAck` — então o relé nunca conseguia decodificá-lo
e descartava o pacote silenciosamente, sem nunca completar o pareamento do seu
lado. Esse bug já existia no protocolo original (não foi introduzido nesta
sessão) e só foi percebido ao comparar os logs dos dois dispositivos lado a
lado.

### Correção aplicada

- [src/lcd_main/main.cpp](../src/lcd_main/main.cpp): o `HelloAck` agora é
  enviado via `sendBroadcast()` (sem criptografia), igual ao `Hello`
  original do relé, permitindo que o relé aprenda o MAC real da tela e só
  então registre a chave de criptografia do seu lado.
- [src/relay_main/main.cpp](../src/relay_main/main.cpp): adicionado um log
  permanente em `case MsgType::SetRelay` (canal + novo estado), útil para
  confirmar em campo que comandos estão chegando.
- [README.md](../README.md): seção "Fluxo de pareamento" atualizada.

**Confirmado fisicamente pelo usuário:** relés acionando ao tocar nos botões
da tela.

---

## 5. Investigação do sensor IMU (QMI8658) — ausente, não é bug

Ao implementar a tela OBD-II com medidores de inclinação (lateral/frontal),
os medidores nunca saíam da posição de repouso. O firmware já logava
`"AVISO: IMU QMI8658 nao detectado."` desde o início da sessão, e a hipótese
inicial (não confirmada à época) era "sensor ausente/com defeito nesta
unidade". Dado o histórico de bugs reais "disfarçados" de limitação de
hardware nas seções 2 e 3, essa suposição foi investigada a fundo em vez de
aceita sem prova.

### Investigação

1. Adicionado log direto no `probeAddress()` do driver (endereço testado +
   resultado da leitura I2C + valor do registrador WHO_AM_I lido) — mostrou
   que a leitura I2C **falhava completamente** (sem resposta/NACK) nos dois
   endereços possíveis do QMI8658 (`0x6A`/`0x6B`), não que respondia com um
   valor errado.
2. Uma falha completa de leitura ainda não prova ausência (poderia ser
   endereço errado, linha de reset/enable não ligada via CH422G, etc.) —
   então foi feita uma **varredura completa do barramento I2C** (0x08-0x77,
   equivalente à ferramenta oficial `i2c-tools` da Espressif). Resultado:
   apenas o **CH422G** (responde numa faixa larga de endereços por usar
   "pseudo-registradores" como endereços, não é um bug nem são 8 chips
   reais), o **GT911** (touch, `0x5D`) e um chip de **RTC** (`0x51`,
   provavelmente PCF8563) respondem. Nada em `0x6A`/`0x6B`, nem em nenhum
   outro endereço inesperado.
3. Cruzado com duas fontes oficiais da Waveshare para este modelo exato:
   o esquemático (`ESP32-S3-Touch-LCD-4.3B-Sch.pdf`, extraído em texto) e o
   pacote de exemplos oficial (`ESP32-S3-Touch-LCD-4.3B-BOX-Demo`, com 8
   exemplos numerados: I2C genérico, RS485, SD, RTC, IO, TWAI tx/rx e LVGL).
   **Nenhum dos dois menciona QMI8658 em lugar nenhum**, e não há nenhum
   exemplo de IMU entre os oficiais.

### Conclusão

A especificação de marketing do produto menciona um QMI8658 onboard, mas
**esta unidade física específica não o tem montado** — confirmado por
evidência direta (varredura de barramento) e por duas fontes documentais
oficiais independentes, não por suposição. Não é um bug deste projeto.

### Ação tomada

- O driver (`lib/imu_qmi8658/`) continua no projeto — `imu::begin()` retorna
  `false` de forma limpa, sem travar nada, para quando uma unidade com o
  sensor populado for usada, ou um módulo QMI8658 externo for ligado ao mesmo
  barramento I2C.
- A UI foi ajustada para refletir isso **de forma honesta e permanente**: em
  vez de deixar os medidores de inclinação "congelados" em `-- graus` (o que
  parece uma tela travada), eles agora mostram explicitamente "sem sensor"
  quando `imu::begin()` falha (ver `setup()` em
  [src/lcd_main/main.cpp](../src/lcd_main/main.cpp)).
- Removido o log de diagnóstico por endereço e a varredura completa do
  barramento (eram só para esta investigação).

---

## 6. Bug de clique fantasma ao voltar de tela por gesto (LVGL)

**Sintoma:** ao implementar a navegação por gesto (deslizar) entre a tela
principal e a tela OBD-II, voltar da tela OBD-II para a tela principal
acionava automaticamente o relé que estivesse embaixo do dedo no momento em
que o gesto terminava — mesmo sem a intenção de tocar em nenhum botão.

### Causa raiz

Pitfall conhecido da LVGL 8.x: quando a tela ativa é trocada
(`lv_scr_load`/`lv_scr_load_anim`) **enquanto o dedo ainda está pressionado**
(no fim de um gesto de arrastar), o evento de "soltar" que chega logo em
seguida pode acabar sendo processado contra o objeto que estiver naquela
mesma posição **na tela nova**, gerando um clique não-intencional nela.

### Correção aplicada

- [src/lcd_main/main.cpp](../src/lcd_main/main.cpp): chamado
  `lv_indev_wait_release(indev)` logo antes de cada `lv_scr_load_anim()` no
  callback de gesto (`screen_gesture_event_cb`) — essa função da própria
  LVGL existe especificamente para este cenário: descarta o processamento do
  toque atual até que o dedo seja fisicamente solto e uma nova pressão
  comece, evitando que o "soltar" vaze para a tela recém-carregada.

**Confirmado pelo usuário:** alternar entre as duas telas várias vezes,
terminando o gesto em cima de diferentes botões, não aciona mais nenhum
relé sozinho.

---

## 7. Lições gerais para a próxima depuração

1. **`ESP_LOGx` não é confiável neste projeto/ambiente** — use sempre
   `Serial.printf`/`Serial.println` para diagnóstico ad-hoc. Isso custou a
   maior parte do tempo perdido na seção 3 (touch).
2. **A porta COM muda a cada reset** (USB-CDC nativo) — sempre rodar
   `platformio device list` antes de cada upload/monitor, nunca assumir a
   porta anterior.
3. **Qualquer atraso entre a inicialização do painel RGB e `lvgl_port_init()`
   é perigoso** neste driver — o registro do callback de vsync precisa
   continuar depois do `lvgl_port_init()`, nunca antes.
4. **Pareamento ESP-NOW "bem-sucedido" de um lado não prova nada do outro
   lado** — ao depurar o protocolo de pareamento/comunicação, monitore o log
   serial **dos dois dispositivos** durante um boot completo.
5. **Correções baseadas em buscas genéricas na internet podem valer para uma
   API/versão diferente da que o projeto realmente usa** — o bug do
   `scl_speed_hz` veio de uma sugestão que servia para a API nova
   (`i2c_master`), não para a legada (`i2c_driver_install`) usada aqui. Sempre
   confirmar contra o código-fonte real da versão do ESP-IDF em uso
   (`framework-arduinoespressif32-libs`) antes de aplicar uma correção às
   cegas.
6. **Comparar com um projeto irmão/referência conhecida (IQ50LCD) é valioso**,
   mas tem limite: o IQ50LCD nunca exercitou touch nem adicionava atrasos
   entre o init do painel e o LVGL, então nunca teria revelado o bug da
   seção 2 mesmo sendo "igual" neste trecho.
7. Ao migrar a placa de relés (ver regra 7 do [CLAUDE.md](../CLAUDE.md)),
   **testar o pareamento com log serial dos dois lados de novo** antes de
   assumir que está tudo OK só porque a tela diz "Pareado".
8. **"Hardware ausente" também precisa de prova, não só suposição** — uma
   varredura completa do barramento I2C (endereços 0x08-0x77) é rápida de
   fazer e resolve a dúvida de vez, em vez de assumir que um sensor "não
   detectado" está com defeito ou foi removido do lote.
9. **Trocar a tela ativa da LVGL durante um gesto de arrastar pode vazar o
   "soltar o dedo" para a tela nova como um clique** — sempre chamar
   `lv_indev_wait_release()` antes de `lv_scr_load()`/`lv_scr_load_anim()`
   dentro de um callback de gesto.
