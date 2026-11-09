/*****************************************************************************
 * | File        :   main.cpp  (lcd_main)
 * | Function    :   Firmware for the Waveshare ESP32-S3 Touch LCD 4.3"
 * |                 "ESPRemotecontrol" - touchscreen UI to remotely switch
 * |                 4 auxiliary relay channels on another ESP32 module over
 * |                 ESP-NOW, plus a tilt (pitch/roll) readout from the
 * |                 MPU6050 mounted on the relay module (received over ESP-NOW).
 * |
 * | Tres telas, navegadas por gesto de arrastar (swipe):
 * |   - Tela principal: botoes dos reles + inclinacao resumida.
 * |   - Tela OBD-II (deslizar para a esquerda): medidores de inclinacao
 * |     lateral/frontal (MPU6050 da placa de reles) e temperatura de motor (dado real via
 * |     ELM327 Bluetooth Classico, lido pela placa de reles e repassado por
 * |     ESP-NOW - MsgType::ObdData, ver README.md). Temperatura de cambio
 * |     fica em "sem dados" permanentemente - nao ha PID Modo 01 padrao para
 * |     isso (e especifico de cada fabricante).
 * |   - Tela OBD-II - Informacoes gerais (deslizar para a esquerda de novo):
 * |     parametros mais comuns em HUDs OBD-II comerciais (velocidade, RPM,
 * |     temperatura do motor, tensao da bateria, consumo estimado, carga do
 * |     motor, MAP e nivel de combustivel) + botao Conectar/Desconectar que
 * |     liga/desliga a conexao Bluetooth Classico da placa de reles com o
 * |     ELM327 (MsgType::ObdControl). Tambem tem o botao "Carro no plano":
 * |     guarda a inclinacao atual (roll/pitch) como nivelado (offset
 * |     persistido na NVS) e os medidores da 2a tela passam a mostrar a
 * |     inclinacao relativa a esse offset (valores brutos nao sao zerados).
 * |   - Tela de DTCs - codigos de falha (deslizar para a esquerda de novo):
 * |     mostra status do MIL (luz de check engine) e a lista de codigos
 * |     armazenados/pendentes (Modo 03/07), com botoes "Ler DTCs" e "Limpar
 * |     codigos" (Modo 04) que pedem a operacao a placa de reles via
 * |     MsgType::DtcControl - resultado chega por MsgType::DtcData. Leitura
 * |     sob demanda (nao fica repetindo sozinha), ja que e mais lenta que o
 * |     ciclo normal de telemetria.
 * |
 * | Indicador de bateria (icone no canto superior direito das 4 telas):
 * |   percentual da LiPo do CONECTOR DA PLACA (nao confundir com a "tensao
 * |   da bateria" do veiculo, acima, que vem do PID OBD-II 0142). Depende de
 * |   um INA219 EXTERNO ligado na LiPo via o terminal I2C da placa - o
 * |   chip gerenciador de carga onboard (CS8501) nao expoe a tensao da
 * |   bateria para nenhum GPIO (ver lib/battery_ina219/battery.h). Mostra
 * |   "sem sensor" ate essa fiacao ser feita.
 * |
 * | NOT implemented yet (future phases, see README.md):
 * |   - Temperatura de cambio (precisaria de PID especifico do fabricante)
 * |   - Magnetic compass heading (needs an external magnetometer module)
 ******************************************************************************/

#include <Arduino.h>
#include <string.h>
#include <Preferences.h>
#include <lvgl.h>
#include "rgb_lcd_port.h"
#include "lvgl_port.h"
#include "touch.h"
#include "gt911.h"
#include "battery.h"
#include "espnow_link.h"

using namespace remote_link;

// ======================= Configuration ====================================

// Shared secret used to derive the ESP-NOW encryption key.
// MUST be identical to the one used in src/relay_main/main.cpp.
static const char *PAIRING_KEY = "ALTERE-ESTA-CHAVE-0001";

// Fixed 2.4GHz Wi-Fi channel used by both devices for ESP-NOW (1-13).
static const uint8_t WIFI_CHANNEL = 6;

static const uint32_t LINK_TIMEOUT_MS     = 8000;  // no status from relay -> "offline"
static const uint32_t HEARTBEAT_INTERVAL_MS = 2000;
// Carga da bateria nao precisa de atualizacao rapida - a leitura
// vem de um INA219 externo (ver lib/battery_ina219/battery.h) ligado na
// bateria LiPo do conector da placa.
static const uint32_t BATTERY_UPDATE_INTERVAL_MS = 5000;

static const char *kChannelLabels[kRelayChannels] = {
    "AUX 1", "AUX 2", "AUX 3", "AUX 4"
};

// Faixas dos medidores (gauges) da tela OBD-II.
static const float kTiltRangeDeg = 45.0f;   // medidores de inclinacao: -45..+45
static const int kEngineTempMin = 40, kEngineTempMax = 130; // graus C
static const int kTransTempMin  = 40, kTransTempMax  = 130; // graus C

// ======================= State =============================================

static uint8_t s_relay_mac[6] = {0};
static bool s_paired = false;
static uint8_t s_ui_mask = 0; // last known/desired relay states shown on screen
static volatile uint32_t s_last_relay_msg_ms = 0; // escrito por onMessage() (task do ESP-NOW)
static uint32_t s_last_heartbeat_ms = 0;
static uint32_t s_last_battery_ms = 0;
static bool s_battery_sensor_present = false;

// Dados do OBD-II, preenchidos a partir de MsgType::ObdData (ver onMessage())
// enviado pela placa de reles, que fala com o adaptador ELM327 via Bluetooth
// Classico (ver README.md). trans_temp_* ficam sempre invalidos - nao ha PID
// Modo 01 padrao para temperatura de cambio (e especifico de cada fabricante).
// rpm/speed/voltage/fuel_rate sao os parametros mais comuns em HUDs OBD-II
// comerciais (ver README.md); fuel_rate_lph e uma ESTIMATIVA calculada a
// partir do MAF (nao e um PID padrao - ver relay_main/main.cpp).
struct ObdData {
    bool connected = false;
    float engine_temp_c = 0.0f;
    bool engine_temp_valid = false;
    float trans_temp_c = 0.0f;
    bool trans_temp_valid = false;
    uint16_t rpm = 0;
    bool rpm_valid = false;
    uint8_t speed_kmh = 0;
    bool speed_valid = false;
    float voltage_v = 0.0f;
    bool voltage_valid = false;
    float fuel_rate_lph = 0.0f;
    bool fuel_rate_valid = false;
    uint8_t engine_load_pct = 0;
    bool engine_load_valid = false;
    uint8_t map_kpa = 0;
    bool map_valid = false;
    uint8_t fuel_level_pct = 0;
    bool fuel_level_valid = false;
};
static ObdData s_obd_data;

// Telemetria da placa de reles (MsgType::StatusReport) alem do estado dos
// canais (s_ui_mask): inclinacao do MPU6050 montado sobre ela e estado do
// sensor de nivel de agua sem contato (XKC-Y25-V) no reservatorio - ver
// onMessage() / update_relay_telemetry_display().
struct RelayTelemetry {
    bool tilt_valid = false;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    bool water_level_valid = false;
    bool water_level_low = false;
};
static RelayTelemetry s_relay_telemetry;

// Resultado da ultima leitura/limpeza de DTCs (MsgType::DtcData), pedida
// pelos botoes da 4a tela - ver update_dtc_screen() / onMessage().
struct DtcInfo {
    bool has_data = false; // false ate a 1a resposta chegar (nunca pedimos ainda)
    bool mil_on = false;
    uint8_t count = 0;
    struct { uint16_t code; bool pending; } entries[kMaxDtcEntries];
};
static DtcInfo s_dtc_info;
static bool s_dtc_request_in_flight = false; // true entre o toque no botao e a resposta chegar

// Estado local (otimista) do botao Conectar/Desconectar da 3a tela - o que
// foi pedido por ultimo pela tela, nao necessariamente o estado real (esse
// vem de s_obd_data.connected, atualizado pela placa de reles).
static bool s_obd_requested_connect = true;

// Calibracao "carro no plano": ultima leitura bruta de inclinacao (MPU6050
// da placa de reles) e os offsets guardados que sao subtraidos dela nos
// medidores da 2a tela e no rotulo da tela principal. Os valores brutos
// mostrados na 3a tela nao sao alterados. Os offsets sao persistidos na NVS.
static bool s_tilt_raw_valid = false;
static float s_tilt_roll_raw = 0.0f, s_tilt_pitch_raw = 0.0f;
static float s_tilt_roll_offset = 0.0f, s_tilt_pitch_offset = 0.0f;
static const char *kTiltPrefsNamespace = "tilt";

// ======================= UI widgets =========================================

static lv_obj_t *s_screen_main = NULL;
static lv_obj_t *s_screen_obd = NULL;
static lv_obj_t *s_screen_obd_info = NULL;
static lv_obj_t *s_screen_dtc = NULL;

static lv_obj_t *s_label_conn = NULL;
static lv_obj_t *s_label_tilt = NULL;
static lv_obj_t *s_relay_buttons[kRelayChannels];

// Indicador de bateria (icone + percentual), replicado no canto superior
// direito das 4 telas - ver update_battery_display().
static lv_obj_t *s_label_battery_main = NULL;
static lv_obj_t *s_label_battery_obd = NULL;
static lv_obj_t *s_label_battery_obd_info = NULL;
static lv_obj_t *s_label_battery_dtc = NULL;

// Tela OBD-II: medidores (gauges) + rotulos de valor numerico.
static lv_obj_t *s_label_obd_status = NULL;
static lv_obj_t *s_meter_roll = NULL, *s_meter_pitch = NULL;
static lv_obj_t *s_meter_engine = NULL, *s_meter_trans = NULL;
static lv_meter_indicator_t *s_needle_roll = NULL, *s_needle_pitch = NULL;
static lv_meter_indicator_t *s_needle_engine = NULL, *s_needle_trans = NULL;
static lv_obj_t *s_label_roll_val = NULL, *s_label_pitch_val = NULL;
static lv_obj_t *s_label_engine_val = NULL, *s_label_trans_val = NULL;

// 3a tela: informacoes gerais do OBD-II + botao Conectar/Desconectar.
static lv_obj_t *s_label_obd_info_status = NULL;
static lv_obj_t *s_label_obd_speed_val = NULL, *s_label_obd_rpm_val = NULL;
static lv_obj_t *s_label_obd_temp_val = NULL;
static lv_obj_t *s_label_obd_voltage_val = NULL, *s_label_obd_fuel_val = NULL;
// 2a coluna: PIDs extras confirmados suportados no veiculo de teste (ver
// tabela.md) que nao cabiam na 1a coluna sem estourar a tela.
static lv_obj_t *s_label_obd_engine_load_val = NULL, *s_label_obd_map_val = NULL;
static lv_obj_t *s_label_obd_fuel_level_val = NULL;
static lv_obj_t *s_btn_obd_connect = NULL, *s_label_obd_connect_btn = NULL;
static lv_obj_t *s_btn_tilt_zero = NULL, *s_label_tilt_zero_btn = NULL;
// Telemetria extra enviada pela placa de reles em MsgType::StatusReport:
// inclinacao do MPU6050 montado nela e estado do nivel do reservatorio
// (sensor XKC-Y25-V) - ver onMessage().
static lv_obj_t *s_label_relay_tilt_val = NULL;
static lv_obj_t *s_label_water_level_val = NULL;

// 4a tela: DTCs (codigos de falha) - status do MIL, lista rolavel de
// codigos e botoes Ler/Limpar (ver create_dtc_screen()/update_dtc_screen()).
static lv_obj_t *s_label_dtc_status = NULL;
static lv_obj_t *s_label_dtc_summary = NULL;
static lv_obj_t *s_list_dtc = NULL;
static lv_obj_t *s_btn_dtc_read = NULL, *s_btn_dtc_clear = NULL;

static void update_connection_label()
{
    if (!lvgl_port_lock(50)) {
        return;
    }
    if (s_paired) {
        char buf[48];
        snprintf(buf, sizeof(buf), "Rele: CONECTADO");
        lv_label_set_text(s_label_conn, buf);
        lv_obj_set_style_text_color(s_label_conn, lv_palette_main(LV_PALETTE_GREEN), 0);
    } else {
        lv_label_set_text(s_label_conn, "Rele: procurando...");
        lv_obj_set_style_text_color(s_label_conn, lv_palette_main(LV_PALETTE_RED), 0);
    }
    lvgl_port_unlock();
}

// Cria o rotulo do indicador de bateria no canto superior direito de uma
// tela, logo abaixo do rotulo de status ja existente nela (ver
// update_battery_display() para o conteudo/cor).
static lv_obj_t *create_battery_label(lv_obj_t *parent)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_label_set_text(lbl, LV_SYMBOL_BATTERY_EMPTY " sem sensor");
    lv_obj_set_style_text_color(lbl, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(lbl, LV_ALIGN_TOP_RIGHT, -20, 45);
    return lbl;
}

// Atualiza o indicador de bateria nas 3 telas a partir de uma leitura do
// INA219 externo (lib/battery_ina219) ligado na LiPo do conector da placa -
// ver battery.h para o porque disso depender de um sensor externo. Se o
// sensor nunca foi detectado (battery::begin() falhou em setup()), mostra
// "sem sensor" em cinza, no mesmo espirito do fallback do IMU.
static void update_battery_display()
{
    if (!lvgl_port_lock(50)) {
        return;
    }

    char buf[24];
    lv_color_t color;

    if (!s_battery_sensor_present) {
        snprintf(buf, sizeof(buf), LV_SYMBOL_BATTERY_EMPTY " sem sensor");
        color = lv_palette_main(LV_PALETTE_GREY);
    } else {
        battery::Reading r = battery::read();
        if (!r.valid) {
            snprintf(buf, sizeof(buf), LV_SYMBOL_BATTERY_EMPTY " sem leitura");
            color = lv_palette_main(LV_PALETTE_GREY);
        } else {
            const char *icon;
            if (r.percent >= 80) {
                icon = LV_SYMBOL_BATTERY_FULL;
            } else if (r.percent >= 55) {
                icon = LV_SYMBOL_BATTERY_3;
            } else if (r.percent >= 30) {
                icon = LV_SYMBOL_BATTERY_2;
            } else if (r.percent >= 10) {
                icon = LV_SYMBOL_BATTERY_1;
            } else {
                icon = LV_SYMBOL_BATTERY_EMPTY;
            }
            snprintf(buf, sizeof(buf), "%s %u%%", icon, (unsigned)r.percent);
            if (r.percent >= 50) {
                color = lv_palette_main(LV_PALETTE_GREEN);
            } else if (r.percent >= 20) {
                color = lv_palette_main(LV_PALETTE_ORANGE);
            } else {
                color = lv_palette_main(LV_PALETTE_RED);
            }
        }
    }

    lv_label_set_text(s_label_battery_main, buf);
    lv_obj_set_style_text_color(s_label_battery_main, color, 0);
    lv_label_set_text(s_label_battery_obd, buf);
    lv_obj_set_style_text_color(s_label_battery_obd, color, 0);
    lv_label_set_text(s_label_battery_obd_info, buf);
    lv_obj_set_style_text_color(s_label_battery_obd_info, color, 0);
    lv_label_set_text(s_label_battery_dtc, buf);
    lv_obj_set_style_text_color(s_label_battery_dtc, color, 0);

    lvgl_port_unlock();
}

static void update_relay_buttons()
{
    if (!lvgl_port_lock(50)) {
        return;
    }
    for (uint8_t ch = 0; ch < kRelayChannels; ch++) {
        bool on = ((s_ui_mask >> ch) & 0x01) != 0;
        if (on) {
            lv_obj_add_state(s_relay_buttons[ch], LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(s_relay_buttons[ch], LV_STATE_CHECKED);
        }
    }
    lvgl_port_unlock();
}

static void relay_button_event_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    uint8_t channel = (uint8_t)(intptr_t)lv_event_get_user_data(e);
    bool on = lv_obj_has_state(btn, LV_STATE_CHECKED);

    if (on) {
        s_ui_mask |= (uint8_t)(1 << channel);
    } else {
        s_ui_mask &= (uint8_t)~(1 << channel);
    }

    if (s_paired) {
        Message msg;
        msg.type = MsgType::SetRelay;
        msg.payload.set_relay.channel = channel;
        msg.payload.set_relay.state = on ? 1 : 0;
        send(s_relay_mac, msg);
    }
}

static void screen_gesture_event_cb(lv_event_t *e)
{
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) {
        return;
    }
    lv_dir_t dir = lv_indev_get_gesture_dir(indev);
    lv_obj_t *scr = lv_event_get_current_target(e);

    // lv_indev_wait_release() evita que o "soltar o dedo" (fim do gesto)
    // seja processado como um clique no widget que ficar embaixo do dedo na
    // tela nova (ex.: acionar um rele sem querer so por ter voltado da tela
    // OBD-II deslizando o dedo ate em cima de um botao).
    if (dir == LV_DIR_LEFT && scr == s_screen_main) {
        lv_indev_wait_release(indev);
        lv_scr_load_anim(s_screen_obd, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, false);
    } else if (dir == LV_DIR_RIGHT && scr == s_screen_obd) {
        lv_indev_wait_release(indev);
        lv_scr_load_anim(s_screen_main, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 250, 0, false);
    } else if (dir == LV_DIR_LEFT && scr == s_screen_obd) {
        lv_indev_wait_release(indev);
        lv_scr_load_anim(s_screen_obd_info, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, false);
    } else if (dir == LV_DIR_RIGHT && scr == s_screen_obd_info) {
        lv_indev_wait_release(indev);
        lv_scr_load_anim(s_screen_obd, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 250, 0, false);
    } else if (dir == LV_DIR_LEFT && scr == s_screen_obd_info) {
        lv_indev_wait_release(indev);
        lv_scr_load_anim(s_screen_dtc, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, false);
    } else if (dir == LV_DIR_RIGHT && scr == s_screen_dtc) {
        lv_indev_wait_release(indev);
        lv_scr_load_anim(s_screen_obd_info, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 250, 0, false);
    }
}

// Medidor circular generico usado tanto para inclinacao quanto para
// temperatura. zone_colors/zone_bounds (opcionais, len zone_count) desenham
// arcos coloridos de faixa (ex.: frio/normal/quente); passe zone_count=0
// para um medidor "neutro" (usado nos de inclinacao).
static void create_gauge(lv_obj_t *parent, int x, int y, int w, const char *title,
                          int32_t min_val, int32_t max_val, uint32_t angle_range, uint32_t rotation,
                          const int32_t *zone_bounds, const lv_color_t *zone_colors, int zone_count,
                          lv_obj_t **meter_out, lv_meter_indicator_t **needle_out, lv_obj_t **value_label_out,
                          const char *initial_value_text, lv_color_t initial_value_color)
{
    lv_obj_t *title_lbl = lv_label_create(parent);
    lv_label_set_text(title_lbl, title);
    lv_obj_set_style_text_color(title_lbl, lv_color_white(), 0);
    lv_obj_set_pos(title_lbl, x, y - 22);

    lv_obj_t *meter = lv_meter_create(parent);
    lv_obj_set_size(meter, w, w);
    lv_obj_set_pos(meter, x, y);
    lv_obj_set_style_bg_color(meter, lv_color_hex(0x1b2127), LV_PART_MAIN);
    lv_obj_set_style_border_width(meter, 0, LV_PART_MAIN);

    lv_meter_scale_t *scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_range(meter, scale, min_val, max_val, angle_range, rotation);
    lv_meter_set_scale_ticks(meter, scale, 9, 2, 8, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, scale, 3, 3, 12, lv_color_white(), 12);

    for (int i = 0; i < zone_count; i++) {
        lv_meter_indicator_t *zone = lv_meter_add_arc(meter, scale, 6, zone_colors[i], 0);
        lv_meter_set_indicator_start_value(meter, zone, zone_bounds[i]);
        lv_meter_set_indicator_end_value(meter, zone, zone_bounds[i + 1]);
    }

    lv_meter_indicator_t *needle = lv_meter_add_needle_line(meter, scale, 3, lv_color_white(), -10);
    lv_meter_set_indicator_value(meter, needle, min_val);

    lv_obj_t *value_lbl = lv_label_create(parent);
    lv_label_set_text(value_lbl, initial_value_text);
    lv_obj_set_style_text_color(value_lbl, initial_value_color, 0);
    lv_obj_set_pos(value_lbl, x, y + w + 6);

    *meter_out = meter;
    *needle_out = needle;
    *value_label_out = value_lbl;
}

static void create_main_screen()
{
    s_screen_main = lv_obj_create(NULL);
    lv_obj_t *scr = s_screen_main;
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr, screen_gesture_event_cb, LV_EVENT_GESTURE, NULL);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Controle Remoto - Luzes Auxiliares");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 20, 15);

    s_label_conn = lv_label_create(scr);
    lv_label_set_text(s_label_conn, "Rele: procurando...");
    lv_obj_align(s_label_conn, LV_ALIGN_TOP_RIGHT, -20, 15);

    s_label_battery_main = create_battery_label(scr);

    static lv_style_t style_btn_off;
    lv_style_init(&style_btn_off);
    lv_style_set_bg_color(&style_btn_off, lv_palette_main(LV_PALETTE_GREY));

    static lv_style_t style_btn_on;
    lv_style_init(&style_btn_on);
    lv_style_set_bg_color(&style_btn_on, lv_palette_main(LV_PALETTE_GREEN));

    // 4 canais em grade 2x2 (reduzido de 6 - ver remote_protocol.h).
    const int cols = 2;
    const int btn_w = 360, btn_h = 150, col_gap = 40, row_gap = 20, margin = 20, top = 70;

    for (uint8_t ch = 0; ch < kRelayChannels; ch++) {
        int col = ch % cols;
        int row = ch / cols;
        int x = margin + col * (btn_w + col_gap);
        int y = top + row * (btn_h + row_gap);

        lv_obj_t *btn = lv_btn_create(scr);
        lv_obj_set_pos(btn, x, y);
        lv_obj_set_size(btn, btn_w, btn_h);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);
        lv_obj_add_style(btn, &style_btn_off, 0);
        lv_obj_add_style(btn, &style_btn_on, LV_STATE_CHECKED);
        lv_obj_add_event_cb(btn, relay_button_event_cb, LV_EVENT_CLICKED, (void *)(intptr_t)ch);

        lv_obj_t *label = lv_label_create(btn);
        lv_label_set_text(label, kChannelLabels[ch]);
        lv_obj_center(label);

        s_relay_buttons[ch] = btn;
    }

    s_label_tilt = lv_label_create(scr);
    lv_label_set_text(s_label_tilt, "Inclinacao: --");
    lv_obj_align(s_label_tilt, LV_ALIGN_BOTTOM_LEFT, 20, -15);
    lv_obj_set_style_text_color(s_label_tilt, lv_color_white(), 0);

    lv_obj_t *hint = lv_label_create(scr);
    lv_label_set_text(hint, "Deslize p/ esquerda: OBD-II >");
    lv_obj_set_style_text_color(hint, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_RIGHT, -20, -15);
}

static void create_obd_screen()
{
    s_screen_obd = lv_obj_create(NULL);
    lv_obj_t *scr = s_screen_obd;
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr, screen_gesture_event_cb, LV_EVENT_GESTURE, NULL);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Diagnostico OBD-II");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 20, 15);

    s_label_obd_status = lv_label_create(scr);
    lv_label_set_text(s_label_obd_status, "OBD: desconectado");
    lv_obj_set_style_text_color(s_label_obd_status, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_align(s_label_obd_status, LV_ALIGN_TOP_RIGHT, -20, 15);

    s_label_battery_obd = create_battery_label(scr);

    // 4 medidores lado a lado: inclinacao lateral/frontal (dados reais, do
    // MPU6050 da placa de reles, via ESP-NOW) + temperatura de motor/cambio (aguardando adaptador
    // OBD-II - ver README.md "Roadmap"). "top" comeca em 100 (em vez de 75)
    // para abrir espaco vertical para o indicador de bateria logo acima,
    // que fica por cima do titulo do 4o medidor (lado direito da tela).
    const int w = 170, gap = 20, margin = 20, top = 100;
    int x0 = margin;
    int x1 = margin + 1 * (w + gap);
    int x2 = margin + 2 * (w + gap);
    int x3 = margin + 3 * (w + gap);

    create_gauge(scr, x0, top, w, "Inclinacao lateral", -(int)kTiltRangeDeg, (int)kTiltRangeDeg, 180, 180,
                 NULL, NULL, 0, &s_meter_roll, &s_needle_roll, &s_label_roll_val, "-- graus", lv_color_white());

    create_gauge(scr, x1, top, w, "Inclinacao frontal", -(int)kTiltRangeDeg, (int)kTiltRangeDeg, 180, 180,
                 NULL, NULL, 0, &s_meter_pitch, &s_needle_pitch, &s_label_pitch_val, "-- graus", lv_color_white());

    static const int32_t engine_zones[4] = {kEngineTempMin, 70, 105, kEngineTempMax};
    static const lv_color_t engine_colors[3] = {
        lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_GREEN), lv_palette_main(LV_PALETTE_RED)
    };
    create_gauge(scr, x2, top, w, "Temp. motor", kEngineTempMin, kEngineTempMax, 270, 135,
                 engine_zones, engine_colors, 3, &s_meter_engine, &s_needle_engine, &s_label_engine_val,
                 "sem dados", lv_palette_main(LV_PALETTE_GREY));

    static const int32_t trans_zones[4] = {kTransTempMin, 70, 105, kTransTempMax};
    static const lv_color_t trans_colors[3] = {
        lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_GREEN), lv_palette_main(LV_PALETTE_RED)
    };
    create_gauge(scr, x3, top, w, "Temp. cambio", kTransTempMin, kTransTempMax, 270, 135,
                 trans_zones, trans_colors, 3, &s_meter_trans, &s_needle_trans, &s_label_trans_val,
                 "sem dados", lv_palette_main(LV_PALETTE_GREY));

    lv_obj_t *note = lv_label_create(scr);
    lv_label_set_text(note,
        "Temp. motor: ELM327 Bluetooth Classico via placa de reles (PID 0105)\n"
        "Temp. cambio: sem PID OBD-II padrao (especifico de fabricante - nao implementado)\n"
        "Bussola requer magnetometro externo (nao instalado)");
    lv_obj_set_style_text_color(note, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(note, LV_ALIGN_BOTTOM_LEFT, 20, -35);

    lv_obj_t *hint = lv_label_create(scr);
    lv_label_set_text(hint, "< Deslize p/ direita: Reles");
    lv_obj_set_style_text_color(hint, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_RIGHT, -20, -15);

    lv_obj_t *hint2 = lv_label_create(scr);
    lv_label_set_text(hint2, "Deslize p/ esquerda: Info/Conexao OBD >");
    lv_obj_set_style_text_color(hint2, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(hint2, LV_ALIGN_BOTTOM_LEFT, 20, -15);
}

// Envia MsgType::ObdControl para a placa de reles (unico chip com Bluetooth
// Classico de verdade - ver README.md) alternando conectar/desconectar do
// ELM327. Atualizacao de rotulo e otimista; o status real ("OBD: conectado/
// desconectado") vem de volta via MsgType::ObdData e atualiza
// s_label_obd_info_status separadamente.
static void obd_connect_button_event_cb(lv_event_t *e)
{
    (void)e;
    s_obd_requested_connect = !s_obd_requested_connect;

    if (s_paired) {
        Message msg;
        msg.type = MsgType::ObdControl;
        msg.payload.obd_control.connect = s_obd_requested_connect ? 1 : 0;
        send(s_relay_mac, msg);
    }

    lv_label_set_text(s_label_obd_connect_btn, s_obd_requested_connect ? "Desconectar" : "Conectar");
}

// Forward declaration - definida mais abaixo, junto de update_tilt_display().
static void update_tilt_display(float roll_deg, float pitch_deg);

static void tilt_zero_feedback_timer_cb(lv_timer_t *t)
{
    (void)t;
    lv_label_set_text(s_label_tilt_zero_btn, "Carro no plano");
}

// Guarda a inclinacao atual como "nivelado" (zero) - os valores brutos nao
// sao zerados, so os medidores/rotulo calibrados passam a descontar o
// offset. Roda dentro do lock do LVGL (callback de evento).
static void tilt_zero_button_event_cb(lv_event_t *e)
{
    (void)e;
    if (!s_tilt_raw_valid) {
        lv_label_set_text(s_label_tilt_zero_btn, "Sem leitura");
        lv_timer_set_repeat_count(lv_timer_create(tilt_zero_feedback_timer_cb, 1500, NULL), 1);
        return;
    }

    s_tilt_roll_offset = s_tilt_roll_raw;
    s_tilt_pitch_offset = s_tilt_pitch_raw;

    Preferences prefs;
    if (prefs.begin(kTiltPrefsNamespace, false)) {
        prefs.putFloat("roll", s_tilt_roll_offset);
        prefs.putFloat("pitch", s_tilt_pitch_offset);
        prefs.end();
    }

    update_tilt_display(s_tilt_roll_raw, s_tilt_pitch_raw);

    lv_label_set_text(s_label_tilt_zero_btn, "Salvo!");
    lv_timer_set_repeat_count(lv_timer_create(tilt_zero_feedback_timer_cb, 1500, NULL), 1);
}

// Cria uma linha "rotulo: valor" na 3a tela (OBD-II - informacoes gerais).
static lv_obj_t *create_info_row(lv_obj_t *parent, int x, int y, const char *label_text)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_label_set_text(lbl, label_text);
    lv_obj_set_style_text_color(lbl, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_pos(lbl, x, y);

    lv_obj_t *val = lv_label_create(parent);
    lv_label_set_text(val, "sem dados");
    lv_obj_set_style_text_color(val, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_pos(val, x + 220, y);
    return val;
}

// 3a tela: parametros mais comuns em HUDs OBD-II comerciais (velocidade,
// RPM, temperatura do motor, tensao, consumo estimado - ver README.md) e o
// botao Conectar/Desconectar do adaptador ELM327.
static void create_obd_info_screen()
{
    s_screen_obd_info = lv_obj_create(NULL);
    lv_obj_t *scr = s_screen_obd_info;
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr, screen_gesture_event_cb, LV_EVENT_GESTURE, NULL);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "OBD-II - Informacoes gerais");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 20, 15);

    s_label_obd_info_status = lv_label_create(scr);
    lv_label_set_text(s_label_obd_info_status, "OBD: desconectado");
    lv_obj_set_style_text_color(s_label_obd_info_status, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_align(s_label_obd_info_status, LV_ALIGN_TOP_RIGHT, -20, 15);

    s_label_battery_obd_info = create_battery_label(scr);

    s_btn_obd_connect = lv_btn_create(scr);
    lv_obj_set_pos(s_btn_obd_connect, 20, 60);
    lv_obj_set_size(s_btn_obd_connect, 220, 70);
    lv_obj_add_event_cb(s_btn_obd_connect, obd_connect_button_event_cb, LV_EVENT_CLICKED, NULL);

    s_label_obd_connect_btn = lv_label_create(s_btn_obd_connect);
    lv_label_set_text(s_label_obd_connect_btn, s_obd_requested_connect ? "Desconectar" : "Conectar");
    lv_obj_center(s_label_obd_connect_btn);

    s_btn_tilt_zero = lv_btn_create(scr);
    lv_obj_set_pos(s_btn_tilt_zero, 260, 60);
    lv_obj_set_size(s_btn_tilt_zero, 220, 70);
    lv_obj_add_event_cb(s_btn_tilt_zero, tilt_zero_button_event_cb, LV_EVENT_CLICKED, NULL);

    s_label_tilt_zero_btn = lv_label_create(s_btn_tilt_zero);
    lv_label_set_text(s_label_tilt_zero_btn, "Carro no plano");
    lv_obj_center(s_label_tilt_zero_btn);

    const int row_x = 20, row_y0 = 170, row_gap = 55;
    s_label_obd_speed_val  = create_info_row(scr, row_x, row_y0 + row_gap * 0, "Velocidade:");
    s_label_obd_rpm_val    = create_info_row(scr, row_x, row_y0 + row_gap * 1, "RPM do motor:");
    s_label_obd_temp_val   = create_info_row(scr, row_x, row_y0 + row_gap * 2, "Temp. motor:");
    s_label_obd_voltage_val = create_info_row(scr, row_x, row_y0 + row_gap * 3, "Tensao da bateria:");
    s_label_obd_fuel_val   = create_info_row(scr, row_x, row_y0 + row_gap * 4, "Consumo (estimado):");

    // 2a coluna: PIDs extras confirmados suportados no veiculo de teste
    // (0104/010B/012F - ver tabela.md), sem precisar de estimativa.
    const int row_x2 = 450;
    s_label_obd_engine_load_val = create_info_row(scr, row_x2, row_y0 + row_gap * 0, "Carga do motor:");
    s_label_obd_map_val         = create_info_row(scr, row_x2, row_y0 + row_gap * 1, "MAP (admissao):");
    s_label_obd_fuel_level_val  = create_info_row(scr, row_x2, row_y0 + row_gap * 2, "Nivel combustivel:");
    // Telemetria da placa de reles (MPU6050 + sensor de nivel XKC-Y25-V),
    // nao vem do OBD-II mas cabe nesta tela de "informacoes gerais".
    s_label_relay_tilt_val  = create_info_row(scr, row_x2, row_y0 + row_gap * 3, "Inclinacao (reles):");
    s_label_water_level_val = create_info_row(scr, row_x2, row_y0 + row_gap * 4, "Nivel reservatorio:");

    lv_obj_t *note = lv_label_create(scr);
    lv_label_set_text(note,
        "Consumo estimado a partir do fluxo de ar (MAF) - assume AFR 14.5:1 e\n"
        "densidade do diesel 832 g/L; pode variar com a qualidade do combustivel.");
    lv_obj_set_style_text_color(note, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(note, LV_ALIGN_BOTTOM_LEFT, 20, -35);

    lv_obj_t *hint = lv_label_create(scr);
    lv_label_set_text(hint, "< Deslize p/ direita: Temp./Inclinacao");
    lv_obj_set_style_text_color(hint, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_RIGHT, -20, -15);

    lv_obj_t *hint2 = lv_label_create(scr);
    lv_label_set_text(hint2, "Deslize p/ esquerda: DTCs >");
    lv_obj_set_style_text_color(hint2, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(hint2, LV_ALIGN_BOTTOM_LEFT, 20, -15);
}

// Forward declaration - definida mais abaixo, perto de update_obd_info_screen().
static void update_dtc_screen();

// Envia MsgType::DtcControl pedindo uma leitura (action=0) dos DTCs - a
// placa de reles responde via MsgType::DtcData quando terminar (nao e
// instantaneo, ver relay_main/main.cpp). Enquanto isso, mostra "Lendo...".
static void dtc_read_button_event_cb(lv_event_t *e)
{
    (void)e;
    if (s_paired) {
        Message msg;
        msg.type = MsgType::DtcControl;
        msg.payload.dtc_control.action = 0;
        send(s_relay_mac, msg);
        s_dtc_request_in_flight = true;
        update_dtc_screen();
    }
}

// Igual ao de cima, mas pede limpeza (Modo 04, action=1) - a placa de reles
// re-le os codigos logo em seguida para confirmar, entao a resposta chega
// pela mesma MsgType::DtcData.
static void dtc_clear_button_event_cb(lv_event_t *e)
{
    (void)e;
    if (s_paired) {
        Message msg;
        msg.type = MsgType::DtcControl;
        msg.payload.dtc_control.action = 1;
        send(s_relay_mac, msg);
        s_dtc_request_in_flight = true;
        update_dtc_screen();
    }
}

// 4a tela: DTCs (codigos de falha) - status do MIL (luz de check engine) e
// lista dos codigos armazenados (Modo 03)/pendentes (Modo 07), com botoes
// para pedir uma leitura nova ou limpar os codigos (Modo 04). Leitura sob
// demanda (nao fica repetindo sozinha) - ver README.md/relay_main.
static void create_dtc_screen()
{
    s_screen_dtc = lv_obj_create(NULL);
    lv_obj_t *scr = s_screen_dtc;
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr, screen_gesture_event_cb, LV_EVENT_GESTURE, NULL);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "DTCs - Codigos de falha");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 20, 15);

    s_label_dtc_status = lv_label_create(scr);
    lv_label_set_text(s_label_dtc_status, "Sem leitura ainda");
    lv_obj_set_style_text_color(s_label_dtc_status, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(s_label_dtc_status, LV_ALIGN_TOP_RIGHT, -20, 15);

    s_label_battery_dtc = create_battery_label(scr);

    s_btn_dtc_read = lv_btn_create(scr);
    lv_obj_set_pos(s_btn_dtc_read, 20, 60);
    lv_obj_set_size(s_btn_dtc_read, 200, 60);
    lv_obj_add_event_cb(s_btn_dtc_read, dtc_read_button_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_read = lv_label_create(s_btn_dtc_read);
    lv_label_set_text(label_read, "Ler DTCs");
    lv_obj_center(label_read);

    s_btn_dtc_clear = lv_btn_create(scr);
    lv_obj_set_pos(s_btn_dtc_clear, 240, 60);
    lv_obj_set_size(s_btn_dtc_clear, 200, 60);
    static lv_style_t style_btn_clear;
    lv_style_init(&style_btn_clear);
    lv_style_set_bg_color(&style_btn_clear, lv_palette_main(LV_PALETTE_RED));
    lv_obj_add_style(s_btn_dtc_clear, &style_btn_clear, 0);
    lv_obj_add_event_cb(s_btn_dtc_clear, dtc_clear_button_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_clear = lv_label_create(s_btn_dtc_clear);
    lv_label_set_text(label_clear, "Limpar codigos");
    lv_obj_center(label_clear);

    s_label_dtc_summary = lv_label_create(scr);
    lv_label_set_text(s_label_dtc_summary, "Toque em \"Ler DTCs\" para consultar o veiculo");
    lv_obj_set_style_text_color(s_label_dtc_summary, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(s_label_dtc_summary, LV_ALIGN_TOP_LEFT, 20, 140);

    s_list_dtc = lv_list_create(scr);
    lv_obj_set_pos(s_list_dtc, 20, 175);
    lv_obj_set_size(s_list_dtc, 760, 250);
    lv_obj_set_style_bg_color(s_list_dtc, lv_color_hex(0x1b2127), LV_PART_MAIN);

    lv_obj_t *hint = lv_label_create(scr);
    lv_label_set_text(hint, "< Deslize p/ direita: OBD-II Info");
    lv_obj_set_style_text_color(hint, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_RIGHT, -20, -15);
}

// Atualiza o rotulo de texto (tela principal) e os medidores de inclinacao
// (tela OBD-II) a partir de uma leitura de inclinacao (MPU6050 da placa de
// reles). Chamado com o lock do LVGL ja tomado (update_relay_telemetry_display()
// ou callback de evento).
static void update_tilt_display(float roll_deg, float pitch_deg)
{
    s_tilt_raw_valid = true;
    s_tilt_roll_raw = roll_deg;
    s_tilt_pitch_raw = pitch_deg;

    float roll = roll_deg - s_tilt_roll_offset;
    float pitch = pitch_deg - s_tilt_pitch_offset;

    char buf[64];
    snprintf(buf, sizeof(buf), "Inclinacao: Roll %.1f  Pitch %.1f", roll, pitch);
    lv_label_set_text(s_label_tilt, buf);

    float roll_clamped = roll < -kTiltRangeDeg ? -kTiltRangeDeg : (roll > kTiltRangeDeg ? kTiltRangeDeg : roll);
    float pitch_clamped = pitch < -kTiltRangeDeg ? -kTiltRangeDeg : (pitch > kTiltRangeDeg ? kTiltRangeDeg : pitch);
    lv_meter_set_indicator_value(s_meter_roll, s_needle_roll, (int32_t)roll_clamped);
    lv_meter_set_indicator_value(s_meter_pitch, s_needle_pitch, (int32_t)pitch_clamped);

    snprintf(buf, sizeof(buf), "%.1f graus", roll);
    lv_label_set_text(s_label_roll_val, buf);
    snprintf(buf, sizeof(buf), "%.1f graus", pitch);
    lv_label_set_text(s_label_pitch_val, buf);
    lv_obj_set_style_text_color(s_label_roll_val, lv_color_white(), 0);
    lv_obj_set_style_text_color(s_label_pitch_val, lv_color_white(), 0);
}

// Atualiza o rotulo de status e o medidor de temperatura do motor (tela
// OBD-II) a partir de uma mensagem MsgType::ObdData recebida do modulo de
// reles (que fala com o ELM327 via Bluetooth Classico - ver README.md).
// Chamado de dentro de onMessage(), fora do lock do LVGL (toma o lock aqui).
static void update_obd_gauges()
{
    if (!lvgl_port_lock(50)) {
        return;
    }

    if (s_obd_data.connected) {
        lv_label_set_text(s_label_obd_status, "OBD: conectado (ELM327)");
        lv_obj_set_style_text_color(s_label_obd_status, lv_palette_main(LV_PALETTE_GREEN), 0);
    } else {
        lv_label_set_text(s_label_obd_status, "OBD: desconectado");
        lv_obj_set_style_text_color(s_label_obd_status, lv_palette_main(LV_PALETTE_RED), 0);
    }

    char buf[32];
    if (s_obd_data.engine_temp_valid) {
        int32_t clamped = (int32_t)s_obd_data.engine_temp_c;
        clamped = clamped < kEngineTempMin ? kEngineTempMin : (clamped > kEngineTempMax ? kEngineTempMax : clamped);
        lv_meter_set_indicator_value(s_meter_engine, s_needle_engine, clamped);
        snprintf(buf, sizeof(buf), "%.0f C", s_obd_data.engine_temp_c);
        lv_label_set_text(s_label_engine_val, buf);
        lv_obj_set_style_text_color(s_label_engine_val, lv_color_white(), 0);
    } else {
        lv_label_set_text(s_label_engine_val, "sem dados");
        lv_obj_set_style_text_color(s_label_engine_val, lv_palette_main(LV_PALETTE_GREY), 0);
    }

    lvgl_port_unlock();
}

// Atualiza os rotulos de texto da 3a tela (OBD-II - informacoes gerais:
// velocidade, RPM, temperatura do motor, tensao da bateria, consumo
// estimado) a partir de uma mensagem MsgType::ObdData. Mesma convencao de
// lock de update_obd_gauges().
static void update_obd_info_screen()
{
    if (!lvgl_port_lock(50)) {
        return;
    }

    if (s_obd_data.connected) {
        lv_label_set_text(s_label_obd_info_status, "OBD: conectado (ELM327)");
        lv_obj_set_style_text_color(s_label_obd_info_status, lv_palette_main(LV_PALETTE_GREEN), 0);
    } else {
        lv_label_set_text(s_label_obd_info_status, "OBD: desconectado");
        lv_obj_set_style_text_color(s_label_obd_info_status, lv_palette_main(LV_PALETTE_RED), 0);
    }

    char buf[32];

    auto set_row = [](lv_obj_t *label, bool valid, const char *text) {
        lv_label_set_text(label, valid ? text : "sem dados");
        lv_obj_set_style_text_color(label, valid ? lv_color_white() : lv_palette_main(LV_PALETTE_GREY), 0);
    };

    if (s_obd_data.speed_valid) {
        snprintf(buf, sizeof(buf), "%u km/h", s_obd_data.speed_kmh);
    }
    set_row(s_label_obd_speed_val, s_obd_data.speed_valid, buf);

    if (s_obd_data.rpm_valid) {
        snprintf(buf, sizeof(buf), "%u rpm", s_obd_data.rpm);
    }
    set_row(s_label_obd_rpm_val, s_obd_data.rpm_valid, buf);

    if (s_obd_data.engine_temp_valid) {
        snprintf(buf, sizeof(buf), "%.0f C", s_obd_data.engine_temp_c);
    }
    set_row(s_label_obd_temp_val, s_obd_data.engine_temp_valid, buf);

    if (s_obd_data.voltage_valid) {
        snprintf(buf, sizeof(buf), "%.2f V", s_obd_data.voltage_v);
    }
    set_row(s_label_obd_voltage_val, s_obd_data.voltage_valid, buf);

    if (s_obd_data.fuel_rate_valid) {
        snprintf(buf, sizeof(buf), "%.1f L/h", s_obd_data.fuel_rate_lph);
    }
    set_row(s_label_obd_fuel_val, s_obd_data.fuel_rate_valid, buf);

    if (s_obd_data.engine_load_valid) {
        snprintf(buf, sizeof(buf), "%u %%", s_obd_data.engine_load_pct);
    }
    set_row(s_label_obd_engine_load_val, s_obd_data.engine_load_valid, buf);

    if (s_obd_data.map_valid) {
        snprintf(buf, sizeof(buf), "%u kPa", s_obd_data.map_kpa);
    }
    set_row(s_label_obd_map_val, s_obd_data.map_valid, buf);

    if (s_obd_data.fuel_level_valid) {
        snprintf(buf, sizeof(buf), "%u %%", s_obd_data.fuel_level_pct);
    }
    set_row(s_label_obd_fuel_level_val, s_obd_data.fuel_level_valid, buf);

    lvgl_port_unlock();
}

// Atualiza os 2 rotulos extras (inclinacao da placa de reles + nivel do
// reservatorio) a partir de uma mensagem MsgType::StatusReport. Chamado de
// dentro de onMessage(), fora do lock do LVGL (toma o lock aqui) - mesma
// convencao de update_obd_info_screen().
static void update_relay_telemetry_display()
{
    if (!lvgl_port_lock(50)) {
        return;
    }

    char buf[32];

    if (s_relay_telemetry.tilt_valid) {
        snprintf(buf, sizeof(buf), "Roll %.1f / Pitch %.1f", s_relay_telemetry.roll_deg, s_relay_telemetry.pitch_deg);
        lv_label_set_text(s_label_relay_tilt_val, buf);
        lv_obj_set_style_text_color(s_label_relay_tilt_val, lv_color_white(), 0);

        update_tilt_display(s_relay_telemetry.roll_deg, s_relay_telemetry.pitch_deg);
    } else {
        lv_label_set_text(s_label_relay_tilt_val, "sem sensor");
        lv_obj_set_style_text_color(s_label_relay_tilt_val, lv_palette_main(LV_PALETTE_GREY), 0);

        s_tilt_raw_valid = false;
        lv_label_set_text(s_label_tilt, "Inclinacao: sensor nao detectado");
        lv_label_set_text(s_label_roll_val, "sem sensor");
        lv_label_set_text(s_label_pitch_val, "sem sensor");
        lv_obj_set_style_text_color(s_label_roll_val, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_obj_set_style_text_color(s_label_pitch_val, lv_palette_main(LV_PALETTE_GREY), 0);
    }

    if (s_relay_telemetry.water_level_valid) {
        if (s_relay_telemetry.water_level_low) {
            lv_label_set_text(s_label_water_level_val, "BAIXO");
            lv_obj_set_style_text_color(s_label_water_level_val, lv_palette_main(LV_PALETTE_RED), 0);
        } else {
            lv_label_set_text(s_label_water_level_val, "OK");
            lv_obj_set_style_text_color(s_label_water_level_val, lv_palette_main(LV_PALETTE_GREEN), 0);
        }
    } else {
        lv_label_set_text(s_label_water_level_val, "sem dados");
        lv_obj_set_style_text_color(s_label_water_level_val, lv_palette_main(LV_PALETTE_GREY), 0);
    }

    lvgl_port_unlock();
}

// Atualiza a 4a tela (DTCs) a partir de s_dtc_info/s_dtc_request_in_flight -
// ver dtc_read_button_event_cb()/dtc_clear_button_event_cb() (que pedem a
// operacao) e onMessage() (que recebe o resultado via MsgType::DtcData).
static void update_dtc_screen()
{
    if (!lvgl_port_lock(50)) {
        return;
    }

    if (s_dtc_request_in_flight) {
        lv_label_set_text(s_label_dtc_status, "Lendo...");
        lv_obj_set_style_text_color(s_label_dtc_status, lv_palette_main(LV_PALETTE_GREY), 0);
    } else if (!s_dtc_info.has_data) {
        lv_label_set_text(s_label_dtc_status, "Sem leitura ainda");
        lv_obj_set_style_text_color(s_label_dtc_status, lv_palette_main(LV_PALETTE_GREY), 0);
    } else if (s_dtc_info.mil_on) {
        lv_label_set_text(s_label_dtc_status, "MIL: LIGADA");
        lv_obj_set_style_text_color(s_label_dtc_status, lv_palette_main(LV_PALETTE_RED), 0);
    } else {
        lv_label_set_text(s_label_dtc_status, "MIL: desligada");
        lv_obj_set_style_text_color(s_label_dtc_status, lv_palette_main(LV_PALETTE_GREEN), 0);
    }

    char summary[64];
    if (s_dtc_info.has_data) {
        uint8_t stored = 0, pending = 0;
        for (uint8_t i = 0; i < s_dtc_info.count; i++) {
            if (s_dtc_info.entries[i].pending) {
                pending++;
            } else {
                stored++;
            }
        }
        snprintf(summary, sizeof(summary), "%u armazenado(s), %u pendente(s)", stored, pending);
    } else {
        snprintf(summary, sizeof(summary), "Toque em \"Ler DTCs\" para consultar o veiculo");
    }
    lv_label_set_text(s_label_dtc_summary, summary);

    lv_obj_clean(s_list_dtc);
    if (!s_dtc_info.has_data) {
        lv_list_add_text(s_list_dtc, "-");
    } else if (s_dtc_info.count == 0) {
        lv_obj_t *row = lv_list_add_text(s_list_dtc, "Nenhum codigo de falha encontrado.");
        lv_obj_set_style_text_color(row, lv_palette_main(LV_PALETTE_GREEN), 0);
    } else {
        for (uint8_t i = 0; i < s_dtc_info.count; i++) {
            char code_str[6];
            formatDtcCode(s_dtc_info.entries[i].code, code_str);
            char row_text[32];
            snprintf(row_text, sizeof(row_text), "%s (%s)", code_str,
                     s_dtc_info.entries[i].pending ? "pendente" : "armazenado");
            lv_obj_t *row = lv_list_add_text(s_list_dtc, row_text);
            lv_obj_set_style_text_color(row, s_dtc_info.entries[i].pending
                                                  ? lv_palette_main(LV_PALETTE_ORANGE)
                                                  : lv_palette_main(LV_PALETTE_RED), 0);
        }
    }

    lvgl_port_unlock();
}



// ======================= ESP-NOW ============================================

static void onMessage(const uint8_t mac[6], const Message &msg, int8_t rssi)
{
    (void)rssi;
    switch (msg.type) {
    case MsgType::Hello:
        // A relay module announced itself: pair with it (first one wins).
        if (!s_paired) {
            memcpy(s_relay_mac, mac, 6);
            addPeer(s_relay_mac);
            s_paired = true;
            Serial.println("[LCD] Pareado com o modulo de reles.");
            update_connection_label();

            // O ACK precisa ir por broadcast (nao criptografado), nao por
            // unicast para s_relay_mac: embora addPeer() acima ja deixe ESTE
            // lado pronto para criptografar, o modulo de reles ainda NAO
            // registrou este controlador como peer criptografado (ele so faz
            // isso ao processar o HelloAck, logo abaixo). Um ACK unicast
            // criptografado chegaria antes dessa chave existir do lado do
            // rele e seria descartado silenciosamente pela pilha ESP-NOW -
            // o LCD imprimia "Pareado" mas o rele nunca recebia comandos.
            Message ack;
            ack.type = MsgType::HelloAck;
            sendBroadcast(ack);
        } else if (macEquals(mac, s_relay_mac)) {
            // O rele JA pareado mandou Hello de novo: ele reiniciou (reset/
            // queda de energia) e perdeu o pareamento do lado dele. Sem
            // reenviar o HelloAck ele nunca volta a parear - e, pior, este
            // Hello renovava s_last_relay_msg_ms abaixo, entao a tela nunca
            // dava timeout e as duas pontas ficavam esperando uma a outra.
            Message ack;
            ack.type = MsgType::HelloAck;
            sendBroadcast(ack);
        } else {
            // Hello de OUTRO modulo enquanto ja pareado: ignora e NAO renova
            // o relogio do link com o rele atual.
            break;
        }
        s_last_relay_msg_ms = millis();
        break;

    case MsgType::StatusReport:
        if (!s_paired || !macEquals(mac, s_relay_mac)) {
            break;
        }
        s_ui_mask = msg.payload.status.relay_mask;
        s_relay_telemetry.tilt_valid = msg.payload.status.tilt_valid != 0;
        s_relay_telemetry.roll_deg = msg.payload.status.roll_deg;
        s_relay_telemetry.pitch_deg = msg.payload.status.pitch_deg;
        s_relay_telemetry.water_level_valid = msg.payload.status.water_level_valid != 0;
        s_relay_telemetry.water_level_low = msg.payload.status.water_level_low != 0;
        s_last_relay_msg_ms = millis();
        update_relay_buttons();
        update_relay_telemetry_display();
        break;

    case MsgType::ObdData:
        if (!s_paired || !macEquals(mac, s_relay_mac)) {
            break;
        }
        s_obd_data.connected = msg.payload.obd_data.connected != 0;
        s_obd_data.engine_temp_valid = msg.payload.obd_data.engine_temp_valid != 0;
        s_obd_data.engine_temp_c = msg.payload.obd_data.engine_temp_c;
        s_obd_data.rpm_valid = msg.payload.obd_data.rpm_valid != 0;
        s_obd_data.rpm = msg.payload.obd_data.rpm;
        s_obd_data.speed_valid = msg.payload.obd_data.speed_valid != 0;
        s_obd_data.speed_kmh = msg.payload.obd_data.speed_kmh;
        s_obd_data.voltage_valid = msg.payload.obd_data.voltage_valid != 0;
        s_obd_data.voltage_v = msg.payload.obd_data.voltage_v;
        s_obd_data.fuel_rate_valid = msg.payload.obd_data.fuel_rate_valid != 0;
        s_obd_data.fuel_rate_lph = msg.payload.obd_data.fuel_rate_lph;
        s_obd_data.engine_load_valid = msg.payload.obd_data.engine_load_valid != 0;
        s_obd_data.engine_load_pct = msg.payload.obd_data.engine_load_pct;
        s_obd_data.map_valid = msg.payload.obd_data.map_valid != 0;
        s_obd_data.map_kpa = msg.payload.obd_data.map_kpa;
        s_obd_data.fuel_level_valid = msg.payload.obd_data.fuel_level_valid != 0;
        s_obd_data.fuel_level_pct = msg.payload.obd_data.fuel_level_pct;
        s_last_relay_msg_ms = millis();
        update_obd_gauges();
        update_obd_info_screen();
        break;

    case MsgType::DtcData:
        if (!s_paired || !macEquals(mac, s_relay_mac)) {
            break;
        }
        s_dtc_request_in_flight = false;
        s_dtc_info.has_data = true;
        s_dtc_info.mil_on = msg.payload.dtc_data.mil_on != 0;
        s_dtc_info.count = msg.payload.dtc_data.dtc_count;
        if (s_dtc_info.count > kMaxDtcEntries) {
            s_dtc_info.count = kMaxDtcEntries; // defensivo - nunca deveria vir maior
        }
        for (uint8_t i = 0; i < s_dtc_info.count; i++) {
            s_dtc_info.entries[i].code = msg.payload.dtc_data.dtcs[i].code;
            s_dtc_info.entries[i].pending = msg.payload.dtc_data.dtcs[i].pending != 0;
        }
        s_last_relay_msg_ms = millis();
        update_dtc_screen();
        break;

    default:
        break;
    }
}

// ======================= Arduino entry points ===============================

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== ESPRemotecontrol - Tela / Controlador ===");

    static esp_lcd_panel_handle_t panel_handle = NULL;
    panel_handle = waveshare_esp32_s3_rgb_lcd_init();
    wavesahre_rgb_lcd_bl_on();

    // touch_gt911_init() also initializes the shared I2C bus and the CH422G
    // IO expander, which the QMI8658 IMU driver below relies on. Its I2C
    // writes/delays can add up to several hundred ms, which is why the RGB
    // panel's vsync callback is registered only below, *after*
    // lvgl_port_init() - see waveshare_rgb_lcd_register_vsync_callback().
    esp_lcd_touch_handle_t tp_handle = touch_gt911_init();

    esp_err_t ret = lvgl_port_init(panel_handle, tp_handle);
    if (ret != ESP_OK) {
        Serial.printf("ERRO ao inicializar LVGL: %d\n", ret);
    }
    waveshare_rgb_lcd_register_vsync_callback(panel_handle);

    if (lvgl_port_lock(-1)) {
        create_main_screen();
        create_obd_screen();
        create_obd_info_screen();
        create_dtc_screen();
        lv_scr_load(s_screen_main);
        lvgl_port_unlock();
    }

    Preferences tilt_prefs;
    if (tilt_prefs.begin(kTiltPrefsNamespace, true)) {
        s_tilt_roll_offset = tilt_prefs.getFloat("roll", 0.0f);
        s_tilt_pitch_offset = tilt_prefs.getFloat("pitch", 0.0f);
        tilt_prefs.end();
    }

    // Indicador de bateria: depende de um INA219 EXTERNO ligado na LiPo via
    // o terminal I2C da placa (ver lib/battery_ina219/battery.h - o CS8501
    // onboard nao expoe a tensao da bateria para nenhum GPIO). Se o sensor
    // ainda nao foi ligado, update_battery_display() mostra "sem sensor".
    s_battery_sensor_present = battery::begin();
    if (s_battery_sensor_present) {
        Serial.println("Bateria: INA219 detectado.");
    } else {
        Serial.println("AVISO: INA219 nao detectado (sensor de bateria nao ligado ainda).");
    }
    update_battery_display();

    if (!begin(PAIRING_KEY, WIFI_CHANNEL, onMessage)) {
        Serial.println("ERRO: falha ao iniciar ESP-NOW!");
    }

    uint8_t mac[6];
    getOwnMac(mac);
    char macstr[18];
    macToStr(mac, macstr);
    Serial.printf("MAC deste modulo: %s (canal %u)\n", macstr, WIFI_CHANNEL);

    update_connection_label();
}

void loop()
{
    uint32_t now = millis();

    if (s_paired && (now - s_last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS)) {
        s_last_heartbeat_ms = now;
        Message hb;
        hb.type = MsgType::Heartbeat;
        hb.payload.heartbeat.uptime_s = now / 1000;
        send(s_relay_mac, hb);
    }

    // Diferenca COM SINAL: s_last_relay_msg_ms e escrito por onMessage()
    // (task do ESP-NOW) com um millis() possivelmente mais novo que o "now"
    // deste loop; em uint32_t isso estouraria e derrubaria o link a toa.
    if (s_paired && (int32_t)(now - s_last_relay_msg_ms) > (int32_t)LINK_TIMEOUT_MS) {
        Serial.println("[LCD] Link com o modulo de reles perdido.");
        s_paired = false;
        update_connection_label();
        // Sem o rele, tambem nao ha mais dados OBD-II frescos (ele que fala
        // com o ELM327) - evita mostrar valores desatualizados como validos.
        s_obd_data.connected = false;
        s_obd_data.engine_temp_valid = false;
        s_obd_data.rpm_valid = false;
        s_obd_data.speed_valid = false;
        s_obd_data.voltage_valid = false;
        s_obd_data.fuel_rate_valid = false;
        s_obd_data.engine_load_valid = false;
        s_obd_data.map_valid = false;
        s_obd_data.fuel_level_valid = false;
        update_obd_gauges();
        update_obd_info_screen();
        // Se um pedido de leitura/limpeza de DTC ficou esperando resposta e
        // o link caiu, nao fica "Lendo..." para sempre.
        if (s_dtc_request_in_flight) {
            s_dtc_request_in_flight = false;
            update_dtc_screen();
        }
    }

    if (s_battery_sensor_present && (now - s_last_battery_ms >= BATTERY_UPDATE_INTERVAL_MS)) {
        s_last_battery_ms = now;
        update_battery_display();
    }

    delay(10);
}
