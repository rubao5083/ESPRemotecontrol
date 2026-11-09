/*****************************************************************************
 * | File        :   main.cpp  (relay_pinscan)
 * | Function    :   Firmware TEMPORARIO de diagnostico - varre candidatos de
 * |                 GPIO (headers expostos perto do modulo ESP32-WROOM-32E)
 * |                 procurando quais realmente acionam os reles. A pinagem
 * |                 generica da comunidade ESPHome (GPIO32/33/25/26) nao
 * |                 produziu nenhum clique nesta unidade - ver
 * |                 docs/HISTORICO-DEPURACAO.md.
 * |
 * | Uso: grave este ambiente (-e relay_pinscan), abra o monitor serial e
 * | escute os reles. Cada pino testado fica 2s em HIGH e 1s em LOW antes de
 * | passar para o proximo. Anote qual(is) pino(s) produziram clique.
 ******************************************************************************/

#include <Arduino.h>

// Candidatos lidos nos headers expostos perto do modulo ESP32-WROOM-32E
// (G15 testado primeiro, por suspeita de rastreamento visual de trilha).
static const uint8_t kCandidates[] = {
    13, 14
};
static const size_t kCandidateCount = sizeof(kCandidates) / sizeof(kCandidates[0]);

// Modo manual: cada byte recebido pela serial avanca um passo (liga o
// candidato atual, ou desliga e passa para o proximo). Isso permite
// sincronizar com precisao qual pino esta ativo no momento do teste.
static size_t s_index = 0;
static bool s_pin_is_on = false;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== Varredura MANUAL de pinos - modulo de reles ===");
    Serial.printf("Mande qualquer byte para avancar. %u candidatos.\n", (unsigned)kCandidateCount);
    Serial.printf(">>> Pronto. Proximo: GPIO%u (candidato %u/%u)\n",
                  kCandidates[s_index], (unsigned)(s_index + 1), (unsigned)kCandidateCount);
}

void loop()
{
    if (Serial.available() <= 0) {
        return;
    }
    while (Serial.available() > 0) {
        Serial.read();
    }

    uint8_t pin = kCandidates[s_index];
    if (!s_pin_is_on) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
        s_pin_is_on = true;
        Serial.printf(">>> GPIO%u LIGADO (candidato %u/%u)\n",
                      pin, (unsigned)(s_index + 1), (unsigned)kCandidateCount);
    } else {
        digitalWrite(pin, LOW);
        pinMode(pin, INPUT);
        s_pin_is_on = false;
        Serial.printf(">>> GPIO%u desligado\n", pin);

        s_index++;
        if (s_index >= kCandidateCount) {
            s_index = 0;
            Serial.println("=== Ciclo completo. Reiniciando do candidato 1. ===");
        }
        Serial.printf(">>> Proximo: GPIO%u (candidato %u/%u)\n",
                      kCandidates[s_index], (unsigned)(s_index + 1), (unsigned)kCandidateCount);
    }
}
