#include "systems_control.h"
#include <stdlib.h>
#include <unistd.h>

#define INTERVALO_PROPULSAO 50000 // 50ms
#define INTERVALO_ENERGIA 200000  // 200ms

// Variáveis para Controlador PID de Descida
static double integral_erro_velocidade = 0.0;
static double erro_velocidade_anterior = 0.0;
static const double kp = 30000.0; // Ganho proporcional
static const double ki = 5000.0;  // Ganho integral
static const double kd = 15000.0; // Ganho derivativo

void *controle_propulsao(void *arg) {
  (void)arg;
  double delta_tempo = INTERVALO_PROPULSAO / 1000000.0;

  while (atomic_load(&estado_nave.sistema_ativo)) {
    pthread_mutex_lock(&mutex_estado);

    int fator_aceleracao = atomic_load(&estado_nave.simulacao_acelerada);
    double dt_real = delta_tempo * fator_aceleracao;

    switch (estado_nave.estado_missao) {
    case LANCAMENTO:
      // Durante o lançamento, usamos empuxo máximo (35 MN)
      estado_nave.empuxo_principal = 35000000.0;
      estado_nave.combustivel_principal -= 15000.0 * dt_real;
      break;

    case ALUNISSAGEM: {
      // Controlador PID para pouso suave (-2.0 m/s na vertical / eixo Y)
      double setpoint = -2.0;

      // Considerando velocidade.y como velocidade de descida
      double velocidade_atual = estado_nave.velocidade.y;
      double erro = setpoint - velocidade_atual;

      integral_erro_velocidade += erro * dt_real;
      double derivada_erro = (erro - erro_velocidade_anterior) / dt_real;

      // Calculando força baseada na malha PID
      double empuxo_pid =
          (kp * erro) + (ki * integral_erro_velocidade) + (kd * derivada_erro);

      // Limites operacionais do motor de descida (Módulo Lunar: max 45kN)
      if (empuxo_pid < 0)
        empuxo_pid = 0;
      if (empuxo_pid > 45000.0)
        empuxo_pid = 45000.0;

      estado_nave.empuxo_principal = empuxo_pid;
      estado_nave.combustivel_principal -=
          (empuxo_pid / 35000000.0 * 15000.0) *
          dt_real; // Escala baseada no empuxo real

      erro_velocidade_anterior = erro;
      break;
    }
    default:
      if (rand() % 100 < 5 && estado_nave.estado_missao != SUPERFICIE_LUNAR &&
          estado_nave.estado_missao != FINALIZACAO) {
        estado_nave.empuxo_rcs = 500.0;
        estado_nave.combustivel_rcs -= 0.1 * dt_real;
      } else {
        estado_nave.empuxo_rcs = 0.0;
      }
      estado_nave.empuxo_principal = 0.0;
      break;
    }

    // Segurança do array de combustíveis
    if (estado_nave.combustivel_principal < 0)
      estado_nave.combustivel_principal = 0;
    if (estado_nave.combustivel_rcs < 0)
      estado_nave.combustivel_rcs = 0;

    pthread_mutex_unlock(&mutex_estado);
    usleep(INTERVALO_PROPULSAO / fator_aceleracao);
  }
  return NULL;
}

void *controle_energia(void *arg) {
  (void)arg;
  double delta_tempo = INTERVALO_ENERGIA / 1000000.0;

  // Semente local / rand em thread separada para não causar deadlocks na base
  // rand() se usada em paralelos perigosos
  unsigned int seed = 12345;

  while (atomic_load(&estado_nave.sistema_ativo)) {
    pthread_mutex_lock(&mutex_estado);

    int fator_aceleracao = atomic_load(&estado_nave.simulacao_acelerada);
    double dt_real = delta_tempo * fator_aceleracao;

    double consumo_base = 80.0;
    double consumo_propulsao = estado_nave.empuxo_principal > 0 ? 50.0 : 0.0;
    double consumo_rcs = estado_nave.empuxo_rcs > 0 ? 20.0 : 0.0;
    double consumo_computadores = 30.0;
    double consumo_suporte_vida = 40.0;

    estado_nave.consumo_energia = consumo_base + consumo_propulsao +
                                  consumo_rcs + consumo_computadores +
                                  consumo_suporte_vida;

    double energia_consumida = estado_nave.consumo_energia * dt_real / 3600.0;
    estado_nave.energia_principal -= energia_consumida;

    if (estado_nave.energia_principal <= 0) {
      estado_nave.energia_reserva += estado_nave.energia_principal;
      estado_nave.energia_principal = 0;

      if (estado_nave.energia_reserva <= 0) {
        estado_nave.energia_reserva = 0;
        if (estado_nave.estado_missao != EMERGENCIA) {
          pthread_mutex_unlock(&mutex_estado);
          acionar_emergencia("Energia esgotada");
          pthread_mutex_lock(&mutex_estado);
        }
      }
    }

    // Simula flutuações de temperatura
    double variacao_temp = ((rand_r(&seed) % 100) - 50) / 500.0;
    estado_nave.temperatura_interna += variacao_temp;

    // Malha termal simulação simples
    if (estado_nave.temperatura_interna < 20.0) {
      estado_nave.temperatura_interna += 0.2 * dt_real;
      estado_nave.consumo_energia += 10.0;
    } else if (estado_nave.temperatura_interna > 24.0) {
      estado_nave.temperatura_interna -= 0.2 * dt_real;
      estado_nave.consumo_energia += 10.0;
    }

    if (estado_nave.estado_missao == TRANSITO_LUNAR ||
        estado_nave.estado_missao == ORBITA_LUNAR ||
        estado_nave.estado_missao == SUPERFICIE_LUNAR) {
      estado_nave.radiacao = 1.0 + ((rand_r(&seed) % 100) / 100.0);
    } else {
      estado_nave.radiacao = 0.1 + ((rand_r(&seed) % 50) / 500.0);
    }

    pthread_mutex_unlock(&mutex_estado);
    usleep(INTERVALO_ENERGIA / fator_aceleracao);
  }
  return NULL;
}
