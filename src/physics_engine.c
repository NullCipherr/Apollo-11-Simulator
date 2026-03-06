#include "physics_engine.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

// Constantes Físicas
#define G 6.67430e-11         // Constante gravitacional em m³/(kg·s²)
#define M_TERRA 5.972e24      // Massa da Terra em kg
#define M_LUA 7.342e22        // Massa da Lua em kg
#define POS_LUA_X 384400000.0 // Distância Terra-Lua ~ 384,400 km
#define POS_LUA_Y 0.0
#define POS_LUA_Z 0.0

// Estrutura auxiliar para o RK4 representativa da derivada (d(Pos)/dt e
// d(Vel)/dt)
typedef struct {
  Vetor3D vel;  // Derivada da posição (velocidade)
  Vetor3D acel; // Derivada da velocidade (aceleração)
} Derivada;

// Calcula o vetor de aceleração gravitacional devido à Terra e à Lua
static Vetor3D calcular_aceleracao_gravitacional(Vetor3D pos) {
  Vetor3D acel_total = {0.0, 0.0, 0.0};

  // --- Influência da Terra ---
  double dist_terra = sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
  // Salvaguarda: Se estiver abaixo da superfície, usamos o raio da Terra para o
  // cálculo de gravidade para evitar força infinita, mas a lógica de colisão
  // cuidará do movimento.
  double r_calc_terra = (dist_terra < 6371000.0) ? 6371000.0 : dist_terra;

  double mag_terra =
      -(G * M_TERRA) / (r_calc_terra * r_calc_terra * r_calc_terra);
  acel_total.x += mag_terra * pos.x;
  acel_total.y += mag_terra * pos.y;
  acel_total.z += mag_terra * pos.z;

  // --- Influência da Lua ---
  double r_lua_x = pos.x - POS_LUA_X;
  double r_lua_y = pos.y - POS_LUA_Y;
  double r_lua_z = pos.z - POS_LUA_Z;
  double dist_lua =
      sqrt(r_lua_x * r_lua_x + r_lua_y * r_lua_y + r_lua_z * r_lua_z);

  // Salvaguarda Lua: Raio da Lua ~1.737 km
  double r_calc_lua = (dist_lua < 1737000.0) ? 1737000.0 : dist_lua;

  double mag_lua = -(G * M_LUA) / (r_calc_lua * r_calc_lua * r_calc_lua);
  acel_total.x += mag_lua * r_lua_x;
  acel_total.y += mag_lua * r_lua_y;
  acel_total.z += mag_lua * r_lua_z;

  return acel_total;
}

// Avalia a derivada para o RK4 em um instante dt
static Derivada avaliar(Vetor3D pos_inicial, Vetor3D vel_inicial,
                        double temporal_dt, Derivada d, double empuxo_principal,
                        double massa_total, Vetor3D dir_empuxo) {

  // Nova posição extrapolada
  Vetor3D pos;
  pos.x = pos_inicial.x + d.vel.x * temporal_dt;
  pos.y = pos_inicial.y + d.vel.y * temporal_dt;
  pos.z = pos_inicial.z + d.vel.z * temporal_dt;

  // Nova velocidade extrapolada
  Vetor3D vel;
  vel.x = vel_inicial.x + d.acel.x * temporal_dt;
  vel.y = vel_inicial.y + d.acel.y * temporal_dt;
  vel.z = vel_inicial.z + d.acel.z * temporal_dt;

  Derivada saida;
  saida.vel = vel; // A derivada da posição é a nova velocidade

  // Calcula as forças gravitacionais na posição provisória
  Vetor3D gravidade = calcular_aceleracao_gravitacional(pos);

  // Aceleração causada pelo empuxo dos motores (a = F / m)
  double acel_empuxo =
      (massa_total > 0) ? (empuxo_principal / massa_total) : 0.0;

  saida.acel.x = gravidade.x + dir_empuxo.x * acel_empuxo;
  saida.acel.y = gravidade.y + dir_empuxo.y * acel_empuxo;
  saida.acel.z = gravidade.z + dir_empuxo.z * acel_empuxo;

  return saida;
}

// Realiza um passo de integração RK4
static void atualizar_fisica_rk4(double dt) {
  pthread_mutex_lock(&mutex_estado);

  // Massa total variável (Dinâmica de Massa)
  double massa_total = estado_nave.massa_vazia +
                       estado_nave.combustivel_principal +
                       estado_nave.combustivel_rcs;

  // Simplificação da direção do empuxo (em um sistema completo viria da
  // orientação da nave) Usando eixo Y para subida vertical / alunissagem
  Vetor3D dir_empuxo = {0.0, 1.0, 0.0};

  Derivada inicial = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

  // K1
  Derivada d1 =
      avaliar(estado_nave.posicao, estado_nave.velocidade, 0.0, inicial,
              estado_nave.empuxo_principal, massa_total, dir_empuxo);
  // K2
  Derivada d2 =
      avaliar(estado_nave.posicao, estado_nave.velocidade, dt * 0.5, d1,
              estado_nave.empuxo_principal, massa_total, dir_empuxo);
  // K3
  Derivada d3 =
      avaliar(estado_nave.posicao, estado_nave.velocidade, dt * 0.5, d2,
              estado_nave.empuxo_principal, massa_total, dir_empuxo);
  // K4
  Derivada d4 = avaliar(estado_nave.posicao, estado_nave.velocidade, dt, d3,
                        estado_nave.empuxo_principal, massa_total, dir_empuxo);

  // Combinação dos K's para a velocidade e posição ( RK4 )
  double dx_dt =
      1.0 / 6.0 * (d1.vel.x + 2.0 * (d2.vel.x + d3.vel.x) + d4.vel.x);
  double dy_dt =
      1.0 / 6.0 * (d1.vel.y + 2.0 * (d2.vel.y + d3.vel.y) + d4.vel.y);
  double dz_dt =
      1.0 / 6.0 * (d1.vel.z + 2.0 * (d2.vel.z + d3.vel.z) + d4.vel.z);

  double dvx_dt =
      1.0 / 6.0 * (d1.acel.x + 2.0 * (d2.acel.x + d3.acel.x) + d4.acel.x);
  double dvy_dt =
      1.0 / 6.0 * (d1.acel.y + 2.0 * (d2.acel.y + d3.acel.y) + d4.acel.y);
  double dvz_dt =
      1.0 / 6.0 * (d1.acel.z + 2.0 * (d2.acel.z + d3.acel.z) + d4.acel.z);

  // Atualiza estado global
  estado_nave.posicao.x += dx_dt * dt;
  estado_nave.posicao.y += dy_dt * dt;
  estado_nave.posicao.z += dz_dt * dt;

  estado_nave.velocidade.x += dvx_dt * dt;
  estado_nave.velocidade.y += dvy_dt * dt;
  estado_nave.velocidade.z += dvz_dt * dt;

  // --- Lógica de Colisão com a Terra (Solo) ---
  double dist_centro = sqrt(estado_nave.posicao.x * estado_nave.posicao.x +
                            estado_nave.posicao.y * estado_nave.posicao.y +
                            estado_nave.posicao.z * estado_nave.posicao.z);

  if (dist_centro < 6378137.0) { // Raio da Terra
    // Reposiciona na superfície
    double fator = 6378137.0 / dist_centro;
    estado_nave.posicao.x *= fator;
    estado_nave.posicao.y *= fator;
    estado_nave.posicao.z *= fator;

    // Se estiver caindo, anula a velocidade vertical (impacto)
    // Simplificação: se o produto escalar Pos . Vel < 0, está indo para o
    // centro
    double dot = estado_nave.posicao.x * estado_nave.velocidade.x +
                 estado_nave.posicao.y * estado_nave.velocidade.y +
                 estado_nave.posicao.z * estado_nave.velocidade.z;

    if (dot < 0) {
      estado_nave.velocidade.x = 0;
      estado_nave.velocidade.y = 0;
      estado_nave.velocidade.z = 0;
    }
  }

  estado_nave.aceleracao.x = dvx_dt;
  estado_nave.aceleracao.y = dvy_dt;
  estado_nave.aceleracao.z = dvz_dt;

  estado_nave.tempo_missao += dt;

  pthread_mutex_unlock(&mutex_estado);
}

// Thread principal de controle da física (Alta Resolução)
void *controle_voo(void *arg) {
  (void)arg;
  struct timespec tempo_anterior, tempo_atual;
  clock_gettime(CLOCK_MONOTONIC, &tempo_anterior);

  double tempo_para_proximo_estado = 30.0;

  while (atomic_load(&estado_nave.sistema_ativo)) {
    // High-Resolution Timer usando CLOCK_MONOTONIC
    clock_gettime(CLOCK_MONOTONIC, &tempo_atual);
    double delta_tempo_real =
        (tempo_atual.tv_sec - tempo_anterior.tv_sec) +
        (tempo_atual.tv_nsec - tempo_anterior.tv_nsec) / 1e9;
    tempo_anterior = tempo_atual;

    // Limita o dt real para evitar saltos enormes em caso de lag na thread
    if (delta_tempo_real > 0.1)
      delta_tempo_real = 0.1;

    int fator_aceleracao = atomic_load(&estado_nave.simulacao_acelerada);
    double dt_simulacao = delta_tempo_real * fator_aceleracao;

    // Atualiza a física com o RK4
    atualizar_fisica_rk4(dt_simulacao);

    pthread_mutex_lock(&mutex_estado);

    // Lógica Fictícia de Avanço de Estados
    if (estado_nave.estado_missao != EMERGENCIA &&
        estado_nave.estado_missao != FINALIZACAO) {
      tempo_para_proximo_estado -= dt_simulacao;
      if (tempo_para_proximo_estado <= 0) {
        pthread_mutex_unlock(&mutex_estado);
        avancar_estado_missao();
        pthread_mutex_lock(&mutex_estado);
        tempo_para_proximo_estado = 30.0;
      }
    }

    pthread_mutex_unlock(&mutex_estado);

    // Usleep curto apenas para não consumir 100% da CPU, mas permitindo o
    // relógio de precisão ditar a taxa de delta_tempo (cerca de ~1ms de espera)
    usleep(1000);
  }

  return NULL;
}
