#include "common.h"
#include "physics_engine.h"
#include "systems_control.h"
#include "telemetry_ui.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

void inicializar_estado() {
  pthread_mutex_lock(&mutex_estado);

  // Posiciona a nave na superfície da Terra (Raio equatorial ~6.378 km)
  estado_nave.posicao = (Vetor3D){0.0, 6378137.0, 0.0};
  estado_nave.velocidade = (Vetor3D){0.0, 0.0, 0.0};
  estado_nave.aceleracao = (Vetor3D){0.0, 0.0, 0.0};
  estado_nave.orientacao = (Vetor3D){0.0, 1.0, 0.0};

  estado_nave.estado_missao = PREPARACAO;
  estado_nave.tempo_missao = 0.0;

  estado_nave.massa_vazia = 100000.0; // aprox 100 toneladas vazio (Sem comb.)
  estado_nave.combustivel_principal = 1924000.0; // Foguete cheio
  estado_nave.combustivel_rcs = 500.0;
  estado_nave.empuxo_principal = 0.0;
  estado_nave.empuxo_rcs = 0.0;

  estado_nave.energia_principal = 10000.0;
  estado_nave.energia_reserva = 5000.0;
  estado_nave.consumo_energia = 100.0;

  estado_nave.temperatura_interna = 22.0;
  estado_nave.pressao_interna = 101.3;
  estado_nave.radiacao = 0.1;

  estado_nave.comunicacao_ativa = true;
  estado_nave.forca_sinal = 100.0;

  atomic_store(&estado_nave.sistema_ativo, true);
  estado_nave.emergencia = false;
  atomic_store(&estado_nave.simulacao_acelerada, 1);

  pthread_mutex_unlock(&mutex_estado);
}

int main() {
  // Inicialização da seed pseudoaleatória principal
  srand(time(NULL));

  // Configurando estado inicial antes de disparar threads
  inicializar_estado();

  // Arrays de threads
  pthread_t thread_voo, thread_propulsao, thread_energia, thread_interface,
      thread_logger;

  // Criando theads funcionais (Pthreads)
  pthread_create(&thread_voo, NULL, controle_voo, NULL);
  pthread_create(&thread_propulsao, NULL, controle_propulsao, NULL);
  pthread_create(&thread_energia, NULL, controle_energia, NULL);
  pthread_create(&thread_logger, NULL, telemetry_logger, NULL);

  // Interface (roda na thread principal ou em uma separada que gerencia o main
  // block)
  pthread_create(&thread_interface, NULL, interface_usuario, NULL);

  // O programa principal esperará a UI finalizar (Usuário apertou Q ou S)
  pthread_join(thread_interface, NULL);

  // As outras threads finalizarão automaticamente em seu próximo laço de poll
  // porque a variável atomic atomic_load(&estado_nave.sistema_ativo) se tornou
  // 'false' no "Sair" da UI
  pthread_join(thread_voo, NULL);
  pthread_join(thread_propulsao, NULL);
  pthread_join(thread_energia, NULL);
  pthread_join(thread_logger, NULL);

  return 0;
}
