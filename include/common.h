#ifndef COMMON_H
#define COMMON_H

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>

// Estados da missão
typedef enum {
  PREPARACAO,
  LANCAMENTO,
  ORBITA_TERRESTRE,
  TRANSITO_LUNAR,
  ORBITA_LUNAR,
  ALUNISSAGEM,
  SUPERFICIE_LUNAR,
  RETORNO_TERRA,
  REENTRADA,
  AMERISSAGEM,
  FINALIZACAO,
  EMERGENCIA
} EstadoMissao;

// Estrutura para vetores 3D
typedef struct {
  double x;
  double y;
  double z;
} Vetor3D;

// Estrutura para armazenar o estado da nave
typedef struct {
  // Posição e movimento
  Vetor3D posicao;
  Vetor3D velocidade;
  Vetor3D aceleracao;
  Vetor3D orientacao;

  // Estado da missão
  EstadoMissao estado_missao;
  double tempo_missao; // em segundos desde o lançamento

  // Propulsão e Massa
  double massa_vazia;           // em kg
  double combustivel_principal; // em kg
  double combustivel_rcs;       // em kg
  double empuxo_principal;      // em Newtons
  double empuxo_rcs;            // em Newtons

  // Energia
  double energia_principal; // em Watts-hora
  double energia_reserva;   // em Watts-hora
  double consumo_energia;   // em Watts

  // Ambiente
  double temperatura_interna; // em Celsius
  double pressao_interna;     // em kPa
  double radiacao;            // em mSv/h

  // Comunicação
  bool comunicacao_ativa;
  double forca_sinal; // em dB

  // Flags de controle (Thread-safe)
  atomic_bool sistema_ativo;
  bool emergencia;
  atomic_int simulacao_acelerada; // fator de aceleração
} EstadoNave;

extern EstadoNave estado_nave;
extern pthread_mutex_t mutex_estado;

// Funções globais básicas
const char *obter_nome_estado(EstadoMissao estado);
void acionar_emergencia(const char *motivo);
void avancar_estado_missao();

#endif // COMMON_H
