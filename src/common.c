#include "common.h"
#include <stdio.h>

EstadoNave estado_nave;
pthread_mutex_t mutex_estado = PTHREAD_MUTEX_INITIALIZER;

const char *obter_nome_estado(EstadoMissao estado) {
  switch (estado) {
  case PREPARACAO:
    return "PREPARACAO";
  case LANCAMENTO:
    return "LANCAMENTO";
  case ORBITA_TERRESTRE:
    return "ORBITA_TERRESTRE";
  case TRANSITO_LUNAR:
    return "TRANSITO_LUNAR";
  case ORBITA_LUNAR:
    return "ORBITA_LUNAR";
  case ALUNISSAGEM:
    return "ALUNISSAGEM";
  case SUPERFICIE_LUNAR:
    return "SUPERFICIE_LUNAR";
  case RETORNO_TERRA:
    return "RETORNO_TERRA";
  case REENTRADA:
    return "REENTRADA";
  case AMERISSAGEM:
    return "AMERISSAGEM";
  case FINALIZACAO:
    return "FINALIZACAO";
  case EMERGENCIA:
    return "EMERGENCIA";
  default:
    return "DESCONHECIDO";
  }
}

void acionar_emergencia(const char *motivo) {
  (void)motivo;
  pthread_mutex_lock(&mutex_estado);
  if (estado_nave.estado_missao != EMERGENCIA) {
    estado_nave.estado_missao = EMERGENCIA;
    estado_nave.emergencia = true;
  }
  // O motivo será tratado pela UI separadamente, mas em um dmesg / log
  // poderíamos salvar aqui
  pthread_mutex_unlock(&mutex_estado);
}

void avancar_estado_missao() {
  pthread_mutex_lock(&mutex_estado);
  switch (estado_nave.estado_missao) {
  case PREPARACAO:
    estado_nave.estado_missao = LANCAMENTO;
    break;
  case LANCAMENTO:
    estado_nave.estado_missao = ORBITA_TERRESTRE;
    break;
  case ORBITA_TERRESTRE:
    estado_nave.estado_missao = TRANSITO_LUNAR;
    break;
  case TRANSITO_LUNAR:
    estado_nave.estado_missao = ORBITA_LUNAR;
    break;
  case ORBITA_LUNAR:
    estado_nave.estado_missao = ALUNISSAGEM;
    break;
  case ALUNISSAGEM:
    estado_nave.estado_missao = SUPERFICIE_LUNAR;
    break;
  case SUPERFICIE_LUNAR:
    estado_nave.estado_missao = RETORNO_TERRA;
    break;
  case RETORNO_TERRA:
    estado_nave.estado_missao = REENTRADA;
    break;
  case REENTRADA:
    estado_nave.estado_missao = AMERISSAGEM;
    break;
  case AMERISSAGEM:
    estado_nave.estado_missao = FINALIZACAO;
    break;
  case FINALIZACAO:
    atomic_store(&estado_nave.sistema_ativo, false);
    break;
  case EMERGENCIA:
    break;
  }
  pthread_mutex_unlock(&mutex_estado);
}
