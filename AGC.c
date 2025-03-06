/*
    Simulador Apollo 11
    Criado por: Andrei Costa
    Data: 06/03/2025

    Descrição: Este programa simula o sistema de controle de voo da Apollo 11.
    O programa é composto por 3 módulos: o módulo de controle de voo, o módulo
    de controle de propulsão e o módulo de controle de energia.
    O módulo de controle de voo é responsável por controlar a nave, o módulo de controle de propulsão é
    responsável por controlar os motores e o módulo de controle de energia é responsável
    por controlar a energia da nave. O programa é composto por 3 threads, cada thread
    representa um módulo da nave. O módulo de controle de voo é a thread principal,
    ele é responsável por criar as threads dos outros módulos e por sincronizar as
    threads. O módulo de controle de propulsão é responsável por controlar a velocidade
    da nave e o módulo de controle de energia é responsável por controlar a energia
    da nave.
*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////// Bibliotecas ////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>   // Biblioteca padrão do C
#include <stdlib.h>  // Biblioteca padrão do C
#include <string.h>  // Biblioteca para manipulação de strings
#include <pthread.h> // Biblioteca para threads
#include <unistd.h>  // Biblioteca para funções do sistema operacional
#include <math.h>    // Biblioteca para funções matemáticas
#include <time.h>    // Biblioteca para gerar números aleatórios

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////// Definições ////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Definição de constantes
#define MAX 100
#define TRUE 1
#define FALSE 0
#define PI 3.14159265358979323846

// Intervalo de atualização em microsegundos
#define INTERVALO_VOO 100000       // 100ms
#define INTERVALO_PROPULSAO 100000 // 50ms
#define INTERVALO_ENERGIA 100000   // 200ms

// Estado da Missão
typedef enum
{
    PREPARACAO,
    LANCAMENTO,
    ORBITA_TERRESTRE,
    TRANSITO_LUNAR,
    ORBITA_LUNAR,
    ALUNISSAGEM,
    SUPERFICIE_LUNAR,
    RETORNO_TERRA,
    REENTRADA,
    AMERRISSAGEM,
    FINALIZACAO,
    EMERGENCIA
} EstadoMissao;

// Estrutura para vetores 3D
typedef struct
{
    double x; // Coordenada x
    double y; // Coordenada y
    double z; // Coordenada z
} Vetor3D;

// Estrutura de armazenamento do estado da nave
typedef struct
{
    // Posição e Movimento
    Vetor3D posicao;    // Posição da nave
    Vetor3D velocidade; // Velocidade da nave
    Vetor3D aceleracao; // Aceleração da nave
    Vetor3D orientacao; // Orientação da nave

    // Estado da Missão
    EstadoMissao estado_missao; // Estado da missão
    double tempo_missao;        // Tempo da missão (Segundos, desde o lançamento)

    // Propulsão
    double combustivel_principal; // Combustível do motor principal (KG)
    double combustivel_rcs;       // Combustível do sistema de controle de reação (KG)
    double empuxo_principal;      // Empuxo do motor principal (Newton)
    double empuxo_rcs;            // Empuxo do sistema de controle de reação (Newton)

    // Energia
    double energia_principal; // Energia do motor principal (Watts-Hora)
    double energia_reserva;   // Energia de reserva (Watts-Hora)
    double consumo_energia;   // Consumo de energia (Watts)

    // Ambiente
    double temperatura_interna; // Temperatura interna da nave (Celsius)
    double pressao_interna;     // Pressão interna da nave (kPa)
    double radiacao;            // Radiação (mSv/hora)

    // Comunicação
    int comunicacao_ativa; // Comunicação ativa (1 ou 0)
    double forca_sinal;    // Força do sinal de comunicação (dB)

    // Flags de Controle
    int sistema_ativo;       // Sistema ativo (1 ou 0)
    int emergencia;          // Emergência (1 ou 0)
    int simulacao_acelerada; // Simulação acelerada (1 ou 0)
} EstadoNave;

// Variáveis Compartilhadas
EstadoNave estado_nave;       // Estado da nave
pthread_mutex_t mutex_estado; // Mutex para sincronização

// Variaveis de Threads
pthread_t thread_voo;       // Thread de controle de voo
pthread_t thread_propulsao; // Thread de controle de propulsão
pthread_t thread_energia;   // Thread de controle de energia
pthread_t thread_interface; // Thread de interface

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////// Funções ////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Função para inicializar o estado da nave
void inicializar_estado()
{
    pthread_mutex_lock(&mutex_estado); // Bloqueia o acesso a variável compartilhada

    // Inicializa a posição e o movimento
    estado_nave.posicao = (Vetor3D){0.0, 0.0, 0.0};    // Posição da nave - Coordenadas x, y e z
    estado_nave.velocidade = (Vetor3D){0.0, 0.0, 0.0}; // Velocidade da nave - Coordenadas x, y e z
    estado_nave.aceleracao = (Vetor3D){0.0, 0.0, 0.0}; // Aceleração da nave - Coordenadas x, y e z
    estado_nave.orientacao = (Vetor3D){0.0, 0.0, 0.0}; // Orientação da nave - Coordenadas x, y e z

    // Estado inicial da missão
    estado_nave.estado_missao = PREPARACAO; // Estado da missão - Preparação
    estado_nave.tempo_missao = 0.0;         // Tempo da missão - 0 segundos

    // Propulsão
    estado_nave.combustivel_principal = 1924000.0; // Combustível do motor principal - ~1.924.000 kg (Saturn V)
    estado_nave.combustivel_rcs = 500.0;           // Combustível do sistema de controle de reação - 1000 kg
    estado_nave.empuxo_principal = 0.0;            // Empuxo do motor principal - 0, pois o motor está desligado
    estado_nave.empuxo_rcs = 0.0;                  // Empuxo do sistema de controle de reação - 0, pois o motor está desligado

    // Energia
    estado_nave.energia_principal = 10000.0; // Energia do motor principal - 10.000 Watts-Hora
    estado_nave.energia_reserva = 5000.0;    // Energia de reserva - 5000 Watts-Hora
    estado_nave.consumo_energia = 100.0;     // Consumo de energia - 100 Watts

    // Ambiente
    estado_nave.temperatura_interna = 22.0; // Temperatura interna da nave - 22 graus Celsius
    estado_nave.pressao_interna = 101.3;    // Pressão interna da nave - 101.3 kPa
    estado_nave.radiacao = 0.1;             // Radiação - 0.1 mSv/hora

    // Comunicação
    estado_nave.comunicacao_ativa = TRUE; // Comunicação ativa - 1 (ativa)
    estado_nave.forca_sinal = 100.0;      // Força do sinal de comunicação - 100 dB

    // Flags de Controle
    estado_nave.sistema_ativo = TRUE;    // Sistema ativo - 1 (ativo)
    estado_nave.emergencia = FALSE;      // Emergência - 0 (sem emergência)
    estado_nave.simulacao_acelerada = 0; // Simulação acelerada - 0 (desligada)

    pthread_mutex_unlock(&mutex_estado); // Libera o acesso a variável compartilhada
}

// Função de controle de voo
// Responsável por controlar a nave
void *controle_voo(void *arg)
{
    printf("Iniciando modulo de controle de voo...\n");
    return NULL;
}

// Função de controle de propulsão
// Responsável por controlar os motores
void *controle_propulsao(void *arg)
{
    printf("Iniciando modulo de controle de propulsao...\n");
    return NULL;
}

// Função de controle de energia
// Responsável por controlar a energia
void *controle_energia(void *arg)
{
    printf("Iniciando modulo de controle de energia...\n");
    return NULL;
}

// Função Main
int main()
{
    printf("Iniciando simulador Apollo 11...\n");

    // Criando as threads
    pthread_create(&thread_voo, NULL, controle_voo, NULL);
    pthread_create(&thread_propulsao, NULL, controle_propulsao, NULL);
    pthread_create(&thread_energia, NULL, controle_energia, NULL);

    // Aguardando as threads
    pthread_join(thread_voo, NULL);
    pthread_join(thread_propulsao, NULL);
    pthread_join(thread_energia, NULL);

    printf("Simulador Apollo 11 finalizado.\n");

    return 0;
}
