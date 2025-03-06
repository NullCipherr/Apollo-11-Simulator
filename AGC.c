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

// Função para obter o nome do estado atual da missão
const char *obter_nome_estado()
{
    switch (estado_nave.estado_missao)
    {
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

// Função para avançar o estado da missão
void avancar_estado_missao()
{
    pthread_mutex_lock(&mutex_estado); // Bloqueia o acesso a variável compartilhada

    switch (estado_nave.estado_missao)
    {
    case PREPARACAO:
        estado_nave.estado_missao = LANCAMENTO;
        printf("Mudança de estado: PREPARACAO -> LANCAMENTO\n");
        break;
    case LANCAMENTO:
        estado_nave.estado_missao = ORBITA_TERRESTRE;
        printf("Mudança de estado: LANCAMENTO -> ORBITA_TERRESTRE\n");
        break;
    case ORBITA_TERRESTRE:
        estado_nave.estado_missao = TRANSITO_LUNAR;
        printf("Mudança de estado: ORBITA_TERRESTRE -> TRANSITO_LUNAR\n");
        break;
    case TRANSITO_LUNAR:
        estado_nave.estado_missao = ORBITA_LUNAR;
        printf("Mudança de estado: TRANSITO_LUNAR -> ORBITA_LUNAR\n");
        break;
    case ORBITA_LUNAR:
        estado_nave.estado_missao = ALUNISSAGEM;
        printf("Mudança de estado: ORBITA_LUNAR -> ALUNISSAGEM\n");
        break;
    case ALUNISSAGEM:
        estado_nave.estado_missao = SUPERFICIE_LUNAR;
        printf("Mudança de estado: ALUNISSAGEM -> SUPERFICIE_LUNAR\n");
        break;
    case SUPERFICIE_LUNAR:
        estado_nave.estado_missao = RETORNO_TERRA;
        printf("Mudança de estado: SUPERFICIE_LUNAR -> RETORNO_TERRA\n");
        break;
    case RETORNO_TERRA:
        estado_nave.estado_missao = REENTRADA;
        printf("Mudança de estado: RETORNO_TERRA -> REENTRADA\n");
        break;
    case REENTRADA:
        estado_nave.estado_missao = AMERRISSAGEM;
        printf("Mudança de estado: REENTRADA -> AMERRISSAGEM\n");
        break;
    case AMERRISSAGEM:
        estado_nave.estado_missao = FINALIZACAO;
        printf("Mudança de estado: AMERRISSAGEM -> FINALIZACAO\n");
        break;
    case FINALIZACAO:
        estado_nave.estado_missao = EMERGENCIA;
        printf("Mudança de estado: FINALIZACAO -> EMERGENCIA\n");
        break;
    case EMERGENCIA:
        // Permance em estado de emergência
        break;
    }

    pthread_mutex_unlock(&mutex_estado); // Libera o acesso a variável compartilhada
}

// Função para acionar a emergência
void acionar_emergencia()
{
    pthread_mutex_lock(&mutex_estado); // Bloqueia o acesso a variável compartilhada

    if (estado_nave.estado_missao != EMERGENCIA)
    {
        estado_nave.estado_missao = EMERGENCIA;
        estado_nave.emergencia = TRUE;
        printf("Emergência acionada!\n");
    }

    pthread_mutex_unlock(&mutex_estado); // Libera o acesso a variável compartilhada
}

// Função para atualizar a fisica da nave
void atualizar_fisica(double delta_tempo)
{
    pthread_mutex_lock(&mutex_estado); // Bloqueia o acesso a variável compartilhada

    // Atualiza a posição com base na velocidade atual
    estado_nave.posicao.x += estado_nave.velocidade.x * delta_tempo; // Atualiza a posição x
    estado_nave.posicao.y += estado_nave.velocidade.y * delta_tempo; // Atualiza a posição y
    estado_nave.posicao.z += estado_nave.velocidade.z * delta_tempo; // Atualiza a posição z

    // Atualiza a velocidade com base na aceleração atual
    estado_nave.velocidade.x += estado_nave.aceleracao.x * delta_tempo; // Atualiza a velocidade x
    estado_nave.velocidade.y += estado_nave.aceleracao.y * delta_tempo; // Atualiza a velocidade y
    estado_nave.velocidade.z += estado_nave.aceleracao.z * delta_tempo; // Atualiza a velocidade z

    // Simulação simples de gravidade
    double distancia_terra = sqrt(pow(estado_nave.posicao.x, 2) +
                                  pow(estado_nave.posicao.y, 2) +
                                  pow(estado_nave.posicao.z, 2));

    // Aceleração gravitacional da Terra simples
    if (distancia_terra > 0)
    {
        double G = 6.67430e-11;                                                     // Constante gravitacional (m^3 kg^-1 s^-2)
        double M_terra = 5.972e24;                                                  // Massa da Terra (kg)
        double aceleracao_grav = G * M_terra / (distancia_terra * distancia_terra); // Aceleração gravitacional

        // Direção da aceleração gravitacional (Para o centro da terra)
        double fator = aceleracao_grav / distancia_terra;
        estado_nave.aceleracao.x = -estado_nave.posicao.x * fator; // Aceleração x
        estado_nave.aceleracao.y = -estado_nave.posicao.y * fator; // Aceleração y
        estado_nave.aceleracao.z = -estado_nave.posicao.z * fator; // Aceleração z
    }

    // Atualiza o tempo da missão
    estado_nave.tempo_missao += delta_tempo;

    pthread_mutex_unlock(&mutex_estado); // Libera o acesso a variável compartilhada
}

// Função de controle de voo - Responsável por controlar a nave
void *controle_voo(void *arg)
{
    printf("Iniciando modulo de controle de voo...\n");

    double delta_tempo = INTERVALO_VOO / 1000000.0; // Converte o intervalo de tempo para segundos
    double tempo_para_proximo_estado = 30.0;        // Tempo para o próximo estado da missão (Segundos)

    // Inicializa o estado da nave, enquanto o sistema estiver ativo
    while (estado_nave.sistema_ativo)
    {
        // Atualiza a fisica da nave
        atualizar_fisica(delta_tempo * estado_nave.simulacao_acelerada);

        // Controle de voo baseada no estado atual
        pthread_mutex_lock(&mutex_estado); // Bloqueia o acesso a variável compartilhada

        // Verificação de Ssegurança
        if (estado_nave.temperatura_interna > 50 && estado_nave.estado_missao != EMERGENCIA)
        {
            acionar_emergencia("Temperatura interna crítica!"); // Aciona a emergência
        }

        if (estado_nave.combustivel_principal <= 0 && (estado_nave.estado_missao == LANCAMENTO || estado_nave.estado_missao == TRANSITO_LUNAR) && estado_nave.estado_missao != EMERGENCIA)
        {
            acionar_emergencia("Combusitível do motor principal esgotado!"); // Aciona a emergência
        }

        // Lógica para mudança de estado
        tempo_para_proximo_estado -= delta_tempo * estado_nave.simulacao_acelerada;
        if (tempo_para_proximo_estado <= 0 && estado_nave.estado_missao != EMERGENCIA && estado_nave.estado_missao != FINALIZACAO)
        {
            pthread_mutex_unlock(&mutex_estado); // Libera o acesso a variável compartilhada
            avancar_estado_missao();             // Avança o estado da missão
            tempo_para_proximo_estado = 30.0;    // Reseta o tempo para o próximo estado
        }
        else
        {
            pthread_mutex_unlock(&mutex_estado); // Libera o acesso a variável compartilhada
        }

        // Pausa entre as atualizações
        usleep(INTERVALO_VOO / estado_nave.simulacao_acelerada); // Pausa a execução por um intervalo de tempo
    }

    printf("Finalizando modulo de controle de voo...\n");
    return NULL;
}

// Função de controle de propulsão - Responsável por controlar os motores
void *controle_propulsao(void *arg)
{
    printf("Iniciando modulo de controle de propulsao...\n");

    double delta_tempo = INTERVALO_PROPULSAO / 1000000.0; // Converte o intervalo de tempo para segundos

    while (estado_nave.sistema_ativo)
    {
        pthread_mutex_lock(&mutex_estado); // Bloqueia o acesso a variável compartilhada

        // Gerencia propulsão baseado no estado da missão
        switch (estado_nave.estado_missao)
        {
        case LANCAMENTO:
            // Durante o lançamneto é utilizado propulsão média
            estado_nave.empuxo_principal = 35000000.0; // 35 MN (aproximado para Saturn V) - 35.000.000 Newton

            // Consumo de combustível
            double consumo = 15000.0 * delta_tempo * estado_nave.simulacao_acelerada; // 15.000 kg/s
            estado_nave.combustivel_principal -= consumo;

            if (estado_nave.combustivel_principal <= 0)
            {
                estado_nave.combustivel_principal = 0;
            }

            // Aceleração resultante simples
            double massa_total = 3000000.0 - (1924000.0 - estado_nave.combustivel_principal); // 3.000.000 kg (aproximado para Saturn V)
            double aceleracao = estado_nave.empuxo_principal / massa_total;                   // Aceleração resultante - a = F / m

            // Direçãol da aceleração simples
            estado_nave.aceleracao.y = aceleracao - 9.81; // Aceleração y - Aceleração gravitacional da Terra
            break;

        case ORBITA_TERRESTRE:
        case TRANSITO_LUNAR:
        case ORBITA_LUNAR:
        case RETORNO_TERRA:
            // Manobras ocasionais
            if (rand() % 100 < 5) // 5 % de chance
            {
                estado_nave.empuxo_rcs = 500.0;                                                     // 500 Newton
                estado_nave.combustivel_rcs -= 0.1 * delta_tempo * estado_nave.simulacao_acelerada; // 0.1 kg/s
            }
            else
            {
                estado_nave.empuxo_rcs = 0.0; // Desliga o RCS
            }
            break;

        case ALUNISSAGEM:
            // Desaceleração controlada para alunissagem
            estado_nave.empuxo_principal = 45000.0;                                                    // 45 kN (aproximado para Lunar Module Descent Engine) - 45.000 Newton
            estado_nave.combustivel_principal -= 50.0 * delta_tempo * estado_nave.simulacao_acelerada; // 50 kg/s
            break;

        default:
            estado_nave.empuxo_principal = 0.0; // Desliga o motor principal
            estado_nave.empuxo_rcs = 0.0;       // Desliga o RCS
        }

        // Garante que o combustível não seja negativo
        if (estado_nave.combustivel_principal < 0)
        {
            estado_nave.combustivel_principal = 0; // Combustível do motor principal
        }
        if (estado_nave.combustivel_rcs < 0)
        {
            estado_nave.combustivel_rcs = 0; // Combustível do RCS
        }

        pthread_mutex_unlock(&mutex_estado); // Libera o acesso a variável compartilhada

        // Pausa entre as atualizações
        usleep(INTERVALO_PROPULSAO / estado_nave.simulacao_acelerada); // Pausa a execução por um intervalo de tempo
    }

    printf("Finalizando modulo de controle de propulsao...\n");
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
