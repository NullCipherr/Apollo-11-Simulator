/*
    Simulador Apollo 11
    Criado por: Andrei Costa
    Data: 06/03/2025

    Este programa simula o sistema de controle de voo da Apollo 11.
    O programa é composto por 3 módulos: o módulo de controle de voo, o módulo
    de controle de propulsão e o módulo de controle de energia.

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

// Definição de constantes
#define MAX 100
#define TRUE 1
#define FALSE 0
#define PI 3.14159265358979323846

// Intervalos de atualização em microsegundos
#define INTERVALO_VOO 100000      // 100ms
#define INTERVALO_PROPULSAO 50000 // 50ms
#define INTERVALO_ENERGIA 200000  // 200ms

// Estados da missão
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
    AMERISSAGEM,
    FINALIZACAO,
    EMERGENCIA
} EstadoMissao;

// Estrutura para vetores 3D
typedef struct
{
    double x;
    double y;
    double z;
} Vetor3D;

// Estrutura para armazenar o estado da nave
typedef struct
{
    // Posição e movimento
    Vetor3D posicao;
    Vetor3D velocidade;
    Vetor3D aceleracao;
    Vetor3D orientacao;

    // Estado da missão
    EstadoMissao estado_missao;
    double tempo_missao; // em segundos desde o lançamento

    // Propulsão
    double combustivel_principal; // em kg
    double combustivel_rcs;       // em kg (Reaction Control System)
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
    int comunicacao_ativa;
    double forca_sinal; // em dB

    // Flags de controle
    int sistema_ativo;
    int emergencia;
    int simulacao_acelerada; // fator de aceleração da simulação
} EstadoNave;

// Variáveis compartilhadas
EstadoNave estado_nave;
pthread_mutex_t mutex_estado;

// Threads
pthread_t thread_voo;
pthread_t thread_propulsao;
pthread_t thread_energia;
pthread_t thread_interface;

// Função para inicializar o estado da nave
void inicializar_estado()
{
    pthread_mutex_lock(&mutex_estado);

    // Inicializa posição e movimento
    estado_nave.posicao = (Vetor3D){0.0, 0.0, 0.0};
    estado_nave.velocidade = (Vetor3D){0.0, 0.0, 0.0};
    estado_nave.aceleracao = (Vetor3D){0.0, 0.0, 0.0};
    estado_nave.orientacao = (Vetor3D){0.0, 0.0, 0.0};

    // Estado inicial da missão
    estado_nave.estado_missao = PREPARACAO;
    estado_nave.tempo_missao = 0.0;

    // Propulsão
    estado_nave.combustivel_principal = 1924000.0; // ~1.924.000 kg de combustível no Saturn V
    estado_nave.combustivel_rcs = 500.0;
    estado_nave.empuxo_principal = 0.0;
    estado_nave.empuxo_rcs = 0.0;

    // Energia
    estado_nave.energia_principal = 10000.0;
    estado_nave.energia_reserva = 5000.0;
    estado_nave.consumo_energia = 100.0;

    // Ambiente
    estado_nave.temperatura_interna = 22.0;
    estado_nave.pressao_interna = 101.3;
    estado_nave.radiacao = 0.1;

    // Comunicação
    estado_nave.comunicacao_ativa = TRUE;
    estado_nave.forca_sinal = 100.0;

    // Flags de controle
    estado_nave.sistema_ativo = TRUE;
    estado_nave.emergencia = FALSE;
    estado_nave.simulacao_acelerada = 1;

    pthread_mutex_unlock(&mutex_estado);
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
    pthread_mutex_lock(&mutex_estado);

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
        estado_nave.estado_missao = AMERISSAGEM;
        printf("Mudança de estado: REENTRADA -> AMERISSAGEM\n");
        break;
    case AMERISSAGEM:
        estado_nave.estado_missao = FINALIZACAO;
        printf("Mudança de estado: AMERISSAGEM -> FINALIZACAO\n");
        break;
    case FINALIZACAO:
        estado_nave.sistema_ativo = FALSE;
        printf("Missão finalizada.\n");
        break;
    case EMERGENCIA:
        // Permanece em emergência até resolução manual
        break;
    }

    pthread_mutex_unlock(&mutex_estado);
}

// Função para acionar emergência
void acionar_emergencia(const char *motivo)
{
    pthread_mutex_lock(&mutex_estado);

    if (estado_nave.estado_missao != EMERGENCIA)
    {
        estado_nave.estado_missao = EMERGENCIA;
        estado_nave.emergencia = TRUE;
        printf("\n*** EMERGENCIA: %s ***\n", motivo);
    }

    pthread_mutex_unlock(&mutex_estado);
}

// Função para atualizar a física da nave
void atualizar_fisica(double delta_tempo)
{
    pthread_mutex_lock(&mutex_estado);

    // Atualizar posição com base na velocidade atual
    estado_nave.posicao.x += estado_nave.velocidade.x * delta_tempo;
    estado_nave.posicao.y += estado_nave.velocidade.y * delta_tempo;
    estado_nave.posicao.z += estado_nave.velocidade.z * delta_tempo;

    // Atualizar velocidade com base na aceleração atual
    estado_nave.velocidade.x += estado_nave.aceleracao.x * delta_tempo;
    estado_nave.velocidade.y += estado_nave.aceleracao.y * delta_tempo;
    estado_nave.velocidade.z += estado_nave.aceleracao.z * delta_tempo;

    // Simulação simples de gravidade (simplificada)
    double distancia_terra = sqrt(pow(estado_nave.posicao.x, 2) +
                                  pow(estado_nave.posicao.y, 2) +
                                  pow(estado_nave.posicao.z, 2));

    // Aceleração gravitacional da Terra (simplificada)
    if (distancia_terra > 0)
    {
        double G = 6.67430e-11;    // Constante gravitacional (m³ kg⁻¹ s⁻²)
        double M_terra = 5.972e24; // Massa da Terra (kg)
        double aceleracao_grav = G * M_terra / (distancia_terra * distancia_terra);

        // Direção da aceleração (para o centro da Terra)
        double fator = aceleracao_grav / distancia_terra;
        estado_nave.aceleracao.x = -estado_nave.posicao.x * fator;
        estado_nave.aceleracao.y = -estado_nave.posicao.y * fator;
        estado_nave.aceleracao.z = -estado_nave.posicao.z * fator;
    }

    // Atualiza o tempo da missão
    estado_nave.tempo_missao += delta_tempo;

    pthread_mutex_unlock(&mutex_estado);
}

// Função de controle de voo
// Responsável por controlar a navegação e o estado geral da nave
void *controle_voo(void *arg)
{
    printf("Iniciando módulo de controle de voo...\n");

    double delta_tempo = INTERVALO_VOO / 1000000.0; // Converter de microssegundos para segundos
    double tempo_para_proximo_estado = 30.0;        // Tempo até o próximo estado (em segundos)

    while (estado_nave.sistema_ativo)
    {
        // Atualizar a física da nave
        atualizar_fisica(delta_tempo * estado_nave.simulacao_acelerada);

        // Lógica de controle de voo baseada no estado atual
        pthread_mutex_lock(&mutex_estado);

        // Verificações de segurança
        if (estado_nave.temperatura_interna > 50.0 && estado_nave.estado_missao != EMERGENCIA)
        {
            acionar_emergencia("Temperatura interna crítica");
        }

        if (estado_nave.combustivel_principal <= 0 &&
            (estado_nave.estado_missao == LANCAMENTO || estado_nave.estado_missao == TRANSITO_LUNAR) &&
            estado_nave.estado_missao != EMERGENCIA)
        {
            acionar_emergencia("Combustível esgotado em fase crítica");
        }

        // Lógica para mudança de estado (simplificada para demonstração)
        tempo_para_proximo_estado -= delta_tempo * estado_nave.simulacao_acelerada;
        if (tempo_para_proximo_estado <= 0 &&
            estado_nave.estado_missao != EMERGENCIA &&
            estado_nave.estado_missao != FINALIZACAO)
        {
            pthread_mutex_unlock(&mutex_estado);
            avancar_estado_missao();
            tempo_para_proximo_estado = 30.0; // Reset para o próximo estado
        }
        else
        {
            pthread_mutex_unlock(&mutex_estado);
        }

        // Pausa entre atualizações
        usleep(INTERVALO_VOO / estado_nave.simulacao_acelerada);
    }

    printf("Módulo de controle de voo finalizado.\n");
    return NULL;
}

// Função de controle de propulsão
// Responsável por controlar os motores e o uso de combustível
void *controle_propulsao(void *arg)
{
    printf("Iniciando módulo de controle de propulsão...\n");

    double delta_tempo = INTERVALO_PROPULSAO / 1000000.0; // Converter de microssegundos para segundos

    while (estado_nave.sistema_ativo)
    {
        pthread_mutex_lock(&mutex_estado);

        // Gerencia propulsão baseado no estado da missão
        switch (estado_nave.estado_missao)
        {
        case LANCAMENTO:
            // Durante o lançamento, usamos propulsão máxima
            estado_nave.empuxo_principal = 35000000.0; // 35 MN (aproximado para Saturn V)

            // Consumo de combustível
            double consumo = 15000.0 * delta_tempo * estado_nave.simulacao_acelerada; // kg/s
            estado_nave.combustivel_principal -= consumo;

            if (estado_nave.combustivel_principal < 0)
                estado_nave.combustivel_principal = 0;

            // Aceleração resultante (simplificada)
            double massa_total = 3000000.0 - (1924000.0 - estado_nave.combustivel_principal);
            double aceleracao = estado_nave.empuxo_principal / massa_total;

            // Direção da aceleração (simplifcada, apenas para cima)
            estado_nave.aceleracao.y = aceleracao - 9.81; // Subtrair gravidade da Terra
            break;

        case ORBITA_TERRESTRE:
        case TRANSITO_LUNAR:
        case ORBITA_LUNAR:
        case RETORNO_TERRA:
            // Manobras ocasionais
            if (rand() % 100 < 5)
            { // 5% de chance de fazer uma manobra
                estado_nave.empuxo_rcs = 500.0;
                estado_nave.combustivel_rcs -= 0.1 * delta_tempo * estado_nave.simulacao_acelerada;
            }
            else
            {
                estado_nave.empuxo_rcs = 0.0;
            }
            break;

        case ALUNISSAGEM:
            // Desaceleração controlada
            estado_nave.empuxo_principal = 45000.0;
            estado_nave.combustivel_principal -= 50.0 * delta_tempo * estado_nave.simulacao_acelerada;
            break;

        default:
            estado_nave.empuxo_principal = 0.0;
            estado_nave.empuxo_rcs = 0.0;
        }

        // Garantir que o combustível não seja negativo
        if (estado_nave.combustivel_principal < 0)
            estado_nave.combustivel_principal = 0;
        if (estado_nave.combustivel_rcs < 0)
            estado_nave.combustivel_rcs = 0;

        pthread_mutex_unlock(&mutex_estado);

        // Pausa entre atualizações
        usleep(INTERVALO_PROPULSAO / estado_nave.simulacao_acelerada);
    }

    printf("Módulo de controle de propulsão finalizado.\n");
    return NULL;
}

// Função de controle de energia
// Responsável por gerenciar energia e sistemas de suporte à vida
void *controle_energia(void *arg)
{
    printf("Iniciando módulo de controle de energia...\n");

    double delta_tempo = INTERVALO_ENERGIA / 1000000.0; // Converter de microssegundos para segundos

    while (estado_nave.sistema_ativo)
    {
        pthread_mutex_lock(&mutex_estado);

        // Cálculo do consumo de energia com base nos sistemas ativos
        double consumo_base = 80.0; // Consumo base de energia (W)
        double consumo_propulsao = estado_nave.empuxo_principal > 0 ? 50.0 : 0.0;
        double consumo_rcs = estado_nave.empuxo_rcs > 0 ? 20.0 : 0.0;
        double consumo_computadores = 30.0;
        double consumo_suporte_vida = 40.0;

        // Consumo total
        estado_nave.consumo_energia = consumo_base + consumo_propulsao +
                                      consumo_rcs + consumo_computadores +
                                      consumo_suporte_vida;

        // Aplicar consumo de energia
        double energia_consumida = estado_nave.consumo_energia * delta_tempo * estado_nave.simulacao_acelerada / 3600.0; // Converter para Wh
        estado_nave.energia_principal -= energia_consumida;

        // Se a energia principal acabar, usa a reserva
        if (estado_nave.energia_principal <= 0)
        {
            estado_nave.energia_reserva += estado_nave.energia_principal; // Adicionar o "negativo" da principal
            estado_nave.energia_principal = 0;

            // Se a reserva também acabar, emergência
            if (estado_nave.energia_reserva <= 0)
            {
                estado_nave.energia_reserva = 0;
                if (estado_nave.estado_missao != EMERGENCIA)
                {
                    acionar_emergencia("Energia esgotada");
                }
            }
        }

        // Simular variação de temperatura
        double variacao_temp = ((rand() % 100) - 50) / 500.0; // Variação aleatória de -0.1 a 0.1 graus
        estado_nave.temperatura_interna += variacao_temp;

        // Corrigir temperatura (sistema de controle térmico)
        if (estado_nave.temperatura_interna < 20.0)
        {
            estado_nave.temperatura_interna += 0.2 * delta_tempo * estado_nave.simulacao_acelerada;
            estado_nave.consumo_energia += 10.0; // Aquecedores
        }
        else if (estado_nave.temperatura_interna > 24.0)
        {
            estado_nave.temperatura_interna -= 0.2 * delta_tempo * estado_nave.simulacao_acelerada;
            estado_nave.consumo_energia += 10.0; // Resfriamento
        }

        // Simulação de radiação
        if (estado_nave.estado_missao == TRANSITO_LUNAR ||
            estado_nave.estado_missao == ORBITA_LUNAR ||
            estado_nave.estado_missao == SUPERFICIE_LUNAR)
        {
            estado_nave.radiacao = 1.0 + ((rand() % 100) / 100.0); // 1.0 a 2.0 mSv/h
        }
        else
        {
            estado_nave.radiacao = 0.1 + ((rand() % 50) / 500.0); // 0.1 a 0.2 mSv/h
        }

        pthread_mutex_unlock(&mutex_estado);

        // Pausa entre atualizações
        usleep(INTERVALO_ENERGIA / estado_nave.simulacao_acelerada);
    }

    printf("Módulo de controle de energia finalizado.\n");
    return NULL;
}

// Função para interface de usuário
void *interface_usuario(void *arg)
{
    printf("Iniciando interface de usuário...\n");

    while (estado_nave.sistema_ativo)
    {
        // Limpar a tela (funciona em sistemas Unix/Linux)
        system("clear");

        pthread_mutex_lock(&mutex_estado);

        // Exibe informações sobre o estado atual da nave
        printf("=======================================================\n");
        printf("           SIMULADOR APOLLO 11 - STATUS                \n");
        printf("=======================================================\n");
        printf("Estado da missão: %s\n", obter_nome_estado());
        printf("Tempo de missão: %.2f horas\n", estado_nave.tempo_missao / 3600.0);
        printf("-------------------------------------------------------\n");
        printf("POSIÇÃO E MOVIMENTO:\n");
        printf("  Posição (km): X=%.2f, Y=%.2f, Z=%.2f\n",
               estado_nave.posicao.x / 1000.0,
               estado_nave.posicao.y / 1000.0,
               estado_nave.posicao.z / 1000.0);
        printf("  Velocidade (m/s): X=%.2f, Y=%.2f, Z=%.2f\n",
               estado_nave.velocidade.x,
               estado_nave.velocidade.y,
               estado_nave.velocidade.z);
        printf("  Velocidade total: %.2f m/s (%.2f km/h)\n",
               sqrt(pow(estado_nave.velocidade.x, 2) +
                    pow(estado_nave.velocidade.y, 2) +
                    pow(estado_nave.velocidade.z, 2)),
               sqrt(pow(estado_nave.velocidade.x, 2) +
                    pow(estado_nave.velocidade.y, 2) +
                    pow(estado_nave.velocidade.z, 2)) *
                   3.6);
        printf("-------------------------------------------------------\n");
        printf("PROPULSÃO:\n");
        printf("  Combustível principal: %.2f kg (%.1f%%)\n",
               estado_nave.combustivel_principal,
               estado_nave.combustivel_principal / 1924000.0 * 100.0);
        printf("  Combustível RCS: %.2f kg (%.1f%%)\n",
               estado_nave.combustivel_rcs,
               estado_nave.combustivel_rcs / 500.0 * 100.0);
        printf("  Empuxo principal: %.2f kN\n", estado_nave.empuxo_principal / 1000.0);
        printf("  Empuxo RCS: %.2f N\n", estado_nave.empuxo_rcs);
        printf("-------------------------------------------------------\n");
        printf("ENERGIA:\n");
        printf("  Energia principal: %.2f Wh (%.1f%%)\n",
               estado_nave.energia_principal,
               estado_nave.energia_principal / 10000.0 * 100.0);
        printf("  Energia reserva: %.2f Wh (%.1f%%)\n",
               estado_nave.energia_reserva,
               estado_nave.energia_reserva / 5000.0 * 100.0);
        printf("  Consumo atual: %.2f W\n", estado_nave.consumo_energia);
        printf("-------------------------------------------------------\n");
        printf("AMBIENTE:\n");
        printf("  Temperatura interna: %.1f °C\n", estado_nave.temperatura_interna);
        printf("  Pressão interna: %.1f kPa\n", estado_nave.pressao_interna);
        printf("  Radiação: %.2f mSv/h\n", estado_nave.radiacao);
        printf("-------------------------------------------------------\n");
        printf("SIMULAÇÃO:\n");
        printf("  Velocidade: %dx\n", estado_nave.simulacao_acelerada);
        printf("=======================================================\n");

        // Se em emergência, mostrar alerta
        if (estado_nave.estado_missao == EMERGENCIA)
        {
            printf("\n*** SITUAÇÃO DE EMERGÊNCIA - SISTEMAS COMPROMETIDOS ***\n");
        }

        printf("\nComandos: [A]celerar simulação, [D]esacelerar, [P]róximo estado, [E]mergência, [S]air\n");

        pthread_mutex_unlock(&mutex_estado);

        // Verificar comandos (não bloqueante)
        char comando = 0;
        scanf("%c", &comando);

        if (comando != '\n' && comando != 0)
        {
            switch (comando)
            {
            case 'a':
            case 'A':
                pthread_mutex_lock(&mutex_estado);
                if (estado_nave.simulacao_acelerada < 100)
                    estado_nave.simulacao_acelerada *= 2;
                pthread_mutex_unlock(&mutex_estado);
                break;

            case 'd':
            case 'D':
                pthread_mutex_lock(&mutex_estado);
                if (estado_nave.simulacao_acelerada > 1)
                    estado_nave.simulacao_acelerada /= 2;
                pthread_mutex_unlock(&mutex_estado);
                break;

            case 'p':
            case 'P':
                avancar_estado_missao();
                break;

            case 'e':
            case 'E':
                acionar_emergencia("Comando manual");
                break;

            case 's':
            case 'S':
                pthread_mutex_lock(&mutex_estado);
                estado_nave.sistema_ativo = FALSE;
                pthread_mutex_unlock(&mutex_estado);
                break;
            }

            // Limpar buffer de entrada
            int c;
            while ((c = getchar()) != '\n' && c != EOF)
                ;
        }

        // Esperar um pouco antes de atualizar a interface
        usleep(200000); // 200ms
    }

    printf("Interface de usuário finalizada.\n");
    return NULL;
}

// Função Main
int main()
{
    printf("Iniciando simulador Apollo 11...\n");

    // Inicialização do gerador de números aleatórios
    srand(time(NULL));

    // Inicialização do mutex
    pthread_mutex_init(&mutex_estado, NULL);

    // Inicialização do estado da nave
    inicializar_estado();

    // Criando as threads
    pthread_create(&thread_voo, NULL, controle_voo, NULL);
    pthread_create(&thread_propulsao, NULL, controle_propulsao, NULL);
    pthread_create(&thread_energia, NULL, controle_energia, NULL);
    pthread_create(&thread_interface, NULL, interface_usuario, NULL);

    // Aguardando as threads
    pthread_join(thread_interface, NULL); // Espera a interface terminar (quando o usuário sai)
    pthread_join(thread_voo, NULL);
    pthread_join(thread_propulsao, NULL);
    pthread_join(thread_energia, NULL);

    // Limpeza
    pthread_mutex_destroy(&mutex_estado);

    printf("Simulador Apollo 11 finalizado.\n");

    return 0;
}
