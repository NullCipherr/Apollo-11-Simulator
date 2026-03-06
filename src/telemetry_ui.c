#include "telemetry_ui.h"
#include <math.h>
#include <ncurses.h>
#include <unistd.h>

void desenhar_interface(WINDOW *win) {
  wclear(win);
  box(win, 0, 0);

  pthread_mutex_lock(&mutex_estado);

  int rows, cols;
  getmaxyx(win, rows, cols);
  (void)rows;

  // ============================================
  // HEADER (Com estilo)
  // ============================================
  wattron(win, COLOR_PAIR(1) | A_BOLD);
  mvwprintw(win, 1, (cols - 30) / 2, "SIMULADOR APOLLO 11 - STATUS");
  wattroff(win, COLOR_PAIR(1) | A_BOLD);
  mvwhline(win, 2, 1, ACS_HLINE, cols - 2);

  // Estado da Missao
  wattron(win, A_BOLD);
  mvwprintw(win, 3, 2, "ESTADO DA MISSAO: ");

  if (estado_nave.estado_missao == EMERGENCIA) {
    wattron(win, COLOR_PAIR(2) | A_BLINK);
    wprintw(win, "%s", obter_nome_estado(estado_nave.estado_missao));
    wattroff(win, COLOR_PAIR(2) | A_BLINK);
  } else {
    wattron(win, COLOR_PAIR(3));
    wprintw(win, "%s", obter_nome_estado(estado_nave.estado_missao));
    wattroff(win, COLOR_PAIR(3));
  }
  wattroff(win, A_BOLD);

  mvwprintw(win, 3, cols - 35, "Tempo de Missao: %.2f horas",
            estado_nave.tempo_missao / 3600.0);

  mvwhline(win, 4, 1, ACS_HLINE, cols - 2);

  // ============================================
  // DADOS VITAIS DA NAVE
  // ============================================

  // Posição e Movimento (Esquerda)
  wattron(win, COLOR_PAIR(4) | A_BOLD);
  mvwprintw(win, 5, 2, " POSICAO E DINAMICA");
  wattroff(win, COLOR_PAIR(4) | A_BOLD);
  mvwprintw(win, 6, 4, "Posicao (km): X=%9.2f  Y=%9.2f  Z=%9.2f",
            estado_nave.posicao.x / 1000.0, estado_nave.posicao.y / 1000.0,
            estado_nave.posicao.z / 1000.0);
  mvwprintw(win, 7, 4, "Acel. (m/s2): X=%9.2f  Y=%9.2f  Z=%9.2f",
            estado_nave.aceleracao.x, estado_nave.aceleracao.y,
            estado_nave.aceleracao.z);

  double vel_total = sqrt(estado_nave.velocidade.x * estado_nave.velocidade.x +
                          estado_nave.velocidade.y * estado_nave.velocidade.y +
                          estado_nave.velocidade.z * estado_nave.velocidade.z);
  mvwprintw(win, 8, 4, "Velocidade total: %9.2f m/s (%9.2f km/h)", vel_total,
            vel_total * 3.6);

  mvwhline(win, 9, 1, ACS_HLINE, cols - 2);

  // Propulsão e Massa
  wattron(win, COLOR_PAIR(5) | A_BOLD);
  mvwprintw(win, 10, 2, " SISTEMAS DE PROPULSAO");
  wattroff(win, COLOR_PAIR(5) | A_BOLD);
  mvwprintw(win, 11, 4, "Combustivel principal: %10.2f kg (%.1f%%)",
            estado_nave.combustivel_principal,
            estado_nave.combustivel_principal / 1924000.0 * 100.0);
  mvwprintw(win, 12, 4, "Combustivel RCS:       %10.2f kg  (%.1f%%)",
            estado_nave.combustivel_rcs,
            estado_nave.combustivel_rcs / 500.0 * 100.0);
  mvwprintw(win, 13, 4, "Empuxo principal:      %10.2f kN",
            estado_nave.empuxo_principal / 1000.0);
  mvwprintw(win, 14, 4, "Empuxo RCS:            %10.2f N",
            estado_nave.empuxo_rcs);

  mvwhline(win, 15, 1, ACS_HLINE, cols - 2);

  // Energia
  wattron(win, COLOR_PAIR(6) | A_BOLD);
  mvwprintw(win, 16, 2, " CELULAS DE ENERGIA");
  wattroff(win, COLOR_PAIR(6) | A_BOLD);
  mvwprintw(win, 17, 4, "Energia principal:     %10.2f Wh (%.1f%%)",
            estado_nave.energia_principal,
            estado_nave.energia_principal / 10000.0 * 100.0);
  mvwprintw(win, 18, 4, "Energia reserva:       %10.2f Wh (%.1f%%)",
            estado_nave.energia_reserva,
            estado_nave.energia_reserva / 5000.0 * 100.0);
  mvwprintw(win, 19, 4, "Consumo atual:         %10.2f W",
            estado_nave.consumo_energia);

  mvwhline(win, 20, 1, ACS_HLINE, cols - 2);

  // Ambiente e Suporte de Vida
  wattron(win, COLOR_PAIR(7) | A_BOLD);
  mvwprintw(win, 21, 2, " SUPORTE DE VIDA");
  wattroff(win, COLOR_PAIR(7) | A_BOLD);
  mvwprintw(win, 22, 4, "Temperatura interna: %5.1f °C",
            estado_nave.temperatura_interna);
  mvwprintw(win, 23, 4, "Pressao interna:     %5.1f kPa",
            estado_nave.pressao_interna);
  mvwprintw(win, 24, 4, "Taxa de Radiacao:    %5.2f mSv/h",
            estado_nave.radiacao);

  // ============================================
  // SIMULAÇÃO E RODAPÉ
  // ============================================
  mvwhline(win, 25, 1, ACS_HLINE, cols - 2);
  mvwprintw(win, 26, 2, "VELOCIDADE DE SIMULACAO: %dx",
            atomic_load(&estado_nave.simulacao_acelerada));

  if (estado_nave.estado_missao == EMERGENCIA) {
    wattron(win, COLOR_PAIR(2) | A_BLINK | A_BOLD);
    mvwprintw(win, 27, (cols - 55) / 2,
              "*** SITUACAO DE EMERGENCIA: SISTEMAS COMPROMETIDOS ***");
    wattroff(win, COLOR_PAIR(2) | A_BLINK | A_BOLD);
  }

  wattron(win, A_DIM);
  mvwprintw(win, 29, 2,
            "Controles: [A]celerar [D]esacelerar [P]roximo Estado [E]mergencia "
            "[S]air");
  wattroff(win, A_DIM);

  pthread_mutex_unlock(&mutex_estado);
  wrefresh(win);
}

void *interface_usuario(void *arg) {
  (void)arg;

  // Inicialização do ncurses
  initscr();
  cbreak();
  noecho();
  nodelay(stdscr, TRUE); // Getch não-bloqueante (Raw mode style)
  keypad(stdscr, TRUE);
  curs_set(0);

  if (has_colors()) {
    start_color();
    use_default_colors();
    init_pair(1, COLOR_BLUE, -1);    // Título principal
    init_pair(2, COLOR_RED, -1);     // Alertas Críticos Emergência
    init_pair(3, COLOR_GREEN, -1);   // Status ok / Missão
    init_pair(4, COLOR_CYAN, -1);    // Header Posição
    init_pair(5, COLOR_MAGENTA, -1); // Header Propulsão
    init_pair(6, COLOR_YELLOW, -1);  // Header Energia
    init_pair(7, COLOR_WHITE, -1);   // Suporte vida
  }

  // Criar o display fixo principal
  WINDOW *win = newwin(31, 85, 1, 2);

  while (atomic_load(&estado_nave.sistema_ativo)) {
    desenhar_interface(win);

    int ch = getch();
    if (ch != ERR) {
      switch (ch) {
      case 'a':
      case 'A': {
        int acel = atomic_load(&estado_nave.simulacao_acelerada);
        if (acel < 8192)
          atomic_store(&estado_nave.simulacao_acelerada, acel * 2);
        break;
      }
      case 'd':
      case 'D': {
        int acel = atomic_load(&estado_nave.simulacao_acelerada);
        if (acel > 1)
          atomic_store(&estado_nave.simulacao_acelerada, acel / 2);
        break;
      }
      case 'p':
      case 'P':
        avancar_estado_missao();
        break;
      case 'e':
      case 'E':
        acionar_emergencia("ACIONAMENTO MANUAL DE EMERGENCIA");
        break;
      case 's':
      case 'S':
      case 'q':
      case 'Q':
        atomic_store(&estado_nave.sistema_ativo, false);
        break;
      }
    }

    usleep(50000); // UI atualiza a 20 FPS (TUI muito responsiva)
  }

  delwin(win);
  endwin();
  return NULL;
}

void *telemetry_logger(void *arg) {
  (void)arg;
  FILE *log_file = fopen("telemetry.csv", "w");
  if (log_file) {
    fprintf(log_file, "Tempo_seg,Estado,PosX_km,PosY_km,PosZ_km,VelX_ms,VelY_"
                      "ms,VelZ_ms,AcelX_ms2,AcelY_ms2,AcelZ_ms2,Combustivel_"
                      "Princ_kg,Combustivel_RCS_kg,Energia_Wh,Temperatura_C\n");
  }

  double tempo_ultimo_log = -1.0;

  while (atomic_load(&estado_nave.sistema_ativo)) {
    pthread_mutex_lock(&mutex_estado);
    double tempo_atual = estado_nave.tempo_missao;

    // Tentar gravar no log a cada aproximadamente 1 segundo de tempo simulado
    // ou 1 segundo de tempo real se simulação acelerar
    if (tempo_atual - tempo_ultimo_log >= 1.0) {
      if (log_file) {
        fprintf(log_file,
                "%.2f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%."
                "2f,%.2f,%.2f\n",
                estado_nave.tempo_missao,
                obter_nome_estado(estado_nave.estado_missao),
                estado_nave.posicao.x / 1000.0, estado_nave.posicao.y / 1000.0,
                estado_nave.posicao.z / 1000.0, estado_nave.velocidade.x,
                estado_nave.velocidade.y, estado_nave.velocidade.z,
                estado_nave.aceleracao.x, estado_nave.aceleracao.y,
                estado_nave.aceleracao.z, estado_nave.combustivel_principal,
                estado_nave.combustivel_rcs, estado_nave.energia_principal,
                estado_nave.temperatura_interna);
        fflush(log_file);
      }
      tempo_ultimo_log = tempo_atual;
    }
    pthread_mutex_unlock(&mutex_estado);
    usleep(500000); // Analisa a thread logger a cada 500ms real
  }

  if (log_file) {
    fclose(log_file);
  }
  return NULL;
}
