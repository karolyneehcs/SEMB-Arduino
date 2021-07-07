/**
Apresentação Geral do Algoritmo: Sobel Robert Cross - Algotitmo de detecção de bordas em imagens pgm.
* O Algoritmo Sobel é comumente aplicado em obter, através de uma imagem que carregam informações numéricas de 0 até 255, de forma a definir o gradiente de intensidade de acordo com estes valores e retornar como saida uma nova imagem entre os limites mais discrepantes de intensidade da imagem. A adaptação Robert Cross tem uma mesma abordagem estratégica.
*
* Estratégia do Algoritmo: Utiliza métodos matemáticos, como convoluções, ou seja, é realizada uma operação entre duas funções de espaços vetoriais, resultando em uma nova função da região compreendida pelos limites da imagem de entrada. Através disso, são inseridas duas matrizes, e a fórmula da convolução é dada através das derivativas entre a imagem original e ambas as matrizes. Ao fim do processo, é realizado também o cálculo da direção do gradiente, calculando a arctg o resultado entre a última convolução, e uma subtração de 135º, ou 3pi/4 para obter uma orientação similar ao que o olho humano percebe. O valor de  0º é correspondente à um contraste máximo.
*
*
* Copyright 2021 por Anthony Jefferson e Ana Karolina
* Instituto Federal de Educação, Ciencia e Tecnologia do Ceara - IFCE
* Todos os Direitos Reservados.
*
*
* Modo de uso da aplicação:
* É comumente utilizado em processamento de imagens e visão computacional, usado na saúde para se reconstruir imagens que não possuem um índice de detalhes altos, como curvas presentes em veias obstruídas, em arquitetura para obter detalhes estruturais nos locais de construção que não possui informações sobre pilares estruturais, vigas, datando épocas de fundação anterior ao surgimento da primeira revolução industrial.

* Entradas e Saídas: Uma imagem de extensão pgm, com valores entre 0 e 255. Saída esperada um imagem de extensão pgm com bordas destacadas, e contraste maior entre o fundo.
*
* Validação e Testes: Testes de benchmark
*
* Estudantes:
* Anthony Jefferson e Ana Karolina
*
* Data:
* 15 de junho de 2021.
*
* Contexto:
* Desenvolvimento de código exclusivo para fins acadêmicos, apresentação de nomenclatura "Trabalho T1 ARM/Linux - Desenvolvimento de SW"
*
* Plataforma alvo:
* Plataforma Linux*/

/* pgm file IO headerfile ------ mypgm.h
* Includes para bibliotecas externas
 */
#include "mypgm.h"

/* Implementação das funções */

/*
 * @note Documentação presente no arquivo de header
 */
void save_image_data( )
{
  char *file_name = "imagemDeSaida.pgm";
  FILE *fp; /* ponteiro do arquivo */
  int x, y; /* variáveis para loop */

  /* abertura do arquivo de saida */
  printf("-----------------------------------------------------\n");
  printf("Rotina de processamento da imagem de saida\n");
  printf("-----------------------------------------------------\n\n");
  fp = fopen(file_name, "wb");
  /* saida das informações de cabeçalho do arquivo */
  fputs("P5\n", fp);
  fputs("# \n", fp);
  fprintf(fp, "%d %d\n", x_size2, y_size2);
  fprintf(fp, "%d\n", MAX_BRIGHTNESS);
  /* saida dos dados de imagem para o arquivo */
  for (y = 0; y < y_size2; y++) {
    for (x = 0; x < x_size2; x++) {
      fputc(ENDORSE(image2[y][x]), fp);
    }
  }
  printf("\n-----Imagem de saida: STATUS OK-----\n\n");
  printf("-----------------------------------------------------\n\n");
  fclose(fp);
}

/*
 * @note Documentação presente no arquivo de header
 */
void load_image_data(const char *file_name)

{
  char buffer[MAX_BUFFERSIZE];
  FILE *fp; /* ponteiro do arquivo */
  int max_gray; /* valor máximo de cinza */
  int x, y; /* variáveis para loop */

  /* inicialização do arquivo do parâmetro */
  printf("\n-----------------------------------------------------\n");
  printf("Rotina de processamento de Imagem de Entrada \n");
  printf("-----------------------------------------------------\n\n");
  printf("     Apenas arquivos PGM sao aceitos.\n\n");
  fp = fopen(file_name, "rb");
  if (NULL == fp) {
    printf("     O arquivo nao existe!\n\n");
    exit(1);
  }
  /* Observa o tipo do arquivo e realiza o primeiro tratamento de erro ---P5 */
  fgets(buffer, MAX_BUFFERSIZE, fp);
  if (buffer[0] != 'P' || buffer[1] != '5') {
    printf("    Formato de aruivo inválido. Use um PGM P5!\n\n");
    exit(1);
  }
  /* entradas de coordenadas */
  x_size1 = 0;
  y_size1 = 0;
  while (x_size1 == 0 || y_size1 == 0) {
    fgets(buffer, MAX_BUFFERSIZE, fp);
    if (buffer[0] != '#') {
      sscanf(buffer, "%d %d", &x_size1, &y_size1);
    }
  }
  /* atribuição na variável de valores máximos de cinza */
  max_gray = 0;
  while (max_gray == 0) {
    fgets(buffer, MAX_BUFFERSIZE, fp);
    if (buffer[0] != '#') {
      sscanf(buffer, "%d", &max_gray);
    }
  }
  /* Demonstra para o usuário informações de entrada como dimensões, valor máximo de cinza na imagem. */
  printf("\n     Largura da Imagem = %d, Comprimento da Imagem = %d\n", x_size1, y_size1);
  printf("     Grau máximo de cinza = %d\n\n",max_gray);
  if (x_size1 > MAX_IMAGESIZE || y_size1 > MAX_IMAGESIZE) {
    printf("     Imagem excedeu o limite %d x %d\n\n",
	   MAX_IMAGESIZE, MAX_IMAGESIZE);
    printf("     Por favor, usar uma imagem com tamanho menor!\n\n");
    exit(1);
  }
  if (max_gray != MAX_BRIGHTNESS) {
    printf("     Grau de cinza inválido!\n\n");
    exit(1);
  }
  /* atribuição dos dados da imagem*/
  for (y = 0; y < y_size1; y++) {
    for (x = 0; x < x_size1; x++) {
      image1[y][x] = (unsigned char)fgetc(fp);
    }
  }
  printf("----- Imagem de entrada: STATUS OK -----\n\n");
  printf("-----------------------------------------------------\n\n");
  fclose(fp);
}

/*
 * @note Documentação presente no arquivo de header
 */
void sobelRobert_filter( )
{
  /* Definição  do filtro de Robert/Sobel na direção horizontal */
  int weight[3][3] = {{ -1,  0,  1 },
		      { -2,  0,  2 },
		      { -1,  0,  1 }};
  double pixel_value;
  double min, max;
  int x, y, i, j;  /* varíaveis do Loop */

  /* Maximum values calculation after filtering*/
  printf("Aplicando o filtro !\n\n");
  min = DBL_MAX;
  max = -DBL_MAX;
  for (y = 1; y < y_size1 - 1; y++) {
    for (x = 1; x < x_size1 - 1; x++) {
      pixel_value = 0.0;
      for (j = -1; j <= 1; j++) {
	    for (i = -1; i <= 1; i++) {
	      pixel_value += weight[j + 1][i + 1] * ENDORSE(image1[y + j][x + i]);
	    }
      }
      if (pixel_value < min) min = pixel_value;
      if (pixel_value > max) max = pixel_value;
    }
  }
  if ((int)(max - min) == 0) {
    printf("Não existe!!!\n\n");
    exit(1);
  }

  /* New loop variables APPROX */
  /*APPROX */int xa;
  /*APPROX */int ya;
  /* New pixel_value APPROX */
  /*APPROX */double pixel_value_app;
  /* Initialization of image2[y][x] */
  x_size2 = x_size1;
  y_size2 = y_size1;
  for (ya = 0; ENDORSE(ya < y_size2); ya++) {
    for (xa = 0; ENDORSE(xa < x_size2); xa++) {
      image2[ya][xa] = 0;
    }
  }
  /* Generation of image2 after linear transformtion */
  for (ya = 1; ENDORSE(ya < y_size1 - 1); ya++) {
    for (xa = 1; ENDORSE(xa < x_size1 - 1); xa++) {
      pixel_value_app = 0.0;
      for (j = -1; j <= 1; j++) {
	    for (i = -1; i <= 1; i++) {
	      pixel_value_app += weight[j + 1][i + 1] * image1[ya + j][xa + i];
	    }
      }
      pixel_value_app = MAX_BRIGHTNESS * (pixel_value_app - min) / (max - min);
      image2[ya][xa] = (unsigned char)pixel_value_app;
    }
  }
}
