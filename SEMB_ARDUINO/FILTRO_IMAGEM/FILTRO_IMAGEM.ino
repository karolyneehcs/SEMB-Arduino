/**
Apresentação Geral do Algoritmo: Sobel Robert Cross - Algotitmo de detecção de bordas em imagens pgm.
* O Algoritmo Sobel é comumente aplicado em obter, através de uma imagem que carregam informações numéricas de 0 até 255, de forma a definir o gradiente de intensidade de acordo com estes valores e retornar como saída uma nova imagem entre os limites mais discrepantes de intensidade da imagem. A adaptação Robert Cross tem uma mesma abordagem estratégica.
*
* EstratÈgia do Algoritmo: Utiliza métodos matemáticos, como convoluções, ou seja, é realizada uma operação entre duas funções de espaços vetoriais, resultando em uma nova função da região compreendida pelos limites da imagem de entrada. Através disso, são inseridas duas matrizes, e a fórmula da convolução é dada através das derivativas entre a imagem original e ambas as matrizes. Ao fim do processo, é realizado também o cálculo da direção do gradiente, calculando a arctg o resultado entre a última convolução, e uma subtração de 135º, ou 3pi/4 para obter uma orientação similar ao que o olho humano percebe. O valor de  0º é correspondente à um contraste máximo.
*
*
* Copyright 2021 por Anthony Jefferson e Ana Karolina
* Instituto Federal de EducaÁ„o, CiÍncia e Tecnologia do Cear· - IFCE
* Todos os Direitos Reservados.
*
* Fonte do código: https://www.programiz.com/dsa/graph-bfs
*
* Modo de uso da aplicação:
* É comumente utilizado em processamento de imagens e visão computacional, usado na saúde para se reconstruir imagens que não possuem um índice de detalhes altos, como curvas presentes em veias obstruídas, em arquitetura para obter detalhes estruturais nos locais de construção que não possui informações sobre pilares estruturais, vigas, datando épocas de fundação anterior ao surgimento da primeira revolução industrial.

* Entradas e Saídas: Uma imagem de extensão pgm, com valores entre 0 e 255. Saída esperada um imagem de extensão pgm com bordas destacadas, e constraste maior entre o fundo.
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
* Plataforma Linux
*
* Para converter a saída do Arduino para uma imagem, utilizar o site https://tomeko.net/online_tools/hex_to_file.php?lang=en
*/

/* pgm file IO headerfile ------ mypgm.h
* Includes para bibliotecas externas
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>

/* Declaração de constantes para fins de simplificação de código */
#define MAX_IMAGESIZE  50
#define MAX_BRIGHTNESS  255 /* Valor máximo de cinza */
#define GRAYLEVEL       256 /* Quantidade de valores na escala de cinza */
#define MAX_FILENAME    256 /* Limite do tamanho do nome do arquivo */
#define MAX_BUFFERSIZE  64
#define DBL_MAX ((double)1.79769313486231570815e+308L)
#define DBL_MIN ((double)2.22507385850720138309e-308L)

/* DECLARAÇÃO DE VARIÁVEIS GLOBAIS */

/* vetores de armazenamento de imagens */
const PROGMEM uint8_t fp[] = {80,53,10,50,48,32,50,48,10,50,53,53,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,255,255,0,255,0,0,0,255,0,0,255,255,0,255,255,0,0,0,0,255,255,255,0,255,0,0,0,255,0,255,255,255,255,255,255,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,255,255,255,255,255,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,255,255,255,255,255,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,255,255,255,255,255,255,0,0,0,255,255,255,0,255,0,0,0,255,0,0,255,255,255,255,255,0,0,0,0,255,0,0,0,255,0,0,0,255,0,0,0,255,255,255,0,0,0,0,0,255,0,0,0,255,0,0,0,255,0,0,0,0,255,0,0,0,0,0,0,255,0,0,0,255,0,0,0,255,0,0,0,0,0,0,0,0,0,0,0,255,255,255,0,255,255,255,0,255,0,0,0,0,0,0,0,0,0,0,0,255,255,255,0,255,255,255,0,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,255,0,0,255,255,255,255,0,0,0,0,0,0,0,0,0,0,0,255,0,0,255,0,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,0,0,255,0,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,255,255,255,0,255,255,255,255,0,0,0,0,0,0,0,0,0,0,0,255,0,0,255,0,0,0,0,255,0,0,0,0,0,0,0,0,0,0,0,255,0,0,255,0,0,0,0,255,0,0,0,0,0,0,0,0,0,0,0,255,0,0,255,0,255,255,255,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,'\0'};
//const PROGMEM uint8_t fp[] = {80,53,10,52,57,32,52,57,10,50,53,53,10,117,162,0,19,44,37,227,178,61,17,244,95,190,204,196,42,132,96,206,189,70,107,204,97,185,60,34,6,123,240,168,24,121,213,120,178,9,28,12,34,92,136,71,251,36,201,85,174,16,209,163,220,10,230,207,15,241,103,186,217,194,96,147,187,198,15,118,193,213,35,0,4,86,81,193,12,47,155,55,95,65,168,189,195,57,55,94,182,253,181,214,250,220,4,239,205,109,118,84,169,52,249,39,197,147,44,44,193,99,178,238,12,187,155,169,39,19,77,238,33,91,9,246,99,162,192,196,46,12,207,238,85,95,93,7,38,116,150,242,218,42,128,50,0,222,72,158,237,124,130,43,26,195,8,229,183,60,77,138,18,17,227,49,18,111,231,45,72,164,98,180,115,215,209,75,208,87,222,11,148,10,147,246,218,59,207,33,28,192,234,52,88,131,230,231,93,173,62,3,46,65,92,240,119,178,13,40,224,200,215,102,231,1,176,253,227,147,226,33,35,244,100,145,190,22,3,35,104,179,31,241,201,112,147,58,72,209,126,205,141,1,238,79,251,195,13,142,185,241,155,111,137,75,112,184,46,69,166,47,184,230,127,112,178,113,98,250,141,203,152,162,97,39,78,157,232,116,145,27,236,74,106,195,158,157,96,162,95,246,40,191,190,61,82,19,44,5,230,101,134,238,88,79,118,40,147,51,192,205,189,209,130,71,228,103,214,8,88,129,164,159,90,221,204,200,88,183,207,104,146,183,214,189,251,1,76,14,233,5,118,40,71,2,224,138,9,125,40,28,225,19,236,134,180,35,30,144,231,209,27,159,237,17,213,186,92,149,180,196,196,202,233,49,141,52,208,44,58,246,166,110,45,0,149,163,93,101,225,23,97,154,2,137,236,1,21,177,158,202,41,175,239,29,199,58,67,208,248,65,193,92,15,61,168,125,136,3,26,223,12,203,55,22,91,70,113,192,48,251,52,68,76,23,145,129,224,72,187,151,132,183,23,149,141,122,117,91,150,73,52,45,10,57,193,51,133,41,192,246,214,227,206,181,242,54,156,27,20,45,58,51,149,106,44,93,79,157,241,164,218,122,220,175,90,39,119,219,84,115,163,233,163,205,33,192,251,34,125,160,237,19,240,221,38,234,121,182,10,159,49,74,161,237,216,48,144,71,223,91,2,126,190,72,72,78,93,253,150,44,121,88,245,75,133,4,140,248,27,230,136,106,193,183,4,138,133,145,150,194,94,111,45,200,22,247,222,117,236,11,104,143,113,95,184,246,168,35,234,55,130,159,63,79,72,67,217,75,50,75,197,245,153,21,5,15,222,251,178,6,179,29,134,244,23,63,243,149,160,230,63,11,245,217,220,232,20,226,114,39,213,84,79,136,250,81,185,231,34,137,179,171,98,180,147,13,31,68,197,38,235,127,77,36,154,111,234,111,247,242,98,210,194,3,51,64,124,222,206,112,134,144,21,62,38,76,56,8,93,74,109,20,93,217,143,164,77,252,239,198,59,172,148,72,123,25,212,92,245,13,208,136,58,60,89,158,196,12,252,158,242,1,94,249,68,124,123,21,245,17,106,144,26,109,196,60,125,222,241,206,197,133,216,223,133,112,188,142,173,153,187,8,171,183,91,81,235,128,213,2,119,178,2,203,46,68,18,184,104,141,40,212,130,245,190,209,209,110,214,82,87,48,150,30,149,168,214,146,187,10,214,164,230,238,214,176,85,59,191,207,179,247,5,176,37,132,250,222,140,159,77,215,102,26,61,69,195,28,200,53,100,195,16,72,117,102,157,226,45,191,21,126,228,118,32,215,195,54,92,182,230,215,99,182,125,197,180,21,183,193,153,112,116,129,102,133,135,236,244,50,167,3,15,187,30,91,19,120,4,73,24,30,211,245,71,222,45,216,98,69,246,71,210,126,103,116,194,35,55,28,44,165,46,80,221,238,84,181,242,213,235,173,236,160,179,235,7,109,57,126,128,120,178,243,185,202,194,120,21,52,172,139,28,244,67,16,91,55,29,134,123,107,95,14,74,100,65,30,147,101,63,172,188,120,33,61,85,129,230,14,208,187,171,31,240,95,126,219,69,46,106,90,104,137,61,186,207,14,9,212,46,236,205,236,17,213,156,117,46,130,57,245,198,100,224,109,210,128,83,17,2,80,177,229,76,88,68,175,10,37,98,41,39,171,132,75,27,191,218,89,243,164,160,53,220,143,219,246,244,212,88,39,110,241,254,180,67,28,176,154,124,16,116,73,140,112,31,93,41,77,106,246,44,173,67,100,9,155,211,115,11,124,72,16,234,181,182,222,212,167,28,116,123,190,96,228,254,165,89,132,131,222,107,41,183,67,69,151,201,207,91,100,211,166,209,107,232,179,74,90,203,201,87,158,140,190,82,200,111,32,8,124,30,143,85,92,170,135,99,16,242,115,48,92,167,165,120,118,26,186,48,174,236,136,248,6,104,85,97,241,234,187,32,216,148,43,253,190,191,97,118,100,133,13,14,253,44,219,131,79,14,128,179,245,139,15,90,121,6,19,185,197,228,36,52,72,52,52,156,17,59,17,14,19,150,140,65,15,87,219,110,93,190,230,67,91,161,109,228,160,106,82,214,5,18,17,212,95,21,162,108,217,30,19,73,160,38,106,110,27,29,184,192,110,184,173,207,245,35,211,82,237,8,172,42,110,75,126,161,91,167,71,17,213,206,80,134,96,166,96,43,108,88,69,212,114,130,122,109,149,216,137,199,56,172,188,39,211,252,157,129,104,52,50,233,133,50,244,15,171,47,118,58,85,193,34,57,34,204,40,132,192,52,225,109,109,211,104,220,6,238,78,76,15,64,215,184,177,192,198,180,202,195,32,170,175,180,77,244,72,213,33,27,183,229,30,203,206,116,1,29,7,167,125,51,145,9,40,46,235,240,213,37,224,75,173,89,205,115,8,254,72,248,106,60,28,125,105,235,25,91,184,64,128,240,104,245,46,201,0,26,200,92,92,171,61,248,220,115,37,252,25,219,105,116,222,172,134,163,97,212,51,239,95,47,221,130,152,55,207,55,90,215,232,143,41,206,54,213,229,131,216,109,68,8,215,146,171,82,30,239,166,150,210,37,251,0,67,11,40,108,134,208,1,225,5,238,248,52,71,95,24,19,235,180,28,86,251,76,45,60,20,228,87,122,234,114,159,92,23,83,131,164,239,134,72,69,45,113,72,245,197,212,106,254,165,47,0,87,215,211,17,236,79,85,65,188,247,61,14,114,63,146,61,179,198,100,187,104,168,84,85,150,247,90,92,200,243,221,139,50,162,183,83,75,60,57,86,160,215,170,203,140,114,185,49,51,33,65,53,12,203,190,137,43,235,105,119,24,67,27,198,171,182,62,250,233,103,96,60,191,31,187,205,100,187,135,26,106,115,37,40,231,50,41,147,49,94,190,27,132,224,9,81,213,233,152,122,150,3,180,91,117,110,55,44,78,63,52,217,44,250,41,217,243,98,134,3,42,69,51,98,104,50,150,214,234,184,119,37,226,11,71,128,239,91,45,152,51,38,94,233,219,160,210,26,117,28,215,44,36,10,156,16,102,111,21,171,190,23,64,87,223,33,173,199,6,220,154,215,230,223,248,33,194,187,53,99,46,179,109,241,120,164,243,31,157,254,194,158,211,185,11,141,100,217,5,11,243,134,27,98,168,131,83,69,62,16,205,75,55,241,62,214,74,18,252,46,89,67,23,149,210,228,93,197,37,30,54,147,189,94,216,50,95,222,2,222,136,242,108,143,144,82,184,174,114,107,220,129,19,64,54,135,101,64,53,194,135,210,76,140,200,61,216,102,216,133,77,117,208,144,3,237,43,207,62,14,80,222,218,175,135,182,207,70,198,146,208,10,168,175,151,230,87,88,227,43,108,33,142,40,7,162,248,2,146,178,126,216,166,15,120,141,223,88,42,121,38,146,77,247,228,154,25,250,166,140,119,141,134,27,46,156,103,102,180,241,20,57,1,36,70,69,178,105,71,136,78,62,222,185,249,39,116,28,51,133,51,213,107,151,23,124,133,50,5,201,64,87,96,237,85,10,149,57,183,155,60,81,33,96,167,75,103,224,174,26,80,72,78,48,167,67,247,91,22,115,163,68,56,40,109,109,252,253,147,245,154,111,161,33,202,185,108,126,142,163,37,190,178,59,216,147,12,150,3,37,33,45,105,241,182,228,245,19,146,119,109,182,203,0,114,174,43,42,40,80,170,126,223,16,57,31,167,220,116,41,228,189,197,1,23,5,237,253,18,104,158,121,62,79,160,13,144,156,83,195,204,239,153,125,237,3,188,207,207,225,32,237,230,29,224,155,143,89,183,24,104,141,109,247,95,53,251,33,250,13,175,194,1,76,90,105,20,124,40,195,139,105,243,164,223,171,44,27,4,209,169,244,144,120,222,189,49,211,0,216,87,74,87,111,54,168,194,35,138,56,217,89,19,198,199,241,186,35,123,204,238,157,166,238,70,175,250,232,54,241,29,25,227,199,32,191,227,41,92,154,245,216,81,70,103,164,162,164,89,104,120,90,17,248,147,206,215,208,93,157,225,70,147,177,244,188,189,240,95,18,84,171,178,2,188,110,52,203,223,117,247,141,4,195,115,218,98,204,219,204,191,134,168,105,173,174,177,199,44,84,248,193,198,234,101,22,49,174,230,75,159,214,42,195,140,165,163,175,31,127,166,116,28,213,215,48,195,49,36,228,74,137,39,239,136,157,171,43,221,243,25,237,228,194,220,155,165,139,53,42,113,137,180,0,11,193,73,187,56,141,226,228,119,181,92,44,219,222,105,61,22,110,31,120,24,185,22,198,206,176,237,25,118,182,106,75,173,130,56,158,136,75,239,193,80,124,64,15,16,182,218,190,140,31,87,28,116,251,179,62,109,184,107,87,133,70,94,96,166,140,124,246,73,56,185,121,25,206,245,5,71,156,187,115,223,211,182,189,233,134,46,49,160,15,14,248,172,46,227,168,214,250,193,166,147,96,134,72,84,236,187,43,189,111,141,201,184,113,9,88,232,201,213,18,99,189,145,253,72,155,243,71,205,192,166,164,138,214,46,73,167,236,107,16,145,11,188,8,4,61,69,12,42,111,134,154,78,101,146,65,244,51,89,142,135,252,75,251,142,136,16,159,84,181,226,229,106,123,120,234,253,249,173,75,89,221,44,142,165,104,212,48,244,68,232,253,75,194,248,82,178,174,36,144,247,131,55,64,87,227,69,246,12,193,97,3,60,47,124,69,98,105,60,63,87,158,214,71,190,253,243,140,182,231,158,54,116,114,23,235,15,78,52,223,71,11,0,216,205,143,10,213,91,208,35,183,69,110,212,82,34,55,22,152,67,174,122,69,101,68,148,132,67,206,'\0'}; 
char buffer_array[MAX_BUFFERSIZE];
//unsigned char image[MAX_IMAGESIZE][MAX_IMAGESIZE];
uint8_t x_size1, y_size1, /* dimensões da imagem1 */
  x_size2, y_size2; /* dimensões da imagem2 */
uint32_t seek_var = 0;

/* Protótipo de funções */
/**
 *@brief Salva imagem processada em um arquivo .pgm .
 *@return void
 */
void save_image_data();

/**
 *@brief Aplica o filtro solbel Robert na imagem carregada.
 *@return void
 */
void sobelRobert_filter();

/**
 *@brief Carrega a imagem escolhida para processar.
 *@param[in]   file_name   ponteiro para string contendo caminho para a imagem.
 *@return     void
 */
void load_image_data(); /* image input */

void setup(){
  Serial.begin(115200);
}

void loop(){
  load_image_data();   /* Input of image1 */
  sobelRobert_filter();   /* Sobel filter is applied to image1 */
  //save_image_data();   /* Output of image2 */
  while(1);
}

/*
 * @note Documentação presente no arquivo de header
 */
/*void save_image_data( )
{
  uint8_t x, y;
  
  Serial.print(("P5"));
  Serial.print(F(" "));
  Serial.print((x_size2));
  Serial.print(F(" "));
  Serial.print((y_size2));
  Serial.print(F(" "));
  Serial.print((MAX_BRIGHTNESS));

  for (y = 0; y < y_size2; y++) {
      for (x = 0; x < x_size2; x++) {
        Serial.flush();
        Serial.print(F(" "));
        Serial.print((image[y][x]));
      }
  }
}*/

/*
 * @note Documentação presente no arquivo de header
 */
void load_image_data()

{
  uint16_t max_gray; /* valor máximo de cinza */
  int x, y; /* variáveis para loop */
  uint32_t aux;

  /* inicialização do arquivo do parâmetro */
  
  if (pgm_read_byte_near(fp + 0) != 'P' || pgm_read_byte_near(fp + 1) != '5') {
    Serial.println("     Mistaken file format, not P5!\n");
    exit(1);
  }
  seek_var = 3;
  
  /* entradas de coordenadas */
  x_size1 = 0;
  y_size1 = 0;
  aux = seek_var;
  while (x_size1 == 0 || y_size1 == 0) {
  while(1){
    if(pgm_read_byte_near(fp + aux) == '\n'){
      aux++;
      break;
    }
    aux++;
  }
  
  for(int i = 0;i<(aux-seek_var);i++){
    buffer_array[i] = pgm_read_byte_near(fp + seek_var + i);
  }
    if (buffer_array[0] != '#') {
      sscanf(buffer_array, "%d %d", &x_size1, &y_size1);
    }
    seek_var = aux;
  }
  
  /* atribuição na variável de valores máximos de cinza */
  max_gray = 0;
  aux = seek_var;
  while (max_gray == 0) {
  while(1){
    if(pgm_read_byte_near(fp + aux) == '\n'){
      aux++;
      break;
    }
    aux++;
  }
  
  for(int i = 0;i<(aux-seek_var);i++){
    buffer_array[i] = pgm_read_byte_near(fp + seek_var + i);
  }
    if (buffer_array[0] != '#') {
      max_gray = atoi(buffer_array);
    }
    seek_var = aux;
  }

  /* Demonstra para o usuário informações de entrada como dimensões, valor máximo de cinza na imagem. */
  
  if (x_size1 > MAX_IMAGESIZE || y_size1 > MAX_IMAGESIZE) {
    //printf("     Image size exceeds %d x %d\n\n",MAX_IMAGESIZE, MAX_IMAGESIZE);
    //printf("     Please use smaller images!\n\n");
    exit(1);
  }
  if (max_gray != MAX_BRIGHTNESS) {
    //printf("     Invalid value of maximum gray level!\n\n");
    exit(1);
  }
  /* atribuição dos dados da imagem*/
  /*aux = seek_var;
  for (y = 0; y < y_size1; y++) {
    for (x = 0; x < x_size1; x++) {

      image1[y][x] = (unsigned char)pgm_read_byte_near(fp + aux);

      aux++;
    }
  }  */
}

/*
 * @note Documentação presente no arquivo de header
 */
void sobelRobert_filter( )
{
  /* Definition of Sobel filter in horizontal direction */
  int weight[3][3] = {{ -1,  0,  1 },
          { -2,  0,  2 },
          { -1,  0,  1 }};
  double pixel_value;
  double min, max;
  int x, y, i, j;  /* Loop variable */

  /* Maximum values calculation after filtering*/
  min = DBL_MAX;
  max = -DBL_MAX;
  
  for (y = 1; y < y_size1 - 1; y++) {
    for (x = 1; x < x_size1 - 1; x++) {
      pixel_value = 0.0;
      for (j = -1; j <= 1; j++) {
      for (i = -1; i <= 1; i++) {
        //pixel_value += weight[j + 1][i + 1] * (image1[y + j][x + i]);
        pixel_value += weight[j + 1][i + 1] * pgm_read_byte_near(fp + seek_var + ((y + j)*y_size1) + (x + i));
      }
      }
      if (pixel_value < min) min = pixel_value;
      if (pixel_value > max) max = pixel_value;
    }
  }
  
  if ((int)(max - min) == 0) {
     
    Serial.println(F("Nothing exists!!!\n"));
    delay(1000);
    exit(1);
  }
  /* New loop variables APPROX */
  int xa;
  int ya;
  /* New pixel_value APPROX */
  double pixel_value_app;
  /* Initialization of image2[y][x] */
  x_size2 = x_size1;
  y_size2 = y_size1;
  /*for (ya = 0; (ya < y_size2); ya++) {
    for (xa = 0; (xa < x_size2); xa++) {
      image[ya][xa] = 0;
    }
  }*/
  
  Serial.println(("P5"));
  //Serial.print(F(" "));
  Serial.print((x_size2));
  Serial.print(F(" "));
  Serial.println((y_size2));
  //Serial.print(F(" "));
  Serial.println((MAX_BRIGHTNESS));

  for(int i=0; i < 1; i++){
    for(int j=0; j < x_size1; j++){
      Serial.flush();
      //Serial.print(F(" 0"));
      Serial.write(0);
    }
  }
  
  /* Generation of image2 after linear transformtion */
  
  for (ya = 1; (ya < y_size1 - 1); ya++) {

    Serial.flush();
    //Serial.print(F(" 0"));
    Serial.write(0);
    
    for (xa = 1; (xa < x_size1 - 1); xa++) {
      pixel_value_app = 0.0;
      for (j = -1; j <= 1; j++) {
      for (i = -1; i <= 1; i++) {
        //pixel_value_app += weight[j + 1][i + 1] * image1[ya + j][xa + i];
        pixel_value_app += weight[j + 1][i + 1] * pgm_read_byte_near(fp + seek_var + ((ya + j)*y_size1) + (xa + i));
      }
      }
      pixel_value_app = MAX_BRIGHTNESS * (pixel_value_app - min) / (max - min);
      //image[ya][xa] = (unsigned char)pixel_value_app;

      
      Serial.flush();
      //Serial.print(F(" "));
      //Serial.print((unsigned char)pixel_value_app);
      Serial.write((unsigned char)pixel_value_app);
    }
    Serial.flush();
    //Serial.print(F(" 0"));
    Serial.write(0);
  }

  for(int i=ya; i < y_size1; i++){
    for(int j=0; j < x_size1; j++){
      Serial.flush();
      //Serial.print(F(" 0"));
      Serial.write(0);
    }
  }
}
