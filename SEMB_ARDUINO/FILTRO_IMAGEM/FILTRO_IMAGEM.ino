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
*
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
const PROGMEM uint8_t fp[] = {80,53,10,53,48,32,53,48,10,50,53,53,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,255,255,255,255,0,255,0,0,0,255,0,255,255,255,255,255,0,255,0,0,0,255,0,255,255,255,255,255,0,255,0,0,0,255,0,255,0,0,0,255,0,0,0,0,0,0,0,0,0,255,255,255,255,255,0,255,0,0,0,255,0,255,255,255,255,255,0,255,0,0,0,255,0,255,255,255,255,255,0,255,255,0,0,255,0,255,0,0,0,255,0,0,0,0,0,0,0,0,0,255,0,0,0,255,0,255,255,0,0,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,255,0,0,255,0,255,0,0,0,255,0,0,0,0,0,0,0,0,0,255,0,0,0,255,0,255,255,0,0,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,255,0,0,255,0,255,0,0,0,255,0,0,0,0,0,0,0,0,0,255,0,0,0,255,0,255,255,0,0,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,255,255,0,255,0,255,0,0,0,255,0,0,111,111,111,111,0,0,0,255,0,0,0,255,0,255,255,0,0,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,255,0,255,0,255,0,0,0,255,0,0,111,111,111,111,0,0,0,255,0,0,0,255,0,255,255,255,0,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,255,0,255,0,255,0,0,0,255,0,0,111,111,111,111,0,0,0,255,255,255,255,255,0,255,255,255,0,255,0,0,0,255,0,0,0,255,255,255,255,255,0,255,0,0,0,255,0,255,0,255,255,255,0,0,255,255,255,255,0,0,0,0,0,0,0,0,0,255,255,255,255,255,0,255,0,255,0,255,0,0,0,255,0,0,0,255,255,255,255,255,0,255,0,0,0,255,0,255,0,0,255,255,0,0,255,255,255,0,0,0,0,200,200,0,0,0,0,255,0,0,0,255,0,255,0,255,0,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,255,255,0,0,0,255,0,0,0,0,0,200,200,0,0,0,0,255,0,0,0,255,0,255,0,255,0,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,255,255,0,0,0,255,0,0,0,0,200,200,200,200,0,0,0,255,0,0,0,255,0,255,0,255,0,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,0,0,255,0,0,0,0,200,200,200,200,0,0,0,255,0,0,0,255,0,255,0,0,255,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,0,0,255,0,0,0,0,200,200,200,200,0,0,0,255,0,0,0,255,0,255,0,0,255,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,0,0,255,0,0,0,0,0,200,200,0,0,0,0,255,0,0,0,255,0,255,0,0,255,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,0,0,255,0,0,0,0,0,200,200,0,0,0,0,255,0,0,0,255,0,255,0,0,255,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,255,255,255,255,0,255,0,0,0,255,0,0,0,255,0,0,0,0,0,0,0,0,0,0,0,255,0,0,0,255,0,255,0,0,255,255,0,0,0,255,0,0,0,255,0,0,0,255,0,255,255,255,255,255,0,255,0,0,0,255,0,0,0,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,230,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,230,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,0,0,0,255,0,255,255,255,255,255,0,255,255,255,255,255,0,255,255,255,255,255,0,255,0,0,0,0,0,255,0,255,255,0,0,255,0,255,255,255,255,255,0,0,0,0,0,0,0,255,0,0,255,255,0,255,255,255,255,255,0,255,255,255,255,255,0,255,255,255,255,255,0,255,0,0,0,0,0,255,0,255,255,0,0,255,0,255,255,255,255,255,0,0,0,0,0,0,0,255,0,255,255,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,0,0,255,0,255,255,255,0,255,0,255,0,0,0,255,0,0,0,0,0,0,0,255,255,255,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,0,0,255,0,255,255,255,0,255,0,255,0,0,0,255,0,0,215,215,0,0,0,255,255,0,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,0,0,255,0,255,0,255,0,255,0,255,0,0,0,255,0,0,215,215,215,0,0,255,255,0,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,0,0,255,0,255,0,255,255,255,0,255,0,0,0,255,0,215,111,111,215,0,0,255,255,255,0,0,0,255,255,0,0,255,0,255,255,255,255,255,0,255,0,0,0,255,0,255,0,0,0,0,0,255,0,255,0,0,255,255,0,255,0,0,0,255,0,215,111,111,215,0,0,255,0,255,0,0,0,255,255,255,255,255,0,255,255,255,255,255,0,255,0,0,0,255,0,255,0,0,0,0,0,255,0,255,0,0,255,255,0,255,255,255,255,255,0,215,111,111,215,0,0,255,0,255,255,0,0,255,255,255,255,255,0,255,255,0,0,0,0,255,0,0,0,255,0,255,0,0,0,0,0,255,0,255,0,0,255,255,0,255,255,255,255,255,0,0,215,215,0,0,0,255,0,0,255,0,0,255,0,0,0,255,0,255,255,255,0,0,0,255,0,0,0,255,0,255,0,0,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,0,215,215,0,0,0,255,0,0,255,255,0,255,0,0,0,255,0,255,255,255,255,0,0,255,0,0,0,255,0,255,0,0,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,0,0,0,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,255,255,255,0,255,0,0,0,255,0,255,0,0,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,0,0,0,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,0,0,0,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,0,0,0,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,0,0,0,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,255,255,255,255,255,0,255,255,255,255,255,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,0,0,0,0,0,0,255,0,0,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,255,255,255,255,255,0,255,255,255,255,255,0,255,0,255,0,0,0,255,0,255,0,0,0,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,'\0'};

char buffer_array[MAX_BUFFERSIZE];
uint8_t x_size1, y_size1, /* dimensões da imagem1 */
  x_size2, y_size2; /* dimensões da imagem2 */
uint32_t seek_var = 0;

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
  double time = micros();
  sobelRobert_filter();   /* Sobel filter is applied to image1 */
  time = micros()-time;
  Serial.println(time);
  
  while(1);
}

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
  
//  Serial.println(("P5"));
//  Serial.print((x_size2));
//  Serial.print(F(" "));
//  Serial.println((y_size2));
//  Serial.println((MAX_BRIGHTNESS));

  for(int i=0; i < 1; i++){
    for(int j=0; j < x_size1; j++){
//      Serial.flush();
//      Serial.write(0);
    }
  }
  
  /* Generation of image2 after linear transformtion */
  
  for (ya = 1; (ya < y_size1 - 1); ya++) {

//    Serial.flush();
//    Serial.write(0);
    
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

      
//      Serial.flush();
//      Serial.write((unsigned char)pixel_value_app);
    }
//    Serial.flush();
//    Serial.write(0);
  }

  for(int i=ya; i < y_size1; i++){
    for(int j=0; j < x_size1; j++){
//      Serial.flush();
//      Serial.write(0);
    }
  }
}
