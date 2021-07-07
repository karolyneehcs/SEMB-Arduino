 /**
Apresentação Geral do Algoritmo: Sobel Robert Cross - Algotitmo de detecção de bordas em imagens pgm.
* O Algoritmo Sobel é comumente aplicado em obter, através de uma imagem que carregam informações numéricas de 0 até 255, de forma a definir o gradiente de intensidade de acordo com estes valores e retornar como saída uma nova imagem entre os limites mais discrepantes de intensidade da imagem. A adaptação Robert Cross tem uma mesma abordagem estratégica.
*
* EstratÈgia do Algoritmo: Utiliza métodos matemáticos, como convoluções, ou seja, é realizada uma operação entre duas funções de espaços vetoriais, resultando em uma nova função da região compreendida pelos limites da imagem de entrada. Através disso, são inseridas duas matrizes, e a fórmula da convolução é dada através das derivativas entre a imagem original e ambas as matrizes. Ao fim do processo, é realizado também o cálculo da direção do gradiente, calculando a arctg o resultado entre a última convolução, e uma subtração de 135º, ou 3pi/4 para obter uma orientação similar ao que o olho humano percebe. O valor de  0º é correspondente à um contraste máximo.
*
*
* Copyright 2021 por Anthony Jefferson e Ana Karolina
* Instituto Federal de Educação, Ciência e Tecnologia do Ceará - IFCE
* Todos os Direitos Reservados.
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

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include "enerc.h"

/* Declaração de constantes para fins de simplificação de código */
#define MAX_IMAGESIZE  4096
#define MAX_BRIGHTNESS  255 /* Valor máximo de cinza */
#define GRAYLEVEL       256 /* Quantidade de valores na escala de cinza */
#define MAX_FILENAME    256 /* Limite do tamanho do nome do arquivo */
#define MAX_BUFFERSIZE  256

/* DECLARAÇÃO DE VARIÁVEIS GLOBAIS */

/* vetores de armazenamento de imagens */
unsigned char image1[MAX_IMAGESIZE][MAX_IMAGESIZE];
unsigned char image2[MAX_IMAGESIZE][MAX_IMAGESIZE];
int x_size1, y_size1, /* dimensões da imagem1 */
  x_size2, y_size2; /* dimensões da imagem2 */


/* Protótipo de funções */
/**
 *@brief Salva imagem processada em um arquivo .pgm .
 *@return void
 */
void save_image_data( );


/**
 *@brief aplica a matriz de Robert/Sobel na imagem.pgm .
 *@return void
 */
void sobelRobert_filter();

/**
 *@brief Carrega a imagem escolhida para processar.
 *@param[in] 	file_name 	ponteiro para string contendo caminho para a imagem.
 *@return 		void
 */
void load_image_data(const char* file_name); /* image input */


