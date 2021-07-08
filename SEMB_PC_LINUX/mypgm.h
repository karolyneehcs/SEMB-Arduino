 /**
ApresentaÃ§Ã£o Geral do Algoritmo: Sobel Robert Cross - Algotitmo de detecÃ§Ã£o de bordas em imagens pgm.
* O Algoritmo Sobel Ã© comumente aplicado em obter, atravÃ©s de uma imagem que carregam informaÃ§Ãµes numÃ©ricas de 0 atÃ© 255, de forma a definir o gradiente de intensidade de acordo com estes valores e retornar como saÃ­da uma nova imagem entre os limites mais discrepantes de intensidade da imagem. A adaptaÃ§Ã£o Robert Cross tem uma mesma abordagem estratÃ©gica.
*
* EstratÃˆgia do Algoritmo: Utiliza mÃ©todos matemÃ¡ticos, como convoluÃ§Ãµes, ou seja, Ã© realizada uma operaÃ§Ã£o entre duas funÃ§Ãµes de espaÃ§os vetoriais, resultando em uma nova funÃ§Ã£o da regiÃ£o compreendida pelos limites da imagem de entrada. AtravÃ©s disso, sÃ£o inseridas duas matrizes, e a fÃ³rmula da convoluÃ§Ã£o Ã© dada atravÃ©s das derivativas entre a imagem original e ambas as matrizes. Ao fim do processo, Ã© realizado tambÃ©m o cÃ¡lculo da direÃ§Ã£o do gradiente, calculando a arctg o resultado entre a Ãºltima convoluÃ§Ã£o, e uma subtraÃ§Ã£o de 135Âº, ou 3pi/4 para obter uma orientaÃ§Ã£o similar ao que o olho humano percebe. O valor de  0Âº Ã© correspondente Ã  um contraste mÃ¡ximo.
*
*
* Copyright 2021 por Anthony Jefferson e Ana Karolina
* Instituto Federal de EducaÃ§Ã£o, CiÃªncia e Tecnologia do CearÃ¡ - IFCE
* Todos os Direitos Reservados.
*
* Modo de uso da aplicaÃ§Ã£o:
* Ã‰ comumente utilizado em processamento de imagens e visÃ£o computacional, usado na saÃºde para se reconstruir imagens que nÃ£o possuem um Ã­ndice de detalhes altos, como curvas presentes em veias obstruÃ­das, em arquitetura para obter detalhes estruturais nos locais de construÃ§Ã£o que nÃ£o possui informaÃ§Ãµes sobre pilares estruturais, vigas, datando Ã©pocas de fundaÃ§Ã£o anterior ao surgimento da primeira revoluÃ§Ã£o industrial.

* Entradas e SaÃ­das: Uma imagem de extensÃ£o pgm, com valores entre 0 e 255. SaÃ­da esperada um imagem de extensÃ£o pgm com bordas destacadas, e contraste maior entre o fundo.
*
* ValidaÃ§Ã£o e Testes: Testes de benchmark
*
* Estudantes:
* Anthony Jefferson e Ana Karolina
*
* Data:
* 15 de junho de 2021.
*
* Contexto:
* Desenvolvimento de cÃ³digo exclusivo para fins acadÃªmicos, apresentaÃ§Ã£o de nomenclatura "Trabalho T1 ARM/Linux - Desenvolvimento de SW"
*
* Plataforma alvo:
* Plataforma Linux*/

/* pgm file IO headerfile ------ mypgm.h
* Includes para bibliotecas externas
 */

#include <stdio.h>
#include <stdlib.h>
#include <float.h>

/* DeclaraÃ§Ã£o de constantes para fins de simplificaÃ§Ã£o de cÃ³digo */
#define MAX_IMAGESIZE  4096
#define MAX_BRIGHTNESS  255 /* Valor mÃ¡ximo de cinza */
#define GRAYLEVEL       256 /* Quantidade de valores na escala de cinza */
#define MAX_FILENAME    256 /* Limite do tamanho do nome do arquivo */
#define MAX_BUFFERSIZE  256

/* DECLARAÃ‡ÃƒO DE VARIÃ�VEIS GLOBAIS */

/* vetores de armazenamento de imagens */
unsigned char image1[MAX_IMAGESIZE][MAX_IMAGESIZE];
unsigned char image2[MAX_IMAGESIZE][MAX_IMAGESIZE];
int x_size1, y_size1, /* dimensÃµes da imagem1 */
  x_size2, y_size2; /* dimensÃµes da imagem2 */


/* ProtÃ³tipo de funÃ§Ãµes */
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


