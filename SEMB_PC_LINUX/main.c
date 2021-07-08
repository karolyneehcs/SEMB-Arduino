/**
Apresenta��o Geral do Algoritmo: Sobel Robert Cross - Algotitmo de detec��o de bordas em imagens pgm.
* O Algoritmo Sobel � comumente aplicado em obter, atrav�s de uma imagem que carregam informa��es num�ricas de 0 at� 255, de forma a definir o gradiente de intensidade de acordo com estes valores e retornar como sa�da uma nova imagem entre os limites mais discrepantes de intensidade da imagem. A adapta��o Robert Cross tem uma mesma abordagem estrat�gica.
*
* Estrat�gia do Algoritmo: Utiliza m�todos matem�ticos, como convolu��es, ou seja, � realizada uma opera��o entre duas fun��es de espa�os vetoriais, resultando em uma nova fun��o da regi�o compreendida pelos limites da imagem de entrada. Atrav�s disso, s�o inseridas duas matrizes, e a f�rmula da convolu��o � dada atrav�s das derivativas entre a imagem original e ambas as matrizes. Ao fim do processo, � realizado tamb�m o c�lculo da dire��o do gradiente, calculando a arctg o resultado entre a �ltima convolu��o, e uma subtra��o de 135�, ou 3pi/4 para obter uma orienta��o similar ao que o olho humano percebe. O valor de  0� � correspondente � um contraste m�ximo.
*
*
* Copyright 2021 por Anthony Jefferson e Ana Karolina
* Instituto Federal de Educa��o, Ci�ncia e Tecnologia do Cear� - IFCE
* Todos os Direitos Reservados.
*
* Fonte do c�digo: https://www.programiz.com/dsa/graph-bfs
*
* Modo de uso da aplica��o:
* � comumente utilizado em processamento de imagens e vis�o computacional, usado na sa�de para se reconstruir imagens que n�o possuem um �ndice de detalhes altos, como curvas presentes em veias obstru�das, em arquitetura para obter detalhes estruturais nos locais de constru��o que n�o possui informa��es sobre pilares estruturais, vigas, datando �pocas de funda��o anterior ao surgimento da primeira revolu��o industrial.

* Entradas e Sa�das: Uma imagem de extens�o pgm, com valores entre 0 e 255. Sa�da esperada um imagem de extens�o pgm com bordas destacadas, e constraste maior entre o fundo.
*
* Valida��o e Testes: Testes de benchmark
*
* Estudantes:
* Anthony Jefferson e Ana Karolina
*
* Data:
* 15 de junho de 2021.
*
* Contexto:
* Desenvolvimento de c�digo exclusivo para fins acad�micos, apresenta��o de nomenclatura "Trabalho T1 ARM/Linux - Desenvolvimento de SW"
*
* Plataforma alvo:
* Plataforma Linux*/

/* pgm file IO headerfile ------ mypgm.h
* Includes para bibliotecas externas
 */


#include "mypgm.h"
#include <time.h>

int main(int argc, const char** argv)
{
  clock_t tempo;
	
  load_image_data(argv[1]);   /* Input of image1 */

  tempo = clock();
  sobelRobert_filter( );   /* Sobel filter is applied to image1 */
  tempo = clock() - tempo;

  double tempo_total = (double)tempo / (double)CLOCKS_PER_SEC;
  printf("Robert Cross time: %f seconds\n",tempo_total);

  save_image_data( );   /* Output of image2 */
  return 0;
}
