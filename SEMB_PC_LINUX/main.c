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
* Plataforma Linux*/

/* pgm file IO headerfile ------ mypgm.h
* Includes para bibliotecas externas
 */


#include "mypgm.h"

int main(int argc, const char** argv)
{
  load_image_data(argv[1]);   /* Input of image1 */ 
  sobelRobert_filter( );   /* Sobel filter is applied to image1 */
  save_image_data( );   /* Output of image2 */
  return 0;
}
