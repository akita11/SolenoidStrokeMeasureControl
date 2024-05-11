#include <stdio.h>

int ADCvalue[9][5] = {
	{75, 82, 101, 116, 144}, // for Ton=1, L[0], L[1], ...
	{73, 81, 101, 116, 143}, // for Ton=2, L[0], L[1], ...
	{68, 79, 100, 113, 143},
	{63, 76, 97, 109, 142},
	{59, 72, 93, 105, 142},
	{56, 67, 88, 100, 140},
	{51, 62, 81, 91, 132},
	{47, 53, 70, 80, 116},
	{38, 42, 53, 61, 87}
};

// Ton = {1, 2, 3, 4, 5, 6, 7, 8, 9}
float L[] = {123.3, 111.5, 94.43, 86.33, 65.3};
#define X 9 // #Ton
#define Y 5 // #ADC

int main()
{
  float Ton0;
  int ADC0;
  float Lint;
  for (Ton0 = 1.0; Ton0 <= 9.0; Ton0 += 0.2){
    for (ADC0 = 30; ADC0 <= 150; ADC0++){
      int x, y;
      x = 0; while(x < X - 1){
	if ((float)(x+1) <= Ton0 && Ton0 < (float)(x+2)) break;
	x++;
      }
      float x0 = (float)(x + 1);
      float t = (Ton0 - x0) / 1.0;
      //  printf("t = %f\n", t);
      
      float x01 = x0 + t;
      float x23 = x0 + t;
      float s;
      
      y = 0; while(y < Y - 1){
	float y01 = (1.0 - t) * (float)ADCvalue[x][y] + t * (float)ADCvalue[x+1][y];
	float y23 = (1.0 - t) * (float)ADCvalue[x][y+1] + t * (float)ADCvalue[x+1][y+1];
	s = ((float)ADC0 - y01) / (y23 - y01);
	//    printf("00[%d %d] 10[%d %d] 01[%d %d] 11[%d %d]\n", x+1, ADCvalue[x][y], x+2, ADCvalue[x][y+1], x+1, ADCvalue[x+1][y], x+2, ADCvalue[x+1][y+1]);
	//    printf("%d (%f, %f), (%f, %f)\n", ADC0, x01, y01, x23, y23);
	//    printf("s = %f\n", s);
	if (0.0 <= s && s <= 1.0) break;
	y++;
      }
      if (y < Y - 1){
	//  printf("x, y = %d, %d\n", x, y);
	//  printf("00[%d %d] 10[%d %d] 01[%d %d] 11[%d %d]\n", x+1, ADCvalue[x][y], x+2, ADCvalue[x][y+1], x+1, ADCvalue[x+1][y], x+2, ADCvalue[x+1][y+1]);
	//  printf("L: %f %f\n", L[y], L[y+1]);
	Lint = (1 - s) * L[y] + s * L[y+1];
	printf("%f %d %f\n", Ton0, ADC0, Lint);
      }
      else{
	Lint = 0.0;
      }
    }
    printf("\n");
  }
}
