#include <stdio.h>
#include "esp32_pos_prediction.c"
#include "esp32_temp_prediction.c"

// option: predict_temp() as well

int main(){
  float param[3];
  // test data
  param[0] = 1.0; param[1] = 479.0; param[2] = 1129.0; // Ton, v0, v1 -> 0.0mm @ 25[degC]
  printf("Pos: %f / Temp: %f\n", predict_pos(param), predict_temp(param));

  param[0] = 1.0; param[1] = 473.0; param[2] = 1123.0; // Ton, v0, v1 -> 0.0mm @ 30[degC]
  printf("Pos: %f / Temp: %f\n", predict_pos(param), predict_temp(param));

  param[0] = 1.0; param[1] = 472.0; param[2] = 1112.0; // Ton, v0, v1 -> 0.0mm @ 35[degC]
  printf("Pos: %f / Temp: %f\n", predict_pos(param), predict_temp(param));

  param[0] = 1.0; param[1] = 461.0; param[2] = 1098.0; // Ton, v0, v1 -> 0.0mm @ 40[degC]
  printf("Pos: %f / Temp: %f\n", predict_pos(param), predict_temp(param));

  param[0] = 3.0; param[1] = 375.0; param[2] = 800.0; // Ton, v0, v1 -> 3.0mm @ 25[degC]
  printf("Pos: %f / Temp: %f\n", predict_pos(param), predict_temp(param));

  param[0] = 2.0; param[1] = 399.0; param[2] = 868.0; // Ton, v0, v1 -> 2.0mm @ 35[degC]
  printf("Pos: %f / Temp: %f\n", predict_pos(param), predict_temp(param));

  return(0);
}
