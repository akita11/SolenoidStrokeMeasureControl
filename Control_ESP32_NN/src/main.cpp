/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

//#include <TensorFlowLite.h>
#include "Adafruit_TFLite.h"

#include "output_handler.h"
#include "SolPosSSBH0830_model.h"

// Create an area of memory to use for input, output, and intermediate arrays.
// Finding the minimum value for your model may require some trial and error.
const int kTensorAreaSize  (2 * 1024);

// Will need tuning for your chipset
const int kInferencesPerCycle = 200;

Adafruit_TFLite ada_tflite(kTensorAreaSize);

void setup() {
	M5.begin()
	if (! ada_tflite.loadModel(g_model)) {
		printf("Failed to load default model\n");
  }
  printf("OK\n");

}

void loop() {
  ada_tflite.input->data.f[0] = 1.0;
  ada_tflite.input->data.f[1] = 479.0;
  ada_tflite.input->data.f[2] = 1129.0;

  TfLiteStatus invoke_status = ada_tflite.interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    printf("Invoke failed\n");
  }

  // Read the predicted y value from the model's output tensor
  float pos = ada_tflite.output->data.f[0];
	printf("pos=%f\n", pos);

//  // Output the results. A custom HandleOutput function can be implemented
//  // for each supported hardware target.
//  HandleOutput(ada_tflite.error_reporter, x_val, y_val);

}