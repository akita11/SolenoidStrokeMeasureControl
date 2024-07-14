/* Copyright 2024 Chirale, TensorFlow Authors. All Rights Reserved.

This sketch is derived from the classic Hello World example of the general 
TensorFlow Lite Micro library. It has been adapted and simplified by Chirale 
to conform to the typical style of Arduino sketches. 
It has been tested on an Arduino Nano 33 BLE.
The sketch implements a Deep Neural Network pre-trained on calculating 
the function sin(x). 
By sending a value between 0 and 2*Pi via the Serial Monitor, 
both the value inferred by the DNN model and the actual value 
calculated using the Arduino math library are displayed.

It shows how to use TensorFlow Lite Library on Arduino.

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

// running TensorFlowLite on ESP32/ESP32-S3
// based on Chirale-TensorFlowLite's example, hello_world
// using M5Stack Core2/S3SE　with library: https://github.com/spaziochirale/Chirale_TensorFlowLite

#include <Chirale_TensorFlowLite.h>

#include "SolPosSSBH0830_model.h"  // generated model file

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

constexpr int kTensorArenaSize = 4000;

uint8_t tensor_arena[kTensorArenaSize];


void setup() {
  Serial.begin(115200);
  while(!Serial);

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_model);

  // Check if model and library have compatible schema version,
  // if not, there is a misalignement between TensorFlow version used
  // to train and generate the TFLite model and the current version of library
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model provided and schema version are not equal!");
    while(true); // stop program here
  }
  // This pulls in all the TensorFlow Lite operators.
  static tflite::AllOpsResolver resolver;
  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  // if an error occurs, stop the program.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    Serial.println("AllocateTensors() failed");
    while(true); // stop program here
  }
  // Obtain pointers to the model's input and output tensors.
  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.println("Initialization done.");
}

float scale_mean[] = {5.0, 948.63425926, 1314.4212963};
float scale_scale[] = { 2.5819889 , 489.5772747 , 389.81080799};

float predict_pos(float Ton, float v0, float v1)
{
  input->data.f[0] = (Ton - scale_mean[0]) / scale_scale[0];
  input->data.f[1] = (v0  - scale_mean[1]) / scale_scale[1];
  input->data.f[2] = (v1  - scale_mean[2]) / scale_scale[2];
  // Run inference, and report if an error occurs
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    Serial.println("Invoke failed!");
    while(1);
  }

  // Obtain the quantized output from model's output tensor

  return(output->data.f[0]);
}

void loop() {
  uint32_t t;
  Serial.println("----");
  t = millis();
  Serial.print(predict_pos(1.0, 479.0, 1129.0)); // 0mm
  Serial.print(' '); Serial.println(millis() - t); t = millis();
  Serial.println(predict_pos(2.0, 475.0, 1120.0)); // 0mm
  Serial.print(' '); Serial.println(millis() - t); t = millis();
  Serial.println(predict_pos(3.0, 478.0, 1119.0)); // 0mm
  Serial.print(' '); Serial.println(millis() - t); t = millis();
  Serial.println(predict_pos(3.0, 440.0, 988.0)); // 1mm
  Serial.print(' '); Serial.println(millis() - t); t = millis();

  delay(1000);
}
