#include <Arduino.h>
#include <M5Unified.h>

//#include <TensorFlowLite_ESP32.h>
//#include <TensorFlowLite.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
//#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "model.h"  // Include the model header file

// Position prediction using NN:
// https://chatgpt.com/share/ebc162db-142f-42d8-82a9-fdc115fb7b4f
// train https://colab.research.google.com/drive/1bI5E3nJ7u63hwzTXQJca03pLSfpk4v60?usp=sharing

// Define the TensorFlow Lite Micro model
const tflite::Model* model = tflite::GetModel(pos_prediction_model_tflite);
static tflite::MicroInterpreter* interpreter;
static uint8_t tensor_arena[10 * 1024];  // Tensor arena

void setup() {
	M5.begin();
  
  // Set up the micro interpreter
  static tflite::MicroMutableOpResolver<10> resolver;
//  tflite::MicroErrorReporter micro_error_reporter;
//  interpreter = new tflite::MicroInterpreter(model, resolver, tensor_arena, sizeof(tensor_arena), &micro_error_reporter);
  interpreter = new tflite::MicroInterpreter(model, resolver, tensor_arena, sizeof(tensor_arena));

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  printf("Model loaded and ready to execute.\n");
}

float predic_position(float Ton, int v0, int v1){
  // Example input: {Ton[ms], v0, v1}
  float input[3] = {Ton, (float)v0, (float)v1};

  // Obtain the input tensor
  float* input_tensor = interpreter->input(0)->data.f;
  
  // Set the input tensor values
  for (uint8_t i = 0; i < 3; i++) input_tensor[i] = input[i];

  // Run inference
  interpreter->Invoke();

  // Obtain the output tensor
  float* output = interpreter->output(0)->data.f;
  return(output[0]);
}

void loop() {
  uint32_t t = millis();
  printf("%d %f\n", millis() - t, predic_position(1.0, 479, 1129));  // 0mm
  t = millis();
  printf("%d %f\n", millis() - t, predic_position(1.0, 361, 722));  // 4mm
  t = millis();
  printf("%d %f\n", millis() - t, predic_position(5.0, 574, 1180));  // 0mm
  t = millis();
  printf("%d %f\n", millis() - t, predic_position(5.0, 850, 1253));  // 2mm
  t = millis();
  delay(1000);
}
