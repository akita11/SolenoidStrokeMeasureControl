#include <Arduino.h>
#include <M5Unified.h>

#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "model.h"  // Include the model header file

// Define the TensorFlow Lite Micro model
const tflite::Model* model = tflite::GetModel(pos_prediction_model_tflite);
static tflite::MicroInterpreter* interpreter;
static uint8_t tensor_arena[10 * 1024];  // Tensor arena

void setup() {
	M5.begin();
  
  // Set up the micro interpreter
  static tflite::MicroMutableOpResolver<10> resolver;
  tflite::MicroErrorReporter micro_error_reporter;
  interpreter = new tflite::MicroInterpreter(model, resolver, tensor_arena, sizeof(tensor_arena), &micro_error_reporter);

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  printf("Model loaded and ready to execute.\n");
}

void loop() {
  // Example input: {Ton[ms], v0, v1}
  float input[3] = {1.0, 479.0, 1129.0};  // Replace with actual input values

  // Obtain the input tensor
  float* input_tensor = interpreter->input(0)->data.f;
  
  // Set the input tensor values
  for (int i = 0; i < 3; i++) {
    input_tensor[i] = input[i];
  }

  // Run inference
  interpreter->Invoke();

  // Obtain the output tensor
  float* output = interpreter->output(0)->data.f;

  // Print the predicted position
  printf("Predicted Pos[mm]: %f\n", output[0]);

  // Wait for a while before running inference again
  delay(1000);
}
