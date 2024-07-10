#include <Arduino.h>
#include <M5Unified.h>

//#include <TensorFlowLite_ESP32.h>
//#include <TensorFlowLite.h>
//#include "tensorflow/lite/micro/all_ops_resolver.h"
//#include "tensorflow/lite/micro/micro_error_reporter.h"
//#include "tensorflow/lite/micro/micro_interpreter.h"
//#include "tensorflow/lite/micro/system_setup.h"
//#include "tensorflow/lite/schema/schema_generated.h"
//#include "model.h"  // Include the model header file

#include "model.h"  // Include the model header file

#include "tensorflow/lite/micro/micro_mutable_op_resolver.h" 
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"
#include "tensorflow/lite/schema/schema_generated.h"

// Position prediction using NN:
// https://chatgpt.com/share/ebc162db-142f-42d8-82a9-fdc115fb7b4f
// train https://colab.research.google.com/drive/1bI5E3nJ7u63hwzTXQJca03pLSfpk4v60?usp=sharing

/*
// Define the TensorFlow Lite Micro model
const tflite::Model* model = tflite::GetModel(pos_prediction_model_tflite);
static tflite::MicroInterpreter* interpreter;
static uint8_t tensor_arena[10 * 1024];  // Tensor arena
*/

#define RAM_SIZE 400000 // 400000 bytes - this is large because of this example model, but you can reduce this for smaller models
uint8_t tensor_arena[RAM_SIZE]; // where the model will be run

// Globally accessible interpreter
std::unique_ptr<tflite::MicroInterpreter> interpreter;

void setup() {
	M5.begin();
  /*
  // Set up the micro interpreter
  static tflite::MicroMutableOpResolver<10> resolver;
//  tflite::MicroErrorReporter micro_error_reporter;
  interpreter = new tflite::MicroInterpreter(model, resolver, tensor_arena, sizeof(tensor_arena), &micro_error_reporter);
//  interpreter = new tflite::MicroInterpreter(model, resolver, tensor_arena, sizeof(tensor_arena));

	static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, RAM_SIZE);
  interpreter = std::unique_ptr<tflite::MicroInterpreter>(&static_interpreter);


  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  printf("Model loaded and ready to execute.\n");
*/

	// set up the error reporter
	static tflite::MicroErrorReporter micro_error_reporter;
	tflite::ErrorReporter* error_reporter = &micro_error_reporter;

	// set up the model
	const tflite::Model* model = tflite::GetModel(pos_prediction_model_tflite);
	// check to make sure the model is compatible
	if (model->version() != TFLITE_SCHEMA_VERSION) {
		printf("Model provided is schema version %d not equal to supported version %d.\n", model->version(), TFLITE_SCHEMA_VERSION);
	}

  static tflite::MicroMutableOpResolver<10> resolver;

	static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, RAM_SIZE);
  interpreter = std::unique_ptr<tflite::MicroInterpreter>(&static_interpreter);

	// Allocate memory for the model's input buffers
	TfLiteStatus allocate_status = interpreter->AllocateTensors();
	if (allocate_status != kTfLiteOk)
	{
		TF_LITE_REPORT_ERROR(error_reporter, "Tensor allocation failed");
	}
	else
	{
		printf("Tensor allocation success, used bytes: %d\n", interpreter->arena_used_bytes());
	}

	// Obtain a pointer to the model's input tensor
	TfLiteTensor *input = interpreter->input(0);

	// Print out the input tensor's details to verify
	// the model is working as expected
	printf("Input size: %d, input bytes: %d\n", input->dims->size, input->bytes);

	for (int i = 0; i < input->dims->size; i++)
	{
		printf("Input dim %d: %d\n", i, input->dims->data[i]);
	}

	input->data.f[0] = 1.0;
	input->data.f[1] = 479.0;
	input->data.f[2] = 1129.0; // Pos = 0mm

	// Invoke the model
	TfLiteStatus invoke_status = interpreter->Invoke();

	if (invoke_status != kTfLiteOk)
	{
		TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed");
	}
	else
	{
		Serial.println("Invoke completed");
	}

	TfLiteTensor *output = interpreter->output(0);

	printf("Output size: %d, output bytes: %d\n", output->dims->size, output->bytes);

	for (int i = 0; i < output->dims->size; i++)
	{
		printf("Output dim %d: %d\n", i, output->dims->data[i]);
	}

  float* outputData = interpreter->output(0)->data.f;
  printf("%f\n", outputData[0]);


/*
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
*/
}

void loop() {
/*
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
*/
}
