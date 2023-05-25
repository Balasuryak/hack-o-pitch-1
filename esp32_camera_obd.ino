/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ---------------------------------------------------------------- */
#include <EnvisionIoT_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include <PubSubClient.h>
#include <base64.h>
#include <ArduinoJson.h>

// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

const char* apiKey = "979ded68289118d014fb3e7471bf5ea6";
// Configure MQTT Broker connection
const char* mqtt_server = "mqtt.flespi.io";
const int mqtt_port = 1883;
const char* mqtt_user = "MMZOJkKnP5RzvVdyfyw0qGpNy3u4cSqMvdr2nZf1UVqfEsBJQXz5rNIPE2JEJXh2";
const char* mqtt_password = "";
const char* id="ADfar34$";

//topic name
const char* send_topic = "envision/ADfar34$/status";
const char* recv_topic= "envision/ADfar34$/request";
const char* wifi_ssid = "Galaxy M219709";
const char* wifi_password = "qwerty@2";
int captureInterval = 60000; // Default capture interval in milliseconds
unsigned long lastCaptureTime = 0;
const char* location;
int app1_mode;
int app2_mode;
int app3_mode;
int app4_mode;
float app1_value;
int app1_smart_time;
float app2_value;
int app2_smart_time;
float app3_value;
int app3_smart_time;  
float app4_value;
int app4_smart_time;
int presence=0;
int presence_count=0;
int app1_last_time;
int app2_last_time;
int app3_last_time;
int app4_last_time;
#define R1 12
#define R2 13
//#define R3 13
//#define R4 14
//#define R5 16

//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#else
#error "Camera model not selected"
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

WiFiClient mqttClient;
PubSubClient client(mqttClient);

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;



void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address : ");
  Serial.println(WiFi.localIP());
}


void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("esp32client", mqtt_user, mqtt_password)) {
      client.subscribe(recv_topic);
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void onMessage (char* topic, byte* message, unsigned int length) {
  StaticJsonDocument<200> jsonBuffer;
  Serial.print(topic);
  Serial.print(" parameter as: ");
  deserializeJson(jsonBuffer, message,length);  
  captureInterval = jsonBuffer["captureInterval"].as<int>();  
  location=jsonBuffer["location"];
  app1_mode=jsonBuffer["app1_mode"].as<int>();
  app2_mode=jsonBuffer["app2_mode"].as<int>();
  app3_mode=jsonBuffer["app3_mode"].as<int>();
  app4_mode=jsonBuffer["app4_mode"].as<int>();  
  if(app1_mode==0){app1_value=jsonBuffer["app1_value"];}
  else{app1_smart_time=jsonBuffer["app1_smart_time"];}
  if(app2_mode==0){app2_value=jsonBuffer["app2_value"];}
  else{app2_smart_time=jsonBuffer["app2_smart_time"];}
  if(app3_mode==0){app3_value=jsonBuffer["app3_value"];}
  else{app1_smart_time=jsonBuffer["app3_smart_time"];}
  if(app4_mode==0){app4_value=jsonBuffer["app4_value"];}
  else{app1_smart_time=jsonBuffer["app4_smart_time"];} 
  if(captureInterval<10000){
    captureInterval=10000;
  }
  Serial.println(captureInterval) ;
  Serial.print("app1 mode: ");
  Serial.print("app1_smart_time");
  Serial.print(app1_smart_time);
  Serial.println(app1_mode);
  Serial.print("app2 mode: ");
  Serial.println(app2_mode) ;  
  Serial.print("app1 value : ");
  Serial.println(app1_value) ;
  Serial.print("app2 value: ");
  Serial.println(app2_value) ;
  Serial.print("location: ");
  Serial.print(location) ;
  Serial.println();
  app1_control();
  app2_control();
}

//control

void app1_control() {

  if (app1_mode == 1) {
          presenceDetection();
    if ((presence/presence_count) >= 0.5) {

      digitalWrite(R1, 0);
      Serial.println("Smartly turned light1 on");
    }
    else {
      digitalWrite(R1, 1);
      Serial.println("Smartly turned light1 off");
    }
  }
  else {
    if (app1_value == 1) {
      digitalWrite(R1, 0);
      Serial.println("Manually turned light1 on");
    }
    else {
      digitalWrite(R1, 1);
      Serial.println("Manually turned light1 off");
    }
  }
  return;
}
void app2_control() {
  if ( app2_mode == 1 ) {
    presenceDetection();
    if ((presence/presence_count) >=0.5) {
      digitalWrite(R2, 0);
      Serial.println("Smartly turned light2 on");
    }
    else {
      digitalWrite(R2, 1);
      Serial.println("Smartly turned light2 off");
    }
  }
  else {
    if (app2_value == 1) {
      digitalWrite(R2, 0);
      Serial.println("Manually turned light2 on");
    }
    else {
      digitalWrite(R2, 1);
      Serial.println("Manually turned light2 off");
    }

  }
  return;
}
//void app3_control() {
//  if ( app3_mode ) {
//    if (presence == 1) {
//      digitalWrite(R3, 0);
//    }
//    else {
//      digitalWrite(R3, 1);
//    }
//  }
//  else {
//    if (app3_value == 1) {
//      digitalWrite(R3, 0);
//      Serial.println("Turned light2 on");
//    }
//    else {
//      digitalWrite(R3, 1);
//    }
//
//  }
//  return;
//}
//void app4_control() {
//  if ( app4_mode ) {
//    if (presence == 1) {
//      digitalWrite(R4, 0);
//    }
//    else {
//      digitalWrite(R4, 1);
//    }
//  }
//  else {
//    if (app4_value == 1) {
//      digitalWrite(R4, 0);
//      Serial.println("Turned fan on");
//    }
//    else {
//      digitalWrite(R4, 1);
//    }
//
//  }
//  return;
//}
void presenceDetection(){
  
    presence_count++;
      // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    // check if allocation was successful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    bool bb_found = result.bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
        auto bb = result.bounding_boxes[ix];
        if (bb.value == 0) {
            continue;
        }
        presence++;
        Serial.print(presence);
        Serial.print(" ");
        Serial.println(presence_count);
        ei_printf("    %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }
    if (!bb_found) {
        ei_printf("No objects found\n");
    }
#else
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label,
                                    result.classification[ix].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif


    free(snapshot_buf);
    return;
}
/**
* @brief      Arduino setup function
*/
void setup()
{   
     

    // put your setup code here, to run once:
    Serial.begin(115200);
    //comment out the below line to start inference immediately after upload
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");
    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    }
    else {
        ei_printf("Camera initialized\r\n");
    }
    
    ei_printf("\nStarting continious inference in 2 seconds...\n");
    ei_sleep(2000);
    setup_wifi();
    pinMode(R1, OUTPUT);
    pinMode(R2, OUTPUT);
    digitalWrite(R1, 1);
    digitalWrite(R2, 1);
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(onMessage);
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/


void loop()
{
    if (!client.connected()) {
          reconnect();
        }

    client.loop();
    if(app1_mode|| app2_mode){
  presenceDetection();
  if(presence_count>=10000){
    presence=1;
    presence_count=1;
    }
    }
  if ((app1_smart_time>0) && (millis() - app1_last_time >= app1_smart_time)) {
    app1_last_time = millis();
    app1_control();
    presence=0;
    presence_count=0;
  }
  if ((app2_smart_time>0) && (millis() - app2_last_time >= app2_smart_time)) {
    app2_last_time = millis();
    app2_control();
    presence=0;
    presence_count=0;
  }
//  if ((app3_smart_time>0) && (millis() - app3_last_time >= app3_smart_time)) {
//    app3_last_time = millis();
//    app3_control();
//  }
//  if ((app4_smart_time>0) && (millis() - app4_last_time >= app4_smart_time)) {
//    app4_last_time = millis();
//    app4_control();
//  }

}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;



    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, 0); // lower the saturation
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
    
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }


    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
