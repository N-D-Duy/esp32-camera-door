#include <ESP32Servo.h>
#include <driver/i2s.h>
#include <controller.h>
#include <Arduino.h>

#define I2S_DOUT        25
#define I2S_BCLK        27
#define I2S_LRC         26
#define BUTTON_PIN      14
#define SERVO_PIN       18

constexpr unsigned long DEBOUNCE_DELAY = 1000;    
constexpr unsigned long RESET_RING_TIME = 20000;

volatile unsigned long lastPressTime = 0;
volatile int pressCount = 0;

Servo myServo;
Controller* controller = nullptr;
SemaphoreHandle_t buttonSemaphore = nullptr;

void initI2S();
void playTone(int frequency, int duration, int times);
void handleButtonPress();

void setup() {
    Serial.begin(115200);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    myServo.attach(SERVO_PIN);
    myServo.write(90);

    buttonSemaphore = xSemaphoreCreateMutex();
    if (buttonSemaphore == nullptr) {
        Serial.println("Failed to create semaphore!");
        return;
    }

    controller = new Controller();
    if (controller != nullptr) {
        controller->setup();
    }

    initI2S();
}

void loop() {
    static int lastButtonState = HIGH;
    int buttonState = digitalRead(BUTTON_PIN);

    if (buttonState != lastButtonState) {
        delay(50);
        buttonState = digitalRead(BUTTON_PIN);
        if (buttonState != lastButtonState) {
            if (buttonState == LOW) {
                handleButtonPress();
            }
            lastButtonState = buttonState;
        }
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastPressTime > RESET_RING_TIME) {

        if (controller != nullptr) {
            controller->setRingStatus(false);
        }
        pressCount = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}

void handleButtonPress() {
    unsigned long currentMillis = millis();

    if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {
        if (currentMillis - lastPressTime > DEBOUNCE_DELAY) {
            lastPressTime = currentMillis;

            if (pressCount == 0) {
                if (controller != nullptr) {
                    controller->setRingStatus(true);
                    Serial.println("Ring status updated on Firebase");
                }
                playTone(2000, 500, 1);
            } else {
                playTone(1500, 500, 1);
            }
            
            pressCount++;
        }

        xSemaphoreGive(buttonSemaphore);
    }
}

void initI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_LRC,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, nullptr);
    i2s_set_pin(I2S_NUM_0, &pin_config);
}

void playTone(int frequency, int duration, int times) {
    constexpr int SAMPLES = 64;
    int16_t* sample = (int16_t*)malloc(SAMPLES * sizeof(int16_t));

    if (sample == nullptr) {
        Serial.println("Failed to allocate memory for tone generation");
        return;
    }

    for (int i = 0; i < SAMPLES; i++) {
        sample[i] = 30000 * sin(2 * PI * frequency * i / 44100.0);
    }

    size_t bytes_written;
    for (int t = 0; t < times; t++) {
        unsigned long startMillis = millis();
        while (millis() - startMillis < duration) {
            i2s_write(I2S_NUM_0, sample, SAMPLES * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    free(sample);
}
