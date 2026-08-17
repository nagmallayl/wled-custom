#include "wled.h"
#include "driver/rmt_rx.h"

#define PASSTHROUGH_INPUT_PIN 25
#define MAX_SYMBOLS 4096
#define MAX_LEDS 300

class PassthroughUsermod : public Usermod {

private:

  rmt_channel_handle_t rxChannel = nullptr;
  rmt_symbol_word_t *rxBuffer = nullptr;

  static bool onReceiveDone(
      rmt_channel_handle_t channel,
      const rmt_rx_done_event_data_t *edata,
      void *user_data) {

    PassthroughUsermod *self =
        static_cast<PassthroughUsermod *>(user_data);

    self->processFrame(
        edata->received_symbols,
        edata->num_symbols
    );

    return false;
  }

  void processFrame(
      const rmt_symbol_word_t *symbols,
      size_t count) {

    if (count < 24) return;

    uint8_t bytes[MAX_LEDS * 3];
    uint16_t byteCount = 0;

    uint8_t currentByte = 0;
    uint8_t bitCount = 0;

    for (size_t i = 0;
         i < count && byteCount < MAX_LEDS * 3;
         i++) {

      uint16_t high = symbols[i].duration0;
      uint16_t low  = symbols[i].duration1;

      uint32_t total = high + low;

      if (total < 5 || total > 40)
        continue;

      bool bit = high > low;

      currentByte <<= 1;

      if (bit)
        currentByte |= 1;

      bitCount++;

      if (bitCount == 8) {

        bytes[byteCount++] = currentByte;

        currentByte = 0;
        bitCount = 0;
      }
    }

    uint16_t ledCount = byteCount / 3;

    if (ledCount == 0)
      return;

    if (ledCount > strip.getLength())
      ledCount = strip.getLength();

    /*
     * Incoming order:
     *
     * GRB
     *
     * Output:
     *
     * RGB
     */

    for (uint16_t i = 0; i < ledCount; i++) {

      uint8_t g = bytes[i * 3 + 0];
      uint8_t r = bytes[i * 3 + 1];
      uint8_t b = bytes[i * 3 + 2];

      strip.setPixelColor(i, r, g, b);
    }

    strip.show();
  }

public:

  void setup() override {

    // GPIO25 input
    pinMode(PASSTHROUGH_INPUT_PIN, INPUT);

    // Diagnostic message
    Serial.println();
    Serial.println("================================");
    Serial.println("Passthrough Usermod STARTED");
    Serial.print("Input GPIO: ");
    Serial.println(PASSTHROUGH_INPUT_PIN);
    Serial.println("================================");

    // Allocate RMT receive buffer
    rxBuffer = (rmt_symbol_word_t *)heap_caps_malloc(
        MAX_SYMBOLS * sizeof(rmt_symbol_word_t),
        MALLOC_CAP_INTERNAL
    );

    if (!rxBuffer) {
      DEBUG_PRINTLN(
          F("Passthrough: RX buffer allocation failed")
      );
      return;
    }

    // RMT configuration
    rmt_rx_channel_config_t rxConfig = {};

    rxConfig.gpio_num =
        (gpio_num_t)PASSTHROUGH_INPUT_PIN;

    rxConfig.clk_src =
        RMT_CLK_SRC_DEFAULT;

    rxConfig.resolution_hz =
        10000000;

    rxConfig.mem_block_symbols =
        64;

    rxConfig.intr_priority =
        1;

    rxConfig.flags.invert_in =
        false;

    rxConfig.flags.with_dma =
        false;

    // Create RMT RX channel
    esp_err_t err =
        rmt_new_rx_channel(
            &rxConfig,
            &rxChannel
        );

    if (err != ESP_OK) {

      DEBUG_PRINTLN(
          F("Passthrough: RMT channel creation failed")
      );

      return;
    }

    // Register callback
    rmt_rx_event_callbacks_t callbacks = {};

    callbacks.on_recv_done =
        PassthroughUsermod::onReceiveDone;

    err =
        rmt_rx_register_event_callbacks(
            rxChannel,
            &callbacks,
            this
        );

    if (err != ESP_OK) {

      DEBUG_PRINTLN(
          F("Passthrough: callback registration failed")
      );

      return;
    }

    // Enable RMT
    err =
        rmt_enable(rxChannel);

    if (err != ESP_OK) {

      DEBUG_PRINTLN(
          F("Passthrough: RMT enable failed")
      );

      return;
    }

    // Receive configuration
    rmt_receive_config_t receiveConfig = {};

    receiveConfig.signal_range_min_ns =
        100;

    receiveConfig.signal_range_max_ns =
        100000;

    // Start receiving
    err =
        rmt_receive(
            rxChannel,
            rxBuffer,
            MAX_SYMBOLS *
                sizeof(rmt_symbol_word_t),
            &receiveConfig
        );

    if (err != ESP_OK) {

      DEBUG_PRINTLN(
          F("Passthrough: receive start failed")
      );

      return;
    }

    DEBUG_PRINTLN(
        F("Passthrough Usermod started")
    );
  }

  void loop() override {
    // Reception handled by RMT callback.
  }

  void addToJsonInfo(JsonObject &root) override {

    JsonObject info =
        root["u"].createNestedObject("Passthrough");

    info["input"] =
        PASSTHROUGH_INPUT_PIN;

    info["status"] =
        "WS2811 RX";
  }

  uint16_t getId() override {
    return 0x5041;
  }
};

static PassthroughUsermod passthroughUsermod;

REGISTER_USERMOD(passthroughUsermod);
