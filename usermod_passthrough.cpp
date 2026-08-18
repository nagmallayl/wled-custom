#include "wled.h"
#include "driver/rmt.h"

#define INPUT_GPIO 25
#define RX_CHANNEL RMT_CHANNEL_0

class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;

  uint32_t lastDump = 0;

  bool gotSample = false;

  void setupRMT()
  {
    rmt_config_t config = {};

    config.rmt_mode = RMT_MODE_RX;
    config.channel = RX_CHANNEL;
    config.gpio_num = (gpio_num_t)INPUT_GPIO;

    /*
     * 80 MHz / 2 = 40 MHz
     * 1 tick = 25 ns
     */
    config.clk_div = 2;

    config.mem_block_num = 1;

    config.flags = 0;

    /*
     * No filter.
     */
    config.rx_config.filter_en = false;

    /*
     * Stop RX after a long idle.
     * This lets us capture one WS2811 frame.
     */
    config.rx_config.idle_threshold = 3000;

    esp_err_t err;

    err = rmt_config(&config);

    if (err != ESP_OK) {

      Serial.print("RMT CONFIG ERROR: ");
      Serial.println(err);

      return;
    }

    err = rmt_driver_install(
      RX_CHANNEL,
      2048,
      0
    );

    if (err != ESP_OK) {

      Serial.print("RMT DRIVER ERROR: ");
      Serial.println(err);

      return;
    }

    err = rmt_get_ringbuf_handle(
      RX_CHANNEL,
      &rxRingBuffer
    );

    if (
      err != ESP_OK ||
      rxRingBuffer == nullptr
    ) {

      Serial.println(
        "RMT RINGBUFFER ERROR"
      );

      return;
    }

    err = rmt_rx_start(
      RX_CHANNEL,
      true
    );

    if (err != ESP_OK) {

      Serial.print(
        "RMT START ERROR: "
      );

      Serial.println(err);

      return;
    }

    rxReady = true;

    Serial.println(
      "RMT DIAGNOSTIC RX READY"
    );

    Serial.println(
      "Waiting for WS2811 signal..."
    );
  }


public:

  void setup() override
  {
    pinMode(
      INPUT_GPIO,
      INPUT
    );

    Serial.println();
    Serial.println(
      "================================"
    );

    Serial.println(
      "PASSTHROUGH RMT DIAGNOSTIC"
    );

    Serial.print(
      "Input GPIO: "
    );

    Serial.println(
      INPUT_GPIO
    );

    Serial.println(
      "Protocol: WS2811"
    );

    Serial.println(
      "Speed: 800 kHz"
    );

    Serial.println(
      "RMT clock: 40 MHz"
    );

    Serial.println(
      "1 tick = 25 ns"
    );

    Serial.println(
      "================================"
    );

    setupRMT();
  }


  void loop() override
  {
    if (
      !rxReady ||
      rxRingBuffer == nullptr
    ) {
      return;
    }

    /*
     * Only capture once every 500 ms.
     *
     * This prevents the diagnostic code from
     * flooding Serial or consuming WLED CPU time.
     */

    if (
      millis() - lastDump < 500
    ) {
      return;
    }

    size_t receivedSize = 0;

    rmt_item32_t *items =
      (rmt_item32_t *)
      xRingbufferReceive(
        rxRingBuffer,
        &receivedSize,
        0
      );

    if (!items) {
      return;
    }

    lastDump = millis();

    size_t count =
      receivedSize /
      sizeof(rmt_item32_t);

    Serial.println();
    Serial.println(
      "========== RMT SAMPLE =========="
    );

    Serial.print(
      "Bytes received: "
    );

    Serial.println(
      receivedSize
    );

    Serial.print(
      "RMT symbols: "
    );

    Serial.println(
      count
    );

    /*
     * Print ONLY the first 40 symbols.
     */

    size_t printCount =
      count > 40 ? 40 : count;

    for (
      size_t i = 0;
      i < printCount;
      i++
    ) {

      uint16_t h =
        items[i].duration0;

      uint16_t l =
        items[i].duration1;

      uint32_t total =
        h + l;

      Serial.print(
        "#"
      );

      Serial.print(i);

      Serial.print(
        "  H="
      );

      Serial.print(h);

      Serial.print(
        "  L="
      );

      Serial.print(l);

      Serial.print(
        "  T="
      );

      Serial.println(total);
    }

    Serial.println(
      "================================"
    );

    /*
     * VERY IMPORTANT:
     * Always return the RMT item.
     */

    vRingbufferReturnItem(
      rxRingBuffer,
      (void *)items
    );

    /*
     * Stop after one useful sample.
     *
     * This prevents continuous RMT processing.
     */

    if (!gotSample) {

      gotSample = true;

      Serial.println();
      Serial.println(
        "RMT SAMPLE CAPTURED."
      );

      Serial.println(
        "Diagnostic mode will remain idle."
      );
    }
  }


  void addToJsonInfo(
    JsonObject &root
  ) override
  {
    JsonObject info =
      root["u"]
        .createNestedObject(
          "Passthrough Diagnostic"
        );

    info["input"] =
      INPUT_GPIO;

    info["protocol"] =
      "WS2811";

    info["speed"] =
      "800 kHz";

    info["mode"] =
      "RMT timing diagnostic";
  }


  uint16_t getId() override
  {
    return 0x5042;
  }
};


static PassthroughUsermod passthroughUsermod;

REGISTER_USERMOD(
  passthroughUsermod
);
