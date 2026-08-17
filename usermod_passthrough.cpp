#include "wled.h"
#include "driver/rmt.h"

#define PASSTHROUGH_INPUT_PIN 25

#define PASSTHROUGH_RMT_CHANNEL RMT_CHANNEL_0

#define MAX_LEDS 300

/*
 * RMT clock:
 *
 * ESP32 APB clock = 80 MHz
 * clk_div = 2
 *
 * RMT clock = 40 MHz
 * 1 tick = 25 ns
 *
 * WS2811 bit ≈ 1.25 us
 * ≈ 50 ticks
 */

#define RMT_CLK_DIV 2

/*
 * WS2811 reset time.
 *
 * 2000 ticks × 25 ns
 * = 50 us
 */

#define RMT_IDLE_THRESHOLD 2000

/*
 * Ignore pulses shorter than:
 *
 * 4 ticks × 25 ns
 * = 100 ns
 */

#define RMT_FILTER_THRESHOLD 4


class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rmtRingBuffer = nullptr;

  bool rmtReady = false;

  uint32_t framesReceived = 0;
  uint32_t lastDebugTime = 0;


  /*
   * Decode one WS2811 bit.
   *
   * Logical 0:
   *
   * HIGH ≈ 0.4 us
   * LOW  ≈ 0.85 us
   *
   * Logical 1:
   *
   * HIGH ≈ 0.8 us
   * LOW  ≈ 0.45 us
   *
   * Therefore:
   *
   * HIGH > LOW = 1
   * HIGH < LOW = 0
   */

  bool decodeBit(
      const rmt_item32_t &item
  ) {

    uint16_t high =
        item.duration0;

    uint16_t low =
        item.duration1;

    uint16_t total =
        high + low;


    /*
     * Expected WS2811 bit:
     *
     * approximately 50 RMT ticks.
     *
     * Allow a reasonable tolerance.
     */

    if (total < 35 ||
        total > 65) {

      return false;
    }


    return high > low;
  }


  /*
   * Decode one complete WS2811 frame.
   */

  void processFrame(
      rmt_item32_t *items,
      size_t itemCount
  ) {

    if (!items)
      return;

    if (itemCount < 24)
      return;


    /*
     * Maximum:
     *
     * 300 LEDs × 3 bytes
     */

    uint8_t bytes[
        MAX_LEDS * 3
    ];


    uint16_t byteCount = 0;

    uint8_t currentByte = 0;

    uint8_t bitCount = 0;


    /*
     * Decode every RMT item.
     */

    for (
        size_t i = 0;
        i < itemCount &&
        byteCount < MAX_LEDS * 3;
        i++
    ) {

      rmt_item32_t &item =
          items[i];


      uint16_t high =
          item.duration0;

      uint16_t low =
          item.duration1;


      /*
       * Ignore empty items.
       */

      if (high == 0 &&
          low == 0) {

        continue;
      }


      uint16_t total =
          high + low;


      /*
       * Ignore anything that does not
       * look like a WS2811 bit.
       */

      if (total < 35 ||
          total > 65) {

        continue;
      }


      bool bit =
          decodeBit(item);


      currentByte <<= 1;


      if (bit)
        currentByte |= 1;


      bitCount++;


      /*
       * Complete byte.
       */

      if (bitCount == 8) {

        bytes[byteCount] =
            currentByte;

        byteCount++;

        currentByte = 0;

        bitCount = 0;
      }
    }


    /*
     * Need at least one RGB LED.
     */

    if (byteCount < 3)
      return;


    /*
     * 3 bytes per LED.
     */

    uint16_t ledCount =
        byteCount / 3;


    if (ledCount > MAX_LEDS)
      ledCount = MAX_LEDS;


    /*
     * Never write beyond the
     * WLED configured strip length.
     */

    if (
        ledCount >
        strip.getLength()
    ) {

      ledCount =
          strip.getLength();
    }


    /*
     * Incoming data:
     *
     * GRB
     *
     * G = byte 0
     * R = byte 1
     * B = byte 2
     *
     * WLED output:
     *
     * RGB
     */

    for (
        uint16_t i = 0;
        i < ledCount;
        i++
    ) {

      uint8_t g =
          bytes[i * 3 + 0];

      uint8_t r =
          bytes[i * 3 + 1];

      uint8_t b =
          bytes[i * 3 + 2];


      strip.setPixelColor(
          i,
          r,
          g,
          b
      );
    }


    /*
     * Send the converted data
     * to the LED strip.
     */

    strip.show();


    framesReceived++;


    /*
     * Diagnostic information.
     *
     * Print once per second.
     */

    uint32_t now =
        millis();


    if (
        now - lastDebugTime >=
        1000
    ) {

      lastDebugTime = now;


      Serial.println();

      Serial.println(
          "----- Passthrough RX -----"
      );


      Serial.print(
          "Frames: "
      );

      Serial.println(
          framesReceived
      );


      Serial.print(
          "RMT symbols: "
      );

      Serial.println(
          itemCount
      );


      Serial.print(
          "Bytes: "
      );

      Serial.println(
          byteCount
      );


      Serial.print(
          "LEDs detected: "
      );

      Serial.println(
          ledCount
      );


      Serial.println(
          "--------------------------"
      );
    }
  }


public:


  /*
   * WLED Usermod setup.
   */

  void setup() override {

    /*
     * GPIO25 as digital input.
     */

    pinMode(
        PASSTHROUGH_INPUT_PIN,
        INPUT
    );


    /*
     * Startup message.
     */

    Serial.println();

    Serial.println(
        "================================"
    );

    Serial.println(
        "Passthrough Usermod STARTED"
    );

    Serial.print(
        "Input GPIO: "
    );

    Serial.println(
        PASSTHROUGH_INPUT_PIN
    );

    Serial.println(
        "RMT: LEGACY RX"
    );

    Serial.println(
        "Protocol: WS2811"
    );

    Serial.println(
        "Input order: GRB"
    );

    Serial.println(
        "Output order: RGB"
    );

    Serial.println(
        "================================"
    );


    /*
     * RMT configuration.
     */

    rmt_config_t config = {};


    config.rmt_mode =
        RMT_MODE_RX;


    config.channel =
        PASSTHROUGH_RMT_CHANNEL;


    config.gpio_num =
        (gpio_num_t)
        PASSTHROUGH_INPUT_PIN;


    /*
     * 80 MHz / 2
     * = 40 MHz
     */

    config.clk_div =
        RMT_CLK_DIV;


    /*
     * ESP32 has 8 RMT memory blocks.
     *
     * Use all 8 for receiving
     * longer LED frames.
     */

    config.mem_block_num = 8;


    config.flags = 0;


    /*
     * Enable input filter.
     */

    config.rx_config.filter_en =
        true;


    config.rx_config
        .filter_ticks_thresh =
        RMT_FILTER_THRESHOLD;


    /*
     * A WS2811 reset is normally
     * longer than 50 us.
     */

    config.rx_config
        .idle_threshold =
        RMT_IDLE_THRESHOLD;


    /*
     * Configure RMT peripheral.
     */

    esp_err_t result =
        rmt_config(
            &config
        );


    if (result != ESP_OK) {

      Serial.print(
          "ERROR: rmt_config = "
      );

      Serial.println(
          result
      );

      return;
    }


    /*
     * Install RMT driver.
     *
     * 8192 bytes ring buffer.
     */

    result =
        rmt_driver_install(
            PASSTHROUGH_RMT_CHANNEL,
            8192,
            0
        );


    if (result != ESP_OK) {

      Serial.print(
          "ERROR: rmt_driver_install = "
      );

      Serial.println(
          result
      );

      return;
    }


    /*
     * Get the ring buffer handle.
     */

    result =
        rmt_get_ringbuf_handle(
            PASSTHROUGH_RMT_CHANNEL,
            &rmtRingBuffer
        );


    if (
        result != ESP_OK ||
        rmtRingBuffer == nullptr
    ) {

      Serial.print(
          "ERROR: RMT ring buffer = "
      );

      Serial.println(
          result
      );

      return;
    }


    /*
     * Start RMT receiver.
     */

    result =
        rmt_rx_start(
            PASSTHROUGH_RMT_CHANNEL,
            true
        );


    if (result != ESP_OK) {

      Serial.print(
          "ERROR: rmt_rx_start = "
      );

      Serial.println(
          result
      );

      return;
    }


    rmtReady = true;


    Serial.println(
        "RMT RX READY"
    );

    Serial.println(
        "Waiting for WS2811 DATA on GPIO25..."
    );

    Serial.println(
        "================================"
    );
  }


  /*
   * WLED main loop.
   */

  void loop() override {

    if (!rmtReady)
      return;


    if (!rmtRingBuffer)
      return;


    size_t receivedSize = 0;


    /*
     * Do NOT block WLED.
     *
     * Timeout = 0.
     */

    rmt_item32_t *items =
        (rmt_item32_t *)
        xRingbufferReceive(
            rmtRingBuffer,
            &receivedSize,
            0
        );


    if (!items)
      return;


    /*
     * Calculate number of
     * RMT symbols received.
     */

    size_t itemCount =
        receivedSize /
        sizeof(rmt_item32_t);


    /*
     * Decode frame.
     */

    processFrame(
        items,
        itemCount
    );


    /*
     * Return buffer to RMT.
     */

    vRingbufferReturnItem(
        rmtRingBuffer,
        (void *)items
    );
  }


  /*
   * Show Usermod information
   * in WLED Info page.
   */

  void addToJsonInfo(
      JsonObject &root
  ) override {

    JsonObject info =
        root["u"]
        .createNestedObject(
            "Passthrough"
        );


    info["input"] =
        PASSTHROUGH_INPUT_PIN;


    info["status"] =
        "WS2811 RX";


    info["rmt"] =
        "Legacy";


    info["frames"] =
        framesReceived;
  }


  /*
   * Unique Usermod ID.
   */

  uint16_t getId() override {

    return 0x5041;
  }
};


/*
 * Create Usermod instance.
 */

static PassthroughUsermod
passthroughUsermod;


/*
 * Register with WLED.
 */

REGISTER_USERMOD(
    passthroughUsermod
);
