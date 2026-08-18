```cpp
#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// CONFIGURATION
// ======================================================

#define PASSTHROUGH_INPUT_PIN 25

// WLED controls the output GPIO.
// We do NOT drive GPIO16 ourselves.
#define PASSTHROUGH_OUTPUT_PIN 16

// ======================================================
// STRIP UNDER TEST
// ======================================================
//
// 15 physical LEDs
// Each WS2811 IC controls 3 physical LEDs
//
// 15 / 3 = 5 WS2811 ICs
//
// The incoming WS2811 frame therefore contains:
// 5 × 3 bytes = 15 bytes
// ======================================================

#define PHYSICAL_LEDS 15
#define PIXEL_COUNT   5
#define FRAME_BYTES   (PIXEL_COUNT * 3)
#define FRAME_BITS    (FRAME_BYTES * 8)

#define WS2811_KHZ 800

// ======================================================
// RMT
// ======================================================
//
// ESP32 classic:
// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns
//
// 50 us reset:
// 50 us / 25 ns = 2000 ticks
// ======================================================

#define RMT_CLK_DIV 2
#define RMT_IDLE_THRESHOLD 2000

#define RX_CHANNEL RMT_CHANNEL_0


// ======================================================
// USERMOD
// ======================================================

class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;

  // ====================================================
  // DOUBLE FRAME BUFFER
  // ====================================================
  //
  // RX writes into rxFrame.
  //
  // WLED reads from displayFrame.
  //
  // This prevents WLED from reading a frame while
  // the RMT decoder is modifying it.
  // ====================================================

  uint8_t rxFrame[FRAME_BYTES];
  uint8_t displayFrame[FRAME_BYTES];

  volatile bool newFrameReady = false;

  uint16_t frameBytes = 0;

  uint8_t currentByte = 0;
  uint8_t bitCount = 0;

  bool passthroughActive = false;

  // ====================================================
  // STATISTICS
  // ====================================================

  uint32_t framesReceived = 0;
  uint32_t framesDropped = 0;

  uint32_t lastFrameTime = 0;
  uint32_t lastDebug = 0;


  // ====================================================
  // RESET DECODER
  // ====================================================

  void resetDecoder()
  {
    frameBytes = 0;
    currentByte = 0;
    bitCount = 0;
  }


  // ====================================================
  // DECODE ONE BIT
  // ====================================================

  bool decodeBit(const rmt_item32_t &item)
  {
    uint16_t high = item.duration0;
    uint16_t low  = item.duration1;

    uint16_t total = high + low;

    // Valid WS2811 symbol window.
    if (total < 35 || total > 65)
      return false;

    // Measured from your actual signal:
    //
    // 0:
    // H ≈ 14-15
    // L ≈ 41-42
    //
    // 1:
    // H ≈ 50-51
    // L ≈ 24-25
    //
    // So HIGH > LOW is a reliable discriminator.

    return high > low;
  }


  // ====================================================
  // COMMIT COMPLETE FRAME
  // ====================================================

  void commitFrame()
  {
    if (
        frameBytes != FRAME_BYTES ||
        bitCount != 0
    ) {
      return;
    }

    // --------------------------------------------------
    // Copy the complete frame atomically from the RX
    // buffer into the display buffer.
    //
    // Only 15 bytes, so this is extremely fast.
    // --------------------------------------------------

    memcpy(
        displayFrame,
        rxFrame,
        FRAME_BYTES
    );

    newFrameReady = true;

    passthroughActive = true;

    lastFrameTime = millis();

    framesReceived++;
  }


  // ====================================================
  // PROCESS RMT SYMBOLS
  // ====================================================

  void processSymbols(
      rmt_item32_t *items,
      size_t count)
  {
    if (!items)
      return;


    for (
        size_t i = 0;
        i < count;
        i++
    ) {

      uint16_t high =
          items[i].duration0;

      uint16_t low =
          items[i].duration1;


      // Ignore empty RMT symbols.

      if (
          high == 0 &&
          low == 0
      ) {
        continue;
      }


      // =================================================
      // WS2811 RESET / FRAME BOUNDARY
      // =================================================

      if (
          low >= RMT_IDLE_THRESHOLD
      ) {

        /*
         * The RESET marks the end of the frame.
         */

        if (
            frameBytes == FRAME_BYTES &&
            bitCount == 0
        ) {

          commitFrame();

        }
        else if (
            frameBytes != 0 ||
            bitCount != 0
        ) {

          framesDropped++;
        }


        // Always resynchronize here.

        resetDecoder();

        continue;
      }


      // =================================================
      // INVALID SYMBOL
      // =================================================

      uint16_t total =
          high + low;


      if (
          total < 35 ||
          total > 65
      ) {

        /*
         * A bad symbol can destroy bit alignment.
         *
         * Immediately resynchronize.
         */

        resetDecoder();

        continue;
      }


      // =================================================
      // PROTECT AGAINST EXTRA DATA
      // =================================================

      if (
          frameBytes >= FRAME_BYTES
      ) {

        resetDecoder();

        continue;
      }


      // =================================================
      // DECODE BIT
      // =================================================

      bool bit =
          decodeBit(items[i]);


      currentByte <<= 1;


      if (bit)
        currentByte |= 1;


      bitCount++;


      // =================================================
      // COMPLETE BYTE
      // =================================================

      if (
          bitCount == 8
      ) {

        rxFrame[
            frameBytes
        ] = currentByte;


        frameBytes++;

        currentByte = 0;

        bitCount = 0;
      }
    }
  }


  // ====================================================
  // APPLY FRAME TO WLED
  // ====================================================
  //
  // IMPORTANT:
  //
  // NO strip.show() here.
  //
  // WLED calls handleOverlayDraw() immediately before
  // its own LED update.
  //
  // This lets WLED remain responsible for the actual
  // LED output timing.
  // ====================================================

  void applyFrameToWLED()
  {
    if (!newFrameReady)
      return;


    // --------------------------------------------------
    // Copy state locally.
    // --------------------------------------------------

    uint8_t localFrame[
        FRAME_BYTES
    ];


    memcpy(
        localFrame,
        displayFrame,
        FRAME_BYTES
    );


    // --------------------------------------------------
    // CURRENT TEST:
    //
    // Incoming data = GRB
    //
    // For now we are NOT changing the color order.
    //
    // G R B
    //
    // becomes temporary RGB:
    //
    // R = input R
    // G = input G
    // B = input B
    //
    // We will restore GRB -> BRG after the frame
    // update is stable.
    // --------------------------------------------------

    for (
        uint16_t i = 0;
        i < PIXEL_COUNT;
        i++
    ) {

      uint16_t p =
          i * 3;


      uint8_t inputG =
          localFrame[p + 0];

      uint8_t inputR =
          localFrame[p + 1];

      uint8_t inputB =
          localFrame[p + 2];


      // TEMPORARY RGB

      uint8_t outputR =
          inputR;

      uint8_t outputG =
          inputG;

      uint8_t outputB =
          inputB;


      strip.setPixelColor(
          i,
          outputR,
          outputG,
          outputB
      );
    }


    /*
     * Frame has now been copied into WLED's LED buffer.
     *
     * DO NOT CALL strip.show().
     *
     * WLED will perform the actual show().
     */

    newFrameReady = false;
  }


  // ====================================================
  // RMT RX SETUP
  // ====================================================

  bool setupRX()
  {
    rmt_config_t config = {};


    config.rmt_mode =
        RMT_MODE_RX;


    config.channel =
        RX_CHANNEL;


    config.gpio_num =
        (gpio_num_t)
        PASSTHROUGH_INPUT_PIN;


    config.clk_div =
        RMT_CLK_DIV;


    // Four RMT memory blocks.

    config.mem_block_num =
        4;


    config.flags = 0;


    // No filtering.

    config.rx_config.filter_en =
        false;


    // 50 us reset.

    config.rx_config.idle_threshold =
        RMT_IDLE_THRESHOLD;


    esp_err_t result;


    result =
        rmt_config(
            &config
        );


    if (
        result != ESP_OK
    ) {

      Serial.print(
          "RX config error: "
      );

      Serial.println(
          result
      );

      return false;
    }


    result =
        rmt_driver_install(
            RX_CHANNEL,
            8192,
            0
        );


    if (
        result != ESP_OK
    ) {

      Serial.print(
          "RX driver error: "
      );

      Serial.println(
          result
      );

      return false;
    }


    result =
        rmt_get_ringbuf_handle(
            RX_CHANNEL,
            &rxRingBuffer
        );


    if (
        result != ESP_OK ||
        rxRingBuffer == nullptr
    ) {

      Serial.println(
          "RX ring buffer error"
      );

      return false;
    }


    result =
        rmt_rx_start(
            RX_CHANNEL,
            true
        );


    if (
        result != ESP_OK
    ) {

      Serial.print(
          "RX start error: "
      );

      Serial.println(
          result
      );

      return false;
    }


    return true;
  }


public:

  // ====================================================
  // SETUP
  // ====================================================

  void setup() override
  {
    pinMode(
        PASSTHROUGH_INPUT_PIN,
        INPUT
    );


    memset(
        rxFrame,
        0,
        sizeof(rxFrame)
    );


    memset(
        displayFrame,
        0,
        sizeof(displayFrame)
    );


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
        "Output GPIO: WLED"
    );


    Serial.println(
        "RMT: LEGACY RX"
    );


    Serial.println(
        "Protocol: WS2811"
    );


    Serial.print(
        "Physical LEDs: "
    );

    Serial.println(
        PHYSICAL_LEDS
    );


    Serial.print(
        "WS2811 ICs: "
    );

    Serial.println(
        PIXEL_COUNT
    );


    Serial.print(
        "Frame bytes: "
    );

    Serial.println(
        FRAME_BYTES
    );


    Serial.print(
        "Frame bits: "
    );

    Serial.println(
        FRAME_BITS
    );


    Serial.println(
        "Speed: 800 kHz"
    );


    Serial.println(
        "Input: GRB"
    );


    Serial.println(
        "Output: TEMP RGB"
    );


    Serial.println(
        "RMT: RX ONLY"
    );


    Serial.println(
        "Output: WLED LED ENGINE"
    );


    Serial.println(
        "Double Buffer: ENABLED"
    );


    Serial.println(
        "================================"
    );


    if (
        setupRX()
    ) {

      rxReady = true;

      Serial.println(
          "RMT RX READY"
      );

    }
    else {

      Serial.println(
          "RMT RX FAILED"
      );

      return;
    }


    Serial.println(
        "Passthrough READY"
    );


    Serial.println(
        "Waiting for WS2811 DATA..."
    );


    Serial.println(
        "================================"
    );
  }


  // ====================================================
  // WLED OVERLAY
  // ====================================================
  //
  // WLED invokes this immediately before its own
  // strip update.
  //
  // This is the correct place to overwrite the frame.
  // ====================================================

  void handleOverlayDraw() override
  {
    if (
        !passthroughActive
    ) {
      return;
    }


    applyFrameToWLED();
  }


  // ====================================================
  // MAIN LOOP
  // ====================================================

  void loop() override
  {
    if (
        !rxReady ||
        rxRingBuffer == nullptr
    ) {
      return;
    }


    // --------------------------------------------------
    // NON-BLOCKING RMT RECEIVE
    // --------------------------------------------------

    size_t receivedSize = 0;


    rmt_item32_t *items =
        (rmt_item32_t *)
        xRingbufferReceive(
            rxRingBuffer,
            &receivedSize,
            0
        );


    if (items) {

      size_t itemCount =
          receivedSize /
          sizeof(rmt_item32_t);


      processSymbols(
          items,
          itemCount
      );


      // Return RMT memory immediately.

      vRingbufferReturnItem(
          rxRingBuffer,
          (void *)items
      );
    }


    // --------------------------------------------------
    // AUTO TIMEOUT
    // --------------------------------------------------

    if (
        passthroughActive
    ) {

      uint32_t now =
          millis();


      if (
          now - lastFrameTime >
          300
      ) {

        passthroughActive =
            false;


        newFrameReady =
            false;


        resetDecoder();


        Serial.println(
            "Passthrough signal lost"
        );


        Serial.println(
            "WLED effects resumed"
        );
      }
    }


    // --------------------------------------------------
    // DEBUG
    // --------------------------------------------------

    uint32_t now =
        millis();


    if (
        now - lastDebug >= 1000
    ) {

      lastDebug =
          now;


      Serial.print(
          "Frames: "
      );

      Serial.print(
          framesReceived
      );


      Serial.print(
          "    Dropped: "
      );

      Serial.print(
          framesDropped
      );


      Serial.print(
          "    NewFrame: "
      );

      Serial.print(
          newFrameReady ?
          "YES" :
          "NO"
      );


      Serial.print(
          "    Mode: "
      );


      if (
          passthroughActive
      ) {

        Serial.println(
            "PASSTHROUGH"
        );

      }
      else {

        Serial.println(
            "WLED"
        );
      }
    }
  }


  // ====================================================
  // WLED INFO
  // ====================================================

  void addToJsonInfo(
      JsonObject &root
  ) override
  {
    JsonObject info =
        root["u"]
        .createNestedObject(
            "Passthrough"
        );


    info["input"] =
        PASSTHROUGH_INPUT_PIN;

    info["output"] =
        "WLED";

    info["physical_leds"] =
        PHYSICAL_LEDS;

    info["ws2811_ics"] =
        PIXEL_COUNT;

    info["frame_bytes"] =
        FRAME_BYTES;

    info["frame_bits"] =
        FRAME_BITS;

    info["speed"] =
        "800 kHz";

    info["rmt"] =
        "LEGACY RX";

    info["double_buffer"] =
        true;

    info["frames"] =
        framesReceived;

    info["dropped"] =
        framesDropped;
  }


  uint16_t getId() override
  {
    return 0x5041;
  }
};


// ======================================================
// REGISTER USERMOD
// ======================================================

static PassthroughUsermod passthroughUsermod;

REGISTER_USERMOD(
    passthroughUsermod
);

