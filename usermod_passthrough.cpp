#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// CONFIG
// ======================================================

#define INPUT_GPIO 25

#define RX_CHANNEL RMT_CHANNEL_0

// ESP32 classic
// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns
#define RMT_CLK_DIV 2

// ======================================================
// WS2811
// ======================================================

// 5 WS2811 ICs
// Each IC controls 3 physical LEDs
// Total physical LEDs = 15
#define WS2811_ICS 5

#define FRAME_BYTES (WS2811_ICS * 3)
#define FRAME_BITS  (FRAME_BYTES * 8)

// ======================================================
// TIMING
// ======================================================

// Measured:
// 0 = H 14~15 / L 41~42
// 1 = H 50~51 / L 24~25
//
// Therefore HIGH > LOW is reliable.

#define BIT_THRESHOLD 30

// 50 us @ 40 MHz
#define RESET_LOW_TICKS 2000

// ======================================================
// USERMOD
// ======================================================

class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;

  // ====================================================
  // RX FRAME
  // ====================================================

  uint8_t rxFrame[FRAME_BYTES];

  uint8_t currentByte = 0;

  uint8_t bitCount = 0;

  uint8_t frameBytes = 0;

  // ====================================================
  // DISPLAY FRAME
  // ====================================================

  uint8_t displayFrame[FRAME_BYTES];

  volatile bool newFrameReady = false;

  // ====================================================
  // MODE
  // ====================================================

  bool passthroughActive = false;

  // ====================================================
  // STATISTICS
  // ====================================================

  uint32_t framesReceived = 0;

  uint32_t framesApplied = 0;

  uint32_t framesDropped = 0;

  uint32_t lastFrameTime = 0;

  uint32_t lastDebugTime = 0;


  // ====================================================
  // RESET DECODER
  // ====================================================

  void resetDecoder()
  {
    currentByte = 0;
    bitCount = 0;
    frameBytes = 0;
  }


  // ====================================================
  // DECODE ONE BIT
  // ====================================================

  bool decodeBit(
      const rmt_item32_t &item,
      bool &bit)
  {
    uint16_t high = item.duration0;
    uint16_t low  = item.duration1;

    uint16_t total =
        high + low;


    // -----------------------------------------------
    // Valid WS2811 symbol
    // -----------------------------------------------

    if (
        total < 35 ||
        total > 65
    ) {
      return false;
    }


    // -----------------------------------------------
    // Measured timing
    // -----------------------------------------------

    bit =
        high >= BIT_THRESHOLD;


    return true;
  }


  // ====================================================
  // COMMIT FRAME
  // ====================================================

  void commitFrame()
  {
    if (
        frameBytes != FRAME_BYTES ||
        bitCount != 0
    ) {
      return;
    }


    // -----------------------------------------------
    // Copy complete frame
    // -----------------------------------------------

    memcpy(
        displayFrame,
        rxFrame,
        FRAME_BYTES
    );


    // -----------------------------------------------
    // Signal WLED overlay
    // -----------------------------------------------

    newFrameReady = true;

    passthroughActive = true;

    lastFrameTime = millis();

    framesReceived++;
  }


  // ====================================================
  // PROCESS RMT
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


      // ---------------------------------------------
      // Ignore empty symbols
      // ---------------------------------------------

      if (
          high == 0 &&
          low == 0
      ) {
        continue;
      }


      // ---------------------------------------------
      // RESET / FRAME BOUNDARY
      // ---------------------------------------------

      if (
          low >= RESET_LOW_TICKS
      ) {

        /*
         * WS2811 reset.
         *
         * If exactly 120 bits were received,
         * commit the frame.
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


        // Always resynchronize
        resetDecoder();

        continue;
      }


      // ---------------------------------------------
      // Validate symbol timing
      // ---------------------------------------------

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


      // ---------------------------------------------
      // Protect frame size
      // ---------------------------------------------

      if (
          frameBytes >= FRAME_BYTES
      ) {

        resetDecoder();

        continue;
      }


      // ---------------------------------------------
      // Decode bit
      // ---------------------------------------------

      bool bit = false;


      if (
          !decodeBit(
              items[i],
              bit
          )
      ) {

        resetDecoder();

        continue;
      }


      // ---------------------------------------------
      // Shift bit into byte
      // ---------------------------------------------

      currentByte <<= 1;


      if (bit)
        currentByte |= 1;


      bitCount++;


      // ---------------------------------------------
      // Complete byte
      // ---------------------------------------------

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

  void applyFrameToWLED()
  {
    if (!newFrameReady)
      return;


    // -----------------------------------------------
    // Local copy
    // -----------------------------------------------

    uint8_t localFrame[
        FRAME_BYTES
    ];


    memcpy(
        localFrame,
        displayFrame,
        FRAME_BYTES
    );


    // =================================================
    // TEMPORARY COLOR TEST
    // =================================================
    //
    // Incoming:
    //
    // G R B
    //
    // WLED receives:
    //
    // R G B
    //
    // NO BRG conversion yet.
    //
    // =================================================

    for (
        uint16_t i = 0;
        i < WS2811_ICS;
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


      // ---------------------------------------------
      // TEMP RGB
      // ---------------------------------------------

      uint8_t outputR =
          inputR;


      uint8_t outputG =
          inputG;


      uint8_t outputB =
          inputB;


      // ---------------------------------------------
      // Write WLED buffer
      // ---------------------------------------------

      strip.setPixelColor(
          i,
          outputR,
          outputG,
          outputB
      );
    }


    // -----------------------------------------------
    // IMPORTANT
    //
    // NO strip.show()
    //
    // WLED performs the actual LED update.
    // -----------------------------------------------

    newFrameReady = false;

    framesApplied++;
  }


  // ====================================================
  // RMT SETUP
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
        INPUT_GPIO;


    config.clk_div =
        RMT_CLK_DIV;


    // Keep four blocks.
    // This was stable in your previous test.

    config.mem_block_num =
        4;


    config.flags =
        0;


    config.rx_config.filter_en =
        false;


    /*
     * 50 us idle threshold.
     */

    config.rx_config.idle_threshold =
        RESET_LOW_TICKS;


    esp_err_t err;


    // -----------------------------------------------
    // Configure RMT
    // -----------------------------------------------

    err =
        rmt_config(
            &config
        );


    if (
        err != ESP_OK
    ) {

      Serial.print(
          "RMT CONFIG ERROR: "
      );

      Serial.println(
          err
      );

      return false;
    }


    // -----------------------------------------------
    // Install driver
    // -----------------------------------------------

    err =
        rmt_driver_install(
            RX_CHANNEL,
            8192,
            0
        );


    if (
        err != ESP_OK
    ) {

      Serial.print(
          "RMT DRIVER ERROR: "
      );

      Serial.println(
          err
      );

      return false;
    }


    // -----------------------------------------------
    // Get ring buffer
    // -----------------------------------------------

    err =
        rmt_get_ringbuf_handle(
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

      return false;
    }


    // -----------------------------------------------
    // Start RX
    // -----------------------------------------------

    err =
        rmt_rx_start(
            RX_CHANNEL,
            true
        );


    if (
        err != ESP_OK
    ) {

      Serial.print(
          "RMT START ERROR: "
      );

      Serial.println(
          err
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
        INPUT_GPIO,
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
        INPUT_GPIO
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
        15
    );


    Serial.print(
        "WS2811 ICs: "
    );

    Serial.println(
        WS2811_ICS
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

  /*
   * WLED calls this immediately before
   * its actual LED output.
   *
   * Therefore:
   *
   * RMT RX
   *   ↓
   * frame
   *   ↓
   * newFrameReady
   *   ↓
   * handleOverlayDraw()
   *   ↓
   * WLED LED buffer
   *   ↓
   * WLED output
   */

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
  // LOOP
  // ====================================================

  void loop() override
  {
    if (
        !rxReady ||
        rxRingBuffer == nullptr
    ) {

      return;
    }


    // -----------------------------------------------
    // Non-blocking RMT receive
    // -----------------------------------------------

    size_t receivedSize = 0;


    rmt_item32_t *items =
        (rmt_item32_t *)
        xRingbufferReceive(
            rxRingBuffer,
            &receivedSize,
            0
        );


    if (items) {

      size_t count =
          receivedSize /
          sizeof(rmt_item32_t);


      processSymbols(
          items,
          count
      );


      vRingbufferReturnItem(
          rxRingBuffer,
          (void *)items
      );
    }


    // -----------------------------------------------
    // Signal timeout
    // -----------------------------------------------

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


    // -----------------------------------------------
    // DEBUG
    // -----------------------------------------------

    uint32_t now =
        millis();


    if (
        now - lastDebugTime >= 2000
    ) {

      lastDebugTime =
          now;


      Serial.print(
          "Frames: "
      );

      Serial.print(
          framesReceived
      );


      Serial.print(
          "    Applied: "
      );

      Serial.print(
          framesApplied
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
  // INFO
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
        INPUT_GPIO;


    info["output"] =
        "WLED";


    info["physical_leds"] =
        15;


    info["ws2811_ics"] =
        WS2811_ICS;


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


    info["applied"] =
        framesApplied;


    info["dropped"] =
        framesDropped;
  }


  uint16_t getId() override
  {
    return 0x5041;
  }
};


// ======================================================
// REGISTER
// ======================================================

static PassthroughUsermod passthroughUsermod;


REGISTER_USERMOD(
    passthroughUsermod
);
