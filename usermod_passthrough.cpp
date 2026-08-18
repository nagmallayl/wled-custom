#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// HARDWARE
// ======================================================

#define INPUT_GPIO 25

// IMPORTANT:
// Move RX away from WLED's likely low-numbered RMT channels.
#define RX_CHANNEL RMT_CHANNEL_6

// ESP32 classic:
// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns
#define RMT_CLK_DIV 2


// ======================================================
// STRIP TEST CONFIGURATION
// ======================================================

#define PHYSICAL_LEDS 15
#define WS2811_ICS    5

#define FRAME_BYTES   (WS2811_ICS * 3)
#define FRAME_BITS    (FRAME_BYTES * 8)


// ======================================================
// MEASURED INPUT TIMING
// ======================================================
//
// Measured on GPIO25:
//
// BIT 0:
// HIGH = 14~15
// LOW  = 41~42
//
// BIT 1:
// HIGH = 50~51
// LOW  = 24~25
//
// We decode primarily from HIGH duration.
// ======================================================

#define BIT_ONE_THRESHOLD 30

#define VALID_HIGH_MIN 5
#define VALID_HIGH_MAX 80

#define RESET_LOW_TICKS 1000


// ======================================================
// AUTO PASSTHROUGH
// ======================================================

#define SIGNAL_TIMEOUT_MS 300


// ======================================================
// USERMOD
// ======================================================

class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;


  // ====================================================
  // DECODER STATE
  // ====================================================

  uint8_t rxFrame[FRAME_BYTES];

  uint8_t currentByte = 0;
  uint8_t bitCount    = 0;
  uint8_t frameBytes  = 0;


  // ====================================================
  // LATEST COMPLETE FRAME
  // ====================================================

  uint8_t latestFrame[FRAME_BYTES];

  bool haveFrame = false;

  bool passthroughActive = false;


  // ====================================================
  // STATISTICS
  // ====================================================

  uint32_t framesReceived = 0;
  uint32_t framesApplied  = 0;
  uint32_t invalidSymbols = 0;
  uint32_t resyncCount    = 0;

  uint32_t lastFrameTime  = 0;
  uint32_t lastDebugTime  = 0;


  // ====================================================
  // RESET DECODER
  // ====================================================

  void resetDecoder()
  {
    currentByte = 0;
    bitCount    = 0;
    frameBytes  = 0;
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

    memcpy(
        latestFrame,
        rxFrame,
        FRAME_BYTES
    );

    haveFrame = true;

    passthroughActive = true;

    framesReceived++;

    lastFrameTime = millis();

    resetDecoder();

    strip.trigger();
  }


  // ====================================================
  // PROCESS RMT SYMBOLS
  // ====================================================

  void processSymbols(
      rmt_item32_t *items,
      size_t count
  )
  {
    if (!items)
      return;


    for (
        size_t i = 0;
        i < count;
        i++
    ) {

      const uint16_t high =
          items[i].duration0;

      const uint16_t low =
          items[i].duration1;


      // Empty symbol
      if (
          high == 0 &&
          low == 0
      ) {
        continue;
      }


      // =================================================
      // TRUE RESET / IDLE
      // =================================================

      if (
          low >= RESET_LOW_TICKS
      ) {

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

          resetDecoder();

          resyncCount++;
        }

        continue;
      }


      // =================================================
      // VALIDATE HIGH DURATION ONLY
      // =================================================

      if (
          high < VALID_HIGH_MIN ||
          high > VALID_HIGH_MAX
      ) {

        invalidSymbols++;

        continue;
      }


      // =================================================
      // DECODE BIT
      // =================================================

      const bool bit =
          high >= BIT_ONE_THRESHOLD;


      currentByte <<= 1;


      if (bit) {
        currentByte |= 1;
      }


      bitCount++;


      // =================================================
      // COMPLETE BYTE
      // =================================================

      if (
          bitCount == 8
      ) {

        if (
            frameBytes < FRAME_BYTES
        ) {

          rxFrame[frameBytes] =
              currentByte;

          frameBytes++;
        }

        currentByte = 0;
        bitCount = 0;
      }


      // =================================================
      // COMPLETE FIXED FRAME
      // =================================================

      if (
          frameBytes == FRAME_BYTES &&
          bitCount == 0
      ) {

        commitFrame();
      }
    }
  }


  // ====================================================
  // APPLY FRAME TO WLED
  // ====================================================

  void applyLatestFrame()
  {
    if (
        !passthroughActive ||
        !haveFrame
    ) {
      return;
    }


    for (
        uint8_t i = 0;
        i < WS2811_ICS;
        i++
    ) {

      const uint8_t p =
          i * 3;


      // Incoming:
      // G R B

      const uint8_t inputG =
          latestFrame[p + 0];

      const uint8_t inputR =
          latestFrame[p + 1];

      const uint8_t inputB =
          latestFrame[p + 2];


      // =================================================
      // GRB -> BRG
      // =================================================

      const uint8_t outputR =
          inputB;

      const uint8_t outputG =
          inputR;

      const uint8_t outputB =
          inputG;


      strip.setPixelColor(
          i,
          outputR,
          outputG,
          outputB
      );
    }


    // NO strip.show()
    framesApplied++;
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
        (gpio_num_t)INPUT_GPIO;


    config.clk_div =
        RMT_CLK_DIV;


    // Keep the settings that previously gave good Frames.
    config.mem_block_num = 2;

    config.flags = 0;


    config.rx_config.filter_en =
        false;


    config.rx_config.idle_threshold =
        3000;


    esp_err_t err;


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

      Serial.println(err);

      return false;
    }


    err =
        rmt_driver_install(
            RX_CHANNEL,
            4096,
            0
        );


    if (
        err != ESP_OK
    ) {

      Serial.print(
          "RMT DRIVER ERROR: "
      );

      Serial.println(err);

      return false;
    }


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


    err =
        rmt_rx_start(
            RX_CHANNEL,
            true
        );


    if (
        err != ESP_OK
    ) {

      Serial.print(
          "RMT RX START ERROR: "
      );

      Serial.println(err);

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
        latestFrame,
        0,
        sizeof(latestFrame)
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
        "RMT RX Channel: 6"
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
        "Input measured timing:"
    );


    Serial.println(
        "0 = H14~15 / L41~42"
    );


    Serial.println(
        "1 = H50~51 / L24~25"
    );


    Serial.println(
        "Bit threshold: H >= 30"
    );


    Serial.println(
        "Input color: GRB"
    );


    Serial.println(
        "Output color: BRG"
    );


    Serial.println(
        "RMT blocks: 2"
    );


    Serial.println(
        "RMT idle threshold: 3000"
    );


    Serial.println(
        "Direct strip.show(): DISABLED"
    );


    Serial.println(
        "Output: WLED OVERLAY"
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

    } else {

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

  void handleOverlayDraw() override
  {
    if (
        passthroughActive &&
        haveFrame
    ) {

      applyLatestFrame();
    }
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


    size_t receivedSize = 0;


    rmt_item32_t *items =
        (rmt_item32_t *)
        xRingbufferReceive(
            rxRingBuffer,
            &receivedSize,
            0
        );


    if (items) {

      const size_t count =
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


    // =================================================
    // AUTO RETURN TO WLED
    // =================================================

    if (
        passthroughActive
    ) {

      const uint32_t now =
          millis();


      if (
          now - lastFrameTime >
          SIGNAL_TIMEOUT_MS
      ) {

        passthroughActive = false;

        haveFrame = false;

        resetDecoder();


        Serial.println(
            "Passthrough OFF -> WLED"
        );
      }
    }


    // =================================================
    // DEBUG
    // =================================================

    const uint32_t now =
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
          "    Invalid: "
      );

      Serial.print(
          invalidSymbols
      );


      Serial.print(
          "    Resync: "
      );

      Serial.print(
          resyncCount
      );


      Serial.print(
          "    RX Bytes: "
      );

      Serial.print(
          frameBytes
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

      } else {

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


    info["rmt_channel"] =
        6;


    info["physical_leds"] =
        PHYSICAL_LEDS;


    info["ws2811_ics"] =
        WS2811_ICS;


    info["frame_bytes"] =
        FRAME_BYTES;


    info["frame_bits"] =
        FRAME_BITS;


    info["input_order"] =
        "GRB";


    info["output_order"] =
        "BRG";


    info["frames"] =
        framesReceived;


    info["applied"] =
        framesApplied;


    info["invalid"] =
        invalidSymbols;


    info["resync"] =
        resyncCount;
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
