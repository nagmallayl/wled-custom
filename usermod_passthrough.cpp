#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// HARDWARE
// ======================================================

#define INPUT_GPIO 25

// WLED هو المسؤول عن منفذ الإخراج.

// ======================================================
// WS2811
// ======================================================

// 15 physical LEDs
// 5 WS2811 ICs
// كل IC يتحكم في 3 LEDs

#define WS2811_ICS 5

// 3 bytes لكل IC
#define FRAME_BYTES (WS2811_ICS * 3)

// 8 bits لكل byte
#define FRAME_BITS (FRAME_BYTES * 8)

// ======================================================
// RMT
// ======================================================

#define RX_CHANNEL RMT_CHANNEL_0

/*
 * ESP32:
 *
 * 80 MHz / 2 = 40 MHz
 *
 * 1 tick = 25 ns
 */

#define RMT_CLK_DIV 2


// ======================================================
// TIMING
// ======================================================
//
// القياسات الفعلية التي حصلنا عليها:
//
// BIT 0:
// H = 14~15
// L = 41~42
//
// BIT 1:
// H = 50~51
// L = 24~25
//
// لذلك نعتمد HIGH فقط:
//
// H < 30  = 0
// H >= 30 = 1
//
// Reset:
// LOW > 1000 ticks
//
// ======================================================

#define BIT_THRESHOLD 30

#define RESET_LOW_TICKS 1000

// ======================================================
// USERMOD
// ======================================================

class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;

  // ----------------------------------------------------
  // Frame decoder state
  // ----------------------------------------------------

  uint8_t frameBuffer[FRAME_BYTES];

  uint8_t currentByte = 0;

  uint8_t bitCount = 0;

  uint8_t frameBytes = 0;

  bool frameReady = false;

  bool passthroughActive = false;

  // ----------------------------------------------------
  // Statistics
  // ----------------------------------------------------

  uint32_t framesReceived = 0;

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
  // START NEW FRAME
  // ====================================================

  void startNewFrame()
  {
    resetDecoder();

    frameReady = false;
  }


  // ====================================================
  // DECODE ONE BIT
  // ====================================================

  bool decodeBit(
      const rmt_item32_t &item,
      bool &bit)
  {
    uint16_t high =
        item.duration0;

    uint16_t low =
        item.duration1;


    // ----------------------------------------------
    // RESET
    // ----------------------------------------------

    if (
        low >= RESET_LOW_TICKS
    ) {

      return false;
    }


    // ----------------------------------------------
    // Ignore obviously invalid noise
    // ----------------------------------------------

    if (
        high < 5 ||
        high > 80
    ) {

      return false;
    }


    // ----------------------------------------------
    // Decode using HIGH duration
    // ----------------------------------------------

    if (
        high < BIT_THRESHOLD
    ) {

      bit = false;

    } else {

      bit = true;
    }


    return true;
  }


  // ====================================================
  // PROCESS RMT BLOCK
  // ====================================================

  void processSymbols(
      rmt_item32_t *items,
      size_t count)
  {
    if (!items)
      return;


    /*
     * مهم جدًا:
     *
     * لا نعتبر انتهاء RMT block
     * نهاية Frame.
     *
     * frameBytes / bitCount تبقى محفوظة
     * للـblock التالي.
     */

    for (
        size_t i = 0;
        i < count;
        i++
    ) {

      if (frameReady)
        return;


      uint16_t high =
          items[i].duration0;

      uint16_t low =
          items[i].duration1;


      // =================================================
      // RESET / FRAME BOUNDARY
      // =================================================

      if (
          low >= RESET_LOW_TICKS
      ) {

        /*
         * إذا كان لدينا Frame كامل
         * قبل الـReset، نعتمده.
         */

        if (
            frameBytes == FRAME_BYTES &&
            bitCount == 0
        ) {

          frameReady = true;

          passthroughActive = true;

          lastFrameTime = millis();

          framesReceived++;

          return;
        }


        /*
         * Frame ناقص.
         *
         * لا نحاول استعماله.
         */

        if (
            frameBytes > 0 ||
            bitCount > 0
        ) {

          framesDropped++;
        }


        startNewFrame();

        continue;
      }


      // =================================================
      // DECODE BIT
      // =================================================

      bool bit = false;


      if (
          !decodeBit(
              items[i],
              bit
          )
      ) {

        /*
         * لا نعمل Reset بسبب LOW طويل
         * مثل 84 ticks.
         *
         * Reset الحقيقي فقط >= 1000.
         */

        continue;
      }


      // =================================================
      // BUILD BYTE
      // =================================================

      currentByte <<= 1;


      if (bit)
        currentByte |= 1;


      bitCount++;


      // =================================================
      // BYTE COMPLETE
      // =================================================

      if (
          bitCount == 8
      ) {

        if (
            frameBytes < FRAME_BYTES
        ) {

          frameBuffer[
              frameBytes
          ] = currentByte;

          frameBytes++;
        }


        currentByte = 0;

        bitCount = 0;
      }


      // =================================================
      // FRAME COMPLETE
      // =================================================

      if (
          frameBytes == FRAME_BYTES &&
          bitCount == 0
      ) {

        /*
         * لا نحتاج انتظار Reset.
         *
         * لدينا 120 bit بالضبط.
         */

        frameReady = true;

        passthroughActive = true;

        lastFrameTime = millis();

        framesReceived++;

        return;
      }
    }
  }


  // ====================================================
  // APPLY FRAME
  //
  // INPUT:
  // GRB
  //
  // REQUIRED:
  // BRG
  //
  // WLED buffer:
  // RGB
  //
  // Therefore:
  //
  // WLED R = Input B
  // WLED G = Input R
  // WLED B = Input G
  // ====================================================

  void applyFrame()
  {
    if (!frameReady)
      return;


    for (
        uint8_t i = 0;
        i < WS2811_ICS;
        i++
    ) {

      uint8_t p =
          i * 3;


      // ----------------------------------------------
      // Incoming GRB
      // ----------------------------------------------

      uint8_t inputG =
          frameBuffer[p + 0];

      uint8_t inputR =
          frameBuffer[p + 1];

      uint8_t inputB =
          frameBuffer[p + 2];


      // ----------------------------------------------
      // GRB → BRG
      // ----------------------------------------------

      uint8_t outputR =
          inputB;

      uint8_t outputG =
          inputR;

      uint8_t outputB =
          inputG;


      // ----------------------------------------------
      // WLED RGB buffer
      // ----------------------------------------------

      strip.setPixelColor(
          i,
          outputR,
          outputG,
          outputB
      );
    }


    frameReady = false;

    resetDecoder();
  }


  // ====================================================
  // SETUP RMT RX
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


    /*
     * Two blocks.
     *
     * Diagnostic showed:
     *
     * 64 symbols per block.
     */

    config.mem_block_num = 2;


    config.flags = 0;


    /*
     * No filter.
     */

    config.rx_config.filter_en =
        false;


    /*
     * 3000 ticks:
     *
     * 3000 × 25 ns
     * = 75 us
     *
     * Enough to terminate the RMT receive chunk.
     *
     * This is NOT our Frame detection.
     */

    config.rx_config.idle_threshold =
        3000;


    esp_err_t err =
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

      Serial.println(
          err
      );

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


    Serial.println(
        "Physical LEDs: 15"
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
        "Input:  GRB"
    );


    Serial.println(
        "Output: BRG"
    );


    Serial.println(
        "RMT: RX ONLY"
    );


    Serial.println(
        "Output: WLED LED ENGINE"
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
    /*
     * لا نلمس RMT هنا.
     *
     * فقط نطبق Frame مكتمل.
     */

    if (
        passthroughActive &&
        frameReady
    ) {

      applyFrame();
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


    /*
     * NON-BLOCKING.
     *
     * timeout = 0
     */

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


      /*
       * تحرير الـRMT block دائمًا.
       */

      vRingbufferReturnItem(
          rxRingBuffer,
          (void *)items
      );
    }


    // =================================================
    // SIGNAL TIMEOUT
    // =================================================

    if (
        passthroughActive
    ) {

      uint32_t now =
          millis();


      if (
          now - lastFrameTime >
          300
      ) {

        passthroughActive = false;

        frameReady = false;

        resetDecoder();


        Serial.println(
            "Passthrough signal lost"
        );


        Serial.println(
            "WLED effects resumed"
        );
      }
    }


    // =================================================
    // DEBUG
    // =================================================

    uint32_t now =
        millis();


    if (
        now - lastDebugTime >= 1000
    ) {

      lastDebugTime = now;


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
          "    Bytes: "
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


    info["input_order"] =
        "GRB";


    info["output_order"] =
        "BRG";


    info["rmt"] =
        "LEGACY RX";


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
