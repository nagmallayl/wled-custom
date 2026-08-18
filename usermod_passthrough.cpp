#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// CONFIG
// ======================================================

#define INPUT_GPIO 25

#define RX_CHANNEL RMT_CHANNEL_0

// ESP32 80MHz / 2 = 40MHz
// 1 tick = 25ns
#define RMT_CLK_DIV 2

// ======================================================
// WS2811
// ======================================================

// 5 IC × 3 LEDs = 15 physical LEDs
#define WS2811_ICS 5

#define FRAME_BYTES (WS2811_ICS * 3)
#define FRAME_BITS  (FRAME_BYTES * 8)

// ======================================================
// REAL MEASURED TIMING
// ======================================================
//
// 0:
// H = 14~15
// L = 41~42
//
// 1:
// H = 50~51
// L = 24~25
//
// لذلك:
// H < 30 = 0
// H >= 30 = 1
//
// Reset الحقيقي أكبر بكثير
// من 84 ticks التي رأيناها.
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
  // Decoder
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


    // --------------------------------------------------
    // Reset detection
    // --------------------------------------------------

    if (
        low >= RESET_LOW_TICKS
    ) {

      return false;
    }


    // --------------------------------------------------
    // Ignore invalid HIGH
    // --------------------------------------------------

    if (
        high < 5 ||
        high > 80
    ) {

      return false;
    }


    // --------------------------------------------------
    // Decode
    // --------------------------------------------------

    bit =
        high >= BIT_THRESHOLD;


    return true;
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

      // ----------------------------------------------
      // Frame already complete
      // ----------------------------------------------

      if (frameReady)
        return;


      uint16_t high =
          items[i].duration0;

      uint16_t low =
          items[i].duration1;


      // ----------------------------------------------
      // RESET
      // ----------------------------------------------

      if (
          low >= RESET_LOW_TICKS
      ) {

        /*
         * إذا وصلنا إلى 120 bit قبل الـReset،
         * الإطار صالح.
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
         */

        if (
            frameBytes > 0 ||
            bitCount > 0
        ) {

          framesDropped++;
        }


        resetDecoder();

        continue;
      }


      // ----------------------------------------------
      // Decode bit
      // ----------------------------------------------

      bool bit = false;


      if (
          !decodeBit(
              items[i],
              bit
          )
      ) {

        continue;
      }


      // ----------------------------------------------
      // Build byte
      // ----------------------------------------------

      currentByte <<= 1;


      if (bit)
        currentByte |= 1;


      bitCount++;


      // ----------------------------------------------
      // Byte complete
      // ----------------------------------------------

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


      // ----------------------------------------------
      // Frame complete
      // ----------------------------------------------

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
    }
  }


  // ====================================================
  // APPLY COMPLETE FRAME
  // ====================================================

  void applyFrame()
  {
    if (!frameReady)
      return;


    /*
     * مهم:
     *
     * نأخذ نسخة محلية من الـFrame قبل الكتابة
     * إلى WLED.
     *
     * هذا يمنع قراءة Frame أثناء تغييره.
     */

    uint8_t localFrame[
        FRAME_BYTES
    ];


    memcpy(
        localFrame,
        frameBuffer,
        FRAME_BYTES
    );


    // ==================================================
    // COLOR MAPPING
    //
    // حاليًا لا نركز على الألوان.
    //
    // نضعها مؤقتًا كما هي:
    //
    // Input GRB
    // WLED RGB
    //
    // لاحقًا نرجع GRB → BRG.
    // ==================================================

    for (
        uint8_t i = 0;
        i < WS2811_ICS;
        i++
    ) {

      uint8_t p =
          i * 3;


      /*
       * مؤقتًا:
       *
       * input:
       * G R B
       *
       * نحافظ على القيم.
       */

      uint8_t g =
          localFrame[p + 0];

      uint8_t r =
          localFrame[p + 1];

      uint8_t b =
          localFrame[p + 2];


      /*
       * WLED RGB buffer.
       *
       * هذه الخطوة مؤقتة لاختبار
       * التحديث والترميش.
       */

      strip.setPixelColor(
          i,
          r,
          g,
          b
      );
    }


    /*
     * Frame كامل أصبح داخل WLED buffer.
     *
     * لا نعمل أي delay.
     */

    strip.show();


    // --------------------------------------------------
    // Frame consumed
    // --------------------------------------------------

    frameReady = false;

    resetDecoder();
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


    config.mem_block_num =
        2;


    config.flags =
        0;


    config.rx_config.filter_en =
        false;


    /*
     * 75us idle threshold.
     */

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


    Serial.println(
        "WS2811 ICs: 5"
    );


    Serial.println(
        "Frame bytes: 15"
    );


    Serial.println(
        "Frame bits: 120"
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


    // --------------------------------------------------
    // NON-BLOCKING RECEIVE
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


    // --------------------------------------------------
    // APPLY ONE COMPLETE FRAME
    // --------------------------------------------------

    if (
        frameReady
    ) {

      applyFrame();
    }


    // --------------------------------------------------
    // SIGNAL TIMEOUT
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

        frameReady =
            false;

        resetDecoder();
      }
    }


    // --------------------------------------------------
    // DEBUG
    // --------------------------------------------------

    uint32_t now =
        millis();


    if (
        now - lastDebugTime >= 2000
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
        5;

    info["frame_bits"] =
        FRAME_BITS;

    info["speed"] =
        "800 kHz";

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
// REGISTER
// ======================================================

static PassthroughUsermod passthroughUsermod;

REGISTER_USERMOD(
    passthroughUsermod
);
