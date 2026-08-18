#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// CONFIG
// ======================================================

#define INPUT_GPIO 25
#define RX_CHANNEL RMT_CHANNEL_0

// ESP32 Classic
// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns
#define RMT_CLK_DIV 2

// ======================================================
// WS2811
// ======================================================

// 5 WS2811 ICs
// 3 channels per IC
// Total = 15 physical LEDs
#define WS2811_ICS 5

#define FRAME_BYTES (WS2811_ICS * 3)
#define FRAME_BITS  (FRAME_BYTES * 8)

// ======================================================
// TIMING
// ======================================================

// Measured:
//
// 0 = H 14~15 / L 41~42
// 1 = H 50~51 / L 24~25
//
// Therefore:
#define BIT_THRESHOLD 30

// RMT idle threshold
// 50 us @ 40 MHz
#define RESET_LOW_TICKS 2000


// ======================================================
// USERMOD
// ======================================================

class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;

  // ----------------------------------------------------
  // RX decoder
  // ----------------------------------------------------

  uint8_t frameBuffer[FRAME_BYTES];

  uint8_t currentByte = 0;

  uint8_t bitCount = 0;

  uint8_t frameBytes = 0;

  // ----------------------------------------------------
  // Statistics
  // ----------------------------------------------------

  uint32_t framesReceived = 0;

  uint32_t framesApplied = 0;

  uint32_t framesDropped = 0;

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
  // DECODE BIT
  // ====================================================

  bool decodeBit(
      const rmt_item32_t &item,
      bool &bit
  )
  {
    uint16_t high = item.duration0;
    uint16_t low  = item.duration1;

    uint16_t total = high + low;

    // -----------------------------------------------
    // Ignore impossible symbols
    // -----------------------------------------------

    if (total < 35 || total > 65)
      return false;

    // -----------------------------------------------
    // Decode using measured HIGH duration
    // -----------------------------------------------

    bit = (high >= BIT_THRESHOLD);

    return true;
  }


  // ====================================================
  // APPLY FRAME
  // ====================================================

  void applyFrame()
  {
    if (frameBytes != FRAME_BYTES)
      return;

    // -----------------------------------------------
    // Write complete frame to WLED
    // -----------------------------------------------

    for (uint8_t i = 0; i < WS2811_ICS; i++)
    {
      uint8_t p = i * 3;

      // Input = GRB
      //
      // Temporarily convert to WLED RGB.
      //
      // We are NOT dealing with final color order yet.

      uint8_t g = frameBuffer[p + 0];
      uint8_t r = frameBuffer[p + 1];
      uint8_t b = frameBuffer[p + 2];

      strip.setPixelColor(
          i,
          r,
          g,
          b
      );
    }

    // -----------------------------------------------
    // IMPORTANT
    //
    // Direct WLED output.
    // -----------------------------------------------

    strip.show();

    framesApplied++;

    // -----------------------------------------------
    // Prepare decoder for next frame
    // -----------------------------------------------

    resetDecoder();
  }


  // ====================================================
  // PROCESS RMT
  // ====================================================

  void processSymbols(
      rmt_item32_t *items,
      size_t count
  )
  {
    if (!items)
      return;

    for (size_t i = 0; i < count; i++)
    {
      uint16_t high = items[i].duration0;
      uint16_t low  = items[i].duration1;

      // ---------------------------------------------
      // Empty symbol
      // ---------------------------------------------

      if (high == 0 && low == 0)
        continue;


      // ---------------------------------------------
      // Reset / idle
      //
      // If a complete frame is already assembled,
      // apply it.
      // ---------------------------------------------

      if (low >= RESET_LOW_TICKS)
      {
        if (
            frameBytes == FRAME_BYTES &&
            bitCount == 0
        )
        {
          applyFrame();

          framesReceived++;
        }
        else
        {
          if (frameBytes > 0 || bitCount > 0)
            framesDropped++;

          resetDecoder();
        }

        continue;
      }


      // ---------------------------------------------
      // Validate WS2811 symbol
      // ---------------------------------------------

      uint16_t total = high + low;

      if (total < 35 || total > 65)
      {
        framesDropped++;

        resetDecoder();

        continue;
      }


      // ---------------------------------------------
      // Prevent overflow
      // ---------------------------------------------

      if (frameBytes >= FRAME_BYTES)
      {
        framesDropped++;

        resetDecoder();

        continue;
      }


      // ---------------------------------------------
      // Decode bit
      // ---------------------------------------------

      bool bit = false;

      if (!decodeBit(items[i], bit))
      {
        framesDropped++;

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
      // Byte complete
      // ---------------------------------------------

      if (bitCount == 8)
      {
        frameBuffer[frameBytes] = currentByte;

        frameBytes++;

        currentByte = 0;

        bitCount = 0;
      }


      // ---------------------------------------------
      // COMPLETE FRAME
      //
      // Do NOT wait for Reset.
      //
      // We know this frame is exactly 120 bits.
      // ---------------------------------------------

      if (
          frameBytes == FRAME_BYTES &&
          bitCount == 0
      )
      {
        applyFrame();

        framesReceived++;
      }
    }
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
        (gpio_num_t)INPUT_GPIO;

    config.clk_div =
        RMT_CLK_DIV;

    // Four memory blocks
    config.mem_block_num =
        4;

    config.flags =
        0;

    config.rx_config.filter_en =
        false;

    config.rx_config.idle_threshold =
        RESET_LOW_TICKS;


    esp_err_t err;


    // -----------------------------------------------
    // Configure RMT
    // -----------------------------------------------

    err = rmt_config(&config);

    if (err != ESP_OK)
    {
      Serial.print("RMT CONFIG ERROR: ");
      Serial.println(err);

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

    if (err != ESP_OK)
    {
      Serial.print("RMT DRIVER ERROR: ");
      Serial.println(err);

      return false;
    }


    // -----------------------------------------------
    // Ring buffer
    // -----------------------------------------------

    err =
        rmt_get_ringbuf_handle(
            RX_CHANNEL,
            &rxRingBuffer
        );

    if (
        err != ESP_OK ||
        rxRingBuffer == nullptr
    )
    {
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

    if (err != ESP_OK)
    {
      Serial.print("RMT START ERROR: ");
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
        frameBuffer,
        0,
        sizeof(frameBuffer)
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
        "Direct strip.show(): ENABLED"
    );

    Serial.println(
        "================================"
    );


    // -----------------------------------------------
    // Start RMT
    // -----------------------------------------------

    if (setupRX())
    {
      rxReady = true;

      Serial.println(
          "RMT RX READY"
      );
    }
    else
    {
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
    )
    {
      return;
    }


    // -----------------------------------------------
    // NON-BLOCKING RECEIVE
    // -----------------------------------------------

    size_t receivedSize = 0;


    rmt_item32_t *items =
        (rmt_item32_t *)
        xRingbufferReceive(
            rxRingBuffer,
            &receivedSize,
            0
        );


    if (items)
    {
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
    // DEBUG
    // -----------------------------------------------

    uint32_t now = millis();


    if (
        now - lastDebugTime >= 2000
    )
    {
      lastDebugTime = now;


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
          "    RX Bytes: "
      );

      Serial.print(
          frameBytes
      );


      Serial.print(
          "    Bits: "
      );

      Serial.print(
          bitCount
      );


      Serial.println();
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
