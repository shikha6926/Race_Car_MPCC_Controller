/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * lighthouse_ootx.c: lighthouse positioning ootx (slow) data receiver
 */
#include <stdbool.h>
#include <stdint.h>

#include "ootx_decoder.h"

// #include "debug.h"

float fp16_to_float(__fp16 in)
{
  uint32_t t1;
  uint32_t t2;
  uint32_t t3;

  t1 = in & 0x7fff;
  t2 = in & 0x8000;
  t3 = in & 0x7c00;
  t1 <<= 13;
  t2 <<= 16;
  t1 += 0x38000000;
  t1 = (t3 == 0 ? 0 : t1);
  t1 |= t2;

  float ret;
  memcpy(&ret, &t1, sizeof(float));
  return ret;
}

uint16_t betole(uint16_t value)
{
  return ((value&0xff00u)>>8) | ((value&0xffu)<<8);
}

// Frame format described there: https://github.com/nairol/LighthouseRedox/blob/master/docs/Light%20Emissions.md#ootx-frame
bool ootxDecoderProcessBit(ootxDecoderState_t * state, int data)
{
  data &= 1;

  // Synchronization finder
  if (state->nZeros == 17 && data == 1) {
    state->synchronized = true;
    state->bitInWord = 0;
    state->wordReceived = 0;
    state->rxState = ootxDecoderState_s::rxLength;
    // DEBUG_PRINT("Synchronized!\n");
    state->isFullyDecoded = false;
    return false;
  }
  if (data == 0) {
    state->nZeros += 1;
  } else {
    state->nZeros = 0;
  }

  if (state->synchronized) {
    // Detect the stuffing bit
    if (state->bitInWord == 16) {
      // If the stuffing bit is ==0 -> framing error
      if (data == 0) {
        // DEBUG_PRINT("Unsynchronized!\n");
        state->synchronized = false;
        state->isFullyDecoded = false;
        return false;
      }
      state->bitInWord = 0;

      // At the stuffing bit after CRC1, we are done!
      // TODO: Check CRC!
      if (state->rxState == ootxDecoderState_s::rxDone) {
        state->synchronized = false;
        state->isFullyDecoded = true;
        return true;
      } else {
        state->isFullyDecoded = false;
        return false;
      }
    }

    state->currentWord = (state->currentWord<<1) | data;
    state->bitInWord += 1;

    // One word received
    if (state->bitInWord == 16) {
      switch (state->rxState) {
        case ootxDecoderState_s::rxLength:
          state->frameLength = betole(state->currentWord);
          // DEBUG_PRINT("Length %0d\n", state->frameLength);
          if (state->frameLength > OOTX_MAX_FRAME_LENGTH) {
            state->synchronized = false;
            state->isFullyDecoded = false;
            return false;
          }
          state->rxState = ootxDecoderState_s::rxData;
          break;
        case ootxDecoderState_s::rxData:
          // DEBUG_PRINT("data[%d]\n", state->wordReceived);
          state->data[state->wordReceived] = betole(state->currentWord);
          state->wordReceived += 1;
          if (2*state->wordReceived >= state->frameLength) {
            state->rxState = ootxDecoderState_s::rxCrc0;
          }
          break;
        case ootxDecoderState_s::rxCrc0:
          // DEBUG_PRINT("CRC0\n");
          state->crc32 = betole(state->currentWord);
          state->rxState = ootxDecoderState_s::rxCrc1;
          break;
        case ootxDecoderState_s::rxCrc1:
          // DEBUG_PRINT("CRC1\n");
          state->crc32 |= (betole(state->currentWord)<<16ul);
          state->rxState = ootxDecoderState_s::rxDone;
          break;
        case ootxDecoderState_s::rxDone: break;
      }
    }
  }
  state->isFullyDecoded = false;
  return false;
}
