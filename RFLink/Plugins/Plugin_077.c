#define AVANTEK_PLUGIN_ID 077
#define PLUGIN_DESC_077 "Avantek doorbells"
#define SerialDebugActivated

#ifdef PLUGIN_077
#include "../4_Display.h"
#include "../1_Radio.h"
#include "../7_Utils.h"

#define PLUGIN_077_ID "Avantek"
//#define PLUGIN_077_DEBUG

const u_int16_t AVTK_PulseDuration = 480;

// could we use the function defined in 7_Utils?
inline bool value_between(uint16_t value, uint16_t min, uint16_t max)
{
    return (value > min && value < max);
}

/**
 * Convert Hex character to Hex value
 **/
byte hexchar2hexvalue(char c)
{
   if ((c>='0') && (c<='9'))
      return c-'0' ;
   if ((c>='A') && (c<='F'))
      return c+10-'A' ;
   if ((c>='a') && (c<='f'))
      return c+10-'a' ;
   return -1 ;
}

bool* convertToBinary(const char* hex, size_t* resultSize)
{
    size_t len = strlen(hex);
    *resultSize = len * 4; // Each hex char is represented by 4 bits

    bool* binaryResult = (bool*)malloc(*resultSize * sizeof(bool));

    if (binaryResult == NULL)
    {
        fprintf(stderr, "Memory allocation failed\n");
        exit(EXIT_FAILURE);
    }

    size_t index = 0;

    for (size_t i = 0; i < len; i++)
    {
        char hexChar = hex[i];
        int hexValue = hexchar2hexvalue(hexChar);

        for (int j = 3; j >= 0; j--)
        {
            bool bit = (hexValue >> j) & 1;
            binaryResult[index++] = bit;
        }
    }

    return binaryResult;
}

/**
 * IN: 1,1,0,0,1,0,1,0,1,1,0,0,1,0,1,0,0,1,0,1,0,0,1,1,0,1,0,1,0,0,1,1
 * (grouped): 11 00 1 0 1 0 11 00 1 0 1 00 1 0 1 00 11 0 1 0 1 00 11
 * OUT: 2,2,1,1,1,1,2,2,1,1,1,2,1,1,1,2,2,1,1,1,1,2,2
*/
size_t* countConsecutive(bool* binaryArray, size_t size, size_t* resultSize) {
    size_t* consecutiveCounts = (size_t*)malloc(size * sizeof(size_t));

    if (consecutiveCounts == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(EXIT_FAILURE);
    }

    size_t count = 1;
    size_t index = 0;

    for (size_t i = 1; i < size; i++) {
        if (binaryArray[i] == binaryArray[i - 1]) {
            count++;
        } else {
            consecutiveCounts[index++] = count;
            count = 1;
        }
    }

    consecutiveCounts[index] = count;
    *resultSize = index + 1;

    return consecutiveCounts;
}

boolean Plugin_077(byte function, const char *string)
{
   const u_int16_t AVTK_PulseMinDuration = AVTK_PulseDuration - 60;
   const u_int16_t AVTK_PulseMaxDuration = AVTK_PulseDuration + 100;
   const u_short AVTK_SyncWordCount = 8;
   const u_short AVTK_MinSyncPairs = 5;

   size_t syncWordSize;
   bool* syncWord = convertToBinary("caca5353", &syncWordSize);
   size_t highLowLengthsSize;
   const size_t* highLowLengths = countConsecutive(syncWord, syncWordSize, &highLowLengthsSize);
   free(syncWord);
   const bool sequenceEndsWithHigh = highLowLengthsSize % 2 != 0;

   int pulseIndex = 1;
   // Check for preamble (0xaaaa)
   u_short preamblePairsFound = 0;
   while (pulseIndex < (RawSignal.Number - 1) && (preamblePairsFound < AVTK_SyncWordCount)) {
      if (value_between(RawSignal.Pulses[pulseIndex], AVTK_PulseMinDuration, AVTK_PulseMaxDuration)
            && value_between(RawSignal.Pulses[pulseIndex + 1], AVTK_PulseMinDuration, AVTK_PulseMaxDuration)) {
         preamblePairsFound++;
      } else if (preamblePairsFound > 0) {
         // if we didn't already had a match, we ignore as mismatch, otherwise we break
         // here
         break;
      }
      pulseIndex += 2;
   }

   if (preamblePairsFound < AVTK_MinSyncPairs) {
      #ifdef PLUGIN_077_DEBUG
      Serial.println(F(PLUGIN_077_ID ": Preamble not found"));
      #endif
      return false;
   }

   for (size_t i = 0; i < highLowLengthsSize; i++) {
      if (pulseIndex >= RawSignal.Number) {
         #ifdef PLUGIN_077_DEBUG
         Serial.println(F(PLUGIN_077_ID ": Sync word not complete"));
         #endif
         return false;
      }
      uint16_t pulse = RawSignal.Pulses[pulseIndex++];
      u_short mul = highLowLengths[i];
      uint16_t min = mul * AVTK_PulseMinDuration;
      uint16_t max = mul * AVTK_PulseMaxDuration;
      boolean syncWordMatch = i < (highLowLengthsSize - 1) 
         ? value_between(pulse, min, max) 
         : pulse > mul * AVTK_PulseMinDuration;
      if (!syncWordMatch) {
         #ifdef PLUGIN_077_DEBUG
         Serial.print(F(PLUGIN_077_ID ": Sync word mismatch at: "));
         Serial.println(pulseIndex);
         #endif
         return false;
      }
   }

   if (sequenceEndsWithHigh) {
      RawSignal.Pulses[pulseIndex - 1] = RawSignal.Pulses[pulseIndex - 1]
            - highLowLengths[highLowLengthsSize - 1] * AVTK_PulseDuration;
   }

   byte packet[] = { 0, 0, 0, 0, 0 };

   if (!decode_pwm(packet, 35, RawSignal.Pulses, RawSignal.Number, pulseIndex, AVTK_PulseMinDuration,
         AVTK_PulseMaxDuration, 2 * AVTK_PulseMinDuration, 2 * AVTK_PulseMaxDuration, 0)) {
      #ifdef PLUGIN_077_DEBUG
      Serial.println(F(PLUGIN_077_ID ": Avantek: PWM decoding failed"));
      #endif
      return false;
   }
   pulseIndex += (35 * 2);

   #ifdef PLUGIN_077_DEBUG
   const size_t buflen = sizeof(PLUGIN_077_ID ": packet = ") + 5 * 2 + 1;
   char printbuf[buflen];
   snprintf(printbuf, buflen, "%s%02x%02x%02x%02x%02x", PLUGIN_077_ID ": packet = ", packet[0], packet[1], packet[2], packet[3], packet[4]);
   SerialDebugPrintln(printbuf);
   #endif

   display_Header();
   display_Name(PLUGIN_077_ID);
   char c_ID[5 * 2 + 1];
   sprintf(c_ID, "%02x%02x%02x%02x%02x", packet[0], packet[1], packet[2], packet[3], packet[4]);
   display_IDc(c_ID);
   display_Footer();
   //==================================================================================
   RawSignal.Repeats = true; // suppress repeats of the same RF packet
   RawSignal.Number = 0;     // do not process the packet any further
   return true;
}
#endif //PLUGIN_077

#ifdef PLUGIN_TX_077
#include "../1_Radio.h"
#include "../2_Signal.h"
#include "../3_Serial.h"
#include <stdlib.h>

inline void send(boolean state)
{
   digitalWrite(Radio::pins::TX_DATA, state ? HIGH : LOW);
   delayMicroseconds(AVTK_PulseDuration);
}

size_t preambleSize;
size_t syncWordSize;
bool* preamble = convertToBinary("aaaa", &preambleSize);
bool* syncWord = convertToBinary("caca5353", &syncWordSize);

boolean PluginTX_077(byte function, const char *string)
{
   //10;AVANTEK;71f1100080
   //012345678901234567890
   if (strncasecmp(InputBuffer_Serial + 3, "AVANTEK;", 8) == 0) {
		short times = 3;
		char* address = InputBuffer_Serial + 3 + 8;

		noInterrupts();

      #ifdef PLUGIN_077_DEBUG
      Serial.println(F(PLUGIN_077_ID ": Sending preamble"));
      #endif
		for (u_short count = 0; count < times; count++) {
         for (u_int i = 0; i < preambleSize; i++) {
            send(preamble[i]);
         }

         #ifdef PLUGIN_077_DEBUG
         Serial.println(F(PLUGIN_077_ID ": Sending syncword"));
         #endif
         for (u_int i = 0; i < syncWordSize; i++) {
            send(syncWord[i]);
         }

         #ifdef PLUGIN_077_DEBUG
         Serial.print(F(PLUGIN_077_ID ": Sending payload "));
         Serial.print(address);
         #endif
			for (size_t i = 0; i < strlen(address); i++) {
				char hexChar = address[i];
				int hexValue = hexchar2hexvalue(hexChar);

				for (int j = 3; j >= 0; j--) {
					bool bit = (hexValue >> j) & 1;
					if (bit) {
						send(true);
						send(false);
						send(false);
					} else {
						send(true);
						send(true);
						send(false);
					}
				}
			}
         // TODO introduce new constant
         delayMicroseconds(5 * AVTK_PulseDuration);
		}
		interrupts();

      return true;
	}
   return false;
}

#endif //PLUGIN_TX_077

