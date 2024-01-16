#define AVANTEK_PLUGIN_ID 077
#define PLUGIN_DESC_077 "Avantek doorbells"
#define SerialDebugActivated

#ifdef PLUGIN_077
#include "../4_Display.h"
#include "../1_Radio.h"
#include "../7_Utils.h"

#define PLUGIN_077_ID "Avantek"
//#define PLUGIN_077_DEBUG

// could we use the function defined in 7_Utils?
inline bool value_between(uint16_t value, uint16_t min, uint16_t max)
{
    return (value > min && value < max);
}

boolean Plugin_077(byte function, const char *string)
{
   const u_int16_t AVTK_PulseDuration = 480;
   const u_int16_t AVTK_PulseMinDuration = AVTK_PulseDuration - 60;
   const u_int16_t AVTK_PulseMaxDuration = AVTK_PulseDuration + 100;
   const u_short AVTK_SyncWordCount = 8;
   const u_short AVTK_MinSyncPairs = 5;

   // HEX: 0xcaca5353
   // BIN: 11001010110010100101001101010011
   // BIN (grouped): 11 00 1 0 1 0 11 00 1 0 1 00 1 0 1 00 11 0 1 0 1 00 11
   const u_short highLowLengths[23] = { 2, 2, 1, 1, 1, 1, 2, 2, 1, 1, 1, 2, 1, 1, 1, 2, 2, 1, 1, 1, 1, 2, 2 };
   const u_short highLowLengthsSize = sizeof(highLowLengths) / sizeof(highLowLengths[0]);
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

   for (int i = 0; i < highLowLengthsSize; i++) {
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

   const size_t buflen = sizeof(PLUGIN_077_ID ": packet = ") + 5 * 2 + 1;
   char printbuf[buflen];
   snprintf(printbuf, buflen, "%s%02x%02x%02x%02x%02x", PLUGIN_077_ID ": packet = ", packet[0], packet[1], packet[2], packet[3], packet[4]);
   SerialDebugPrintln(printbuf);

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

boolean PluginTX_077(byte function, const char *string)
{
   // not yet supported
   return false;
}

#endif //PLUGIN_TX_077

