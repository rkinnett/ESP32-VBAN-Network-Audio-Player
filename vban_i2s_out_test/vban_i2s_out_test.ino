/****************************************************
 * R. Kinnett, 2020
 * 
 * For use with VBAN Voicemeeter
 * https://www.vb-audio.com/Voicemeeter/index.htm
 * 
 * Receives VBAN packets over UDP and funnels audio
 * data into I2S DMA buffers to drive analog output.
 * 
 * *************************************************/

#include <Arduino.h>
#include <WiFi.h>
//#include <WiFiAP.h>
#include <WiFiUDP.h>
#include <driver/dac.h>
#include <driver/i2s.h>
#include "vban.h"
#include "esp_err.h"

#include "my_wifi.h"  // this defines my wifi ssid and password
// Set these to your desired credentials.
//const char *ssid = "esp32";

WiFiUDP udpIn;
uint16_t udpInPort = 6981;
char udpIncomingPacket[VBAN_PACKET_MAX_LEN_BYTES];


// IMPORTANT Note:
// VBAN adjusts the number of samples per packet according to sample rate.
// Assuming 16-bit PCM mono, sample rates 11025, 22050, 44100, and 88200 yield 
// packets containing 64, 128, 256, and 256 samples per packet, respectively.
// The even-thousands sample rates below 48000 yield non-power-of-2 lengths.
// For example, sample rate 24000 yields 139 samples per packet.
// This VBAN->DMA->DAC method seems to require the dma buffer length be set
// equal to the number of samples in each VBAN packet.
// ESP32 I2S/DMA does not seem to handle non-power-of-2 buffer lengths well.
// Sample rate 24000 doesn't work reliably at all.
// Sample rate 32000 is stable but stutters.
// Recommend selecting from sample rates 11025, 22050, 44100, and above
// And set samplesPerPacket to 64 for 11025, 128 for 22050, or 256 for all else.

const uint32_t initialSampleRate        = 44100;  // samples/sec; this must match VBAN Outgoing Stream SampleRate parameter
const uint16_t initialSamplesPerPacket  = 256;   // Use 64 for sample rate 11025 Hz, 128 for 22050 Hz, or 256 for 44100 and above.


i2s_port_t i2s_num = I2S_NUM_0; // i2s port number (Built-in DAC functions are only supported on I2S0 for current ESP32 chip, purportedly)
i2s_config_t i2s_config;
bool i2s_running = false;

TaskHandle_t outputAudioStreamTaskHandle;
static void outputAudioStreamTask(void * pvParameters);

uint16_t noDataCounter = 0;




void configure_i2s(int newSampleRate, uint16_t newDmaBufLen){
  Serial.printf("Configuring i2s driver with new sample rate %u and buffer length %u\n",newSampleRate,newDmaBufLen);

  i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = newSampleRate,    // This must match VBAN Outgoing Stream SampleRate parameter
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,   // apparently built-in DAC only enabled on I2S0 which only allows 16bit?
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 1, // default interrupt priority
    .dma_buf_count = 16,
    .dma_buf_len = newDmaBufLen,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  if(i2s_running){
    ESP_ERROR_CHECK( i2s_stop(i2s_num) );
    ESP_ERROR_CHECK( i2s_driver_uninstall(i2s_num) );
  }
  
  ESP_ERROR_CHECK( i2s_driver_install(i2s_num, &i2s_config, 0, NULL)  );   //install and start i2s driver
  ESP_ERROR_CHECK( i2s_set_pin(i2s_num, NULL) );
  ESP_ERROR_CHECK( i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN)  );  // RIGHT=GPIO25, LEFT=GPIO26
  ESP_ERROR_CHECK( i2s_zero_dma_buffer(i2s_num) );

  i2s_running = true;

  Serial.println("done");
}


//////////////////////////   SETUP   ////////////////////////////////////////
void setup() {
  Serial.begin(500000);

  // Setup Wifi:
  //WiFi.softAP(ssid);
  //IPAddress myIP = WiFi.softAPIP();
  
  WiFi.begin(ssid, password);     //Connect to your WiFi router
  Serial.println("");
  while (WiFi.status() != WL_CONNECTED) {  // Wait for connection
    delay(500);
    Serial.print(".");
  }
  IPAddress myIP = WiFi.localIP();
  
  Serial.print("AP IP address: ");  
  Serial.print(myIP);
  Serial.print(";  Subnet Mask: ");
  Serial.println(WiFi.subnetMask());

  Serial.println("initializing i2s");
  configure_i2s(initialSampleRate, initialSamplesPerPacket);

  Serial.println("initializing udp");
  Serial.println( udpIn.begin(myIP, udpInPort) );

  // Iniitiate the udp receiving and audio output stream loop task:
  xTaskCreatePinnedToCore(outputAudioStreamTask, "outputAudioStreamTask", 20000, NULL, 0, &outputAudioStreamTaskHandle, 0 );

  Serial.println("Setup done");
}


void loop() {
  // Note: udp receiving and dma -> i2s -> dac streaming is handled in dedicated loop
  // Reserve primary loop for infrastructural things you might want to check periodically.
}



static void outputAudioStreamTask(void * pvParameters){
  // uint16_t delayMsec = (uint16_t) (1000.0 * i2s_config.dma_buf_len / i2s_config.sample_rate);
  for( ;; ){
    receiveUdp(); 
    vTaskDelay(1);  // resets watchdog with 1-tick (1ms) delay
  }
}



void receiveUdp(){
  uint16_t  vban_rx_data_bytes, vban_rx_sample_count;
  int16_t*  vban_rx_data;
  uint32_t* vban_rx_pkt_nbr;
  uint16_t  outBuf[VBAN_PACKET_MAX_SAMPLES+1];
  size_t    bytesOut;

  // read UDP buffer and process if non-empty:
  int packetSize = udpIn.parsePacket();

  if (packetSize) {

    // receive incoming UDP packet
    int len = udpIn.read(udpIncomingPacket, VBAN_PACKET_MAX_LEN_BYTES);
    //Serial.printf("%d B\n", len);

    // Check if packet length meets VBAN specification:
    if (len<=(VBAN_PACKET_HEADER_BYTES+VBAN_PACKET_COUNTER_BYTES) || len>VBAN_PACKET_MAX_LEN_BYTES) {
      Serial.printf("Error: packet length %u bytes\n", len);
      return;
    }
    
    // Check if preamble matches VBAN format:
    if(strncmp("VBAN",udpIncomingPacket,4)!=0){
      Serial.printf("Unrecognized preamble %.4s\n", udpIncomingPacket);
      return;
    }
      
    vban_rx_data_bytes = len - (VBAN_PACKET_HEADER_BYTES+VBAN_PACKET_COUNTER_BYTES);
    vban_rx_pkt_nbr = (uint32_t*)&udpIncomingPacket[VBAN_PACKET_HEADER_BYTES];
    vban_rx_data = (int16_t*)&udpIncomingPacket[VBAN_PACKET_HEADER_BYTES+VBAN_PACKET_COUNTER_BYTES];
    vban_rx_sample_count = vban_rx_data_bytes/2;
    uint8_t vbanSampleRateIdx = udpIncomingPacket[4] & VBAN_SR_MASK;
    uint32_t vbanSampleRate = VBanSRList[vbanSampleRateIdx];

    //Serial.printf("%u %u %u\n", vbanSampleRate, vban_rx_sample_count, i2s_config.dma_buf_len);

    // Just to be safe, re-check sample count against max sample count to avoid overrunning outBuf later
    if(vban_rx_sample_count > VBAN_PACKET_MAX_SAMPLES){
      Serial.printf("error: unexpected packet size: %u\n",vban_rx_sample_count);
      return;
    }

    // If necessary, reconfigre I2S to match VBAN format:
    if(!i2s_running || vbanSampleRate!=i2s_config.sample_rate || vban_rx_sample_count!=i2s_config.dma_buf_len){
      configure_i2s(vbanSampleRate, vban_rx_sample_count);
    }

    // Important notes from:  https://esp32.com/viewtopic.php?t=8234:  
    // 1. The I2S needs 16-bit samples.
    // 2. The I2S reads the samples as 32-bit words and outputs the high 16-bit first and the low 16-bit second. 
    //      This means that if you want to send it 16-bit samples, you'll need to swap the even and odd 16-bit words.
    // 3. The DAC only plays the high byte of the 16-bit sample as an unsigned value (0..255, not -128..127).

    //Serial.printf("%08X\n", *vban_rx_pkt_nbr);
    
    for(int i=0; i<vban_rx_sample_count; i++){
      // Input stream is signed int16, with min -32768 and max +32768.
      // Output needs to be uint16 with audio range in upper 8 bits,
      // Add 2^16/2 to remap from 0 to 65535.
      // Note:  index i^1 (XOR 1) swaps even and odd samples, per note above.
      outBuf[i^1] = (uint16_t)(vban_rx_data[i] + 32768);
      //Serial.printf("\t%X", (uint8_t)(outBuf[i^1]>>8));
    }
    //Serial.println();

    ESP_ERROR_CHECK( i2s_write( i2s_num, &outBuf, vban_rx_sample_count*sizeof(uint16_t), &bytesOut, portMAX_DELAY ) );
    //Serial.printf("pkt %04ul, %u vals, %i, %i, %i\n",vban_rx_pkt_nbr, vban_rx_data_vals, pkt_data_min_val, pkt_data_max_val, bytesOut);

    // to-do:  report gaps in vban frame numbers

    //Serial.println(emptyCounter);
    noDataCounter = 0;  // reset no-data counter
    
  } else {  // handle no-data
    // If haven't received any data in awhile, then clear the dma buffers.
    // Otherwise i2s will continue to loop through old data.  
    noDataCounter++;
    if(noDataCounter>=50){
      if(i2s_running){
        Serial.println("Stopping i2s");
        ESP_ERROR_CHECK( i2s_stop(i2s_num) );
        ESP_ERROR_CHECK( i2s_driver_uninstall(i2s_num) );
        i2s_running = false;
      }
      noDataCounter = 0;
    }
    vTaskDelay(1);
  }
}
