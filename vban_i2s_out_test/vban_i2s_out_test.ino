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
#include <WiFiUDP.h>
#include <driver/dac.h>
#include <driver/i2s.h>
#include "esp_err.h"

#include "my_wifi.h"  // this defines my wifi ssid and password

WiFiUDP udpIn;
uint16_t udpInPort = 6981;

#define VBPACKET_HEADER_BYTES 24  
#define VBPACKET_COUNTER_BYTES 4  
#define VBPACKET_MAX_SAMPLE_COUNT 256
const int maxPacketSizeBytes = VBPACKET_HEADER_BYTES + VBPACKET_COUNTER_BYTES + VBPACKET_MAX_SAMPLE_COUNT;
char udpIncomingPacket[maxPacketSizeBytes];

i2s_port_t i2s_num = I2S_NUM_0; // i2s port number (Built-in DAC functions are only supported on I2S0 for current ESP32 chip, purportedly)
i2s_config_t i2s_config;


TaskHandle_t outputAudioStreamTaskHandle;
static void outputAudioStreamTask(void * pvParameters);



void setup_i2s(){
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = 11025,    // This must match VBAN Outgoing Stream SampleRate parameter
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,   // apparently built-in DAC only enabled on I2S0 which only allows 16bit?
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false
  };

  ESP_ERROR_CHECK( i2s_driver_install(i2s_num, &i2s_config, 0, NULL)  );   //install and start i2s driver
  ESP_ERROR_CHECK( i2s_set_pin(i2s_num, NULL) );
  ESP_ERROR_CHECK( i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN)  );  // RIGHT=GPIO25, LEFT=GPIO26
}


//////////////////////////   SETUP   ////////////////////////////////////////
void setup() {
  Serial.begin(500000);

  // Setup Wifi:
  WiFi.begin(ssid, password);     //Connect to your WiFi router
  Serial.println("");
  while (WiFi.status() != WL_CONNECTED) {  // Wait for connection
    delay(500);
    Serial.print(".");
  }
  IPAddress myIP = WiFi.localIP();
  Serial.print("Wifi connected. IP: ");
  Serial.print(WiFi.localIP());
  Serial.print(";  Subnet Mask: ");
  Serial.println(WiFi.subnetMask());

  Serial.println("initializing i2s");
  setup_i2s();

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
  for( ;; ){
    receiveUdp(); 
    vTaskDelay(1);  // resets watchdog with 1-tick (1ms) delay
  }
}




void receiveUdp(){
  uint16_t vban_rx_data_bytes, vban_rx_sample_count;
  int16_t* vban_rx_data;
  uint32_t vban_rx_pkt_nbr;
  uint16_t outBuf[VBPACKET_MAX_SAMPLE_COUNT+1];
  size_t bytesOut;

  // read UDP buffer and process if non-empty:
  int packetSize = udpIn.parsePacket();
  if (packetSize) {
    // receive incoming UDP packets
    //Serial.printf("Received %d bytes from %s\n", packetSize, udpIn.remoteIP().toString().c_str());
    int len = udpIn.read(udpIncomingPacket, 255);
    if (len>0 && len<=maxPacketSizeBytes) {
      // Make sure packet length and preamble match VBAN format:
      if(len>(VBPACKET_HEADER_BYTES+VBPACKET_COUNTER_BYTES) && strncmp("VBAN",udpIncomingPacket,4)==0){
        udpIncomingPacket[len] = 0;
        vban_rx_data_bytes = len - (VBPACKET_HEADER_BYTES+VBPACKET_COUNTER_BYTES);
        vban_rx_pkt_nbr = (uint32_t)udpIncomingPacket[VBPACKET_HEADER_BYTES];
        vban_rx_data = (int16_t*)&udpIncomingPacket[VBPACKET_HEADER_BYTES+VBPACKET_COUNTER_BYTES];
        vban_rx_sample_count = vban_rx_data_bytes/2;

        // Just to be safe, make sure the sample count meets expectations
        if(vban_rx_sample_count > VBPACKET_MAX_SAMPLE_COUNT){
          Serial.printf("error: unexpected packet size: %u\n",vban_rx_sample_count);
          return;
        }

        // Important notes from:  https://esp32.com/viewtopic.php?t=8234:  
        // 1. The I2S needs 16-bit samples.
        // 2. The I2S reads the samples as 32-bit words and outputs the high 16-bit first and the low 16-bit second. 
        //      This means that if you want to send it 16-bit samples, you'll need to swap the even and odd 16-bit words.
        // 3. The DAC only plays the high byte of the 16-bit sample as an unsigned value (0..255, not -128..127).
        
        for(int i=0; i<vban_rx_sample_count; i++){
          // Input stream is signed int16, with min -32768 and max +32768.
          // Output needs to be uint16 with audio range in upper 8 bits,
          // Therefore input value -32768 maps to output value +32768
          //       and input value +32768 maps to output value +65535.
          // So:  Outval = Inval*0.5 + 65535*3/4
          // You could do this with the Arduino map function.
          // Temporarily convert to signed int32 to scale and adjust offset.
          // Note:  index i^1 (XOR 1) to swap even and odd samples, per note above.
          outBuf[i^1] = (uint16_t) ((int32_t)(vban_rx_data[i]*0.5 + 49152));         
        }     

        ESP_ERROR_CHECK( i2s_write( i2s_num, &outBuf, vban_rx_sample_count*sizeof(uint16_t), &bytesOut, portMAX_DELAY ) );
        //Serial.printf("pkt %04ul, %u vals, %i, %i, %i\n",vban_rx_pkt_nbr, vban_rx_data_vals, pkt_data_min_val, pkt_data_max_val, bytesOut);

        // consider checking for gaps in vban frame numbers
      }      
    }    
  }
}
