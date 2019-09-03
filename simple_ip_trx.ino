/*
 * Simple IP Transceiver with N-Law by 7m4mon.
 * Hardware : Arduino Due, W55000
 * 2019 licenced under MIT.
*/

#include "nlaw.h"

//#define PC_TEST
#define RX_DEBUG           0
#define SEND_BUFF_EN       1
#define SEND_BUFF_SIZE     1024       //MTUが1500バイトなので。ちなみにヘッダは42バイトでした。
#define SEND_PKT_SIZE      SEND_BUFF_SIZE

#define RECV_BUFF_SIZE      8192        //1.024秒分
#define RECV_PKT_BUFF_SIZE  RECV_BUFF_SIZE

/* pins */
// VCC, GND, RESET, MODE(IN), STATE(OUT), MICIN, SPOUT, PTT(IN), BUSY(OUT), UART_TX, UART_RX
#define PIN_MIC        A11
#define PIN_VREF       DAC0
#define PIN_PTT         52
#define PIN_TX_LED      50
#define PIN_AB_SEL      53
#define PIN_SP         DAC1
#define PIN_BUSY        48
#define PIN_ONB_LED     13
#define ETHERNET_SPI_CS_PIN   10        //default
//#define ETHERNET_RESET_PIN    //Resetと共通

/* for interrupt 8ksps */
#include <DueTimer.h>           // https://github.com/ivanseidel/DueTimer

/* for W5500 */
#include <SPI.h> 
#include <Ethernet2.h>
#include <EthernetUdp2.h>

byte mac[6];
IPAddress dst_ip_addr;
IPAddress my_ip_addr;

byte mac_a[] = { 0x10,0x69,0x69,0x2D,0x30,0x41 };
byte mac_b[] = { 0x10,0x69,0x69,0x2D,0x30,0x42 };
IPAddress ip_addr_a(192,168,0,1);
IPAddress ip_addr_b(192,168,0,2);
IPAddress ip_addr_test(192,168,0,3);

unsigned int ip_port = 8649;      // Hello CQ
EthernetUDP Udp;

bool ptt_en, play_en, send_ready;
int16_t buff_available, recv_buff_counter, play_buff_counter, send_buff_counter;

uint8_t recv_buffer[RECV_BUFF_SIZE];
uint8_t send_buffer[SEND_BUFF_SIZE];        //マイク入力をn-lawに変換して保持しておく
uint8_t send_packet[SEND_BUFF_SIZE];        //send_bufferがある一定数溜まったら、実際にSerial/W5500に渡すバッファ

void setup(){
    // ADC,DACを12bit精度にする。Mapleではデフォルト12bitらしい。
    analogReadResolution(12);
    analogWriteResolution(12);
    analogWrite(PIN_VREF, 2048);
    analogWrite(PIN_SP, 2048);

    pinMode(PIN_ONB_LED, OUTPUT);
    digitalWrite(PIN_ONB_LED, LOW);
    pinMode(PIN_TX_LED, OUTPUT);
    digitalWrite(PIN_TX_LED, LOW);    
    pinMode(PIN_AB_SEL, INPUT_PULLUP);
    
    // CONNECTION_UART (for DEBUG) 
    Serial2.begin(230400);
    
    // PTT
    pinMode(PIN_PTT, INPUT_PULLUP);
    
    // BUSY
    pinMode(PIN_BUSY, OUTPUT);
    digitalWrite(PIN_BUSY, LOW);

    //Ethernet set ip, mac by slide switch.
    if(digitalRead(PIN_AB_SEL) == LOW){
        memcpy(mac, mac_a, 6);
        my_ip_addr = ip_addr_a;
        dst_ip_addr = ip_addr_b;
    }else{
        memcpy(mac, mac_b, 6);
        my_ip_addr = ip_addr_b;
        dst_ip_addr = ip_addr_a;
    }
    #ifdef PC_TEST
    dst_ip_addr = ip_addr_test;
    #endif
    
    Ethernet.begin(mac, my_ip_addr);    //default CS pin is 10.
    Udp.begin(ip_port);

    // initialize valiable
    ptt_en = false;
    play_en = false;
    send_ready = false;
    buff_available = 0;
    recv_buff_counter = 0;
    play_buff_counter = 0;
    send_buff_counter = 0;
    memset(&recv_buffer[0],0x80,RECV_BUFF_SIZE);
    memset(&send_buffer[0],0x80,SEND_BUFF_SIZE);

    // Timer 8ksps(125us毎に割り込み)
    Timer1.attachInterrupt(timer_int_8ksps);
    Timer1.start(125); // Calls every 125us
    
}


void timer_int_8ksps(){
    
    digitalWrite(PIN_ONB_LED, HIGH);
    
    /*****************/
    /* adc -> serial */
    /*****************/
    // PTTの状態を取得。
    ptt_en = (digitalRead(PIN_PTT) == LOW) ? true : false;
    
    if(ptt_en){
        
        int16_t raw_wav;
        if(RX_DEBUG){
            raw_wav = send_buff_counter * 4;        //send_buff_counter == SEND_BUFF_SIZE == 1024として 鋸歯を発生
        }else{
            raw_wav = analogRead(PIN_MIC);
        }
        raw_wav -= 2048;    //中点を合わせる
        
        uint8_t nlaw_wav;
        nlaw_wav = nlaw_compress(raw_wav);   
        send_buffer[send_buff_counter++] = nlaw_wav;
        if(send_buff_counter == SEND_BUFF_SIZE){
            send_buff_counter = 0;
            memcpy(send_packet,send_buffer,SEND_BUFF_SIZE);
            send_ready = true;
        }
    }
    
    
    /*****************/
    /* serial -> dac */
    /*****************/
    if((buff_available > (RECV_BUFF_SIZE >> 1 )) && !play_en)
        { //バッファに半分溜まったら再生開始
            play_en = true;
        }
    // read to N-Law
    uint8_t read_byte;
    int16_t dac_out_data;
    if((buff_available > 0) && play_en)
    {
        read_byte = recv_buffer[play_buff_counter];
        dac_out_data = nlaw_decompress(read_byte);
        play_buff_counter++;
        play_buff_counter &= (RECV_BUFF_SIZE -1);
        buff_available--;
    }else{
        dac_out_data = 0;  
    }
    
    if(buff_available < 1){
        play_en = false;
    }
    
    dac_out_data += 2048;    //中点を合わせる。
    analogWrite(PIN_SP, dac_out_data);
    digitalWrite(PIN_BUSY, play_en);
    digitalWrite(PIN_TX_LED, ptt_en);
    digitalWrite(PIN_ONB_LED, LOW);    
}

//リングバッファ recv_buffer[recv_buff_counter] に貯めていき、buff_availableをカウントアップする。
void proc_rcv_byte(uint8_t read_byte){
    if(read_byte != 0)
    {
        recv_buffer[recv_buff_counter] = read_byte;
        recv_buff_counter++;
        recv_buff_counter &= (RECV_BUFF_SIZE -1);
        buff_available++;
    }
}

// W5500でUDPパケットを投げる。
// 先にint32_t dst_ip_addr, int32_t ip_port, を入れておくこと。
void send_udp_pkt(uint8_t* data_ptr, uint16_t pkt_size){
    //if(dst_ip_addr != null && ip_port != null)
    {
        Udp.beginPacket(dst_ip_addr, ip_port);
        Udp.write(data_ptr, pkt_size);
        Udp.endPacket();
    }
}



void loop(){
    uint8_t read_byte;
    /*
     * W5500から受信
     */
    int packetSize = Udp.parsePacket();
    uint16_t i;
    uint8_t packetBuffer[RECV_PKT_BUFF_SIZE];
    /* W5500 からの受信処理 */
    if (Udp.available()){
        IPAddress send_ip_addr = Udp.remoteIP();
        Udp.read(packetBuffer, packetSize); 
        if (send_ip_addr == dst_ip_addr){   
            for (i = 0; i < packetSize; i++){
                // 受信データ処理プロセスに回す。
                read_byte = packetBuffer[i];
                proc_rcv_byte(read_byte);
            }
        }
    }

    /*
     * バッファが一杯になったらネットワークに送信
     */
    //Transmit send_packet
    if(send_ready){
        //noInterrupts(); //  Bluepillでは送信中に割り込みが入るとデータが欠ける。（DMAを使えば解決かも）
        Udp.beginPacket(dst_ip_addr, ip_port);
        Udp.write(send_packet, SEND_PKT_SIZE);
        Udp.endPacket();
        //interrupts();
        
        send_ready = false;
    }
}
    
    

