/*******************************************
* nlaw.h
* 2019 7m4mon licenced under MIT.
* N-LAW is 12bit implementation of A-LAW
* For VoIP Using 12bit DAC/ADC.
* Please refer to simple_ip_trx project for specific usage.
*******************************************/


/* N-Lawの伸長テーブル, 128 elements */
const int16_t nlaw_decompress_table [] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 
    33, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 
    66, 70, 74, 78, 82, 86, 90, 94, 98, 102, 106, 110, 114, 118, 122, 126, 
    132, 140, 148, 156, 164, 172, 180, 188, 196, 204, 212, 220, 228, 236, 244, 252, 
    264, 280, 296, 312, 328, 344, 360, 376, 392, 408, 424, 440, 456, 472, 488, 504, 
    528, 560, 592, 624, 656, 688, 720, 752, 784, 816, 848, 880, 912, 944, 976, 1008, 
    1056, 1120, 1184, 1248, 1312, 1376, 1440, 1504, 1568, 1632, 1696, 1760, 1824, 1888, 1952, 2018, 
//    2048, 
};

int16_t nlaw_decompress(uint8_t nlaw_wave){
    int16_t linear_wave, pol;
    linear_wave = nlaw_wave - 0x80;
    pol = (linear_wave < 0) ? -1:1;
    linear_wave *= pol;
    linear_wave = nlaw_decompress_table[linear_wave];
    linear_wave *= pol;
    return linear_wave;
}

uint8_t nlaw_compress(int16_t /* 12bit Signed */ wave){
    int16_t pol;
    int8_t order;
    int16_t retval;
    pol = (wave < 0) ? -1:1;
    wave *= pol;
    if(wave > 2047) wave = 2047;
    
    if (wave < 32){
        retval = wave;
    }else{
        order = 6;              // 残したいのは10から7ビット目の下位4ビット有効  -> 10-4
        while(!(wave&0x0800)){  // 最上位ビットがどこにあるかを探す（11ビット目が1になるまで左シフト）
            wave <<=1;          // 左シフトして最上位ビットを11まで移動する。
            order--;            // 上位ビットが現れるまで減らしていく
        }
        wave >>=7;              //右シフトして 11ビット目を5ビット目にする。
        retval =16 * order + 32;    //32は0になるので32を足す。
        retval += (wave & 0x0F);    //下位4ビットを足す
    }
    retval *= pol;        //符号を戻す
    retval += 0x80;       //中点の移動。WAVEファイル 8bit互換
    
    return (uint8_t)retval;
}
