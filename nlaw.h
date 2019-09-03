/*******************************************
* nlaw.h
* 2019 7m4mon licenced under MIT.
* N-LAW is 12bit implementation of A-LAW
* For VoIP Using 12bit DAC/ADC.
* Please refer to simple_ip_trx project for specific usage.
*******************************************/


/* N-Law�̐L���e�[�u��, 128 elements */
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
        order = 6;              // �c�������̂�10����7�r�b�g�ڂ̉���4�r�b�g�L��  -> 10-4
        while(!(wave&0x0800)){  // �ŏ�ʃr�b�g���ǂ��ɂ��邩��T���i11�r�b�g�ڂ�1�ɂȂ�܂ō��V�t�g�j
            wave <<=1;          // ���V�t�g���čŏ�ʃr�b�g��11�܂ňړ�����B
            order--;            // ��ʃr�b�g�������܂Ō��炵�Ă���
        }
        wave >>=7;              //�E�V�t�g���� 11�r�b�g�ڂ�5�r�b�g�ڂɂ���B
        retval =16 * order + 32;    //32��0�ɂȂ�̂�32�𑫂��B
        retval += (wave & 0x0F);    //����4�r�b�g�𑫂�
    }
    retval *= pol;        //������߂�
    retval += 0x80;       //���_�̈ړ��BWAVE�t�@�C�� 8bit�݊�
    
    return (uint8_t)retval;
}
