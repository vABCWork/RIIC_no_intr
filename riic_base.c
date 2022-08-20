
#include "iodefine.h"
#include "misratypes.h"
#include "riic_base.h"



uint8_t iic_slave_adrs;  // IIC �X���[�u�A�h���X  00: 7bit�A�h���X( ��:100 0000 = 0x40 )


volatile uint8_t iic_rcv_data[16];   // IIC��M�f�[�^
volatile uint8_t iic_sd_data[32];    // ���M�f�[�^

				// �T�[���p�C���p��CRC-8(SMBus PEC�R�[�h)(x8 + x2 + x1 + 1)
uint8_t smbus_crc_8;	        //  CRC���Z�Ώۃf�[�^: �ŏ��̃X���[�u�A�h���X����A�R�}���h�A�X���[�u�A�h���X(Read�p)�A��M�f�[�^(low���j�A��M�f�[�^(high��)��5�o�C�g��
uint8_t smbus_crc_ng;          // ��L��5byte�ɁA�X���[�u�����瑗�M���ꂽ�APEC�����āACRC�v�Z�����l=0�Ȃ�΁A�ُ�Ȃ��B
			        // ( 32.2.3 CRC �f�[�^�o�̓��W�X�^�iCRCDOR�j�uRX23E-A�O���[�v ���[�U�[�Y�}�j���A���@�n�[�h�E�F�A�ҁv (R01UH0801JJ0120 Rev.1.20) )  
			
uint8_t riic_sensor_status;	   // �Z���T�̃X�e�[�^�X
uint32_t riic_sensor_humidity;	   // �Z���T����̎��x�f�[�^ 10�{�����l (��: 784�Ȃ��78.4%)
uint32_t riic_sensor_temperature;  // �Z���T����̉��x�f�[�^ 10�{�����l (��: 784�Ȃ��78.4%)
	
float	float_sensor_humidity;		// �Z���T����̎��x�f�[�^
float	float_sensor_temperature;	// �Z���T����̉��x�f�[�^

uint8_t  crc_x8_x5_x4_1;	// �����x�Z���T�p�@CRC-8 (x8 + x5 + X4 + 1)
uint8_t  riic_crc_ng;


uint16_t  ta_word_temp;
float  ta_celsius;		// Self temperature (�Z���T�̎��͉��x Ta)[��]


uint16_t  to_word_temp;
float  to_celsius;		// Object temperature (����Ώە��̉��x To)[��]


// 
//  �����x�Z���T���瓾���f�[�^���A
//  ���x�Ɖ��x���v�Z����B
//    CRC�ُ�̏ꍇ�́A0�Ƃ���B
//
void Cal_humidity_temperature(void)
{
	uint32_t dt;
	uint32_t dt_h;
	uint32_t dt_m;
	uint32_t dt_l;
	
	
	crc_x8_x5_x4_1 = Calc_crc_x8_x5_x4_1(&iic_rcv_data[0],6);   // CRC-8(X8+X5+X4+1)�̌v�Z
	riic_crc_ng =  Calc_crc_x8_x5_x4_1(&iic_rcv_data[0],7);     // ���M���ꂽCRC���܂߂Čv�Z
	
	
	if ( riic_crc_ng == 0 ) { // CRC����v�����ꍇ�A�����x�̌v�Z
	
		dt_h = iic_rcv_data[1];		// ���x�f�[�^(b19-b12)
		dt_h = dt_h << 12;
	
        	dt_m = iic_rcv_data[2];		// ���x�f�[�^(b11-b4)
		dt_m = dt_m << 4;
	
		dt_l = iic_rcv_data[3];		// b7-b4: ���x�f�[�^(b3-b0)
		dt_l = dt_l >> 4;
	
		dt = dt_h | dt_m | dt_l;
	
		dt =  dt * 1000;		
		dt = dt >> 10;			// 1/1024 = 1/(2^10)
		dt = dt >> 10;
		riic_sensor_humidity = dt;     // ���x�f�[�^ (784�Ȃ��78.4%)
	
	
		dt_h = iic_rcv_data[3] & 0x0f; // b3-b0: ���x�f�[�^(b19-b16)
		dt_h = dt_h << 16;
	
		dt_m = iic_rcv_data[4];		// ���x�f�[�^(b15-b8)
		dt_m = dt_m << 8;
	
		dt_l = iic_rcv_data[5];		// ���x�f�[�^(b7-b0)
	
		dt = dt_h | dt_m | dt_l;
	
		dt =  dt * 200 *10;		
		dt = dt >> 10;
		dt = dt >> 10;
		dt = dt - 500;
	
		riic_sensor_temperature = dt;		// ���x�f�[�^ (283�Ȃ��28.3��)
	}
        else {
		riic_sensor_humidity = 0;
		riic_sensor_temperature = 0;
	}
	
	
	float_sensor_humidity = riic_sensor_humidity / 10.0;
	
	float_sensor_temperature = riic_sensor_temperature/ 10.0;
	
}



// CRC-8�̌v�Z (AHT25�p)
// CRC-8-Maxim: X8+X5+X4+1 (0x31) �����l=0xff
//
// ���L�T���v���v���O���������p
// STM32 �� AHT20 ���[�`�� (aht20_stm32 demo v1_4)�v (http://www.aosong.com/class-36.html)
// 
//
uint8_t Calc_crc_x8_x5_x4_1(volatile uint8_t *data, uint8_t num)
{
        uint8_t i;
        uint8_t pt;
        uint8_t crc;
	
	crc = 0xff;

	for ( pt = 0; pt < num; pt++ ) {
  
         crc ^= data[pt];
    
	 for ( i = 8 ;i >0 ; --i)  {
    
           if ( crc & 0x80 ) {
               crc = ( crc << 1 ) ^ 0x31;
	   }
           else{
	       crc = ( crc << 1 );
	   }
	 }
       }
 
       return crc;
}


// PEC�ɂ��f�[�^�`�F�b�N (�T�[���p�C���p)
// CRC-8-ATM: X8+X2+X1+1 (0x07) �����l=0x00
//
// ����M�f�[�^����ACPU��CRC���Z����g�p���ċ��߂�B
// ��:
//�@�@iic_sd_data[0] = 0x7a;   (�X���[�u�A�h���X=3D + R/W#(Write=0))
//    iic_sd_data[1] = 0x71;   (�R�}���h Object temperature read)
//    iic_sd_data[2] = 0x7b;   (�X���[�u�A�h���X=3D + R/W#(Read=1))
//
//    iic_rcv_data[0] = 0xdd;  (�Ώە��̉��x ���ʃo�C�g)
//    iic_rcv_data[1] = 0x01;  (�Ώە��̉��x ��ʃo�C�g)
//    iic_rcv_data[1] = 0xb8;  PEC(Packet error code)
//
//   �S�f�[�^(0x7a,0x71,0x7b,0xdd,0x01,0xb8)���ACRC.CRCDIR�ɓ����B
//   CRC.CRCDOR = 0�ł���΁A�f�[�^�Ɍ�薳���B
// 
// �Q�l: �uRX23E-A�O���[�v ���[�U�[�Y�}�j���A���@�n�[�h�E�F�A�� (R01UH0801JJ0120 Rev.1.20)�v
//�@�@�@�@32.2.3 CRC �f�[�^�o�̓��W�X�^�iCRCDOR�j
//    
void Cal_crc_thermo_pile(void)
{
	uint32_t i;
	
	CRC.CRCCR.BYTE = 0x85;		     // CRCDOR���W�X�^���N���A, MSB�t�@�[�X�g�ʐM�p��CRC�𐶐�, 8�r�b�gCRC�iX8 + X2 + X + 1�j

	for ( i = 0 ; i < 3 ; i++ ) {	     // CRC-8�̌v�Z(���M�f�[�^)
	   CRC.CRCDIR = iic_sd_data[i];
	}
	
	CRC.CRCDIR = iic_rcv_data[0];	    // CRC-8�̌v�Z(��M�f�[�^)
	CRC.CRCDIR = iic_rcv_data[1];
		     
	smbus_crc_8 = CRC.CRCDOR;	   // CRC�v�Z����(PEC)
 
	CRC.CRCDIR = iic_rcv_data[2];     // PEC�@
	       
	smbus_crc_ng = CRC.CRCDOR;        // ��M����PEC�܂�CRC�v�Z�B0�Ȃ�΃f�[�^����
}





// 
//  ���ˉ��x�v(�T�[���p�C��)���瓾���f�[�^���A
//  Self temperature��Object temperature���v�Z����B
//    CRC�ُ�̏ꍇ�́A0�Ƃ���B
//
void Cal_Ta_To_temperature(void)
{
	if ( smbus_crc_ng == 0 ) {   // CRC ����̏ꍇ
		     
	    if( iic_sd_data[1] == 0x70 ) {					// Ta(Self temperature)�̓ǂݏo�̏ꍇ
			  ta_word_temp =  iic_rcv_data[1];
		    	  ta_word_temp =  ( ta_word_temp << 8 );
		          ta_word_temp =  (ta_word_temp | iic_rcv_data[0]);
		          ta_celsius = ( ta_word_temp * 0.125) - 20.0;  
	     }
	     else if ( iic_sd_data[1] == 0x71 ){				// To(Object temperature)�̓ǂݏo�̏ꍇ
			  to_word_temp =  iic_rcv_data[1];
		          to_word_temp =  ( to_word_temp << 8 );
		          to_word_temp =  ( to_word_temp | iic_rcv_data[0]);
		          to_celsius = ( to_word_temp * 0.125) - 30.0;  
	      	        }
         }

	 else{			// PEC �ُ�
		ta_celsius = 0.0;
		to_celsius = 0.0;
	 }

	
}


//  I2C(SMBus)�C���^�[�t�F�C�X �̏����� 
// 
//   	PORT16 = SCL
//      PORT17 = SDA
//
//      PCLKB = 32MHz:
//
//      �]�����x= 1 / { ( (ICBRH + 1) + (ICBRL + 1) ) / (IIC Phy) + SCLn ���C�������オ�莞��(tr) + SCLn ���C�����������莞��(tf) }
//
//     ( The maximum frequency of the MLX90614 SMBus is 100KHz and the minimum is 10 KHz.)
//
//     1) IIC������N���b�N(IIC Phy) = 4MHz = (32/8)MHz �̏ꍇ
//     �@ ICBRH=15(0xEF), ICBRL=18(0xF2)
//   
//        SCL0���C���̗����オ�莞��(tr)��1000 ns�ASCL0���C���̗��������莞��(tf)��300 ns
//
//       �]�����x = 1 / { (15+1+18+1)/(4 MHz) + 1000nsec+300nsec} = 1 / ( 8.75usec + 1.3usec) = 1/(10.05usec) => 99.5 Kbps
//       (������  29.2.14 I2C �o�X�r�b�g���[�gHigh ���W�X�^(ICBRH)�@���)
//
//     ( ����:�u RX23E-A�O���[�v ���[�U�[�Y�}�j���A���@�n�[�h�E�F�A�ҁv (R01UH0801JJ0120 Rev.1.20)�j 
//


void RIIC0_Init(void)
{
	RIIC0.ICCR1.BIT.ICE = 0;    // RIIC�͋@�\��~(SCL,SDA�[�q��쓮���)
	RIIC0.ICCR1.BIT.IICRST = 1; // RIIC���Z�b�g�A
	RIIC0.ICCR1.BIT.ICE = 1;    // �������Z�b�g��� �ASCL0�ASDA0�[�q�쓮���
		
	RIIC0.ICSER.BYTE = 0x00;    // I2C�o�X�X�e�[�^�X�����W�X�^ �i�}�X�^����̂��߃X���[�u�ݒ�͖���)	
	
				    //  �ʐM���x = 100 kbps 
	RIIC0.ICMR1.BIT.CKS = 3;    // RIIC�̓�����N���b�N = 32/8 = 4 MHz�@
	RIIC0.ICBRH.BIT.BRH = 0xEF; // �����́@�u�\29.5 �]�����x�ɑ΂���ICBRH�AICBRL���W�X�^�̐ݒ��v��� (PCLK=PCLKB=32[MHz])
	RIIC0.ICBRL.BIT.BRL = 0xF2;
	
	RIIC0.ICMR3.BIT.ACKWP = 1;	// ACKBT�r�b�g�ւ̏������݋���		
						
					
					
	RIIC0.ICMR3.BIT.RDRFS = 1;	// RDRF�t���O(��M�f�[�^�t��)�Z�b�g�^�C�~���O
					// 1�FRDRF �t���O��8 �N���b�N�ڂ̗����オ��Łg1�h �ɂ��A8 �N���b�N�ڂ̗����������SCL0 ���C����Low �Ƀz�[���h���܂��B
					// ����SCL0 ���C����Low �z�[���h��ACKBT �r�b�g�ւ̏������݂ɂ���������܂��B
					//���̐ݒ�̂Ƃ��A�f�[�^��M��A�N�m���b�W�r�b�g���o�O��SCL0 ���C���������I��Low �Ƀz�[���h���邽�߁A
					// ��M�f�[�^�̓��e�ɉ�����ACK (ACKBT �r�b�g���g0�h) �܂���NACK (ACKBT �r�b�g���g1�h) �𑗏o���鏈�����\�ł��B
			
					
	RIIC0.ICMR3.BIT.WAIT = 0;	// WAIT�Ȃ� (9�N���b�N�ڂ�1�N���b�N�ڂ̊Ԃ�Low�Ƀz�[���h���Ȃ�)	
	
	RIIC0.ICMR3.BIT.SMBS = 1;       // SMBus�I�� 				
	
	 
	RIIC0.ICCR1.BIT.IICRST = 0;	 // RIIC���Z�b�g����
}




//
//
//  I2C(SMBus)�C���^�[�t�F�C�X�p�̃|�[�g��ݒ�
// 
//   	PORT16 = SCL
//      PORT17 = SDA
//

void RIIC0_Port_Set(void)
{
	
	MPC.PWPR.BIT.B0WI = 0;      // �}���`�t�@���N�V�����s���R���g���[���@�v���e�N�g����
    	MPC.PWPR.BIT.PFSWE = 1;     // PmnPFS ���C�g�v���e�N�g����
    
    	MPC.P16PFS.BYTE = 0x0f;     // PORT16 = SCL0
    	MPC.P17PFS.BYTE = 0x0f;     // PORT17 = SDA0
          
    	MPC.PWPR.BYTE = 0x80;      //  PmnPFS ���C�g�v���e�N�g �ݒ�
  
    	PORT1.PMR.BIT.B6 = 1;     // PORT16:���Ӄ��W���[���Ƃ��Ďg�p
    	PORT1.PMR.BIT.B7 = 1;     // PORT17:      :
}

