#include "iodefine.h"
#include "misratypes.h"

#include "riic_base.h"
#include "riic_no_intr.h"



//  �����x�Z���T�̃X�e�[�^�X�ǂݏo�� (�}�X�^��M)(���荞�ݖ��g�p)
// RIIC ���M�o�b�t�@
//   �@iic_sd_data[0] : �X���[�u�A�h���X(7bit) + 1(R/W#�r�b�g=Read)
// RIIC ��M�o�b�t�@
//     iic_rcv_data[0]: �X�e�[�^�X

void riic_sensor_rd_status(void)
{
	iic_sd_data[0] = (( iic_slave_adrs << 1 ) | 0x01 ) ;  // �X���[�u�A�h���X����Ǐo��

	riic_master_rcv_nbyte( 1 );	// �}�X�^��M(���荞�ݖ��g�p) ( 1 byte )
	
	riic_sensor_status = iic_rcv_data[0];  // �Z���T�̃X�e�[�^�X
	
}




// �����x�Z���T�ւ̑���J�n�R�}���h���M�@(�}�X�^���M)(���荞�ݖ��g�p)
//	 IIC ���M�o�b�t�@
//   �@iic_sd_data[0] : �X���[�u�A�h���X(7bit) + 0(=wite)
//                [1] : Trigger measure(0xAC)
//                [2] : Data0(0x33)
//                [3] : Data1(0x00)
void riic_sensor_wr_cmd(void)
{
	
	iic_sd_data[0] = ( iic_slave_adrs << 1 ) ;  // �X���[�u�A�h���X�֏�������
	iic_sd_data[1] = 0xac;
	iic_sd_data[2] = 0x33;
	iic_sd_data[3] = 0x00;
	
	riic_master_snd_nbyte( 4 );	// �}�X�^���M(���荞�ݖ��g�p)

}




//  �����x�Z���T����X�e�[�^�X�Ɖ����x�f�[�^�̓ǂݏo�� (�}�X�^��M)(���荞�ݖ��g�p)
// RIIC ���M�o�b�t�@
//   �@iic_sd_data[0] : �X���[�u�A�h���X(7bit) + 1(=Read)
// RIIC ��M�o�b�t�@
//     iic_rcv_data[0]: �X�e�[�^�X
//             :   [1]: ���x�f�[�^(b19-b12)
//             :   [2]: ���x�f�[�^(b11-b4)
//             :   [3]��b7-b4: ���x�f�[�^(b3-b0)
//             :   [3]��b3-b0: ���x�f�[�^(b19-b16)
//             :   [4]: ���x�f�[�^(b15-b8)
//             :   [5]: ���x�f�[�^(b7-b0)
//             :   [6]: CRC 
void riic_sensor_rd_humi_temp(void)
{
	
	
	iic_sd_data[0] = (( iic_slave_adrs << 1 ) | 0x01 ) ;  // �X���[�u�A�h���X����Ǐo��

	riic_master_rcv_nbyte( 7 );	// �}�X�^��M(���荞�ݖ��g�p) ( 7 byte )(750usec������) (SCLK = 100 KHz)
	
	
	crc_x8_x5_x4_1 = Calc_crc_x8_x5_x4_1(&iic_rcv_data[0],6);   // CRC-8(X8+X5+X4+1)�̌v�Z
	riic_crc_ng =  Calc_crc_x8_x5_x4_1(&iic_rcv_data[0],7);     // ���M���ꂽCRC���܂߂Čv�Z
	
	 Cal_humidity_temperature();	//  ���x�Ɖ��x���v�Z
	 
}

//
// I2C �����x�Z���T(AHT25)�̃��W�X�^������
//
void aht25_reg_ini ( uint8_t reg_adrs )
{
	uint8_t d1, d2, d3;
	
	iic_sd_data[0] = ( iic_slave_adrs << 1 ) ;  // �X���[�u�A�h���X�֏�������
	iic_sd_data[1] = reg_adrs; 
	iic_sd_data[2] = 0x00;
	iic_sd_data[3] = 0x00;
	
	riic_master_snd_nbyte( 4 );	// �}�X�^���M(���荞�ݖ��g�p)
	
	delay_msec(5);		// 5msec �҂�
	
	iic_sd_data[0] = (( iic_slave_adrs << 1 ) | 0x01 ) ;  // �X���[�u�A�h���X����Ǐo��

	riic_master_rcv_nbyte( 3 );	// �}�X�^��M(���荞�ݖ��g�p) ( 3 byte )
	
	d1 = iic_rcv_data[0];
	d2 = iic_rcv_data[1];
	d3 = iic_rcv_data[2];
	
	delay_msec(10);		// 10msec �҂�
	
	iic_sd_data[0] = ( iic_slave_adrs << 1 ) ;  // �X���[�u�A�h���X�֏�������
	iic_sd_data[1] = ( 0xB0 | reg_adrs);
	iic_sd_data[2] = d2;
	iic_sd_data[3] = d3;
	
	riic_master_snd_nbyte( 4 );	// �}�X�^���M(���荞�ݖ��g�p)
}






// �}�X�^ ���M(���荞�ݖ��g�p)
// ����: ���M�o�C�g��(�X���[�u�A�h���X���܂�)
//
// ����:
//  1. �X���[�u�A�h���X��R/W# �r�b�g(W#=0) ���M
//  2. �R�}���h�ƃp�����[�^�̑��M

void riic_master_snd_nbyte(uint32_t sd_num)
{
	uint32_t i;
	
	
	while(RIIC0.ICCR2.BIT.BBSY != 0 ){ 	// �o�X�����Ԃ̊m�F (I2C�o�X�r�W�[��Ԃ̏ꍇ�A���[�v)
	}
	
	RIIC0.ICCR2.BIT.ST = 1;		// �X�^�[�g�R���f�B�V�����̔��s  (�}�X�^���M�̊J�n)
	while(RIIC0.ICSR2.BIT.START != 1 ) {    // �X�^�[�g�R���f�B�V�����̔��s�m�F (�m�F����Ȃ��ꍇ�A���[�v)
	}
	RIIC0.ICSR2.BIT.START = 0;	// �X�^�[�g�R���f�B�V�������o�t���O�̃N���A
	
	while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// ���M�f�[�^�G���v�e�B�̊m�F(ICDRT���W�X�^�ɑ��M�f�[�^����ꍇ�A���[�v)
	}
	
	RIIC0.ICDRT = iic_sd_data[0];  // ���M�f�[�^��������(�X���[�u�A�h���X��R/W# �r�b�g)
				       // R/W# �r�b�g=0 �̏ꍇ�A���M���[�h�ɂȂ�
	
	
	if ( RIIC0.ICSR2.BIT.NACKF == 0 ) {     // ACK��M�����ꍇ
           
	     for ( i = 0 ; i < sd_num - 1; i++ ) {
	        
	        while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// ���M�f�[�^�G���v�e�B�̊m�F(ICDRT���W�X�^�ɑ��M�f�[�^����ꍇ�A���[�v)
	        }
	
	        RIIC0.ICDRT = iic_sd_data[ i + 1 ];    // ���M�f�[�^��������(�R�}���h�܂��̓p�����[�^)
		
	     }
	       
	     while(RIIC0.ICSR2.BIT.TEND != 1 ){ 	// ���M�I���̊m�F�i���M���̏ꍇ�A���[�v) ,( 1�ɂȂ����:TDRE�t���O���g1�h �̏�ԂŁA9 �ڂ�SCL �̗����オ��)
	     }
	     
	     RIIC0.ICSR2.BIT.STOP = 0;		// �X�g�b�v�R���f�B�V�������o�t���O�̃N���A
	     RIIC0.ICCR2.BIT.SP = 1;		// �X�g�b�v�R���f�B�V�����̔��s�v���̐ݒ�
	}
	else {					// ��M�f�o�C�X����A�N�m���b�W���Ȃ�����(NACK ����M����) �ꍇ
	
	    RIIC0.ICSR2.BIT.STOP = 0;		// �X�g�b�v�R���f�B�V�������o�t���O�̃N���A
	    RIIC0.ICCR2.BIT.SP = 1;		// �X�g�b�v�R���f�B�V�����̔��s�v���̐ݒ�
	}
	
	
	while(RIIC0.ICSR2.BIT.STOP != 1 ){ 	// �X�g�b�v�R���f�B�V�������o�̊m�F(�X�g�b�v�R���f�B�V���������o�ꍇ�A���[�v)
	}
	
	RIIC0.ICSR2.BIT.NACKF = 0;	  // NACK ���o�t���O�̃N���A
	RIIC0.ICSR2.BIT.STOP = 0;	 //  STOP ���o�t���O�̃N���A
}




// �}�X�^ ��M (���荞�ݖ��g�p)
// ����: rcv_num ��M�o�C�g��
//
// ����:
//  1. �X���[�u�A�h���X��R/W# �r�b�g(R=1) ���M
//  2. �_�~�[�f�[�^�̃��[�h�iSCL�@�N���b�N����)
//  3.�@��M�f�[�^�ǂݏo����ACK�܂���NACK���M�@�i�ŏI��M�f�[�^�̏ꍇ�́ANACK���M)
//

void riic_master_rcv_nbyte(uint32_t rcv_num)
{
	uint8_t dummy_data;
	uint32_t i;
	
	
	
	while(RIIC0.ICCR2.BIT.BBSY != 0 ){ 	// �o�X�����Ԃ̊m�F (I2C�o�X�r�W�[��Ԃ̏ꍇ�A���[�v)
	}
	
	RIIC0.ICCR2.BIT.ST = 1;		// �X�^�[�g�R���f�B�V�����̔��s  (�}�X�^���M�̊J�n)
	while(RIIC0.ICSR2.BIT.START != 1 ) {    // �X�^�[�g�R���f�B�V�����̔��s�m�F (�m�F����Ȃ��ꍇ�A���[�v)
	}
	RIIC0.ICSR2.BIT.START = 0;	// �X�^�[�g�R���f�B�V�������o�t���O�̃N���A
	
	while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// ���M�f�[�^�G���v�e�B�̊m�F(ICDRT���W�X�^�ɑ��M�f�[�^����ꍇ�A���[�v)
	}
	
	RIIC0.ICDRT = iic_sd_data[0];  // ���M�f�[�^��������(�X���[�u�A�h���X��R/W# �r�b�g)
					       // R/W# �r�b�g=1 �̏ꍇ�A��M���[�h�ɂȂ�
	
	while(RIIC0.ICSR2.BIT.RDRF != 1 ){ 	// ��M�f�[�^�t���t���O�̊m�F(CDRR���W�X�^�Ɏ�M�f�[�^�Ȃ��̏ꍇ�A���[�v)
	}
	
	if ( RIIC0.ICSR2.BIT.NACKF == 0 ) {     // ACK��M�����ꍇ
             
	     dummy_data = RIIC0.ICDRR;		// �_�~�[���[�h�@(SCL�N���b�N���o�͂��āA��M����J�n)
	  
	     for ( i = 0 ; i < rcv_num; i++ ) {
	         while(RIIC0.ICSR2.BIT.RDRF != 1 ){ 	// ��M�f�[�^�t���t���O(RDRF)�̊m�F(CDRR���W�X�^�Ɏ�M�f�[�^�Ȃ��̏ꍇ�A���[�v)
                 }
		 					// RDRFS�t���O( RIIC0.ICMR3.BIT.RDRFS)��1�̂��߁ARDRF�́A8 �N���b�N�ڂ̗����オ��Łg1�h�ƂȂ�B
                 iic_rcv_data[i] = RIIC0.ICDRR;         // ��M�f�[�^�ǂݏo��
	        
		 if ( i < ( rcv_num - 1 ) ) {           // �ŏI��M�f�[�^�łȂ��ꍇ
			 RIIC0.ICMR3.BIT.ACKBT = 0;	// ACK ���M�ݒ�  
		 }
		 else { 				// �ŏI��M�f�[�^�̏ꍇ
		   RIIC0.ICMR3.BIT.ACKBT = 1;		// NACK ���M�ݒ� 
	     	   RIIC0.ICSR2.BIT.STOP = 0;		// �X�g�b�v�R���f�B�V�������o�t���O�̃N���A
	           RIIC0.ICCR2.BIT.SP = 1;		// �X�g�b�v�R���f�B�V�����̔��s�v���̐ݒ�
		 }
	     }
	}
	else {					// ��M�f�o�C�X����A�N�m���b�W���Ȃ�����(NACK ����M����) �ꍇ
	
	    RIIC0.ICSR2.BIT.STOP = 0;		// �X�g�b�v�R���f�B�V�������o�t���O�̃N���A
	    RIIC0.ICCR2.BIT.SP = 1;		// �X�g�b�v�R���f�B�V�����̔��s�v���̐ݒ�
	   
	    dummy_data = RIIC0.ICDRR;		// �_�~�[���[�h�@(SCL�N���b�N���o�͂��āA��M����J�n)
	    
	}
	
	
	while(RIIC0.ICSR2.BIT.STOP != 1 ){ 	// �X�g�b�v�R���f�B�V�������o�̊m�F(�X�g�b�v�R���f�B�V���������o�ꍇ�A���[�v)
	}
	
	RIIC0.ICSR2.BIT.NACKF = 0;	  // NACK ���o�t���O�̃N���A
	RIIC0.ICSR2.BIT.STOP = 0;	 //  STOP ���o�t���O�̃N���A
}





//
//  ���ˉ��x�v(�T�[���p�C��)����̃f�[�^�ǂݏo���@(�}�X�^����M) �i���荞�ݖ��g�p)
//
// ����: rd_obj= 0: �Z���T�̎��͉��x(Self temperature)(TA)��ǂݏo��
///            = 1: ����ΏۑΏە��̉��x(TO)��ǂݏo��
//   IIC ���M�o�b�t�@
//   �@iic_sd_data[0] : �X���[�u�A�h���X(7bit) + 0(=Write)
//     iic_sd_data[1] : �R�}���h(�ǂݏo���A�h���X)  0x70=���͉��x�ǂݏo��, 0x71=����Ώۉ��x�ǂݏo��
//     iic_sd_data[2] : �X���[�u�A�h���X(7bit) + 1(=Read)
//
void rd_thermo_pile_no_intr(uint32_t rd_obj)
{
	
	
	if ( rd_obj == 0 ) {	// �Z���T�̎��͉��x(Self temperature)(TA)��ǂݏo��
		iic_sd_data[1] = 0x70;
		
	}
	else {				// ����ΏۑΏە��̉��x(TO)��ǂݏo��
		iic_sd_data[1] = 0x71;
		
	}
	
	iic_sd_data[0] = ( iic_slave_adrs << 1 );   // �������ݗp �X���[�u�A�h���X
	iic_sd_data[2] = (iic_sd_data[0] | 0x01);   // �ǂݏo���p�@�X���[�u�A�h���X 
	
	riic_master_snd_rcv_3byte();			// �}�X�^����M(���X�^�[�g���Ď�M)

	Cal_crc_thermo_pile();		// PEC�ɂ��f�[�^�`�F�b�N 
		     
	Cal_Ta_To_temperature();	// ���x�̌v�Z
	
	
}


// �}�X�^ ����M�@(���X�^�[�g���ă}�X�^��M)
//
// ����:
//  �X���[�u�A�h���X��R/W# �r�b�g(W#=0) ���M
//  ACK��M��: 
//    �R�}���h���M
//    ���X�^�[�g���s
//  �@�X���[�u�A�h���X��R/W# �r�b�g(R=1) ���M
//  �@�f�[�^(Low,Hig)��PEC�R�[�h�i���v 3byte) ��M
//  �@ACK���M
//    STOP�R���f�B�V�������s
//
//  NACK��M:
//    STOP�R���f�B�V�������s
//
void riic_master_snd_rcv_3byte(void)
{
	uint8_t dummy_data;
	uint32_t i;
	
	
	while(RIIC0.ICCR2.BIT.BBSY != 0 ){ 	// �o�X�����Ԃ̊m�F (I2C�o�X�r�W�[��Ԃ̏ꍇ�A���[�v)
	}
	
	RIIC0.ICCR2.BIT.ST = 1;		// �X�^�[�g�R���f�B�V�����̔��s  (�}�X�^���M�̊J�n)
	while(RIIC0.ICSR2.BIT.START != 1 ) {    // �X�^�[�g�R���f�B�V�����̔��s�m�F (�m�F����Ȃ��ꍇ�A���[�v)
	}
	RIIC0.ICSR2.BIT.START = 0;	// �X�^�[�g�R���f�B�V�������o�t���O�̃N���A
	
	
	while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// ���M�f�[�^�G���v�e�B�̊m�F(ICDRT���W�X�^�ɑ��M�f�[�^����ꍇ�A���[�v)
	}
	
	RIIC0.ICDRT = iic_sd_data[0];  // ���M�f�[�^��������(�X���[�u�A�h���X��R/W# �r�b�g)
				       // R/W# �r�b�g=0 �̏ꍇ�A���M���[�h�ɂȂ�
	
	
	if ( RIIC0.ICSR2.BIT.NACKF == 0 ) {     // ACK��M�����ꍇ
           		       
		while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// ���M�f�[�^�G���v�e�B�̊m�F(ICDRT���W�X�^�ɑ��M�f�[�^����ꍇ�A���[�v)
		}
	
		RIIC0.ICDRT = iic_sd_data[ 1 ];    // �R�}���h(�ǂݏo���A�h���X)���M   0x70(Selft temperature) or 0x71(Object temperature)

        	while(RIIC0.ICSR2.BIT.TEND != 1 ){ 	// ���M�I���̊m�F�i���M���̏ꍇ�A���[�v) ,( 1�ɂȂ����:TDRE�t���O���g1�h �̏�ԂŁA9 �ڂ�SCL �̗����オ��)
       		 }
	    
		RIIC0.ICCR2.BIT.RS = 1;		// ���X�^�[�g�R���f�B�V�����̔��s  
		while(RIIC0.ICSR2.BIT.START != 1 ) {    // ���X�^�[�g�R���f�B�V�����̔��s�m�F (�m�F����Ȃ��ꍇ�A���[�v)
		}
		RIIC0.ICSR2.BIT.START = 0;	// �X�^�[�g�R���f�B�V�������o�t���O�̃N���A
	
		RIIC0.ICDRT = iic_sd_data[2];       // ���M�f�[�^��������(�X���[�u�A�h���X��R/W# �r�b�g)
				               // R/W# �r�b�g=1 �̏ꍇ�A��M���[�h�ɂȂ�
	  
		while(RIIC0.ICSR2.BIT.RDRF != 1 ){ 	// ��M�f�[�^�t���t���O�̊m�F(CDRR���W�X�^�Ɏ�M�f�[�^�Ȃ��̏ꍇ�A���[�v)
		}
        
		dummy_data = RIIC0.ICDRR;		// �_�~�[���[�h�@(SCL�N���b�N���o�͂��āA��M����J�n)
	  
        	for ( i = 0 ; i < 3; i++ ) {
	    		while(RIIC0.ICSR2.BIT.RDRF != 1 ){ 	// ��M�f�[�^�t���t���O(RDRF)�̊m�F(CDRR���W�X�^�Ɏ�M�f�[�^�Ȃ��̏ꍇ�A���[�v)
            		}
		 					// RDRFS�t���O( RIIC0.ICMR3.BIT.RDRFS)��1�̂��߁ARDRF�́A8 �N���b�N�ڂ̗����オ��Łg1�h�ƂȂ�B
							// 8�N���b�N�ڂ̗������SCL���C����Low�z�[���h�B(ACKBT�r�b�g�������݂�Low�z�[���h����)
            		iic_rcv_data[i] = RIIC0.ICDRR;      // ��M�f�[�^�ǂݏo��
	        
             		RIIC0.ICMR3.BIT.ACKBT = 0;	// ACK ���M�ݒ� 
		
		}
	
		RIIC0.ICSR2.BIT.STOP = 0;	// �X�g�b�v�R���f�B�V�������o�t���O�̃N���A
		RIIC0.ICCR2.BIT.SP = 1;		// �X�g�b�v�R���f�B�V�����̔��s�v���̐ݒ�
	}
		
	else {					 // NACK��M�����ꍇ
		RIIC0.ICSR2.BIT.STOP = 0;	// �X�g�b�v�R���f�B�V�������o�t���O�̃N���A
		RIIC0.ICCR2.BIT.SP = 1;		// �X�g�b�v�R���f�B�V�����̔��s�v���̐ݒ�
	}
	
	while(RIIC0.ICSR2.BIT.STOP != 1 ){ 	// �X�g�b�v�R���f�B�V�������o�̊m�F(�X�g�b�v�R���f�B�V���������o�ꍇ�A���[�v)
	}
	
	RIIC0.ICSR2.BIT.NACKF = 0;	  // NACK ���o�t���O�̃N���A
	RIIC0.ICSR2.BIT.STOP = 0;	 //  STOP ���o�t���O�̃N���A
}













// �}�X�^ ����M�@(���X�^�[�g���ă}�X�^��M)(��)
//
// ����:
//  1. �X���[�u�A�h���X��R/W# �r�b�g(W#=0) ���M
//  2. �R�}���h���M
//  3. ���X�^�[�g���s
//  4. �X���[�u�A�h���X��R/W# �r�b�g(R=1) ���M
//  5. �f�[�^(Low,Hig)��PEC�R�[�h�i���v 3byte) ��M
//  6. ACK���M�ASTOP�R���f�B�V�������s

void riic_master_snd_rcv_3byte_old(void)
{
	uint8_t dummy_data;
	uint32_t i;
	
	
	while(RIIC0.ICCR2.BIT.BBSY != 0 ){ 	// �o�X�����Ԃ̊m�F (I2C�o�X�r�W�[��Ԃ̏ꍇ�A���[�v)
	}
	
	RIIC0.ICCR2.BIT.ST = 1;		// �X�^�[�g�R���f�B�V�����̔��s  (�}�X�^���M�̊J�n)
	while(RIIC0.ICSR2.BIT.START != 1 ) {    // �X�^�[�g�R���f�B�V�����̔��s�m�F (�m�F����Ȃ��ꍇ�A���[�v)
	}
	RIIC0.ICSR2.BIT.START = 0;	// �X�^�[�g�R���f�B�V�������o�t���O�̃N���A
	
	
	while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// ���M�f�[�^�G���v�e�B�̊m�F(ICDRT���W�X�^�ɑ��M�f�[�^����ꍇ�A���[�v)
	}
	
	RIIC0.ICDRT = iic_sd_data[0];  // ���M�f�[�^��������(�X���[�u�A�h���X��R/W# �r�b�g)
				       // R/W# �r�b�g=0 �̏ꍇ�A���M���[�h�ɂȂ�
	
	while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// ���M�f�[�^�G���v�e�B�̊m�F(ICDRT���W�X�^�ɑ��M�f�[�^����ꍇ�A���[�v)
	}
	
	RIIC0.ICDRT = iic_sd_data[ 1 ];    // �R�}���h(�ǂݏo���A�h���X)���M   0x70(Selft temperature) or 0x71(Object temperature)

        while(RIIC0.ICSR2.BIT.TEND != 1 ){ 	// ���M�I���̊m�F�i���M���̏ꍇ�A���[�v) ,( 1�ɂȂ����:TDRE�t���O���g1�h �̏�ԂŁA9 �ڂ�SCL �̗����オ��)
        }
	    
	RIIC0.ICCR2.BIT.RS = 1;		// ���X�^�[�g�R���f�B�V�����̔��s  
	while(RIIC0.ICSR2.BIT.START != 1 ) {    // ���X�^�[�g�R���f�B�V�����̔��s�m�F (�m�F����Ȃ��ꍇ�A���[�v)
	}
	RIIC0.ICSR2.BIT.START = 0;	// �X�^�[�g�R���f�B�V�������o�t���O�̃N���A
	
	
	RIIC0.ICDRT = iic_sd_data[2];       // ���M�f�[�^��������(�X���[�u�A�h���X��R/W# �r�b�g)
				               // R/W# �r�b�g=1 �̏ꍇ�A��M���[�h�ɂȂ�
	  
	while(RIIC0.ICSR2.BIT.RDRF != 1 ){ 	// ��M�f�[�^�t���t���O�̊m�F(CDRR���W�X�^�Ɏ�M�f�[�^�Ȃ��̏ꍇ�A���[�v)
	}
        
	dummy_data = RIIC0.ICDRR;		// �_�~�[���[�h�@(SCL�N���b�N���o�͂��āA��M����J�n)
	  
        for ( i = 0 ; i < 3; i++ ) {
	    while(RIIC0.ICSR2.BIT.RDRF != 1 ){ 	// ��M�f�[�^�t���t���O(RDRF)�̊m�F(CDRR���W�X�^�Ɏ�M�f�[�^�Ȃ��̏ꍇ�A���[�v)
            }
		 				// RDRFS�t���O( RIIC0.ICMR3.BIT.RDRFS)��1�̂��߁ARDRF�́A8 �N���b�N�ڂ̗����オ��Łg1�h�ƂȂ�B
						// 8�N���b�N�ڂ̗������SCL���C����Low�z�[���h�B(ACKBT�r�b�g�������݂�Low�z�[���h����)
            iic_rcv_data[i] = RIIC0.ICDRR;      // ��M�f�[�^�ǂݏo��
	        
             RIIC0.ICMR3.BIT.ACKBT = 0;	// ACK ���M�ݒ� 
		
	}
	
	
	RIIC0.ICSR2.BIT.STOP = 0;		// �X�g�b�v�R���f�B�V�������o�t���O�̃N���A
	RIIC0.ICCR2.BIT.SP = 1;		// �X�g�b�v�R���f�B�V�����̔��s�v���̐ݒ�
		   
	while(RIIC0.ICSR2.BIT.STOP != 1 ){ 	// �X�g�b�v�R���f�B�V�������o�̊m�F(�X�g�b�v�R���f�B�V���������o�ꍇ�A���[�v)
	}
	
	RIIC0.ICSR2.BIT.NACKF = 0;	  // NACK ���o�t���O�̃N���A
	RIIC0.ICSR2.BIT.STOP = 0;	 //  STOP ���o�t���O�̃N���A
}





