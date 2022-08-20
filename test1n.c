#include	<machine.h>
#include	 "iodefine.h"
#include	 "misratypes.h"
#include	"delay.h"
#include 	"riic_base.h"
#include	"riic_no_intr.h"


void clear_module_stop(void);
void test_aht25(void);
void test_thermo_pile(void);

void main(void)
{
	
	clear_module_stop();	//  ���W���[���X�g�b�v�̉���
	
		
	RIIC0_Port_Set();	//  I2C(SMBus)�C���^�[�t�F�C�X�p�̃|�[�g�ݒ�	
	RIIC0_Init();		//  I2C(SMBus)�C���^�[�t�F�C�X �̏�����
	
	delay_msec(100);	// �Z���T����҂� (100msec �҂�) 
	
	test_aht25();		// �����x�Z���T (AHT25) �̃e�X�g
	
	//test_thermo_pile();	// �T�[���p�C���̃e�X�g

}


//
// �T�[���p�C���̃e�X�g(���荞�ݖ��g�p)
//
void test_thermo_pile(void)
{			
	iic_slave_adrs = 0x3d;    	//  �X���[�u�A�h���X = 0x3D (7bit; 011 1101)  (�T�[���p�C�� A3D01S)
		
	while (1) {
	     rd_thermo_pile_no_intr(0);    // �Z���T�̎��͉��x��ǂݏo���A���͉��x�̌v�Z
	     
	     delay_msec(50);		  // 50msec �҂� 
	
	     rd_thermo_pile_no_intr(1);    // ����Ώە��̉��x��ǂݏo���A�Ώە��̉��x�̌v�Z
	     
	     delay_msec(50);		  // 50msec �҂� 
	}
}





//
// �����x�Z���T (AHT25) �̃e�X�g
//
//
void test_aht25(void)
{
	iic_slave_adrs = 0x38;    	//  �X���[�u�A�h���X = 0x3B (�����x�Z���T AHT25)

	riic_sensor_rd_status();	//  �����x�Z���T�̃X�e�[�^�X�ǂݏo�� (���荞�ݖ��g�p)
 
	if ( riic_sensor_status != 0x18 ) {  // calibration check, (�����x�ǂݏo����A�d��ON�̂܂܂��� 0x1C�ɂȂ�)
		aht25_reg_ini( 0x1B );	// ���W�X�^0x1B ������
	 	aht25_reg_ini( 0x1C );	// ���W�X�^0x1C ������
		aht25_reg_ini( 0x1E );	// ���W�X�^0x1E ������
		
		delay_msec(10);		// 10msec �҂�
	}


	while(1) {
	    riic_sensor_wr_cmd();	// �����x�Z���T�ւ̃R�}���h��������(���荞�ݖ��g�p)
  
	    delay_msec(80);		// 80msec �҂�
		
	    riic_sensor_rd_humi_temp(); //  �����x�Z���T�̉��x�Ǝ��x�̓ǂݏo��(���荞�ݖ��g�p)
	
	    delay_msec(2000);		// 2�b �҂@(�p�ɂɑ��肷��ƃZ���T�̉��x�㏸�ɂ�萸�x�ɉe�����o��)
	}
	
}



// ���W���[���X�g�b�v�̉���
//  I2C �o�X�C���^�t�F�[�X(RIICa)
//  CRC ���Z��iCRC�j(RIIC I2C�ʐM�p)
//
void clear_module_stop(void)
{
	SYSTEM.PRCR.WORD = 0xA50F;	// �N���b�N�����A����d�͒ጸ�@�\�֘A���W�X�^�̏������݋���	

	MSTP(RIIC0) = 0;                //  RIIC0���W���[���X�g�b�v���� (I2C�ʐM)
	MSTP(CRC) = 0;			// CRC ���W���[���X�g�b�v�̉���
	
	SYSTEM.PRCR.WORD = 0xA500;	// �N���b�N�����A����d�͒ጸ�@�\�֘A���W�X�^�������݋֎~
}

