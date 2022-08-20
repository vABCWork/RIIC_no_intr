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
	
	clear_module_stop();	//  モジュールストップの解除
	
		
	RIIC0_Port_Set();	//  I2C(SMBus)インターフェイス用のポート設定	
	RIIC0_Init();		//  I2C(SMBus)インターフェイス の初期化
	
	delay_msec(100);	// センサ安定待ち (100msec 待つ) 
	
	test_aht25();		// 温湿度センサ (AHT25) のテスト
	
	//test_thermo_pile();	// サーモパイルのテスト

}


//
// サーモパイルのテスト(割り込み未使用)
//
void test_thermo_pile(void)
{			
	iic_slave_adrs = 0x3d;    	//  スレーブアドレス = 0x3D (7bit; 011 1101)  (サーモパイル A3D01S)
		
	while (1) {
	     rd_thermo_pile_no_intr(0);    // センサの周囲温度を読み出し、周囲温度の計算
	     
	     delay_msec(50);		  // 50msec 待つ 
	
	     rd_thermo_pile_no_intr(1);    // 測定対象物の温度を読み出し、対象物の温度の計算
	     
	     delay_msec(50);		  // 50msec 待つ 
	}
}





//
// 温湿度センサ (AHT25) のテスト
//
//
void test_aht25(void)
{
	iic_slave_adrs = 0x38;    	//  スレーブアドレス = 0x3B (温湿度センサ AHT25)

	riic_sensor_rd_status();	//  温湿度センサのステータス読み出し (割り込み未使用)
 
	if ( riic_sensor_status != 0x18 ) {  // calibration check, (温湿度読み出し後、電源ONのままだと 0x1Cになる)
		aht25_reg_ini( 0x1B );	// レジスタ0x1B 初期化
	 	aht25_reg_ini( 0x1C );	// レジスタ0x1C 初期化
		aht25_reg_ini( 0x1E );	// レジスタ0x1E 初期化
		
		delay_msec(10);		// 10msec 待つ
	}


	while(1) {
	    riic_sensor_wr_cmd();	// 温湿度センサへのコマンド書き込み(割り込み未使用)
  
	    delay_msec(80);		// 80msec 待つ
		
	    riic_sensor_rd_humi_temp(); //  温湿度センサの温度と湿度の読み出し(割り込み未使用)
	
	    delay_msec(2000);		// 2秒 待つ　(頻繁に測定するとセンサの温度上昇により精度に影響が出る)
	}
	
}



// モジュールストップの解除
//  I2C バスインタフェース(RIICa)
//  CRC 演算器（CRC）(RIIC I2C通信用)
//
void clear_module_stop(void)
{
	SYSTEM.PRCR.WORD = 0xA50F;	// クロック発生、消費電力低減機能関連レジスタの書き込み許可	

	MSTP(RIIC0) = 0;                //  RIIC0モジュールストップ解除 (I2C通信)
	MSTP(CRC) = 0;			// CRC モジュールストップの解除
	
	SYSTEM.PRCR.WORD = 0xA500;	// クロック発生、消費電力低減機能関連レジスタ書き込み禁止
}

