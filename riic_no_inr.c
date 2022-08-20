#include "iodefine.h"
#include "misratypes.h"

#include "riic_base.h"
#include "riic_no_intr.h"



//  温湿度センサのステータス読み出し (マスタ受信)(割り込み未使用)
// RIIC 送信バッファ
//   　iic_sd_data[0] : スレーブアドレス(7bit) + 1(R/W#ビット=Read)
// RIIC 受信バッファ
//     iic_rcv_data[0]: ステータス

void riic_sensor_rd_status(void)
{
	iic_sd_data[0] = (( iic_slave_adrs << 1 ) | 0x01 ) ;  // スレーブアドレスから読出し

	riic_master_rcv_nbyte( 1 );	// マスタ受信(割り込み未使用) ( 1 byte )
	
	riic_sensor_status = iic_rcv_data[0];  // センサのステータス
	
}




// 温湿度センサへの測定開始コマンド送信　(マスタ送信)(割り込み未使用)
//	 IIC 送信バッファ
//   　iic_sd_data[0] : スレーブアドレス(7bit) + 0(=wite)
//                [1] : Trigger measure(0xAC)
//                [2] : Data0(0x33)
//                [3] : Data1(0x00)
void riic_sensor_wr_cmd(void)
{
	
	iic_sd_data[0] = ( iic_slave_adrs << 1 ) ;  // スレーブアドレスへ書き込み
	iic_sd_data[1] = 0xac;
	iic_sd_data[2] = 0x33;
	iic_sd_data[3] = 0x00;
	
	riic_master_snd_nbyte( 4 );	// マスタ送信(割り込み未使用)

}




//  温湿度センサからステータスと温湿度データの読み出し (マスタ受信)(割り込み未使用)
// RIIC 送信バッファ
//   　iic_sd_data[0] : スレーブアドレス(7bit) + 1(=Read)
// RIIC 受信バッファ
//     iic_rcv_data[0]: ステータス
//             :   [1]: 湿度データ(b19-b12)
//             :   [2]: 湿度データ(b11-b4)
//             :   [3]のb7-b4: 湿度データ(b3-b0)
//             :   [3]のb3-b0: 温度データ(b19-b16)
//             :   [4]: 温度データ(b15-b8)
//             :   [5]: 温度データ(b7-b0)
//             :   [6]: CRC 
void riic_sensor_rd_humi_temp(void)
{
	
	
	iic_sd_data[0] = (( iic_slave_adrs << 1 ) | 0x01 ) ;  // スレーブアドレスから読出し

	riic_master_rcv_nbyte( 7 );	// マスタ受信(割り込み未使用) ( 7 byte )(750usecかかる) (SCLK = 100 KHz)
	
	
	crc_x8_x5_x4_1 = Calc_crc_x8_x5_x4_1(&iic_rcv_data[0],6);   // CRC-8(X8+X5+X4+1)の計算
	riic_crc_ng =  Calc_crc_x8_x5_x4_1(&iic_rcv_data[0],7);     // 送信されたCRCを含めて計算
	
	 Cal_humidity_temperature();	//  湿度と温度を計算
	 
}

//
// I2C 温湿度センサ(AHT25)のレジスタ初期化
//
void aht25_reg_ini ( uint8_t reg_adrs )
{
	uint8_t d1, d2, d3;
	
	iic_sd_data[0] = ( iic_slave_adrs << 1 ) ;  // スレーブアドレスへ書き込み
	iic_sd_data[1] = reg_adrs; 
	iic_sd_data[2] = 0x00;
	iic_sd_data[3] = 0x00;
	
	riic_master_snd_nbyte( 4 );	// マスタ送信(割り込み未使用)
	
	delay_msec(5);		// 5msec 待つ
	
	iic_sd_data[0] = (( iic_slave_adrs << 1 ) | 0x01 ) ;  // スレーブアドレスから読出し

	riic_master_rcv_nbyte( 3 );	// マスタ受信(割り込み未使用) ( 3 byte )
	
	d1 = iic_rcv_data[0];
	d2 = iic_rcv_data[1];
	d3 = iic_rcv_data[2];
	
	delay_msec(10);		// 10msec 待つ
	
	iic_sd_data[0] = ( iic_slave_adrs << 1 ) ;  // スレーブアドレスへ書き込み
	iic_sd_data[1] = ( 0xB0 | reg_adrs);
	iic_sd_data[2] = d2;
	iic_sd_data[3] = d3;
	
	riic_master_snd_nbyte( 4 );	// マスタ送信(割り込み未使用)
}






// マスタ 送信(割り込み未使用)
// 入力: 送信バイト数(スレーブアドレスを含む)
//
// 処理:
//  1. スレーブアドレスとR/W# ビット(W#=0) 送信
//  2. コマンドとパラメータの送信

void riic_master_snd_nbyte(uint32_t sd_num)
{
	uint32_t i;
	
	
	while(RIIC0.ICCR2.BIT.BBSY != 0 ){ 	// バス解放状態の確認 (I2Cバスビジー状態の場合、ループ)
	}
	
	RIIC0.ICCR2.BIT.ST = 1;		// スタートコンディションの発行  (マスタ送信の開始)
	while(RIIC0.ICSR2.BIT.START != 1 ) {    // スタートコンディションの発行確認 (確認されない場合、ループ)
	}
	RIIC0.ICSR2.BIT.START = 0;	// スタートコンディション検出フラグのクリア
	
	while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// 送信データエンプティの確認(ICDRTレジスタに送信データある場合、ループ)
	}
	
	RIIC0.ICDRT = iic_sd_data[0];  // 送信データ書き込み(スレーブアドレスとR/W# ビット)
				       // R/W# ビット=0 の場合、送信モードになる
	
	
	if ( RIIC0.ICSR2.BIT.NACKF == 0 ) {     // ACK受信した場合
           
	     for ( i = 0 ; i < sd_num - 1; i++ ) {
	        
	        while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// 送信データエンプティの確認(ICDRTレジスタに送信データある場合、ループ)
	        }
	
	        RIIC0.ICDRT = iic_sd_data[ i + 1 ];    // 送信データ書き込み(コマンドまたはパラメータ)
		
	     }
	       
	     while(RIIC0.ICSR2.BIT.TEND != 1 ){ 	// 送信終了の確認（送信中の場合、ループ) ,( 1になる条件:TDREフラグが“1” の状態で、9 個目のSCL の立ち上がり)
	     }
	     
	     RIIC0.ICSR2.BIT.STOP = 0;		// ストップコンディション検出フラグのクリア
	     RIIC0.ICCR2.BIT.SP = 1;		// ストップコンディションの発行要求の設定
	}
	else {					// 受信デバイスからアクノリッジがなかった(NACK を受信した) 場合
	
	    RIIC0.ICSR2.BIT.STOP = 0;		// ストップコンディション検出フラグのクリア
	    RIIC0.ICCR2.BIT.SP = 1;		// ストップコンディションの発行要求の設定
	}
	
	
	while(RIIC0.ICSR2.BIT.STOP != 1 ){ 	// ストップコンディション検出の確認(ストップコンディション未検出場合、ループ)
	}
	
	RIIC0.ICSR2.BIT.NACKF = 0;	  // NACK 検出フラグのクリア
	RIIC0.ICSR2.BIT.STOP = 0;	 //  STOP 検出フラグのクリア
}




// マスタ 受信 (割り込み未使用)
// 入力: rcv_num 受信バイト数
//
// 処理:
//  1. スレーブアドレスとR/W# ビット(R=1) 送信
//  2. ダミーデータのリード（SCL　クロック生成)
//  3.　受信データ読み出しとACKまたはNACK送信　（最終受信データの場合は、NACK送信)
//

void riic_master_rcv_nbyte(uint32_t rcv_num)
{
	uint8_t dummy_data;
	uint32_t i;
	
	
	
	while(RIIC0.ICCR2.BIT.BBSY != 0 ){ 	// バス解放状態の確認 (I2Cバスビジー状態の場合、ループ)
	}
	
	RIIC0.ICCR2.BIT.ST = 1;		// スタートコンディションの発行  (マスタ送信の開始)
	while(RIIC0.ICSR2.BIT.START != 1 ) {    // スタートコンディションの発行確認 (確認されない場合、ループ)
	}
	RIIC0.ICSR2.BIT.START = 0;	// スタートコンディション検出フラグのクリア
	
	while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// 送信データエンプティの確認(ICDRTレジスタに送信データある場合、ループ)
	}
	
	RIIC0.ICDRT = iic_sd_data[0];  // 送信データ書き込み(スレーブアドレスとR/W# ビット)
					       // R/W# ビット=1 の場合、受信モードになる
	
	while(RIIC0.ICSR2.BIT.RDRF != 1 ){ 	// 受信データフルフラグの確認(CDRRレジスタに受信データなしの場合、ループ)
	}
	
	if ( RIIC0.ICSR2.BIT.NACKF == 0 ) {     // ACK受信した場合
             
	     dummy_data = RIIC0.ICDRR;		// ダミーリード　(SCLクロックを出力して、受信動作開始)
	  
	     for ( i = 0 ; i < rcv_num; i++ ) {
	         while(RIIC0.ICSR2.BIT.RDRF != 1 ){ 	// 受信データフルフラグ(RDRF)の確認(CDRRレジスタに受信データなしの場合、ループ)
                 }
		 					// RDRFSフラグ( RIIC0.ICMR3.BIT.RDRFS)が1のため、RDRFは、8 クロック目の立ち上がりで“1”となる。
                 iic_rcv_data[i] = RIIC0.ICDRR;         // 受信データ読み出し
	        
		 if ( i < ( rcv_num - 1 ) ) {           // 最終受信データでない場合
			 RIIC0.ICMR3.BIT.ACKBT = 0;	// ACK 送信設定  
		 }
		 else { 				// 最終受信データの場合
		   RIIC0.ICMR3.BIT.ACKBT = 1;		// NACK 送信設定 
	     	   RIIC0.ICSR2.BIT.STOP = 0;		// ストップコンディション検出フラグのクリア
	           RIIC0.ICCR2.BIT.SP = 1;		// ストップコンディションの発行要求の設定
		 }
	     }
	}
	else {					// 受信デバイスからアクノリッジがなかった(NACK を受信した) 場合
	
	    RIIC0.ICSR2.BIT.STOP = 0;		// ストップコンディション検出フラグのクリア
	    RIIC0.ICCR2.BIT.SP = 1;		// ストップコンディションの発行要求の設定
	   
	    dummy_data = RIIC0.ICDRR;		// ダミーリード　(SCLクロックを出力して、受信動作開始)
	    
	}
	
	
	while(RIIC0.ICSR2.BIT.STOP != 1 ){ 	// ストップコンディション検出の確認(ストップコンディション未検出場合、ループ)
	}
	
	RIIC0.ICSR2.BIT.NACKF = 0;	  // NACK 検出フラグのクリア
	RIIC0.ICSR2.BIT.STOP = 0;	 //  STOP 検出フラグのクリア
}





//
//  放射温度計(サーモパイル)からのデータ読み出し　(マスタ送受信) （割り込み未使用)
//
// 入力: rd_obj= 0: センサの周囲温度(Self temperature)(TA)を読み出す
///            = 1: 測定対象対象物の温度(TO)を読み出す
//   IIC 送信バッファ
//   　iic_sd_data[0] : スレーブアドレス(7bit) + 0(=Write)
//     iic_sd_data[1] : コマンド(読み出しアドレス)  0x70=周囲温度読み出し, 0x71=測定対象温度読み出し
//     iic_sd_data[2] : スレーブアドレス(7bit) + 1(=Read)
//
void rd_thermo_pile_no_intr(uint32_t rd_obj)
{
	
	
	if ( rd_obj == 0 ) {	// センサの周囲温度(Self temperature)(TA)を読み出す
		iic_sd_data[1] = 0x70;
		
	}
	else {				// 測定対象対象物の温度(TO)を読み出す
		iic_sd_data[1] = 0x71;
		
	}
	
	iic_sd_data[0] = ( iic_slave_adrs << 1 );   // 書き込み用 スレーブアドレス
	iic_sd_data[2] = (iic_sd_data[0] | 0x01);   // 読み出し用　スレーブアドレス 
	
	riic_master_snd_rcv_3byte();			// マスタ送受信(リスタートして受信)

	Cal_crc_thermo_pile();		// PECによるデータチェック 
		     
	Cal_Ta_To_temperature();	// 温度の計算
	
	
}


// マスタ 送受信　(リスタートしてマスタ受信)
//
// 処理:
//  スレーブアドレスとR/W# ビット(W#=0) 送信
//  ACK受信時: 
//    コマンド送信
//    リスタート発行
//  　スレーブアドレスとR/W# ビット(R=1) 送信
//  　データ(Low,Hig)とPECコード（合計 3byte) 受信
//  　ACK送信
//    STOPコンディション発行
//
//  NACK受信:
//    STOPコンディション発行
//
void riic_master_snd_rcv_3byte(void)
{
	uint8_t dummy_data;
	uint32_t i;
	
	
	while(RIIC0.ICCR2.BIT.BBSY != 0 ){ 	// バス解放状態の確認 (I2Cバスビジー状態の場合、ループ)
	}
	
	RIIC0.ICCR2.BIT.ST = 1;		// スタートコンディションの発行  (マスタ送信の開始)
	while(RIIC0.ICSR2.BIT.START != 1 ) {    // スタートコンディションの発行確認 (確認されない場合、ループ)
	}
	RIIC0.ICSR2.BIT.START = 0;	// スタートコンディション検出フラグのクリア
	
	
	while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// 送信データエンプティの確認(ICDRTレジスタに送信データある場合、ループ)
	}
	
	RIIC0.ICDRT = iic_sd_data[0];  // 送信データ書き込み(スレーブアドレスとR/W# ビット)
				       // R/W# ビット=0 の場合、送信モードになる
	
	
	if ( RIIC0.ICSR2.BIT.NACKF == 0 ) {     // ACK受信した場合
           		       
		while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// 送信データエンプティの確認(ICDRTレジスタに送信データある場合、ループ)
		}
	
		RIIC0.ICDRT = iic_sd_data[ 1 ];    // コマンド(読み出しアドレス)送信   0x70(Selft temperature) or 0x71(Object temperature)

        	while(RIIC0.ICSR2.BIT.TEND != 1 ){ 	// 送信終了の確認（送信中の場合、ループ) ,( 1になる条件:TDREフラグが“1” の状態で、9 個目のSCL の立ち上がり)
       		 }
	    
		RIIC0.ICCR2.BIT.RS = 1;		// リスタートコンディションの発行  
		while(RIIC0.ICSR2.BIT.START != 1 ) {    // リスタートコンディションの発行確認 (確認されない場合、ループ)
		}
		RIIC0.ICSR2.BIT.START = 0;	// スタートコンディション検出フラグのクリア
	
		RIIC0.ICDRT = iic_sd_data[2];       // 送信データ書き込み(スレーブアドレスとR/W# ビット)
				               // R/W# ビット=1 の場合、受信モードになる
	  
		while(RIIC0.ICSR2.BIT.RDRF != 1 ){ 	// 受信データフルフラグの確認(CDRRレジスタに受信データなしの場合、ループ)
		}
        
		dummy_data = RIIC0.ICDRR;		// ダミーリード　(SCLクロックを出力して、受信動作開始)
	  
        	for ( i = 0 ; i < 3; i++ ) {
	    		while(RIIC0.ICSR2.BIT.RDRF != 1 ){ 	// 受信データフルフラグ(RDRF)の確認(CDRRレジスタに受信データなしの場合、ループ)
            		}
		 					// RDRFSフラグ( RIIC0.ICMR3.BIT.RDRFS)が1のため、RDRFは、8 クロック目の立ち上がりで“1”となる。
							// 8クロック目の立下りでSCLラインのLowホールド。(ACKBTビット書き込みでLowホールド解除)
            		iic_rcv_data[i] = RIIC0.ICDRR;      // 受信データ読み出し
	        
             		RIIC0.ICMR3.BIT.ACKBT = 0;	// ACK 送信設定 
		
		}
	
		RIIC0.ICSR2.BIT.STOP = 0;	// ストップコンディション検出フラグのクリア
		RIIC0.ICCR2.BIT.SP = 1;		// ストップコンディションの発行要求の設定
	}
		
	else {					 // NACK受信した場合
		RIIC0.ICSR2.BIT.STOP = 0;	// ストップコンディション検出フラグのクリア
		RIIC0.ICCR2.BIT.SP = 1;		// ストップコンディションの発行要求の設定
	}
	
	while(RIIC0.ICSR2.BIT.STOP != 1 ){ 	// ストップコンディション検出の確認(ストップコンディション未検出場合、ループ)
	}
	
	RIIC0.ICSR2.BIT.NACKF = 0;	  // NACK 検出フラグのクリア
	RIIC0.ICSR2.BIT.STOP = 0;	 //  STOP 検出フラグのクリア
}













// マスタ 送受信　(リスタートしてマスタ受信)(旧)
//
// 処理:
//  1. スレーブアドレスとR/W# ビット(W#=0) 送信
//  2. コマンド送信
//  3. リスタート発行
//  4. スレーブアドレスとR/W# ビット(R=1) 送信
//  5. データ(Low,Hig)とPECコード（合計 3byte) 受信
//  6. ACK送信、STOPコンディション発行

void riic_master_snd_rcv_3byte_old(void)
{
	uint8_t dummy_data;
	uint32_t i;
	
	
	while(RIIC0.ICCR2.BIT.BBSY != 0 ){ 	// バス解放状態の確認 (I2Cバスビジー状態の場合、ループ)
	}
	
	RIIC0.ICCR2.BIT.ST = 1;		// スタートコンディションの発行  (マスタ送信の開始)
	while(RIIC0.ICSR2.BIT.START != 1 ) {    // スタートコンディションの発行確認 (確認されない場合、ループ)
	}
	RIIC0.ICSR2.BIT.START = 0;	// スタートコンディション検出フラグのクリア
	
	
	while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// 送信データエンプティの確認(ICDRTレジスタに送信データある場合、ループ)
	}
	
	RIIC0.ICDRT = iic_sd_data[0];  // 送信データ書き込み(スレーブアドレスとR/W# ビット)
				       // R/W# ビット=0 の場合、送信モードになる
	
	while(RIIC0.ICSR2.BIT.TDRE != 1 ){ 	// 送信データエンプティの確認(ICDRTレジスタに送信データある場合、ループ)
	}
	
	RIIC0.ICDRT = iic_sd_data[ 1 ];    // コマンド(読み出しアドレス)送信   0x70(Selft temperature) or 0x71(Object temperature)

        while(RIIC0.ICSR2.BIT.TEND != 1 ){ 	// 送信終了の確認（送信中の場合、ループ) ,( 1になる条件:TDREフラグが“1” の状態で、9 個目のSCL の立ち上がり)
        }
	    
	RIIC0.ICCR2.BIT.RS = 1;		// リスタートコンディションの発行  
	while(RIIC0.ICSR2.BIT.START != 1 ) {    // リスタートコンディションの発行確認 (確認されない場合、ループ)
	}
	RIIC0.ICSR2.BIT.START = 0;	// スタートコンディション検出フラグのクリア
	
	
	RIIC0.ICDRT = iic_sd_data[2];       // 送信データ書き込み(スレーブアドレスとR/W# ビット)
				               // R/W# ビット=1 の場合、受信モードになる
	  
	while(RIIC0.ICSR2.BIT.RDRF != 1 ){ 	// 受信データフルフラグの確認(CDRRレジスタに受信データなしの場合、ループ)
	}
        
	dummy_data = RIIC0.ICDRR;		// ダミーリード　(SCLクロックを出力して、受信動作開始)
	  
        for ( i = 0 ; i < 3; i++ ) {
	    while(RIIC0.ICSR2.BIT.RDRF != 1 ){ 	// 受信データフルフラグ(RDRF)の確認(CDRRレジスタに受信データなしの場合、ループ)
            }
		 				// RDRFSフラグ( RIIC0.ICMR3.BIT.RDRFS)が1のため、RDRFは、8 クロック目の立ち上がりで“1”となる。
						// 8クロック目の立下りでSCLラインのLowホールド。(ACKBTビット書き込みでLowホールド解除)
            iic_rcv_data[i] = RIIC0.ICDRR;      // 受信データ読み出し
	        
             RIIC0.ICMR3.BIT.ACKBT = 0;	// ACK 送信設定 
		
	}
	
	
	RIIC0.ICSR2.BIT.STOP = 0;		// ストップコンディション検出フラグのクリア
	RIIC0.ICCR2.BIT.SP = 1;		// ストップコンディションの発行要求の設定
		   
	while(RIIC0.ICSR2.BIT.STOP != 1 ){ 	// ストップコンディション検出の確認(ストップコンディション未検出場合、ループ)
	}
	
	RIIC0.ICSR2.BIT.NACKF = 0;	  // NACK 検出フラグのクリア
	RIIC0.ICSR2.BIT.STOP = 0;	 //  STOP 検出フラグのクリア
}





