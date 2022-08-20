
#include "iodefine.h"
#include "misratypes.h"
#include "riic_base.h"



uint8_t iic_slave_adrs;  // IIC スレーブアドレス  00: 7bitアドレス( 例:100 0000 = 0x40 )


volatile uint8_t iic_rcv_data[16];   // IIC受信データ
volatile uint8_t iic_sd_data[32];    // 送信データ

				// サーモパイル用のCRC-8(SMBus PECコード)(x8 + x2 + x1 + 1)
uint8_t smbus_crc_8;	        //  CRC演算対象データ: 最初のスレーブアドレスから、コマンド、スレーブアドレス(Read用)、受信データ(low側）、受信データ(high側)の5バイト文
uint8_t smbus_crc_ng;          // 上記の5byteに、スレーブ側から送信された、PECを入れて、CRC計算した値=0ならば、異常なし。
			        // ( 32.2.3 CRC データ出力レジスタ（CRCDOR）「RX23E-Aグループ ユーザーズマニュアル　ハードウェア編」 (R01UH0801JJ0120 Rev.1.20) )  
			
uint8_t riic_sensor_status;	   // センサのステータス
uint32_t riic_sensor_humidity;	   // センサからの湿度データ 10倍した値 (例: 784ならば78.4%)
uint32_t riic_sensor_temperature;  // センサからの温度データ 10倍した値 (例: 784ならば78.4%)
	
float	float_sensor_humidity;		// センサからの湿度データ
float	float_sensor_temperature;	// センサからの温度データ

uint8_t  crc_x8_x5_x4_1;	// 温湿度センサ用　CRC-8 (x8 + x5 + X4 + 1)
uint8_t  riic_crc_ng;


uint16_t  ta_word_temp;
float  ta_celsius;		// Self temperature (センサの周囲温度 Ta)[℃]


uint16_t  to_word_temp;
float  to_celsius;		// Object temperature (測定対象物の温度 To)[℃]


// 
//  温湿度センサから得たデータより、
//  湿度と温度を計算する。
//    CRC異常の場合は、0とする。
//
void Cal_humidity_temperature(void)
{
	uint32_t dt;
	uint32_t dt_h;
	uint32_t dt_m;
	uint32_t dt_l;
	
	
	crc_x8_x5_x4_1 = Calc_crc_x8_x5_x4_1(&iic_rcv_data[0],6);   // CRC-8(X8+X5+X4+1)の計算
	riic_crc_ng =  Calc_crc_x8_x5_x4_1(&iic_rcv_data[0],7);     // 送信されたCRCを含めて計算
	
	
	if ( riic_crc_ng == 0 ) { // CRCが一致した場合、温湿度の計算
	
		dt_h = iic_rcv_data[1];		// 湿度データ(b19-b12)
		dt_h = dt_h << 12;
	
        	dt_m = iic_rcv_data[2];		// 湿度データ(b11-b4)
		dt_m = dt_m << 4;
	
		dt_l = iic_rcv_data[3];		// b7-b4: 湿度データ(b3-b0)
		dt_l = dt_l >> 4;
	
		dt = dt_h | dt_m | dt_l;
	
		dt =  dt * 1000;		
		dt = dt >> 10;			// 1/1024 = 1/(2^10)
		dt = dt >> 10;
		riic_sensor_humidity = dt;     // 湿度データ (784ならば78.4%)
	
	
		dt_h = iic_rcv_data[3] & 0x0f; // b3-b0: 温度データ(b19-b16)
		dt_h = dt_h << 16;
	
		dt_m = iic_rcv_data[4];		// 温度データ(b15-b8)
		dt_m = dt_m << 8;
	
		dt_l = iic_rcv_data[5];		// 温度データ(b7-b0)
	
		dt = dt_h | dt_m | dt_l;
	
		dt =  dt * 200 *10;		
		dt = dt >> 10;
		dt = dt >> 10;
		dt = dt - 500;
	
		riic_sensor_temperature = dt;		// 温度データ (283ならば28.3℃)
	}
        else {
		riic_sensor_humidity = 0;
		riic_sensor_temperature = 0;
	}
	
	
	float_sensor_humidity = riic_sensor_humidity / 10.0;
	
	float_sensor_temperature = riic_sensor_temperature/ 10.0;
	
}



// CRC-8の計算 (AHT25用)
// CRC-8-Maxim: X8+X5+X4+1 (0x31) 初期値=0xff
//
// 下記サンプルプログラムより引用
// STM32 の AHT20 ルーチン (aht20_stm32 demo v1_4)」 (http://www.aosong.com/class-36.html)
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


// PECによるデータチェック (サーモパイル用)
// CRC-8-ATM: X8+X2+X1+1 (0x07) 初期値=0x00
//
// 送受信データから、CPUのCRC演算器を使用して求める。
// 例:
//　　iic_sd_data[0] = 0x7a;   (スレーブアドレス=3D + R/W#(Write=0))
//    iic_sd_data[1] = 0x71;   (コマンド Object temperature read)
//    iic_sd_data[2] = 0x7b;   (スレーブアドレス=3D + R/W#(Read=1))
//
//    iic_rcv_data[0] = 0xdd;  (対象物の温度 下位バイト)
//    iic_rcv_data[1] = 0x01;  (対象物の温度 上位バイト)
//    iic_rcv_data[1] = 0xb8;  PEC(Packet error code)
//
//   全データ(0x7a,0x71,0x7b,0xdd,0x01,0xb8)を、CRC.CRCDIRに入れる。
//   CRC.CRCDOR = 0であれば、データに誤り無し。
// 
// 参考: 「RX23E-Aグループ ユーザーズマニュアル　ハードウェア編 (R01UH0801JJ0120 Rev.1.20)」
//　　　　32.2.3 CRC データ出力レジスタ（CRCDOR）
//    
void Cal_crc_thermo_pile(void)
{
	uint32_t i;
	
	CRC.CRCCR.BYTE = 0x85;		     // CRCDORレジスタをクリア, MSBファースト通信用にCRCを生成, 8ビットCRC（X8 + X2 + X + 1）

	for ( i = 0 ; i < 3 ; i++ ) {	     // CRC-8の計算(送信データ)
	   CRC.CRCDIR = iic_sd_data[i];
	}
	
	CRC.CRCDIR = iic_rcv_data[0];	    // CRC-8の計算(受信データ)
	CRC.CRCDIR = iic_rcv_data[1];
		     
	smbus_crc_8 = CRC.CRCDOR;	   // CRC計算結果(PEC)
 
	CRC.CRCDIR = iic_rcv_data[2];     // PEC　
	       
	smbus_crc_ng = CRC.CRCDOR;        // 受信したPECまでCRC計算。0ならばデータ正常
}





// 
//  放射温度計(サーモパイル)から得たデータより、
//  Self temperatureとObject temperatureを計算する。
//    CRC異常の場合は、0とする。
//
void Cal_Ta_To_temperature(void)
{
	if ( smbus_crc_ng == 0 ) {   // CRC 正常の場合
		     
	    if( iic_sd_data[1] == 0x70 ) {					// Ta(Self temperature)の読み出の場合
			  ta_word_temp =  iic_rcv_data[1];
		    	  ta_word_temp =  ( ta_word_temp << 8 );
		          ta_word_temp =  (ta_word_temp | iic_rcv_data[0]);
		          ta_celsius = ( ta_word_temp * 0.125) - 20.0;  
	     }
	     else if ( iic_sd_data[1] == 0x71 ){				// To(Object temperature)の読み出の場合
			  to_word_temp =  iic_rcv_data[1];
		          to_word_temp =  ( to_word_temp << 8 );
		          to_word_temp =  ( to_word_temp | iic_rcv_data[0]);
		          to_celsius = ( to_word_temp * 0.125) - 30.0;  
	      	        }
         }

	 else{			// PEC 異常
		ta_celsius = 0.0;
		to_celsius = 0.0;
	 }

	
}


//  I2C(SMBus)インターフェイス の初期化 
// 
//   	PORT16 = SCL
//      PORT17 = SDA
//
//      PCLKB = 32MHz:
//
//      転送速度= 1 / { ( (ICBRH + 1) + (ICBRL + 1) ) / (IIC Phy) + SCLn ライン立ち上がり時間(tr) + SCLn ライン立ち下がり時間(tf) }
//
//     ( The maximum frequency of the MLX90614 SMBus is 100KHz and the minimum is 10 KHz.)
//
//     1) IIC内部基準クロック(IIC Phy) = 4MHz = (32/8)MHz の場合
//     　 ICBRH=15(0xEF), ICBRL=18(0xF2)
//   
//        SCL0ラインの立ち上がり時間(tr)を1000 ns、SCL0ラインの立ち下がり時間(tf)を300 ns
//
//       転送速度 = 1 / { (15+1+18+1)/(4 MHz) + 1000nsec+300nsec} = 1 / ( 8.75usec + 1.3usec) = 1/(10.05usec) => 99.5 Kbps
//       (資料の  29.2.14 I2C バスビットレートHigh レジスタ(ICBRH)　より)
//
//     ( 資料:「 RX23E-Aグループ ユーザーズマニュアル　ハードウェア編」 (R01UH0801JJ0120 Rev.1.20)） 
//


void RIIC0_Init(void)
{
	RIIC0.ICCR1.BIT.ICE = 0;    // RIICは機能停止(SCL,SDA端子非駆動状態)
	RIIC0.ICCR1.BIT.IICRST = 1; // RIICリセット、
	RIIC0.ICCR1.BIT.ICE = 1;    // 内部リセット状態 、SCL0、SDA0端子駆動状態
		
	RIIC0.ICSER.BYTE = 0x00;    // I2Cバスステータス許可レジスタ （マスタ動作のためスレーブ設定は無効)	
	
				    //  通信速度 = 100 kbps 
	RIIC0.ICMR1.BIT.CKS = 3;    // RIICの内部基準クロック = 32/8 = 4 MHz　
	RIIC0.ICBRH.BIT.BRH = 0xEF; // 資料の　「表29.5 転送速度に対するICBRH、ICBRLレジスタの設定例」より (PCLK=PCLKB=32[MHz])
	RIIC0.ICBRL.BIT.BRL = 0xF2;
	
	RIIC0.ICMR3.BIT.ACKWP = 1;	// ACKBTビットへの書き込み許可		
						
					
					
	RIIC0.ICMR3.BIT.RDRFS = 1;	// RDRFフラグ(受信データフル)セットタイミング
					// 1：RDRF フラグは8 クロック目の立ち上がりで“1” にし、8 クロック目の立ち下がりでSCL0 ラインをLow にホールドします。
					// このSCL0 ラインのLow ホールドはACKBT ビットへの書き込みにより解除されます。
					//この設定のとき、データ受信後アクノリッジビット送出前にSCL0 ラインを自動的にLow にホールドするため、
					// 受信データの内容に応じてACK (ACKBT ビットが“0”) またはNACK (ACKBT ビットが“1”) を送出する処理が可能です。
			
					
	RIIC0.ICMR3.BIT.WAIT = 0;	// WAITなし (9クロック目と1クロック目の間をLowにホールドしない)	
	
	RIIC0.ICMR3.BIT.SMBS = 1;       // SMBus選択 				
	
	 
	RIIC0.ICCR1.BIT.IICRST = 0;	 // RIICリセット解除
}




//
//
//  I2C(SMBus)インターフェイス用のポートを設定
// 
//   	PORT16 = SCL
//      PORT17 = SDA
//

void RIIC0_Port_Set(void)
{
	
	MPC.PWPR.BIT.B0WI = 0;      // マルチファンクションピンコントローラ　プロテクト解除
    	MPC.PWPR.BIT.PFSWE = 1;     // PmnPFS ライトプロテクト解除
    
    	MPC.P16PFS.BYTE = 0x0f;     // PORT16 = SCL0
    	MPC.P17PFS.BYTE = 0x0f;     // PORT17 = SDA0
          
    	MPC.PWPR.BYTE = 0x80;      //  PmnPFS ライトプロテクト 設定
  
    	PORT1.PMR.BIT.B6 = 1;     // PORT16:周辺モジュールとして使用
    	PORT1.PMR.BIT.B7 = 1;     // PORT17:      :
}

