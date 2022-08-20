
extern	uint8_t iic_slave_adrs; 
extern  volatile uint8_t iic_rcv_data[16];
extern  volatile uint8_t iic_sd_data[32];


extern  uint8_t smbus_crc_8;
extern  uint8_t smbus_crc_ng; 

extern uint8_t riic_sensor_status;
extern float	float_sensor_humidity;	
extern float	float_sensor_temperature;

extern	uint8_t  crc_x8_x5_x4_1;
extern	uint8_t  riic_crc_ng;


void Cal_humidity_temperature(void);
uint8_t Calc_crc_x8_x5_x4_1(volatile uint8_t *data, uint8_t num);

void Cal_crc_thermo_pile(void);
void Cal_Ta_To_temperature(void);


void RIIC0_Init(void);
void RIIC0_Port_Set(void);