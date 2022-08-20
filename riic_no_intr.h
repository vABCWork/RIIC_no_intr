
void riic_sensor_rd_status(void);
void riic_sensor_wr_cmd(void);
void riic_sensor_rd_humi_temp(void);
void aht25_reg_ini (uint8_t reg_adrs );

void riic_master_snd_nbyte(uint32_t para_num);
void riic_master_rcv_nbyte(uint32_t rcv_num);

void rd_thermo_pile_no_intr(uint32_t rd_obj);
void riic_master_snd_rcv_3byte(void);