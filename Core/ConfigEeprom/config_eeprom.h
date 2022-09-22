/*
 * config_eeprom.h
 *
 *  Created on: 22.09.2022
 *      Author: kai
 */

#ifndef CONFIGEEPROM_CONFIG_EEPROM_H_
#define CONFIGEEPROM_CONFIG_EEPROM_H_

#include "EEPROM.h"
/*
 * configuration is stored as key value pair.
 * Each value is stored as double and uses four bytes of the eeprom.
 * Each module has a designated eeprom page.
 */
namespace configuration
{

// L1 params are stored into page 1
namespace l1_params
{
constexpr uint8_t l1_page = 1;
struct page_content
{
	double kp;
	double ki;
	double kd;
	double bias;
};
void save_kp(double kp)
{
	EEPROM_Write_NUM(l1_page, 0, kp);
}
void save_ki(double ki)
{
	EEPROM_Write_NUM(l1_page, 4, ki);
}
void save_kd(double kd)
{
	EEPROM_Write_NUM(l1_page, 8, kd);
}
void save_bias(double bias)
{
	EEPROM_Write_NUM(l1_page, 12, bias);
}

void write_page(double kp, double ki,double kd, double bias){
	save_kp(kp);
	save_ki(ki);
	save_kd(kd);
	save_bias(bias);
}
page_content load_page()
{
	page_content cont;
	cont.kp = EEPROM_Read_NUM(l1_page, 0);
	cont.ki = EEPROM_Read_NUM(l1_page, 4);
	cont.kd = EEPROM_Read_NUM(l1_page, 8);
	cont.bias = EEPROM_Read_NUM(l1_page, 12);

	printf(" Kp = %.4f\r\n Ki= %.4f \r\n kd= %.4f \r\n bias = %.4f \r\n",
			cont.kp,cont.ki,cont.kd,cont.bias);
	return cont;
}
}
;
void recall();
void blank();
void upload_hexfile();
void download_hexfile();

}
;

#endif /* CONFIGEEPROM_CONFIG_EEPROM_H_ */
