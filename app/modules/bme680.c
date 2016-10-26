// ***************************************************************************
// BME680 module for ESP8266 with nodeMCU
// 
// Written by Ying Shaodong (shaodong.ying@sg.bosch.com)
// 
// MIT license, http://opensource.org/licenses/MIT
// ***************************************************************************

#include "module.h"
#include "lauxlib.h"
#include "platform.h"
#include "c_math.h"

#define NODEMCU_I2C_ID                      (0)
#define BME680_CHIP_ID                      (0x60)
#define BME680_I2C_ADDRESS1                 (0x76)
#define BME680_I2C_ADDRESS2                 (0x77)
/****************************************************/
/**\name	OVER SAMPLING DEFINITIONS  */
/***************************************************/
#define BME680_OVERSAMP_1X                  (0x01)
#define BME680_OVERSAMP_2X                  (0x02)
#define BME680_OVERSAMP_4X                  (0x03)
#define BME680_OVERSAMP_8X                  (0x04)
#define BME680_OVERSAMP_16X                 (0x05)

#define BME680_REGISTER_CHIPID              (0xD0)
#define BME680_REGISTER_CONTROL_HUM         (0x72)
#define BME680_REGISTER_CONTROL_MEAS        (0x74)
#define BME680_REGISTER_GAS_WAIT_0          (0x64)
#define BME680_REGISTER_RES_HEAT_0          (0x5A)
#define BME680_REGISTER_CONTROL_GAS_1       (0X71)
#define BME680_REGISTER_CONFIG              (0x75)

#define BME680_SLEEP_MODE                   (0x00)
#define BME680_FORCED_MODE                  (0x01)

#define BME680_S32_t int32_t
#define BME680_U32_t uint32_t
#define BME680_S64_t int64_t

#define BME680_SAMPLING_DELAY  113 //maximum measurement time in ms for maximum oversampling for all measures = 1.25 + 2.3*16 + 2.3*16 + 0.575 + 2.3*16 + 0.575 ms

#define r16s(addr, reg)  ((int16_t)r16u(addr, reg))
#define r16sLE(addr, reg)  ((int16_t)r16uLE(addr, reg))

#define bme680_adc_T(addr) r24u(addr, BME680_REGISTER_TEMP)
#define bme680_adc_P(addr) r24u(addr, BME680_REGISTER_PRESS)
#define bme680_adc_H(addr) r16u(addr, BME680_REGISTER_HUM)


static uint8_t bme680_osrs_t = BME680_OVERSAMP_2X;
static uint8_t bme680_osrs_p = BME680_OVERSAMP_16X;
static uint8_t bme680_osrs_h = BME680_OVERSAMP_1X;
static uint8_t bme680_heat_duration = 0x59;  // 100ms

os_timer_t bme680_timer; // timer for forced mode readout
int lua_connected_readout_ref; // callback when readout is ready

/*
static struct bme680_config_t {
    uint8_t     osrs_h;
    uint8_t     osrs_t;
    uint8_t     osrs_p;
    uint8_t     gas_wait_0;
    uint8_t     res_heat_0;
} bme680_config;

static struct {
	uint16_t  dig_T1;
	int16_t   dig_T2;
	int16_t   dig_T3;
	uint16_t  dig_P1;
	int16_t   dig_P2;
	int16_t   dig_P3;
	int16_t   dig_P4;
	int16_t   dig_P5;
	int16_t   dig_P6;
	int16_t   dig_P7;
	int16_t   dig_P8;
	int16_t   dig_P9;
	uint8_t   dig_H1;
	int16_t   dig_H2;
	uint8_t   dig_H3;
	int16_t   dig_H4;
	int16_t   dig_H5;
	int8_t    dig_H6;
} bme680_calib;

static BME680_S32_t bme680_t_fine;
static uint32_t bme680_h = 0;
static double bme680_hc = 0.0;
*/

static uint8_t r8u(uint8_t addr, uint8_t reg) {
	uint8_t ret;
	platform_i2c_send_start(NODEMCU_I2C_ID);
	platform_i2c_send_address(NODEMCU_I2C_ID, addr, PLATFORM_I2C_DIRECTION_TRANSMITTER);
	platform_i2c_send_byte(NODEMCU_I2C_ID, reg);
	platform_i2c_send_stop(NODEMCU_I2C_ID);
	platform_i2c_send_start(NODEMCU_I2C_ID);
	platform_i2c_send_address(NODEMCU_I2C_ID, addr, PLATFORM_I2C_DIRECTION_RECEIVER);
	ret = platform_i2c_recv_byte(NODEMCU_I2C_ID, 0);
	platform_i2c_send_stop(NODEMCU_I2C_ID);
	//NODE_DBG("reg:%x, value:%x \n", reg, ret);
	return ret;
}

static uint8_t w8u(uint8_t addr, uint8_t reg, uint8_t val) {
	platform_i2c_send_start(NODEMCU_I2C_ID);
	platform_i2c_send_address(NODEMCU_I2C_ID, addr, PLATFORM_I2C_DIRECTION_TRANSMITTER);
	platform_i2c_send_byte(NODEMCU_I2C_ID, reg);
	platform_i2c_send_byte(NODEMCU_I2C_ID, val);
	platform_i2c_send_stop(NODEMCU_I2C_ID);
}

static uint16_t r16u(uint8_t addr, uint8_t reg) {
	uint8_t high = r8u(addr, reg);
	uint8_t low  = r8u(addr, ++reg);
	return (high << 8) | low;
}

static uint16_t r16uLE(uint8_t addr, uint8_t reg) {
	uint8_t low = r8u(addr, reg);
	uint8_t high  = r8u(addr, ++reg);
	return (high << 8) | low;
}

static uint32_t r24u(uint8_t addr, uint8_t reg) {
	uint8_t high = r8u(addr, reg);
	uint8_t mid  = r8u(addr, ++reg);
	uint8_t low  = r8u(addr, ++reg);
	return (uint32_t)(((high << 16) | (mid << 8) | low) >> 4);
}

static uint8_t convertAddress(uint8_t id) {
    if(id == 1)
        return BME680_I2C_ADDRESS2;
    else
        return BME680_I2C_ADDRESS1;
}

static uint8_t calculate_res_heat(uint8_t addr, double target_temp, double amb_temp) {
    // formula from datasheet section 6.3.5
    uint8_t par_g1 = r8u(addr, 0xED);
    uint16_t par_g2 = r8u(addr, 0xEB) | (r8u(addr, 0xEC)<<8); 
    uint8_t par_g3 = r8u(addr, 0xEE);
    uint8_t res_heat_range = (r8u(addr, 0x02)>>4) & 0x3;      // not sure
    int8_t res_heat_val = (int8_t)r8u(addr, 0x0);
    double var1 = ((double)par_g1 / 16.0) + 49.0;
    double var2 = (((double)par_g2 / 32768.0) * 0.0005) + 0.00235;
    double var3 = (double)par_g3 / 1024.0;
    double var4 = var1 * (1.0 + (var2 * target_temp));
    //Use ambient temperature. This could be either hardcoded or read from temperature sensor
    double var5 = var4 + (var3 * amb_temp);
    uint8_t res_heat_x = (uint8_t)(((var5 * (4.0 / (4.0 + (double)res_heat_range)) - 25.0)) * 3.4 / (((double)res_heat_val * 0.002) + 1));
    return res_heat_x;
}

/*
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.  
// t_fine carries fine temperature as global value 
static BME680_S32_t bme680_compensate_T(BME680_S32_t adc_T) {
	BME680_S32_t var1, var2, T; 
	var1  = ((((adc_T>>3) - ((BME680_S32_t)bme680_calib.dig_T1<<1))) * ((BME680_S32_t)bme680_calib.dig_T2)) >> 11; 
	var2  = (((((adc_T>>4) - ((BME680_S32_t)bme680_calib.dig_T1)) * ((adc_T>>4) - ((BME680_S32_t)bme680_calib.dig_T1))) >> 12) *  
		((BME680_S32_t)bme680_calib.dig_T3)) >> 14; 
	bme680_t_fine = var1 + var2; 
	T  = (bme680_t_fine * 5 + 128) >> 8; 
	return T; 
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits). 
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa 
static BME680_U32_t bme680_compensate_P(BME680_S32_t adc_P) {
	BME680_S64_t var1, var2, p; 
	var1 = ((BME680_S64_t)bme680_t_fine) - 128000; 
	var2 = var1 * var1 * (BME680_S64_t)bme680_calib.dig_P6; 
	var2 = var2 + ((var1*(BME680_S64_t)bme680_calib.dig_P5)<<17); 
	var2 = var2 + (((BME680_S64_t)bme680_calib.dig_P4)<<35); 
	var1 = ((var1 * var1 * (BME680_S64_t)bme680_calib.dig_P3)>>8) + ((var1 * (BME680_S64_t)bme680_calib.dig_P2)<<12); 
	var1 = (((((BME680_S64_t)1)<<47)+var1))*((BME680_S64_t)bme680_calib.dig_P1)>>33; 
	if (var1 == 0) { 
		return 0; // avoid exception caused by division by zero 
	} 
	p = 1048576-adc_P; 
	p = (((p<<31)-var2)*3125)/var1; 
	var1 = (((BME680_S64_t)bme680_calib.dig_P9) * (p>>13) * (p>>13)) >> 25; 
	var2 = (((BME680_S64_t)bme680_calib.dig_P8) * p) >> 19; 
	p = ((p + var1 + var2) >> 8) + (((BME680_S64_t)bme680_calib.dig_P7)<<4); 
	p = (p * 10) >> 8;
	return (BME680_U32_t)p; 
} 
 
// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits). 
// Output value of “47445” represents 47445/1024 = 46.333 %RH 
static BME680_U32_t bme680_compensate_H(BME680_S32_t adc_H) {
	BME680_S32_t v_x1_u32r; 

	v_x1_u32r = (bme680_t_fine - ((BME680_S32_t)76800)); 
	v_x1_u32r = (((((adc_H << 14) - (((BME680_S32_t)bme680_calib.dig_H4) << 20) - (((BME680_S32_t)bme680_calib.dig_H5) * v_x1_u32r)) + 
		((BME680_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME680_S32_t)bme680_calib.dig_H6)) >> 10) * (((v_x1_u32r * 
		((BME680_S32_t)bme680_calib.dig_H3)) >> 11) + ((BME680_S32_t)32768))) >> 10) + ((BME680_S32_t)2097152)) * 
		((BME680_S32_t)bme680_calib.dig_H2) + 8192) >> 14)); 
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME680_S32_t)bme680_calib.dig_H1)) >> 4)); 
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);  
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);   
	v_x1_u32r = v_x1_u32r>>12;
	return (BME680_U32_t)((v_x1_u32r * 1000)>>10); 
} 
*/

static int bme680_lua_init(lua_State* L) {

	uint8_t sda;
	uint8_t scl;
    uint8_t addr;

    // required arguments
	if (!lua_isnumber(L, 1) || !lua_isnumber(L, 2) || !lua_isnumber(L, 3)) {
		return luaL_error(L, "wrong arguments");
	}
    uint8_t addr_id = luaL_checkinteger(L, 1);
	addr = convertAddress(addr_id);
    sda = luaL_checkinteger(L, 2);
    scl = luaL_checkinteger(L, 3);
    
    // Init I2C
	platform_i2c_setup(NODEMCU_I2C_ID, sda, scl, PLATFORM_I2C_SPEED_SLOW);
    // Search for i2c device
	platform_i2c_send_start(NODEMCU_I2C_ID);
	uint8_t ack = platform_i2c_send_address(NODEMCU_I2C_ID, addr, PLATFORM_I2C_DIRECTION_TRANSMITTER);
	platform_i2c_send_stop(NODEMCU_I2C_ID);
	if (!ack) {
		NODE_DBG("No ACK on address: %x\n", addr);
        return 0;
	}
	uint8_t chipid = r8u(addr, BME680_REGISTER_CHIPID);
	if(chipid != BME680_CHIP_ID) {
        NODE_DBG("Expected chipd_id: %x, found chip_id: %x\n", BME680_CHIP_ID, chipid);
        return 0;
    }
/*
    // Get calibration
	uint8_t reg = BME680_REGISTER_DIG_T;
	bme680_calib.dig_T1 = r16uLE(addr, reg); reg+=2;
	bme680_calib.dig_T2 = r16sLE(addr, reg); reg+=2;
	bme680_calib.dig_T3 = r16sLE(addr, reg);
	//NODE_DBG("dig_T: %d\t%d\t%d\n", bme680_calib.dig_T1, bme680_calib.dig_T2, bme680_calib.dig_T3);
	reg = BME680_REGISTER_DIG_P;
	bme680_calib.dig_P1 = r16uLE(addr, reg); reg+=2;
	bme680_calib.dig_P2 = r16sLE(addr, reg); reg+=2;
	bme680_calib.dig_P3 = r16sLE(addr, reg); reg+=2;
	bme680_calib.dig_P4 = r16sLE(addr, reg); reg+=2;
	bme680_calib.dig_P5 = r16sLE(addr, reg); reg+=2;
	bme680_calib.dig_P6 = r16sLE(addr, reg); reg+=2;
	bme680_calib.dig_P7 = r16sLE(addr, reg); reg+=2;
	bme680_calib.dig_P8 = r16sLE(addr, reg); reg+=2;
	bme680_calib.dig_P9 = r16sLE(addr, reg);
	// NODE_DBG("dig_P: %d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", bme680_calib.dig_P1, bme680_calib.dig_P2, bme680_calib.dig_P3, bme680_calib.dig_P4, bme680_calib.dig_P5, bme680_calib.dig_P6, bme680_calib.dig_P7, bme680_calib.dig_P8, bme680_calib.dig_P9);
    reg = BME680_REGISTER_DIG_H1;
    bme680_calib.dig_H1 = r8u(addr, reg);
	reg = BME680_REGISTER_DIG_H2;
	bme680_calib.dig_H2 = r16sLE(addr, reg); reg+=2;
	bme680_calib.dig_H3 = r8u(addr, reg); reg++;
	bme680_calib.dig_H4 = ((int16_t)r8u(addr, reg) << 4 | (r8u(addr, reg+1) & 0xF)); reg+=2;
	bme680_calib.dig_H5 = ((int16_t)r8u(addr, reg+1) << 4 | (r8u(addr, reg) >> 4)); reg+=2;
	bme680_calib.dig_H6 = (int8_t)r8u(addr, reg);
	// NODE_DBG("dig_H: %d\t%d\t%d\t%d\t%d\t%d\n", bme680_calib.dig_H1, bme680_calib.dig_H2, bme680_calib.dig_H3, bme680_calib.dig_H4, bme680_calib.dig_H5, bme680_calib.dig_H6);
*/    
    // Config
    w8u(addr, BME680_REGISTER_CONTROL_HUM, bme680_osrs_h);
    w8u(addr, BME680_REGISTER_CONTROL_MEAS, bme680_osrs_t<<5 | bme680_osrs_p<<2);
    w8u(addr, BME680_REGISTER_GAS_WAIT_0, bme680_heat_duration);
    w8u(addr, BME680_REGISTER_RES_HEAT_0, calculate_res_heat(addr, 300.0, 25.0));
    w8u(addr, BME680_REGISTER_CONTROL_GAS_1, 0x10);
	
	return 1;
}

static void bme680_readoutdone (void *arg)
{
	NODE_DBG("timer out\n");
	lua_State *L = lua_getstate();
	lua_rawgeti (L, LUA_REGISTRYINDEX, lua_connected_readout_ref);
    lua_call (L, 0, 0);
	luaL_unref (L, LUA_REGISTRYINDEX, lua_connected_readout_ref);
	os_timer_disarm (&bme680_timer);
}

static int bme680_lua_startsampling(lua_State* L) {
    uint8_t addr; 
	uint32_t delay;

	if (lua_isnumber(L, 1)) {
		uint8_t addr_id = luaL_checkinteger(L, 1);
		addr = convertAddress(addr_id);
	}
	
	if (lua_isnumber(L, 2)) {
		delay = luaL_checkinteger(L, 2);
		if (!delay) {delay = BME680_SAMPLING_DELAY;} // if delay is 0 then set the default delay
	}
	
	if (!lua_isnoneornil(L, 3)) {
		lua_pushvalue(L, 3);
		lua_connected_readout_ref = luaL_ref(L, LUA_REGISTRYINDEX);
	} else {
		lua_connected_readout_ref = LUA_NOREF;
	}

	uint8_t current_mode = r8u(addr, BME680_REGISTER_CONTROL_MEAS);
	w8u(addr, BME680_REGISTER_CONTROL_MEAS, (current_mode & 0xFC) | BME680_FORCED_MODE);
	NODE_DBG("control register changed: %x -> %x, start sampling, delay: %d\n", current_mode, (current_mode & 0xFC) | BME680_FORCED_MODE, delay);

	if (lua_connected_readout_ref != LUA_NOREF) {
		os_timer_disarm (&bme680_timer);
		os_timer_setfn (&bme680_timer, (os_timer_func_t *)bme680_readoutdone, L);
		os_timer_arm (&bme680_timer, delay, 0); // trigger callback when readout is ready
        NODE_DBG("timer started.\n");		
	}
	return 0;
}
/*
static int bme680_lua_temp(lua_State* L) {
	uint32_t adc_T = bme680_adc_T();
	if (adc_T == 0x80000 || adc_T == 0xfffff)
		return 0;
	lua_pushinteger(L, bme680_compensate_T(adc_T));
	lua_pushinteger(L, bme680_t_fine);
	return 2;
}

static int bme680_lua_baro(lua_State* L) {
	uint32_t adc_T = bme680_adc_T();
	uint32_t T = bme680_compensate_T(adc_T);
	uint32_t adc_P = bme680_adc_P();
	if (adc_T == 0x80000 || adc_T == 0xfffff || adc_P ==0x80000 || adc_P == 0xfffff)
		return 0;
	lua_pushinteger(L, bme680_compensate_P(adc_P));
	lua_pushinteger(L, T);
	return 2;
}

static int bme680_lua_humi(lua_State* L) {
	if (!bme680_isbme) return 0;
	uint32_t adc_T = bme680_adc_T();
	uint32_t T = bme680_compensate_T(adc_T);
	uint32_t adc_H = bme680_adc_H();
	if (adc_T == 0x80000 || adc_T == 0xfffff || adc_H == 0x8000 || adc_H == 0xffff)
		return 0;
	lua_pushinteger(L, bme680_compensate_H(adc_H));
	lua_pushinteger(L, T);
	return 2;
}

static int bme680_lua_qfe2qnh(lua_State* L) {
	if (!lua_isnumber(L, 2)) {
		return luaL_error(L, "wrong arg range");
	}
	int32_t qfe = luaL_checkinteger(L, 1);
	int32_t h = luaL_checkinteger(L, 2);
	
	double hc;
	if (bme680_h == h) {
		hc = bme680_hc;
	} else {
		hc = pow((double)(1.0 - 2.25577e-5 * h), (double)(-5.25588));
		bme680_hc = hc; bme680_h = h;
	}
	double qnh = (double)qfe * hc;
	lua_pushinteger(L, (int32_t)(qnh + 0.5));
	return 1;
}

static int bme680_lua_altitude(lua_State* L) {
	if (!lua_isnumber(L, 2)) {
		return luaL_error(L, "wrong arg range");
	}
	int32_t P = luaL_checkinteger(L, 1);
	int32_t qnh = luaL_checkinteger(L, 2);

	double h = (1.0 - pow((double)P/(double)qnh, 1.0/5.25588)) / 2.25577e-5 * 100.0;
	lua_pushinteger(L, (int32_t)(h + (((h<0)?-1:(h>0)) * 0.5)));
	return 1;
}

static double ln(double x) {
	double y = (x-1)/(x+1);
	double y2 = y*y;
	double r = 0;
	for (int8_t i=33; i>0; i-=2) { //we've got the power
		r = 1.0/(double)i + y2 * r;
	}
	return 2*y*r;
}

static int bme680_lua_dewpoint(lua_State* L) {
	const double c243 = 243.5;
	const double c17 = 17.67;
	if (!lua_isnumber(L, 2)) {
		return luaL_error(L, "wrong arg range");
	}
	double H = luaL_checkinteger(L, 1)/100000.0;
	double T = luaL_checkinteger(L, 2)/100.0;

	double c = ln(H) + ((c17 * T) / (c243 + T));
	double d = (c243 * c)/(c17 - c) * 100.0;
	lua_pushinteger(L, (int32_t)(d + (((d<0)?-1:(d>0)) * 0.5)));
	return 1;
}
*/
static const LUA_REG_TYPE bme680_map[] = {
	{ LSTRKEY( "init" ), LFUNCVAL(bme680_lua_init)},
	{ LSTRKEY( "startsampling" ),  LFUNCVAL(bme680_lua_startsampling)},
//	{ LSTRKEY( "temp" ),  LFUNCVAL(bme680_lua_temp)},
//	{ LSTRKEY( "baro" ),  LFUNCVAL(bme680_lua_baro)},
//	{ LSTRKEY( "humi" ),  LFUNCVAL(bme680_lua_humi)},
//	{ LSTRKEY( "qfe2qnh" ),  LFUNCVAL(bme680_lua_qfe2qnh)},
//	{ LSTRKEY( "altitude" ),  LFUNCVAL(bme680_lua_altitude)},
//	{ LSTRKEY( "dewpoint" ),  LFUNCVAL(bme680_lua_dewpoint)},
	{ LNILKEY, LNILVAL}
};

NODEMCU_MODULE(BME680, "bme680", bme680_map, NULL);
