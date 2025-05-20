/*
 * Driver for the aw32207 charger.
 * Author: zhuqian
 *
 */
#include <linux/alarmtimer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/power/charger-manager.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/usb/phy.h>
#include <linux/usb/otg.h>
#include <uapi/linux/usb/charger.h>
#include <linux/gpio.h>
#include "../../usb/musb/musb_core.h"
#include "prj/prj_config.h"

#define AW32207_REG_0					0x0
#define AW32207_REG_1					0x1
#define AW32207_REG_2					0x2
#define AW32207_REG_3					0x3
#define AW32207_REG_4					0x4
#define AW32207_REG_5					0x5
#define AW32207_REG_6					0x6
#define AW32207_REG_10					0x10

#define AW32207_BATTERY_NAME				"sc27xx-fgu"
#define BIT_DP_DM_BC_ENB				BIT(0)
#define AW32207_OTG_VALID_MS				500
#define AW32207_FEED_WATCHDOG_VALID_MS			50
#define AW32207_OTG_ALARM_TIMER_MS			15000

#define AW32207_REG_HZ_MODE_MASK			GENMASK(1, 1)
#define AW32207_REG_OPA_MODE_MASK			GENMASK(0, 0)

#define AW32207_REG_SAFETY_VOL_MASK			GENMASK(3, 0)
#define AW32207_REG_SAFETY_CUR_MASK			GENMASK(7, 4)
#define AW32207_REG_SAFETY_CUR_SHIFT		4

#define AW32207_REG_RESET_MASK				GENMASK(7, 7)
#define AW32207_REG_RESET				BIT(7)

#define AW32207_REG_VSP_MASK				GENMASK(2, 0)

/*AW32207_REG_5----VSP: BIT0-BIT2
HEX  VSP(V	 HEX 	VSP(V)
0 	 4.250	 1 		4.325
2 	 4.400	 3 		4.475
4 	 4.550	 5 		4.625
6 	 4.700	 7 		4.775
*/

#if 1
//reduce vsp voltage is 4.25
#define AW32207_REG_VSP				0
#else
//default is 4.625
#define AW32207_REG_VSP				(BIT(2) | BIT(0))
#endif

#define AW32207_REG_TERMINAL_CURRENT_MASK		GENMASK(3, 3)

#define AW32207_REG_TERMINAL_VOLTAGE_MASK		GENMASK(7, 2)
#define AW32207_REG_TERMINAL_VOLTAGE_SHIFT		2

#define AW32207_REG_CHARGE_CONTROL_MASK		GENMASK(2, 2)
#define AW32207_REG_CHARGE_DISABLE			BIT(2)
#define AW32207_REG_CHARGE_ENABLE			0

#define AW32207_REG_CURRENT_MASK			GENMASK(6, 3)
#define AW32207_REG_CURRENT_MASK_SHIFT			3

#define AW32207_REG_LIMIT_CURRENT_MASK			GENMASK(7, 6)
#define AW32207_REG_LIMIT_CURRENT_SHIFT		6

#define AW32207_DISABLE_PIN_MASK_2730			BIT(0)
#define AW32207_DISABLE_PIN_MASK_2721			BIT(15)
#define AW32207_DISABLE_PIN_MASK_2720			BIT(0)
#define AW32207_WAKE_UP_MS                      2000
#define AW32207_REG_EN_CHARGER_MASK			GENMASK(2, 2)
#define AW32207_REG_EN_CHARGER_SHIFT		2
#define AW32207_REG_EN_CHARGER				(BIT(2))

#define AW32207_REG_VENDOR_MASK			GENMASK(7, 5)
#define AW32207_REG_VENDOR_MASK_SHIFT			5
#define AW32207_REG_PN_MASK			GENMASK(4, 3)
#define AW32207_REG_PN_MASK_SHIFT			3
#define AW32207_REG_REV_MASK				GENMASK(2, 0)
#define AW32207_REG_REV_MASK_SHIFT				0

struct aw32207_charger_info *global_info;

#define AW_PN_32207  0
#define AW_PN_32257  2

#if defined(CONFIG_SOC_PIKE2)&&!defined(CONFIG_BOARD_B88)&&!defined(CONFIG_BOARD_B801)&&!defined(CONFIG_BOARD_B863)
#define CHGR_PD_USE_GPIO
#endif

#if defined(CHGR_PD_USE_GPIO)
int revo_get_dtReal_gpio_num(int num);
static int charge_power_en_gpio = 0;
#endif

struct aw32207_charge_current {
	int sdp_limit;
	int sdp_cur;
	int dcp_limit;
	int dcp_cur;
	int cdp_limit;
	int cdp_cur;
	int unknown_limit;
	int unknown_cur;
};
struct aw32207_charger_info {
	struct i2c_client *client;
	struct device *dev;
	struct usb_phy *usb_phy;
	struct notifier_block usb_notify;
	struct power_supply *psy_usb;
	struct aw32207_charge_current cur;
	struct work_struct work;
	struct mutex lock;
	bool charging;
	u32 limit;
	struct delayed_work otg_work;
	struct regmap *pmic;
	u32 charger_detect;
	u32 charger_pd;
	u32 charger_pd_mask;
	struct gpio_desc *gpiod;
	struct extcon_dev *edev;
	bool otg_enable;
	struct alarm otg_timer;
#ifdef PRJ_FEATURE_H_BOARD_DOCKING_SUPPORT
	struct platform_device *musb_pdev;
#endif
};

//##############################declare function###################################
static int aw32207_read(struct aw32207_charger_info *info, u8 reg, u8 *data);


//##############################define function###################################
#if defined(CHGR_PD_USE_GPIO)
static void sprd_chg_pd_en(int enable)
{
	if(enable)
		gpio_direction_output(charge_power_en_gpio, 0);
	else
		gpio_direction_output(charge_power_en_gpio, 1);
}
#endif

static u8 aw32207_get_vendor_id(void)
{
	u8 reg_val;
	aw32207_read(global_info, AW32207_REG_3, &reg_val);

	reg_val &= AW32207_REG_VENDOR_MASK;
	reg_val = reg_val >> AW32207_REG_VENDOR_MASK_SHIFT;
	return reg_val;
}

static u8 aw32207_get_pn(void)
{
	u8 reg_val;
	aw32207_read(global_info, AW32207_REG_3, &reg_val);

	reg_val &= AW32207_REG_PN_MASK;
	reg_val = reg_val >> AW32207_REG_PN_MASK_SHIFT;
	return reg_val;
}

static u8 aw32207_get_rev_code(void)
{
	u8 reg_val;
	aw32207_read(global_info, AW32207_REG_3, &reg_val);

	reg_val &= AW32207_REG_REV_MASK;
	reg_val = reg_val >> AW32207_REG_REV_MASK_SHIFT;
	return reg_val;
}

static bool aw32207_charger_is_bat_present(struct aw32207_charger_info *info)
{
#if defined(PRJ_FEATURE_H_BOARD_BATTERY_DETECT)
	struct power_supply *psy;
	union power_supply_propval val;
	bool present = false;
	int ret;

	psy = power_supply_get_by_name(AW32207_BATTERY_NAME);
	if (!psy) {
		dev_err(info->dev, "Failed to get psy of sc27xx_fgu\n");
		return present;
	}
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,
					&val);
	if (ret == 0 && val.intval)
		present = true;
	power_supply_put(psy);

	if (ret)
		dev_err(info->dev,
			"Failed to get property of present:%d\n", ret);

	return present;
#else
  return true;
#endif
}

static int aw32207_read(struct aw32207_charger_info *info, u8 reg, u8 *data)
{
	int ret;

	ret = i2c_smbus_read_byte_data(info->client, reg);
	if (ret < 0)
		return ret;

	*data = ret;
	return 0;
}

static int aw32207_write(struct aw32207_charger_info *info, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(info->client, reg, data);
}

static int aw32207_update_bits(struct aw32207_charger_info *info, u8 reg,
		u8 mask, u8 data)
{
	u8 v;
	int ret;

	ret = aw32207_read(info, reg, &v);
	if (ret < 0)
		return ret;

	v &= ~mask;
	v |= (data & mask);

	return aw32207_write(info, reg, v);
}

static int
aw32207_charger_set_safety_vol(struct aw32207_charger_info *info, u32 vol)
{
	u8 reg_val;

	if (vol < 4200)
		vol = 4200;
	if (vol > 4500)
		vol = 4500;
	reg_val = (vol - 4200) / 20 + 1;   
  
  //BIT0-BIT3 max is 0xF
  if(reg_val > 0xF)
  	reg_val =  0xF;
  	
  return aw32207_update_bits(info, AW32207_REG_6, AW32207_REG_SAFETY_VOL_MASK, reg_val);
}

static int
aw32207_charger_set_termina_vol(struct aw32207_charger_info *info, u32 vol)
{
	u8 reg_val;

	if (vol < 3500)
		reg_val = 0x0;
	else if (vol >= 4500)
		reg_val = 0x32;
	else
		reg_val = (vol - 3499) / 20;
  
  	return aw32207_update_bits(info, AW32207_REG_2,
  				    AW32207_REG_TERMINAL_VOLTAGE_MASK,
  				    reg_val << AW32207_REG_TERMINAL_VOLTAGE_SHIFT);
}


static int aw32207_charger_set_safety_cur(struct aw32207_charger_info *info, u32 cur)
{
	u8 reg_val=0;

/*
    //this is 68mo 
  	if (cur < 400000)
  		reg_val = 0x0;
  	else if (cur >= 400000 && cur < 700000)
  		reg_val = 0x1;
  	else if (cur >= 700000 && cur < 800000)
  		reg_val = 0x2;
  	else if (cur >= 800000 && cur < 900000)
  		reg_val = 0x3;
  	else if (cur >= 900000 && cur < 1000000)
  		reg_val = 0x4;
  	else if (cur >= 1000000 && cur < 1100000)
  		reg_val = 0x5;
  	else if (cur >= 1100000 && cur < 1200000)
  		reg_val = 0x6;
  	else if (cur >= 1200000 && cur < 1300000)
  		reg_val = 0x7;
  	else if (cur >= 1300000 && cur < 1400000)
  		reg_val = 0x8;
  	else if (cur >= 1400000 && cur < 1500000)
  		reg_val = 0x9;
  	else if (cur >= 1500000)
  		reg_val = 0xa;
  */
	if(aw32207_get_pn() == AW_PN_32257)
	{
			//this is aw32257 33mo
		  if (cur < 496000)
		  	reg_val = 0x0;//496
		  else if (cur >= 496000 && cur < 620000)
		  	reg_val = 0x1;//620
		  else if (cur >= 620000 && cur < 868000)
		  	reg_val = 0x2;//868
		  else if (cur >= 868000 && cur < 992000)
		  	reg_val = 0x3;//992
		  else if (cur >= 992000 && cur < 1116000)
		  	reg_val = 0x4;//1116
		  else if (cur >= 1116000 && cur < 1240000)
		  	reg_val = 0x5;//1240
		  else if (cur >= 1240000 && cur < 1364000)
		  	reg_val = 0x6;//1364
		  else if (cur >= 1364000 && cur < 1488000)
		  	reg_val = 0x7;//1488
		  else if (cur >= 1488000 && cur < 1612000)
		  	reg_val = 0x8;//1612
		  else if (cur >= 1612000 && cur < 1736000)
		  	reg_val = 0x9;//1736
		  else if (cur >= 1736000 && cur < 1860000)
		    reg_val = 0xa;//1860
		  else if (cur >= 1860000 && cur < 1984000)
		  	reg_val = 0xb;//1984
		  else if (cur >= 1984000 && cur < 2108000)
		  	reg_val = 0xc;//2108
		  else if (cur >= 2108000 && cur < 2232000)
		  	reg_val = 0xd;//2232
		  else if (cur >= 2232000 && cur < 2356000)
		  	reg_val = 0xe;//2356
		  else if (cur >= 2356000)
		    reg_val = 0xf;//2480
	}
	else
	{
		  //this is 56mo
		  if (cur < 510000)
		  	reg_val = 0x0;//485
		  else if (cur >= 510000 && cur < 850000)
		  	reg_val = 0x1;//607
		  else if (cur >= 850000 && cur < 971000)
		  	reg_val = 0x2;//850
		  else if (cur >= 971000 && cur < 1092000)
		  	reg_val = 0x3;//971
		  else if (cur >= 1092000 && cur < 1214000)
		  	reg_val = 0x4;//1092
		  else if (cur >= 1214000 && cur < 1336000)
		  	reg_val = 0x5;//1214
		  else if (cur >= 1336000 && cur < 1457000)
		  	reg_val = 0x6;//1336
		  else if (cur >= 1457000 && cur < 1578000)
		  	reg_val = 0x7;//1457
		  else if (cur >= 1578000 && cur < 1699000)
		  	reg_val = 0x8;//1578
		  else if (cur >= 1699000 && cur < 1821000)
		  	reg_val = 0x9;//1699
		  else if (cur >= 1821000)
		    reg_val = 0xa;//1821
	}
	
	return aw32207_update_bits(info, AW32207_REG_6,AW32207_REG_SAFETY_CUR_MASK,reg_val << AW32207_REG_SAFETY_CUR_SHIFT);
}

int aw32207_get_battery_cur(struct power_supply *psy,
				  struct aw32207_charge_current *bat_cur)
{
	struct device_node *battery_np;
	const char *value;
	int err;

	bat_cur->sdp_cur = -EINVAL;
	bat_cur->sdp_limit = -EINVAL;
	bat_cur->dcp_cur = -EINVAL;
	bat_cur->dcp_limit = -EINVAL;
	bat_cur->cdp_cur = -EINVAL;
	bat_cur->cdp_limit = -EINVAL;
	bat_cur->unknown_cur = -EINVAL;
	bat_cur->unknown_limit = -EINVAL;

	if (!psy->of_node) {
		dev_warn(&psy->dev, "%s currently only supports devicetree\n",
			 __func__);
		return -ENXIO;
	}

	battery_np = of_parse_phandle(psy->of_node, "monitored-battery", 0);
	if (!battery_np)
		return -ENODEV;

	err = of_property_read_string(battery_np, "compatible", &value);
	if (err)
		goto out_put_node;

	if (strcmp("simple-battery", value)) {
		err = -ENODEV;
		goto out_put_node;
	}

	of_property_read_u32_index(battery_np, "charge-sdp-current-microamp", 0,
				   &bat_cur->sdp_cur);
	of_property_read_u32_index(battery_np, "charge-sdp-current-microamp", 1,
				   &bat_cur->sdp_limit);
	of_property_read_u32_index(battery_np, "charge-dcp-current-microamp", 0,
				   &bat_cur->dcp_cur);
	of_property_read_u32_index(battery_np, "charge-dcp-current-microamp", 1,
				   &bat_cur->dcp_limit);
	of_property_read_u32_index(battery_np, "charge-cdp-current-microamp", 0,
				   &bat_cur->cdp_cur);
	of_property_read_u32_index(battery_np, "charge-cdp-current-microamp", 1,
				   &bat_cur->cdp_limit);
	of_property_read_u32_index(battery_np, "charge-unknown-current-microamp", 0,
				   &bat_cur->unknown_cur);
	of_property_read_u32_index(battery_np, "charge-unknown-current-microamp", 1,
				   &bat_cur->unknown_limit);
out_put_node:
	of_node_put(battery_np);
	return err;
}

static int aw32207_charger_hw_init(struct aw32207_charger_info *info)
{
	struct power_supply_battery_info bat_info = { };
	int voltage_max_microvolt;
	int ret;

	ret = aw32207_get_battery_cur(info->psy_usb, &info->cur);
	if (ret) {
		dev_warn(info->dev, "no battery current information is supplied\n");

		/*
		 * If no battery information is supplied, we should set
		 * default charge termination current to 100 mA, and default
		 * charge termination voltage to 4.2V.
		 */
		info->cur.sdp_limit = 500000;
		info->cur.sdp_cur = 500000;
		info->cur.dcp_limit = 5000000;
		info->cur.dcp_cur = 500000;
		info->cur.cdp_limit = 5000000;
		info->cur.cdp_cur = 1500000;
		info->cur.unknown_limit = 5000000;
		info->cur.unknown_cur = 500000;
	}

	ret = power_supply_get_battery_info(info->psy_usb, &bat_info);
	if (ret) {
		dev_warn(info->dev, "no battery information is supplied\n");
		voltage_max_microvolt = 4440;
	} else {
		voltage_max_microvolt = bat_info.constant_charge_voltage_max_uv / 1000;
		power_supply_put_battery_info(info->psy_usb, &bat_info);

		if (of_device_is_compatible(info->dev->of_node,
					    "fairchild,aw32207_chg")) {
			ret = aw32207_charger_set_safety_vol(info,
						voltage_max_microvolt);
			if (ret) {
				dev_err(info->dev,
					"set aw32207 safety vol failed\n");
				return ret;
			}

			ret = aw32207_charger_set_safety_cur(info,
						info->cur.dcp_cur);
			if (ret) {
				dev_err(info->dev,
					"set aw32207 safety cur failed\n");
				return ret;
			}
		}

		ret = aw32207_update_bits(info, AW32207_REG_4,
					   AW32207_REG_RESET_MASK,
					   AW32207_REG_RESET);
		if (ret) {
			dev_err(info->dev, "reset aw32207 failed\n");
			return ret;
		}
		
		msleep(60);
		

		ret = aw32207_update_bits(info, AW32207_REG_5,
				   AW32207_REG_VSP_MASK,
				   AW32207_REG_VSP);
		
		if (ret) {
			dev_err(info->dev, "set aw32207 vsp failed\n");
			return ret;
		}

		ret = aw32207_update_bits(info, AW32207_REG_1,
					   AW32207_REG_TERMINAL_CURRENT_MASK, 0);
		if (ret) {
			dev_err(info->dev, "set aw32207 terminal cur failed\n");
			return ret;
		}

		ret = aw32207_charger_set_termina_vol(info, voltage_max_microvolt);
		if (ret) {
			dev_err(info->dev, "set aw32207 terminal vol failed\n");
			return ret;
		}
	}

	return ret;
}

static void aw32207_dump_reg(struct aw32207_charger_info *info) {
	int i = 0;
	int ret;
	unsigned char v = 0;
	for (i = 0; i < 0x0b; i++) {
		ret = aw32207_read(info, i, &v);
		if (ret < 0) {
			dev_err(info->dev, "read [%d] error", i);
		}
		dev_info(info->dev, "reg[%02x] = %02x\n", i, v);
	}
}

static int aw32207_charger_start_charge(struct aw32207_charger_info *info)
{
  int ret=0;
  aw32207_update_bits(info, AW32207_REG_1, AW32207_REG_EN_CHARGER_MASK, 0 << AW32207_REG_EN_CHARGER_SHIFT);
  
  aw32207_dump_reg(info);
  
  #if defined(CHGR_PD_USE_GPIO)
    sprd_chg_pd_en(1);
    return ret;
  #else   
    ret = regmap_update_bits(info->pmic, info->charger_pd,
  			 info->charger_pd_mask, 0);
    if (ret)
    	dev_err(info->dev, "enable aw32207 charge failed\n");
    	
    return ret;
  #endif
}

static void aw32207_charger_stop_charge(struct aw32207_charger_info *info)
{
  aw32207_update_bits(info, AW32207_REG_1, AW32207_REG_EN_CHARGER_MASK, 1 << AW32207_REG_EN_CHARGER_SHIFT);
  
#if defined(CHGR_PD_USE_GPIO)
  sprd_chg_pd_en(0);
#else
	regmap_update_bits(info->pmic, info->charger_pd,
				 info->charger_pd_mask,
				 info->charger_pd_mask);
#endif
}


static int aw32207_charger_set_current(struct aw32207_charger_info *info,
					u32 cur)
{
	u8 reg_val=0;

/*
    //this is 68mo 
  	if (cur < 400000)
  		reg_val = 0x0;
  	else if (cur >= 400000 && cur < 700000)
  		reg_val = 0x1;
  	else if (cur >= 700000 && cur < 800000)
  		reg_val = 0x2;
  	else if (cur >= 800000 && cur < 900000)
  		reg_val = 0x3;
  	else if (cur >= 900000 && cur < 1000000)
  		reg_val = 0x4;
  	else if (cur >= 1000000 && cur < 1100000)
  		reg_val = 0x5;
  	else if (cur >= 1100000 && cur < 1200000)
  		reg_val = 0x6;
  	else if (cur >= 1200000 && cur < 1300000)
  		reg_val = 0x7;
  	else if (cur >= 1300000 && cur < 1400000)
  		reg_val = 0x8;
  	else if (cur >= 1400000 && cur < 1500000)
  		reg_val = 0x9;
  	else if (cur >= 1500000)
  		reg_val = 0xa;
  */

/*
    //this is 56mo
  	if (cur < 510000)
  		reg_val = 0x0;//485
  	else if (cur >= 510000 && cur < 850000)
  		reg_val = 0x1;//607
  	else if (cur >= 850000 && cur < 971000)
  		reg_val = 0x2;//850
  	else if (cur >= 971000 && cur < 1092000)
  		reg_val = 0x3;//971
  	else if (cur >= 1092000 && cur < 1214000)
  		reg_val = 0x4;//1092
  	else if (cur >= 1214000 && cur < 1336000)
  		reg_val = 0x5;//1214
  	else if (cur >= 1336000 && cur < 1457000)
  		reg_val = 0x6;//1336
  	else if (cur >= 1457000 && cur < 1578000)
  		reg_val = 0x7;//1457
  	else if (cur >= 1578000 && cur < 1699000)
  		reg_val = 0x8;//1578
  	else if (cur >= 1699000 && cur < 1821000)
  		reg_val = 0x9;//1699
  	else if (cur >= 1821000)
  		reg_val = 0xa;//1821
*/
	if(aw32207_get_pn() == AW_PN_32257)
	{
			//this is aw32257 33mo
		  if (cur < 496000)
			reg_val = 0x0;//496
		  else if (cur >= 496000 && cur < 620000)
			reg_val = 0x1;//620
		  else if (cur >= 620000 && cur < 868000)
			reg_val = 0x2;//868
		  else if (cur >= 868000 && cur < 992000)
			reg_val = 0x3;//992
		  else if (cur >= 992000 && cur < 1116000)
			reg_val = 0x4;//1116
		  else if (cur >= 1116000 && cur < 1240000)
			reg_val = 0x5;//1240
		  else if (cur >= 1240000 && cur < 1364000)
			reg_val = 0x6;//1364
		  else if (cur >= 1364000 && cur < 1488000)
			reg_val = 0x7;//1488
		  else if (cur >= 1488000 && cur < 1612000)
			reg_val = 0x8;//1612
		  else if (cur >= 1612000 && cur < 1736000)
			reg_val = 0x9;//1736
		  else if (cur >= 1736000 && cur < 1860000)
			reg_val = 0xa;//1860
		  else if (cur >= 1860000 && cur < 1984000)
			reg_val = 0xb;//1984
		  else if (cur >= 1984000 && cur < 2108000)
			reg_val = 0xc;//2108
		  else if (cur >= 2108000 && cur < 2232000)
			reg_val = 0xd;//2232
		  else if (cur >= 2232000 && cur < 2356000)
			reg_val = 0xe;//2356
		  else if (cur >= 2356000)
			reg_val = 0xf;//2480
	}
	else
	{
	    //this is 51mo
	  	if (cur < 550000)
	  		reg_val = 0x0;//533
	  	else if (cur >= 550000 && cur < 933000)
	  		reg_val = 0x1;//666
	  	else if (cur >= 933000 && cur < 1066000)
	  		reg_val = 0x2;//933
	  	else if (cur >= 1066000 && cur < 1200000)
	  		reg_val = 0x3;//1066
	  	else if (cur >= 1200000 && cur < 1333000)
	  		reg_val = 0x4;//1200
	  	else if (cur >= 1333000 && cur < 1466000)
	  		reg_val = 0x5;//1333
	  	else if (cur >= 1466000 && cur < 1600000)
	  		reg_val = 0x6;//1466
	  	else if (cur >= 1600000 && cur < 1733000)
	  		reg_val = 0x7;//1600
	  	else if (cur >= 1733000 && cur < 1866000)
	  		reg_val = 0x8;//1733
	  	else if (cur >= 1866000 && cur < 2000000)
	  		reg_val = 0x9;//1866
	  	else if (cur >= 2000000)
	  		reg_val = 0xa;//2000
	}

	return aw32207_update_bits(info, AW32207_REG_4,
			    AW32207_REG_CURRENT_MASK,
			    reg_val << AW32207_REG_CURRENT_MASK_SHIFT);
}

static int aw32207_charger_get_current(struct aw32207_charger_info *info,
					u32 *cur)
{
	u8 reg_val;
	int ret;

	ret = aw32207_read(info, AW32207_REG_4, &reg_val);
	if (ret < 0)
		return ret;

	reg_val &= AW32207_REG_CURRENT_MASK;
	reg_val = reg_val >> AW32207_REG_CURRENT_MASK_SHIFT;

	if(aw32207_get_pn() == AW_PN_32257)
	{
		  switch (reg_val) {
		  	case 0:
			  		*cur = 496000;
			  		break;
		  	case 1:
			  		*cur = 620000;
			  		break;
		  	case 2:
			  		*cur = 868000;
			  		break;
		  	case 3:
			  		*cur = 992000;
			  		break;
		  	case 4:
			  		*cur = 1116000;
			  		break;
		  	case 5:
			  		*cur = 1240000;
			  		break;
		  	case 6:
			  		*cur = 1364000;
			  		break;
		  	case 7:
			  		*cur = 1488000;
			  		break;
		  	case 8:
			  		*cur = 1612000;
			  		break;
		  	case 9:
			  		*cur = 1736000;
			  		break;
		  	case 0xa:
			  		*cur = 1860000;
			  		break;
		  	case 0xb:
			  		*cur = 1984000;
			  		break;
		  	case 0xc:
			  		*cur = 2108000;
			  		break;
		  	case 0xd:
			  		*cur = 2232000;
			  		break;
		  	case 0xe:
			  		*cur = 2356000;
			  		break;
		  	case 0xf:
			  		*cur = 2480000;
			  		break;
		  	default:
			  		*cur = 2480000;
			  		break;
		  }
	}
	else
	{
		  switch (reg_val) {
		  	case 0:
		  		*cur = 533000;
		  		break;
		  	case 1:
		  		*cur = 666000;
		  		break;
		  	case 2:
		  		*cur = 933000;
		  		break;
		  	case 3:
		  		*cur = 1066000;
		  		break;
		  	case 4:
		  		*cur = 1200000;
		  		break;
		  	case 5:
		  		*cur = 1333000;
		  		break;
		  	case 6:
		  		*cur = 1466000;
		  		break;
		  	case 7:
		  		*cur = 1600000;
		  		break;
		  	case 8:
		  		*cur = 1733000;
		  		break;
		  	case 9:
		  		*cur = 1866000;
		  		break;
		  	case 0xa:
		  		*cur = 2000000;
		  		break;
		  	default:
		  		*cur = 2000000;
		  		break;
		  }
	}
  
	return 0;
}

static u32
aw32207_charger_get_limit_current(struct aw32207_charger_info *info,
				   u32 *limit_cur)
{
	switch (info->usb_phy->chg_type) {
		case SDP_TYPE:
			*limit_cur = 500000;
			break;
		case DCP_TYPE:
		case CDP_TYPE:
			*limit_cur = 2000000;
			break;
		default:
			*limit_cur = 500000;
	}
	return 0;
}

static int aw32207_charger_get_health(struct aw32207_charger_info *info,
				     u32 *health)
{
	*health = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int aw32207_charger_get_online(struct aw32207_charger_info *info,
				     u32 *online)
{
	if (info->limit)
		*online = true;
	else
		*online = false;

	return 0;
}


static int aw32207_charger_get_status(struct aw32207_charger_info *info)
{
	if (info->charging == true)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int aw32207_charger_set_status(struct aw32207_charger_info *info,
				       int val)
{
	int ret = 0;

#ifdef REVO_CHARGER_DEBUG
	dev_err(info->dev, "aw32207_charger_set_status enable/disable:%d  ischarging:%d\n", val, info->charging);
#endif
	if (!val && info->charging) {
		aw32207_charger_stop_charge(info);
		info->charging = false;
	} else if (val && !info->charging) {
		ret = aw32207_charger_start_charge(info);
		if (ret)
			dev_err(info->dev, "start charge failed\n");
		else
			info->charging = true;
	}

	return ret;
}

static void aw32207_charger_work(struct work_struct *data)
{
	struct aw32207_charger_info *info =
		container_of(data, struct aw32207_charger_info, work);
	int limit_cur, cur, ret;
	bool present = aw32207_charger_is_bat_present(info);

	mutex_lock(&info->lock);

	if (info->limit > 0 && !info->charging && present) {
		/* set current limitation and start to charge */
		switch (info->usb_phy->chg_type) {
		case SDP_TYPE:
			limit_cur = info->cur.sdp_limit;
			cur = info->cur.sdp_cur;
			break;
		case DCP_TYPE:
			limit_cur = info->cur.dcp_limit;
			cur = info->cur.dcp_cur;
			break;
		case CDP_TYPE:
			limit_cur = info->cur.cdp_limit;
			cur = info->cur.cdp_cur;
			break;
		default:
			limit_cur = info->cur.unknown_limit;
			cur = info->cur.unknown_cur;
		}

		ret = aw32207_charger_set_current(info, cur);
		if (ret)
			goto out;

		ret = aw32207_charger_start_charge(info);
		if (ret)
			goto out;

		info->charging = true;
	} else if ((!info->limit && info->charging) || !present) {
		/* Stop charging */
		info->charging = false;
		aw32207_charger_stop_charge(info);
	}

out:
	mutex_unlock(&info->lock);
	dev_info(info->dev, "battery present = %d, charger type = %d\n",
		 present, info->usb_phy->chg_type);
	cm_notify_event(info->psy_usb, CM_EVENT_CHG_START_STOP, NULL);
}


static int aw32207_charger_usb_change(struct notifier_block *nb,
				       unsigned long limit, void *data)
{
	struct aw32207_charger_info *info =
		container_of(nb, struct aw32207_charger_info, usb_notify);

	if (!info) {
		pr_err("%s:line%d: NULL pointer!!!\n", __func__, __LINE__);
		return NOTIFY_OK;
	}
	info->limit = limit;
#ifdef REVO_CHARGER_DEBUG
	dev_info(info->dev, "aw32207_charger_usb_change limit = %d\n",(int)limit);
#endif

	pm_wakeup_event(info->dev, AW32207_WAKE_UP_MS);
	schedule_work(&info->work);
	return NOTIFY_OK;
}

static int aw32207_charger_usb_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct aw32207_charger_info *info = power_supply_get_drvdata(psy);
	u32 cur, online, health;
	enum usb_charger_type type;
	int ret = 0;

	if (!info) {
		pr_err("%s:line%d: NULL pointer!!!\n", __func__, __LINE__);
		return -EINVAL;
	}

	mutex_lock(&info->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (info->limit)
			val->intval = aw32207_charger_get_status(info);
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (!info->charging) {
			val->intval = 0;
		} else {
			ret = aw32207_charger_get_current(info, &cur);
			if (ret)
				goto out;

			val->intval = cur;
		}
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (!info->charging) {
			val->intval = 0;
		} else {
			ret = aw32207_charger_get_limit_current(info, &cur);
			if (ret)
				goto out;

			val->intval = cur;
		}
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		ret = aw32207_charger_get_online(info, &online);
		if (ret)
			goto out;

		val->intval = online;

		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (info->charging) {
			val->intval = 0;
		} else {
			ret = aw32207_charger_get_health(info, &health);
			if (ret)
				goto out;

			val->intval = health;
		}
		break;

	case POWER_SUPPLY_PROP_USB_TYPE:
		type = info->usb_phy->chg_type;

		switch (type) {
		case SDP_TYPE:
			val->intval = POWER_SUPPLY_USB_TYPE_SDP;
			break;

		case DCP_TYPE:
			val->intval = POWER_SUPPLY_USB_TYPE_DCP;
			break;

		case CDP_TYPE:
			val->intval = POWER_SUPPLY_USB_TYPE_CDP;
			break;

		default:
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		}

		break;

	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&info->lock);
	return ret;
}

static int aw32207_charger_usb_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct aw32207_charger_info *info = power_supply_get_drvdata(psy);
	int ret;

	if (!info) {
		pr_err("%s:line%d: NULL pointer!!!\n", __func__, __LINE__);
		return -EINVAL;
	}

	mutex_lock(&info->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = aw32207_charger_set_current(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set charge current failed\n");
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = aw32207_charger_set_current(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set charge current limit failed\n");
		break;

	case POWER_SUPPLY_PROP_STATUS:
		ret = aw32207_charger_set_status(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set charge status failed\n");
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = aw32207_charger_set_termina_vol(info, val->intval / 1000);
		if (ret < 0)
			dev_err(info->dev, "failed to set terminate voltage\n");
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&info->lock);
	return ret;
}

static int aw32207_charger_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_STATUS:
		ret = 1;
		break;

	default:
		ret = 0;
	}

	return ret;
}

static enum power_supply_usb_type aw32207_charger_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};

static enum power_supply_property aw32207_usb_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc aw32207_charger_desc = {
	.name			= "aw32207_charger",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= aw32207_usb_props,
	.num_properties		= ARRAY_SIZE(aw32207_usb_props),
	.get_property		= aw32207_charger_usb_get_property,
	.set_property		= aw32207_charger_usb_set_property,
	.property_is_writeable	= aw32207_charger_property_is_writeable,
	.usb_types		= aw32207_charger_usb_types,
	.num_usb_types		= ARRAY_SIZE(aw32207_charger_usb_types),
};

static void aw32207_charger_detect_status(struct aw32207_charger_info *info)
{
	int min, max;

	/*
	 * If the USB charger status has been USB_CHARGER_PRESENT before
	 * registering the notifier, we should start to charge with getting
	 * the charge current.
	 */
	if (info->usb_phy->chg_state != USB_CHARGER_PRESENT)
		return;

	usb_phy_get_charger_current(info->usb_phy, &min, &max);
	info->limit = min;
	schedule_work(&info->work);
}

#if IS_ENABLED(CONFIG_REGULATOR)
static void aw32207_charger_otg_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct aw32207_charger_info *info = container_of(dwork,
			struct aw32207_charger_info, otg_work);
	int ret;

	if (!info) {
		pr_err("%s:line%d: NULL pointer!!!\n", __func__, __LINE__);
		return;
	}
	if (!extcon_get_state(info->edev, EXTCON_USB)) {
		ret = aw32207_update_bits(info, AW32207_REG_1,
					   AW32207_REG_HZ_MODE_MASK |
					   AW32207_REG_OPA_MODE_MASK,
					   AW32207_REG_OPA_MODE_MASK);
		if (ret)
			dev_err(info->dev, "restart aw32207 charger otg failed\n");
	}

	schedule_delayed_work(&info->otg_work, msecs_to_jiffies(500));
}

static int aw32207_charger_enable_otg(struct regulator_dev *dev)
{
	struct aw32207_charger_info *info = rdev_get_drvdata(dev);
	int ret;
#ifdef PRJ_FEATURE_H_BOARD_DOCKING_SUPPORT
	struct sprd_glue *glue;
#endif
	if (!info) {
		pr_err("%s:line%d: NULL pointer!!!\n", __func__, __LINE__);
		return -EINVAL;
	}
#ifdef PRJ_FEATURE_H_BOARD_DOCKING_SUPPORT
	if(info->musb_pdev) {
		glue = platform_get_drvdata(info->musb_pdev);
		if(glue) {
			dev_err(info->dev, "%s:is_docking_online:%d\n",__func__,glue->is_docking_online);
		}
	}
#endif
	/*
	 * Disable charger detection function in case
	 * affecting the OTG timing sequence.
	 */
	ret = regmap_update_bits(info->pmic, info->charger_detect,
				 BIT_DP_DM_BC_ENB, BIT_DP_DM_BC_ENB);
	if (ret) {
		dev_err(info->dev, "failed to disable bc1.2 detect function.\n");
		return ret;
	}

  
	ret = aw32207_update_bits(info, AW32207_REG_1,
				   AW32207_REG_HZ_MODE_MASK |
				   AW32207_REG_OPA_MODE_MASK,
				   AW32207_REG_OPA_MODE_MASK);
	if (ret) {
		dev_err(info->dev, "enable aw32207 otg failed\n");
		regmap_update_bits(info->pmic, info->charger_detect,
				   BIT_DP_DM_BC_ENB, 0);
		return ret;
	}

	info->otg_enable = true;
	schedule_delayed_work(&info->otg_work,
			      msecs_to_jiffies(AW32207_OTG_VALID_MS));

	return 0;
}

static int aw32207_charger_disable_otg(struct regulator_dev *dev)
{
	struct aw32207_charger_info *info = rdev_get_drvdata(dev);
	int ret;
#ifdef PRJ_FEATURE_H_BOARD_DOCKING_SUPPORT
	struct sprd_glue *glue;
	
	if(info->musb_pdev) {
		glue = platform_get_drvdata(info->musb_pdev);
		if(glue) {
			dev_err(info->dev, "%s:is_docking_online:%d\n",__func__,glue->is_docking_online);
		}
		dev_err(info->dev, "%s: musb_pdev(0x%p)\n",__func__, info->musb_pdev);
	}
#endif
	info->otg_enable = false;
	cancel_delayed_work_sync(&info->otg_work);
	ret = aw32207_update_bits(info, AW32207_REG_1,
				   AW32207_REG_HZ_MODE_MASK |
				   AW32207_REG_OPA_MODE_MASK,
				   0);
	if (ret) {
		dev_err(info->dev, "disable aw32207 otg failed\n");
		return ret;
	}

	/* Enable charger detection function to identify the charger type */
	ret = regmap_update_bits(info->pmic, info->charger_detect,
				 BIT_DP_DM_BC_ENB, 0);
	if (ret)
		dev_err(info->dev, "enable BC1.2 failed\n");
	return ret;
}

static int aw32207_charger_vbus_is_enabled(struct regulator_dev *dev)
{
	struct aw32207_charger_info *info = rdev_get_drvdata(dev);
	int ret;
	u8 val;

	if (!info) {
		pr_err("%s:line%d: NULL pointer!!!\n", __func__, __LINE__);
		return -EINVAL;
	}
	ret = aw32207_read(info, AW32207_REG_1, &val);
	if (ret) {
		dev_err(info->dev, "failed to get aw32207 otg status\n");
		return ret;
	}

	val &= AW32207_REG_OPA_MODE_MASK;

	return val;
}

static const struct regulator_ops aw32207_charger_vbus_ops = {
	.enable = aw32207_charger_enable_otg,
	.disable = aw32207_charger_disable_otg,
	.is_enabled = aw32207_charger_vbus_is_enabled,
};

static const struct regulator_desc aw32207_charger_vbus_desc = {
	.name = "otg-vbus",
	.of_match = "otg-vbus",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &aw32207_charger_vbus_ops,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static int
aw32207_charger_register_vbus_regulator(struct aw32207_charger_info *info)
{
	struct regulator_config cfg = { };
	struct regulator_dev *reg;
	int ret = 0;

	cfg.dev = info->dev;
	cfg.driver_data = info;
	reg = devm_regulator_register(info->dev,
				      &aw32207_charger_vbus_desc, &cfg);
	if (IS_ERR(reg)) {
		ret = PTR_ERR(reg);
		dev_err(info->dev, "Can't register regulator:%d\n", ret);
	}

	return ret;
}

#else
static int
aw32207_charger_register_vbus_regulator(struct aw32207_charger_info *info)
{
	return 0;
}
#endif

#ifdef PRJ_FEATURE_H_BOARD_DOCKING_SUPPORT
extern struct platform_device *find_sprd_musb_dev(struct device *dev);
#endif
static int aw32207_charger_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct power_supply_config charger_cfg = { };
	struct aw32207_charger_info *info;
	struct device_node *regmap_np;
	struct platform_device *regmap_pdev;
	int ret;

	if (!adapter) {
		pr_err("%s:line%d: NULL pointer!!!\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (!dev) {
		pr_err("%s:line%d: NULL pointer!!!\n", __func__, __LINE__);
		return -EINVAL;
	}
	
	dev_info(dev, "%s:line%d: in\n", __func__, __LINE__);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "No support for SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->client = client;
	info->dev = dev;

	alarm_init(&info->otg_timer, ALARM_BOOTTIME, NULL);

	mutex_init(&info->lock);
	INIT_WORK(&info->work, aw32207_charger_work);

	i2c_set_clientdata(client, info);
	
	global_info = info;

	info->usb_phy = devm_usb_get_phy_by_phandle(dev, "phys", 0);
	if (IS_ERR(info->usb_phy)) {
		dev_err(dev, "failed to find USB phy\n");
		return PTR_ERR(info->usb_phy);
	}

	info->edev = extcon_get_edev_by_phandle(info->dev, 0);
	if (IS_ERR(info->edev)) {
		dev_err(dev, "failed to find vbus extcon device.\n");
		return PTR_ERR(info->edev);
	}

	ret = aw32207_charger_register_vbus_regulator(info);
	if (ret) {
		dev_err(dev, "failed to register vbus regulator.\n");
		return ret;
	}

	regmap_np = of_find_compatible_node(NULL, NULL, "sprd,sc27xx-syscon");
	if (!regmap_np) {
		dev_err(dev, "unable to get syscon node\n");
		return -ENODEV;
	}

	ret = of_property_read_u32_index(regmap_np, "reg", 1,
					 &info->charger_detect);
	if (ret) {
		dev_err(dev, "failed to get charger_detect\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_index(regmap_np, "reg", 2,
					 &info->charger_pd);
	if (ret) {
		dev_err(dev, "failed to get charger_pd reg\n");
#if defined(CHGR_PD_USE_GPIO)
#else
 		return ret;
#endif 
	}

	if (of_device_is_compatible(regmap_np->parent, "sprd,sc2730"))
		info->charger_pd_mask = AW32207_DISABLE_PIN_MASK_2730;
	else if (of_device_is_compatible(regmap_np->parent, "sprd,sc2721"))
		info->charger_pd_mask = AW32207_DISABLE_PIN_MASK_2721;
	else if (of_device_is_compatible(regmap_np->parent, "sprd,sc2720"))
		info->charger_pd_mask = AW32207_DISABLE_PIN_MASK_2720;
	else {
		dev_err(dev, "failed to get charger_pd mask\n");
		return -EINVAL;
	}

	regmap_pdev = of_find_device_by_node(regmap_np);
	if (!regmap_pdev) {
		of_node_put(regmap_np);
		dev_err(dev, "unable to get syscon device\n");
		return -ENODEV;
	}

	of_node_put(regmap_np);
	info->pmic = dev_get_regmap(regmap_pdev->dev.parent, NULL);
	if (!info->pmic) {
		dev_err(dev, "unable to get pmic regmap device\n");
		return -ENODEV;
	}
#ifdef PRJ_FEATURE_H_BOARD_DOCKING_SUPPORT
	info->musb_pdev = find_sprd_musb_dev(dev);
	if(!info->musb_pdev) {
		dev_err(dev, "%s: get sprd musb fail\n", __func__);
	}
#endif
	charger_cfg.drv_data = info;
	charger_cfg.of_node = dev->of_node;
	info->psy_usb = devm_power_supply_register(dev,
						   &aw32207_charger_desc,
						   &charger_cfg);
	if (IS_ERR(info->psy_usb)) {
		dev_err(dev, "failed to register power supply\n");
		return PTR_ERR(info->psy_usb);
	}

	ret = aw32207_charger_hw_init(info);
	if (ret)
		return ret;

	device_init_wakeup(info->dev, true);
	info->usb_notify.notifier_call = aw32207_charger_usb_change;
	ret = usb_register_notifier(info->usb_phy, &info->usb_notify);
	if (ret) {
		dev_err(dev, "failed to register notifier:%d\n", ret);
		return ret;
	}

	aw32207_charger_detect_status(info);
	INIT_DELAYED_WORK(&info->otg_work, aw32207_charger_otg_work);

#if defined(CHGR_PD_USE_GPIO)
	charge_power_en_gpio = revo_get_dtReal_gpio_num(19);
	gpio_request(charge_power_en_gpio, "charge_en");	
#endif

	dev_info(dev, "<revo_log:%s> %s = 0x%x %s = 0x%x \n", __func__, 
	    "aw32207_get_rev_code" ,aw32207_get_rev_code() ,
	    "aw32207_get_vendor_id" ,aw32207_get_vendor_id());

	return 0;
}

static void aw32207_shutdown(struct i2c_client *client)
{
	int ret;
	struct aw32207_charger_info *info = i2c_get_clientdata(client);

	ret = aw32207_update_bits(info, AW32207_REG_4, AW32207_REG_RESET_MASK, AW32207_REG_RESET);
	if (ret) {
		dev_err(info->dev, "reset aw32207 failed\n");
	}
}

static int aw32207_charger_remove(struct i2c_client *client)
{
	int ret;
	struct aw32207_charger_info *info = i2c_get_clientdata(client);

	usb_unregister_notifier(info->usb_phy, &info->usb_notify);


	ret = aw32207_update_bits(info, AW32207_REG_4, AW32207_REG_RESET_MASK, AW32207_REG_RESET);
	if (ret) {
		dev_err(info->dev, "reset aw32207 failed\n");
		return ret;
	}
		
	return 0;
}

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int aw32207_charger_suspend(struct device *dev)
{
	struct aw32207_charger_info *info = dev_get_drvdata(dev);
	ktime_t now, add;
	unsigned int wakeup_ms = AW32207_OTG_ALARM_TIMER_MS;

	if (!info->otg_enable)
		return 0;

	now = ktime_get_boottime();
	add = ktime_set(wakeup_ms / MSEC_PER_SEC,
			(wakeup_ms % MSEC_PER_SEC) * NSEC_PER_MSEC);
	alarm_start(&info->otg_timer, ktime_add(now, add));

	return 0;
}

static int aw32207_charger_resume(struct device *dev)
{
	struct aw32207_charger_info *info = dev_get_drvdata(dev);

	if (!info->otg_enable)
		return 0;

	alarm_cancel(&info->otg_timer);

	return 0;
}
#endif

static const struct dev_pm_ops aw32207_charger_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(aw32207_charger_suspend,
				aw32207_charger_resume)
};

static const struct i2c_device_id aw32207_i2c_id[] = {
	{"aw32207_chg", 0},
	{}
};

static const struct of_device_id aw32207_charger_of_match[] = {
	{ .compatible = "fairchild,aw32207_chg", },
	{ }
};

MODULE_DEVICE_TABLE(of, aw32207_charger_of_match);

static struct i2c_driver aw32207_charger_driver = {
	.driver = {
		.name = "aw32207_charger",
		.of_match_table = aw32207_charger_of_match,
		.pm = &aw32207_charger_pm_ops,
	},
	.probe = aw32207_charger_probe,
	.remove = aw32207_charger_remove,
	.shutdown = aw32207_shutdown,
	.id_table = aw32207_i2c_id,
};

module_i2c_driver(aw32207_charger_driver);
MODULE_DESCRIPTION("AW32207 Charger Driver");
MODULE_LICENSE("GPL v2");
