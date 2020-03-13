#include "platform.h"
#include "i2c.h"
#include "bq24158.h"

int g_bq24158_log_en=0;

/**********************************************************
  *
  *   [I2C Slave Setting] 
  *
  *********************************************************/
#define bq24158_SLAVE_ADDR_WRITE   0xD4
#define bq24158_SLAVE_ADDR_Read    0xD5

/**********************************************************
  *
  *   [Global Variable] 
  *
  *********************************************************/
#define bq24158_REG_NUM 7  
kal_uint8 bq24158_reg[bq24158_REG_NUM] = {0};

#define BQ24158_I2C_ID	I2C1
static struct mt_i2c_t bq24158_i2c;

/**********************************************************
  *
  *   [I2C Function For Read/Write bq24158] 
  *
  *********************************************************/
kal_uint32 bq24158_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    bq24158_i2c.id = BQ24158_I2C_ID;
    /* Since i2c will left shift 1 bit, we need to set BQ24158 I2C address to >>1 */
    bq24158_i2c.addr = (bq24158_SLAVE_ADDR_WRITE >> 1);
    bq24158_i2c.mode = ST_MODE;
    bq24158_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&bq24158_i2c, write_data, len);
    printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}

kal_uint32 bq24158_read_byte (kal_uint8 addr, kal_uint8 *dataBuffer) 
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint16 len;
    *dataBuffer = addr;

    bq24158_i2c.id = BQ24158_I2C_ID;
    /* Since i2c will left shift 1 bit, we need to set BQ24158 I2C address to >>1 */
    bq24158_i2c.addr = (bq24158_SLAVE_ADDR_WRITE >> 1);
    bq24158_i2c.mode = ST_MODE;
    bq24158_i2c.speed = 100;
    len = 1;

    ret_code = i2c_write_read(&bq24158_i2c, dataBuffer, len, len);
    printf("%s: i2c_read: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}

/**********************************************************
  *
  *   [Read / Write Function] 
  *
  *********************************************************/
kal_uint32 bq24158_read_interface (kal_uint8 RegNum, kal_uint8 *val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 bq24158_reg = 0;
    int ret = 0;
    
    printf("--------------------------------------------------PL\n");

    ret = bq24158_read_byte(RegNum, &bq24158_reg);
    printf("[bq24158_read_interface] Reg[%x]=0x%x\n", RegNum, bq24158_reg);
    
    bq24158_reg &= (MASK << SHIFT);
    *val = (bq24158_reg >> SHIFT);    
    printf("[bq24158_read_interface] val=0x%x\n", *val);

    return ret;
}

kal_uint32 bq24158_config_interface (kal_uint8 RegNum, kal_uint8 val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 bq24158_reg = 0;
    int ret = 0;

    printf("--------------------------------------------------PL\n");

    ret = bq24158_read_byte(RegNum, &bq24158_reg);
    printf("[bq24158_config_interface] Reg[%x]=0x%x\n", RegNum, bq24158_reg);
    
    bq24158_reg &= ~(MASK << SHIFT);
    bq24158_reg |= (val << SHIFT);

    ret = bq24158_write_byte(RegNum, bq24158_reg);
    printf("[bq24158_config_interface] write Reg[%x]=0x%x\n", RegNum, bq24158_reg);

    // Check
    //bq24158_read_byte(RegNum, &bq24158_reg);
    //printf("[bq24158_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq24158_reg);

    return ret;
}

kal_uint32 bq24158_get_boost_status(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24158_read_interface(     (kal_uint8)(bq24158_CON0), 
                                    (&val),
                                    (kal_uint8)(CON0_BOOST_MASK),
                                    (kal_uint8)(CON0_BOOST_SHIFT)
                                    );
    if(g_bq24158_log_en>1)        
        printf("%d\n", ret);
    
    return val;
}

void bq24158_set_opa_mode(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24158_config_interface(   (kal_uint8)(bq24158_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_OPA_MODE_MASK),
                                    (kal_uint8)(CON1_OPA_MODE_SHIFT)
                                    );
    if(g_bq24158_log_en>1)        
        printf("%d\n", ret);
}

void bq24158_set_otg_en(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24158_config_interface(   (kal_uint8)(bq24158_CON2), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON2_OTG_EN_MASK),
                                    (kal_uint8)(CON2_OTG_EN_SHIFT)
                                    );
    if(g_bq24158_log_en>1)        
        printf("%d\n", ret);
}

