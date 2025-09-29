#include "ov7670.h"
#include "ov7670_regs.h"

int OV7670_WriteReg(uint8_t regAddr, const uint8_t *pData)
{
	uint8_t tt[2];
	tt[0] = regAddr;
	tt[1] = pData[0];
	if (HAL_I2C_Master_Transmit(hcamera.hi2c, hcamera.addr,tt,2,hcamera.timeout) == HAL_OK)
	{
		return OV7670_OK;
	}
	else
	{
		return OV7670_ERROR;
	}
}

int OV7670_ReadReg(uint8_t regAddr, uint8_t *pData)
{
	HAL_I2C_Master_Transmit(hcamera.hi2c, hcamera.addr+1,&regAddr,1,hcamera.timeout);
	if (HAL_I2C_Master_Receive(hcamera.hi2c, hcamera.addr+1,pData,1,hcamera.timeout) == HAL_OK)
	{
		return OV7670_OK;
	}
	else
	{
		return OV7670_ERROR;
	}
}

int OV7670_Reset(void)
{
	HAL_Delay(100);
	uint8_t data = COM7_RESET;
	if (OV7670_WriteReg(REG_COM7, &data) != OV7670_OK)
	{
		return OV7670_ERROR;
	}
	HAL_Delay(100);
	return OV7670_OK;
}

int OV7670_WriteRegList(const struct regval_t *reg_list)
{
	const struct regval_t *pReg = reg_list;
	while (pReg->reg_addr != 0xFF && pReg->value != 0xFF)
	{
		int write_result = OV7670_WriteReg( pReg->reg_addr, &(pReg->value));
		if (write_result != OV7670_OK)
		{
			return write_result;
		}
		pReg++;
	}
	return OV7670_OK;
}

////////////////////////////////////////////////////////////////////////////
//OV7670��������
//��ƽ������
//0:�Զ�
//1:̫��sunny
//2,����cloudy
//3,�칫��office
//4,����home
void OV7670_Light_Mode(uint8_t mode)
{
	uint8_t reg13val=0XE7;//Ĭ�Ͼ�������Ϊ�Զ���ƽ��
	uint8_t reg01val=0;
	uint8_t reg02val=0;
	switch(mode)
	{
		case 1://sunny
			reg13val=0XE5;
			reg01val=0X5A;
			reg02val=0X5C;
			break;	
		case 2://cloudy
			reg13val=0XE5;
			reg01val=0X58;
			reg02val=0X60;
			break;	
		case 3://office
			reg13val=0XE5;
			reg01val=0X84;
			reg02val=0X4c;
			break;	
		case 4://home
			reg13val=0XE5;
			reg01val=0X96;
			reg02val=0X40;
			break;	
	}
	OV7670_WriteReg(REG_COM8, &reg13val);//COM8���� 
	OV7670_WriteReg(REG_BLUE,&reg01val);//AWB��ɫͨ������ 
	OV7670_WriteReg(REG_RED,&reg02val);//AWB��ɫͨ������ 
}				

//ɫ������
//0:-2
//1:-1
//2,0
//3,1
//4,2
void OV7670_Color_Saturation(uint8_t sat)
{
	uint8_t reg4f5054val=0X80;//Ĭ�Ͼ���sat=2,��������ɫ�ȵ�����
 	uint8_t reg52val=0X22;
	uint8_t reg53val=0X5E;
	uint8_t reg54val=0X9E;
 	switch(sat)
	{
		case 0://-2
			reg4f5054val=0X40;  	 
			reg52val=0X11;
			reg53val=0X2F;	 	 
			break;	
		case 1://-1
			reg4f5054val=0X66;	    
			reg52val=0X1B;
			reg53val=0X4B;	  
			break;	
		case 3://1
			reg4f5054val=0X99;	   
			reg52val=0X28;
			reg53val=0X71;	   
			break;	
		case 4://2
			reg4f5054val=0XC0;	   
			reg52val=0X33;
			reg53val=0X8D;	   
			break;	
	}
	OV7670_WriteReg(MTX1,&reg4f5054val);	//ɫ�ʾ���ϵ��1
	OV7670_WriteReg(MTX2,&reg4f5054val);	//ɫ�ʾ���ϵ��2 
	OV7670_WriteReg(MTX3,0X00);			//ɫ�ʾ���ϵ��3  
	OV7670_WriteReg(MTX4,&reg52val);		//ɫ�ʾ���ϵ��4 
	OV7670_WriteReg(MTX5,&reg53val);		//ɫ�ʾ���ϵ��5 
	OV7670_WriteReg(MTX6,&reg4f5054val);	//ɫ�ʾ���ϵ��6  
	OV7670_WriteReg(MTXS,&reg54val);			//MTXS 
}

//��������
//0:-2
//1:-1
//2,0
//3,1
//4,2
void OV7670_Brightness(uint8_t bright)
{
	uint8_t reg55val=0X00;//Ĭ�Ͼ���bright=2
  	switch(bright)
	{
		case 0://-2
			reg55val=0XB0;	 	 
			break;	
		case 1://-1
			reg55val=0X98;	 	 
			break;	
		case 3://1
			reg55val=0X18;	 	 
			break;	
		case 4://2
			reg55val=0X30;	 	 
			break;	
	}
	OV7670_WriteReg(REG_BRIGHT,&reg55val);	//���ȵ��� 
}

//�Աȶ�����
//0:-2
//1:-1
//2,0
//3,1
//4,2
void OV7670_Contrast(uint8_t contrast)
{
	uint8_t reg56val=0X40;//Ĭ�Ͼ���contrast=2
  	switch(contrast)
	{
		case 0://-2
			reg56val=0X30;	 	 
			break;	
		case 1://-1
			reg56val=0X38;	 	 
			break;	
		case 3://1
			reg56val=0X50;	 	 
			break;	
		case 4://2
			reg56val=0X60;	 	 
			break;	
	}
	OV7670_WriteReg(REG_CONTRAS,&reg56val);	//�Աȶȵ��� 
}

//��Ч����
//0:��ͨģʽ    
//1,��Ƭ
//2,�ڰ�   
//3,ƫ��ɫ
//4,ƫ��ɫ
//5,ƫ��ɫ
//6,����	    
void OV7670_Special_Effects(uint8_t eft)
{
	uint8_t reg3aval=0X04;//Ĭ��Ϊ��ͨģʽ
	uint8_t reg67val=0XC0;
	uint8_t reg68val=0X80;
	switch(eft)
	{
		case 1://��Ƭ
			reg3aval=0X24;
			reg67val=0X80;
			reg68val=0X80;
			break;	
		case 2://�ڰ�
			reg3aval=0X14;
			reg67val=0X80;
			reg68val=0X80;
			break;	
		case 3://ƫ��ɫ
			reg3aval=0X14;
			reg67val=0Xc0;
			reg68val=0X80;
			break;	
		case 4://ƫ��ɫ
			reg3aval=0X14;
			reg67val=0X40;
			reg68val=0X40;
			break;	
		case 5://ƫ��ɫ
			reg3aval=0X14;
			reg67val=0X80;
			reg68val=0XC0;
			break;	
		case 6://����
			reg3aval=0X14;
			reg67val=0XA0;
			reg68val=0X40;
			break;	 
	}
	OV7670_WriteReg(REG_TSLB,&reg3aval);//TSLB���� 
	OV7670_WriteReg(0X68,&reg67val);//MANU,�ֶ�Uֵ 
	OV7670_WriteReg(0X67,&reg68val);//MANV,�ֶ�Vֵ 
}

//����ͼ���������
//��QVGA���á�
void ov7670_Window_Set(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height)
{
	uint16_t endx;
	uint16_t endy;
	uint8_t temp; 
	endx=sx+width*2;	//V*2
 	endy=sy+height*2;
	if(endy>784) endy-=784;
	
	OV7670_ReadReg(0X03,&temp);				//��ȡVref֮ǰ��ֵ
	temp&=0XF0;
	temp|=((endx&0X03)<<2)|(sx&0X03);
	OV7670_WriteReg(0X03,&temp);				//����Vref��start��end�����2λ
	sx = (sx >>2)&0xFF;
	endx = (endx >>2)&0xFF;
	OV7670_WriteReg(0X19,(uint8_t *)&sx);			//����Vref��start��8λ
	OV7670_WriteReg(0X1A,(uint8_t *)&endx);			//����Vref��end�ĸ�8λ

	OV7670_ReadReg(0X32,&temp);				//��ȡHref֮ǰ��ֵ
	temp&=0XC0;
	temp|=((endy&0X07)<<3)|(sy&0X07);
	OV7670_WriteReg(0X32,&temp);
	sy = (sy>>3)&0xFF;
	OV7670_WriteReg(0X17,(uint8_t *)&sy);			//����Href��start��8λ
	endy = (endy>>3)&0xFF;
	OV7670_WriteReg(0X18,(uint8_t *)&endy);			//����Href��end�ĸ�8λ
}

int OV7670_Config(void)
{
	int ov_reset_result = OV7670_Reset();
	if (ov_reset_result != OV7670_OK)
	{
		return ov_reset_result;
	}
	int ov_read_reg_result = OV7670_ERROR;
	int ov_write_reg_result = OV7670_ERROR;

	ov_write_reg_result = OV7670_WriteRegList( ov7670_default_regs);
	if (ov_write_reg_result != OV7670_OK)
	{
		return ov_write_reg_result;
	}
#if 0
	uint8_t ov_com3 = 0x04; // REG_COM3 enable scaling
	ov_write_reg_result = OV7670_WriteReg( REG_COM3, &ov_com3);
	if (ov_write_reg_result != OV7670_OK)
	{
		return ov_write_reg_result;
	}
#endif
	ov_write_reg_result = OV7670_WriteRegList( vga_ov7670);
	if (ov_write_reg_result != OV7670_OK)
	{
		return ov_write_reg_result;
	}

	ov_write_reg_result = OV7670_WriteRegList( rgb565_ov7670);
	if (ov_write_reg_result != OV7670_OK)
	{
		return ov_write_reg_result;
	}
	
	// use 25mhz clock directly

	uint8_t ov_clk_rc = 0;
	ov_read_reg_result = OV7670_ReadReg( REG_CLKRC, &ov_clk_rc);
	if (ov_read_reg_result != OV7670_OK)
	{
		return ov_read_reg_result;
	}
	ov_clk_rc = (ov_clk_rc & 0x80) | 0x40; // use clock directly
	ov_write_reg_result = OV7670_WriteReg( REG_CLKRC, &ov_clk_rc);
	if (ov_write_reg_result != OV7670_OK)
	{
		return ov_write_reg_result;
	}

	uint8_t ov_dblv = 0;
	ov_read_reg_result = OV7670_ReadReg( REG_DBLV, &ov_dblv);
	if (ov_read_reg_result != OV7670_OK)
	{
		return ov_read_reg_result;
	}
	ov_dblv = (ov_dblv & 0x3F) | DBLV_PLLOFF; // to enable PLL x4
	ov_write_reg_result = OV7670_WriteReg( REG_DBLV, &ov_dblv);
	if (ov_write_reg_result != OV7670_OK)
	{
		return ov_write_reg_result;
	}
	
	HAL_Delay(100);
	
//	OV7670_Light_Mode(4);
//	OV7670_Color_Saturation(2);
//	OV7670_Brightness(2);
//	OV7670_Contrast(2);
//	OV7670_Special_Effects(0);
//	ov7670_Window_Set(0,320,160,120);
	
	return OV7670_OK;
}

int OV7670_ReadRegList(uint8_t *regs){
	int res = 0;
	for (int i=0;i<0xc9+1;i++){
		res+=OV7670_ReadReg(i, regs++);
	}
	return res;
}


