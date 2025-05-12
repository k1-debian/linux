#ifndef __PANEL_CREALITY_LCD_H__
#define __PANEL_CREALITY_LCD_H__

struct creality_panel_detect {
  const char* name;
  int (*detect)(void);
  void (*init)(void);
};

void SPI_SendCommand(unsigned char i);
void SPI_SendData(unsigned char i);
void SPI_ReadData(unsigned char reg, unsigned char* dest, unsigned char len);

// Panel Model Functions
int lcd_gc9503cv_ue_detect(void);
void lcd_gc9503cv_ue_init(void);

int lcd_st7701_tjc_detect(void);
void lcd_st7701_tjc_init(void);

int lcd_st7701_pj_detect(void);
void lcd_st7701_pj_init(void);

int lcd_st7701_dc_detect(void);
void lcd_st7701_dc_init(void);

int lcd_st7701_tjc_lc_detect(void);
void lcd_st7701_tjc_lc_init(void);

int lcd_st7701_pj_lc_detect(void);
void lcd_st7701_pj_lc_init(void);

int lcd_st7701_dc_lc_detect(void);
void lcd_st7701_dc_lc_init(void);


#endif // __PANEL_CREALITY_LCD_H__
