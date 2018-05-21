/*******************************************************************************
Lazurite用 LCD HD44780 ドライバ

                                           Copyright (c) 2010-2018 Wataru KUNINO
*******************************************************************************/

/*
作成者：xbee@dream.jp
ＵＲＬ：http://www.geocities.jp/bokunimowakaru/diy/xbee/
Ｃ表示：Copyright (c) 2010-2018 Wataru KUNINO                       */

/*********************************************************************
本ソースリストは下記からダウンロードして手を加えたものです。

    http://www.pluto.dti.ne.jp/~nak/h8/h8_tiny.html

                      http://www.geocities.jp/bokunimowakaru/diy/xbee/
*********************************************************************/

/*
Ｈ８版
作成者：nak@pluto.dti.ne.jp
ＵＲＬ：http://www.pluto.dti.ne.jp/~nak/h8/h8_tiny.html
取得日：2009年6月20日
元名称：HD44780 LCD driver (lcd_h8.ZIP -> lcd_h8.c)
Ｃ表示：なし                                                 */
/*
 * HD44780 LCD driver
 *      4bitモード、busyチェック無し
 *
 *  Original is ....
 *   Program:    LCD driver
 *   Created:    28.8.99 21:32
 *   Author:     Brajer Vlado
 *   Comments:   HITACHI LCD driver
 */
 
/*
原作
作成者：Brajer Vlado (vlado.brajer@kks.s-net.net)
ＵＲＬ：http://bray.velenje.cx/avr/lcd/lcd.html
取得日：2007年2月23日
元名称：LCD driver (lcd.zip -> lcd.c)
Ｃ表示：なし                                                 */
 /***********************************************************
  *   Program:    LCD driver
  *   Created:    28.8.99 21:32
  *   Author:     Brajer Vlado
  *                 vlado.brajer@kks.s-net.net
  *   Comments:   HITACHI LCD driver
  ************************************************************/

#ifndef LCD_DRV_HD44780_H
    #define LCD_DRV_HD44780_H
    void lcd_delay(unsigned int);
    void lcd_toggle_E(void);
    void lcd_setup(void);
    void lcd_out(unsigned char  uB);
    void lcd_cls(void);
    void lcd_home(void);
    void lcd_control(unsigned char , unsigned char , unsigned char );
    void lcd_goto(unsigned char);
		void lcd_goto_line(unsigned char);
    void lcd_shift(unsigned char);
    void lcd_putch(char);
    void lcd_putstr(const char *);
    void lcd_disp_bin(unsigned char);
    void lcd_disp_hex(unsigned char);
    void lcd_disp_1(unsigned int);
    void lcd_disp_2(unsigned int);
    void lcd_disp_3(unsigned int);
    void lcd_disp_5(unsigned int);
    void lcd_init(void);
    #ifndef LCD_DRV_HD44780_C
        #define LCD_DRV_HD44780_C
        // #include "lcd_drv_hd44780.c"
    #endif
#endif
