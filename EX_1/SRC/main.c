/*
 * @Description:
 * @Author: MALossov
 * @Date: 2022-04-15 18:23:01
 * @LastEditTime: 2022-04-15 20:41:54
 * @LastEditors: MALossov
 * @Reference:
 */
#include "msp430.h"
#include "oled.h"
#include "type.h"
#include "Pic.h"

void SetVcoreUp(unsigned int level)
{
    // Open PMM registers for write
    PMMCTL0_H = PMMPW_H;
    // Set SVS/SVM high side new level
    SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
    // Set SVM low side to new level
    SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
    // Wait till SVM is settled
    while ((PMMIFG & SVSMLDLYIFG) == 0)
        ;
    // Clear already set flags
    PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
    // Set VCore to new level
    PMMCTL0_L = PMMCOREV0 * level;
    // Wait till new level reached
    if ((PMMIFG & SVMLIFG))
        while ((PMMIFG & SVMLVLRIFG) == 0)
            ;
    // Set SVS/SVM low side to new level
    SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
    // Lock PMM registers for write access
    PMMCTL0_H = 0x00;
}

void initClock(void)
{
    // Increase Vcore setting to level3 to support fsystem=25MHz
    // NOTE: Change core voltage one level at a time..
    SetVcoreUp(0x01);
    SetVcoreUp(0x02);
    SetVcoreUp(0x03);

    UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                        // Set ACLK = REFO

    __bis_SR_register(SCG0);                  // Disable the FLL control loop
    UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_7;                     // Select DCO range 50MHz operation
    UCSCTL2 = FLLD_0 + 762;                   // Set DCO Multiplier for 25MHz
                                              // (N + 1) * FLLRef = Fdco
                                              // (762 + 1) * 32768 = 25MHz
                                              // Set FLL Div = fDCOCLK/2
    __bic_SR_register(SCG0);                  // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
    // UG for optimization.
    // 32 x 32 x 25 MHz / 32,768 Hz ~ 780k MCLK cycles for DCO to settle
    __delay_cycles(782000);

    // Loop until XT1,XT2 & DCO stabilizes - In this case only DCO has to stabilize
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                      // Clear fault flags
    } while (SFRIFG1 & OFIFG);                   // Test oscillator fault flag

}

void main(void)
{
    WDTCTL = WDTPW + WDTHOLD;// stop watchdog timer
    initClock();

    /*
    P1DIR |= BIT0;                //配置p1.0为输出模式
    P4DIR |= BIT7;                //配置p4.7为输出模式
    P1DIR &= ~BIT1;               //配置p1.1为输入模式
    P2DIR &= ~BIT1;               //配置p2.1为输入模式
    */

    P1REN |= BIT1;                //
    P2REN |= BIT1;                //
    P1OUT |= BIT1;                //
    P2OUT |= BIT1;                //配置P1.1,P2.1上拉电阻
    P1OUT &= ~BIT0;               //默认p1.0LED灯关闭
    P4OUT &= ~BIT7;               //默认p4.7LED灯关闭
    P6DIR |= BIT0;                //配置p6.0为输出模式
    P6DIR |= BIT5;                //配置p6.5为输出模式
    P6OUT |= BIT0;                //默认p6.0为开
    P6OUT |= BIT5;                //默认p6.5为开
    int i;

    OLED_Init();
    OLED_Clear();


    P4OUT |= BIT7;
    P1OUT |= BIT0;

    P4OUT &= ~BIT7;
    P1OUT &= ~BIT0;

    OLED_Clear();
    OLED_ShowString(0, 0, "Ready2ShowPics", 16);
    __delay_cycles(20000);
    OLED_Clear();
    int preImgId = -1;
    int curImgId = -1;

    while (1)
    {
        if (!(P1IN & BIT1)) {
            P4OUT |= BIT7;

            // Right
            if (!(P1IN & BIT1))
            {
                while (!(P1IN & BIT1));
                curImgId++;
                P4OUT &= ~BIT7;
            }
        }

        if (!(P2IN & BIT1)) {
            P1OUT |= BIT0;

            // Left
            if (!(P2IN & BIT1)) {
                while (!(P2IN & BIT1));
                curImgId--;
                P1OUT &= ~BIT0;
            }
        }

        if(curImgId < 0)
            curImgId = 0;
        if(curImgId >= picAmount)
            curImgId = picAmount - 1;

        if(curImgId != preImgId) {
            preImgId = curImgId;

            OLED_DrawBMP(curImgId, picMode_Center);
            OLED_DrawBMP_P((SCREEN_WIDTH - picSizeInfo[dockerPicId][0] / 2) * (curImgId + 1) / picAmount, SCREEN_HIGHT - 1, dockerPicId, picMode_Pic_Center);
        }
    }
}

