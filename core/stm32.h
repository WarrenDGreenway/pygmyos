/**************************************************************************
    PygmyOS ( Pygmy Operating System )
    Copyright (C) 2011-2014  Warren D Greenway

    This file is part of PygmyOS.

    PygmyOS is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PygmyOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with PygmyOS.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/
/**************************************************************************
This file implements a standard/mandatory set of functions and macros to
provide a Hardware Abstraction Layer (HAL) for an MCU. 
***************************************************************************/

#pragma once
using namespace std;
#include "pygmy_uart.h"
#include "pygmy_profile.h"
/**************************************************************************
                          Hardware Definitions
**************************************************************************/
//STM32  External and Internal Peripheral defines
#define SRAM_BASE             ((u32)0x20000000)
#define PERIPH_BASE           ((u32)0x40000000)
#define PERIPH_BB_BASE        ((u32)0x42000000) // Alias
#define SRAM_BB_BASE          ((u32)0x22000000) // Alias
#define FLASH_BASE            ((u32)0x40022000) // Flash Registers Starting Address
#define OB_BASE               ((u32)0x1FFFF800) // Flash Option Registers Starting Address

#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)

#ifndef RTC_BASE
    #define RTC_BASE                (APB1PERIPH_BASE + 0x2800)
#endif
#ifndef WWDG_BASE
    #define WWDG_BASE               (APB1PERIPH_BASE + 0x2C00)
#endif
#ifndef IWDG_BASE
    #define IWDG_BASE             (APB1PERIPH_BASE + 0x3000)
#endif

#define BKP_BASE              (APB1PERIPH_BASE + 0x6C00)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)

//#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000)
//#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)


#define DAC_BASE                    (APB1PERIPH_BASE + 0x7400)
#define DAC                 ((DAC_TYPEDEF *) DAC_BASE)

/**************************************************************************
                                Clocks
***************************************************************************/
typedef struct
{
  vu32 CR;
  vu32 CFGR;
  vu32 CIR;
  vu32 APB2RSTR;
  vu32 APB1RSTR;
  vu32 AHBENR;
  vu32 APB2ENR;
  vu32 APB1ENR;
  vu32 BDCR;
  vu32 CSR;
  vu32 CFGR2;
} RCC_TYPEDEF;

#define RCC_BASE              (AHBPERIPH_BASE + 0x1000)

#define RCC_PLLRDY              BIT25                   
#define RCC_PLLON               BIT24                  
#define RCC_CSSON               BIT19                   
#define RCC_HSEBYP              BIT18                   
#define RCC_HSERDY              BIT17                  
#define RCC_HSEON               BIT16                   
#define RCC_HSIRDY              BIT1                  
#define RCC_HSION               BIT0                    
#define RCC_MCO_CLEAR           (BIT26|BIT25|BIT24)       
#define RCC_MCO_SYSCLOCK        BIT26                   
#define RCC_MCO_INTERNALRC      (BIT26|BIT24)             
#define RCC_MCO_EXTCLOCK        (BIT26|BIT25)             
#define RCC_MCO_PLLDIV2         (BIT26|BIT25|BIT24)       
#define RCC_USBPRE              BIT22                   
#define RCC_PLL_CLEAR           (BIT21|BIT20|BIT19|BIT18) 
#define RCC_PLL_MUL2            0                      
#define RCC_PLL_MUL3            BIT18                   
#define RCC_PLL_MUL4            BIT19                  
#define RCC_PLL_MUL5            (BIT19|BIT18)         
#define RCC_PLL_MUL6            BIT20                   
#define RCC_PLL_MUL7            (BIT20|BIT18)            
#define RCC_PLL_MUL8            (BIT20|BIT19)            
#define RCC_PLL_MUL9            (BIT20|BIT19|BIT18)     
#define RCC_PLL_MUL10           BIT21                   
#define RCC_PLL_MUL11           (BIT21|BIT18)            
#define RCC_PLL_MUL12           (BIT21|BIT19)            
#define RCC_PLL_MUL13           (BIT21|BIT19|BIT18)     
#define RCC_PLL_MUL14           (BIT21|BIT20)            
#define RCC_PLL_MUL15           (BIT21|BIT20|BIT18)       
#define RCC_PLL_MUL16           (BIT21|BIT20|BIT19)    
#define RCC_PLLXTPRE            BIT17                  
#define RCC_PLLSRC              BIT16                  
#define RCC_ADC_CLEAR           (BIT15|BIT14)           
#define RCC_ADC_PRE2            0                     
#define RCC_ADC_PRE4            BIT14                 
#define RCC_ADC_PRE6            BIT15                  
#define RCC_ADC_PRE8            (BIT15|BIT14)           
//PLL HCLK for APB
#define RCC_PRE2_CLEAR          (BIT13|BIT12|BIT11)       
#define RCC_PRE2_DIV1           0                     
#define RCC_PRE2_DIV2           BIT13                   
#define RCC_PRE2_DIV4           (BIT13|BIT11)             
#define RCC_PRE2_DIV8           (BIT13|BIT12)           
#define RCC_PRE2_DIV16          (BIT13|BIT12|BIT11)      
#define RCC_PRE1_CLEAR          (BIT10|BIT9|BIT8)        
#define RCC_PRE1_DIV1           0                     
#define RCC_PRE1_DIV2           BIT10                   
#define RCC_PRE1_DIV4           (BIT10|BIT8)             
#define RCC_PRE1_DIV8           (BIT10|BIT9)              
#define RCC_PRE1_DIV16          (BIT10|BIT9|BIT8)         
//PLL SYSCLK for AHB
#define RCC_HPRE_CLEAR          (BIT7|BIT6|BIT5|BIT4)    
#define RCC_HPRE_DIV1           0                       
#define RCC_HPRE_DIV2           BIT7                  
#define RCC_HPRE_DIV4           (BIT7|BIT4)              
#define RCC_HPRE_DIV8           (BIT7|BIT5)               
#define RCC_HPRE_DIV16          (BIT7|BIT5|BIT4)         
#define RCC_HPRE_DIV64          (BIT7|BIT6)              
#define RCC_HPRE_DIV128         (BIT7|BIT6|BIT4)        
#define RCC_HPRE_DIV256         (BIT7|BIT6|BIT5 )         
#define RCC_HPRE_DIV512         (BIT7|BIT6|BIT5|BIT4)     
//PLL System Clock Swtich Status
#define RCC_SWS                 (BIT3|BIT2)          
#define RCC_SWS_HSI             0                     
#define RCC_SWS_HSE             BIT2                  
#define RCC_SWS_PLL             BIT3                   
#define RCC_SW_CLEAR            BIT1|BIT0              
#define RCC_SW_HSI              0                     
#define RCC_SW_HSE              BIT0                   
#define RCC_SW_PLL              BIT1                   
//RCC_CIR Clock Interrupt 
#define RCC_CSSC                BIT23                  
#define RCC_PLLRDYC             BIT20                  
#define RCC_HSERDYC             BIT19                 
#define RCC_HSIRDYC             BIT18                  
#define RCC_LSERDYC             BIT17                 
#define RCC_LSIRDYC             BIT16                  
#define RCC_PLLRDYIE            BIT12                  
#define RCC_HSERDYIE            BIT11                  
#define RCC_HSIRDYIE            BIT10                  
#define RCC_LSERDYIE            BIT9                   
#define RCC_LSIRDYIE            BIT8                    
#define RCC_CSSF                BIT7                   
#define RCC_PLLRDYF             BIT4                   
#define RCC_HSERDYF             BIT3                   
#define RCC_HSIRDYF             BIT2                  
#define RCC_LSERDYF             BIT1                   
#define RCC_LSIRDYF             BIT0                  
//RCC_APB2RSTR Peripheral Reset Register
#define RCC_ADC3RST             BIT15                   
#define RCC_USART1RST           BIT14                  
#define RCC_TIM8RST             BIT13                  
#define RCC_SPI1RST             BIT12                  
#define RCC_TIM1RST             BIT11                 
#define RCC_ADC2RST             BIT10                   
#define RCC_ADC1RST             BIT9                    
#define RCC_IOPGRST             BIT8                    
#define RCC_IOPFRST             BIT7                   
#define RCC_IOPERST             BIT6                   
#define RCC_IOPDRST             BIT5                   
#define RCC_IOPCRST             BIT4                    
#define RCC_IOPBRST             BIT3                  
#define RCC_IOPARST             BIT2                   
#define RCC_AFIORST             BIT0                   
//RCC_APB1RSTR Peripheral Reset Register
#define RCC_DACRST              BIT29                   
#define RCC_PWRRST              BIT28                 
#define RCC_BKPRST              BIT27                
#define RCC_CANRST              BIT25                  
#define RCC_USBRST              BIT23                  
#define RCC_I2C2RST             BIT22                  
#define RCC_I2C1RST             BIT21                   
#define RCC_UART5RST            BIT20                 
#define RCC_UART4RST            BIT19               
#define RCC_UART3RST            BIT18                  
#define RCC_UART2RST            BIT17                  
#define RCC_SPI3RST             BIT15                  
#define RCC_SPI2RST             BIT14                  
#define RCC_WWDGRST             BIT11                
#define RCC_TIM7RST             BIT5                  
#define RCC_TIM6RST             BIT4                   
#define RCC_TIM5RST             BIT3                    
#define RCC_TIM4RST             BIT2                   
#define RCC_TIM3RST             BIT1                 
#define RCC_TIM2RST             BIT0                    
//RCC_AHBENR Peripheral Clock Enable
#define RCC_SDIOEN              BIT10                  
#define RCC_FSMCEN              BIT8                  
#define RCC_CRCEN               BIT6                    
#define RCC_FLITFEN             BIT4                 
#define RCC_SRAMEN              BIT2                  
#define RCC_DMA2EN              BIT1                    
#define RCC_DMA1EN              BIT0                   
//RCC_APB2ENR Peripheral Clock Enable
#define RCC_TIM11EN             BIT21
#define RCC_TIM10EN             BIT20
#define RCC_TIM9EN              BIT19
#define RCC_TIM17EN             BIT18
#define RCC_TIM16EN             BIT17
#define RCC_TIM15EN             BIT16
#define RCC_ADC3EN              BIT15                 
#define RCC_USART1EN            BIT14                  
#define RCC_TIM8EN              BIT13                 
#define RCC_SPI1EN              BIT12                
#define RCC_TIM1EN              BIT11                  
#define RCC_ADC2EN              BIT10                  
#define RCC_ADC1EN              BIT9                 
#define RCC_IOPGEN              BIT8                    
#define RCC_IOPFEN              BIT7                   
#define RCC_IOPEEN              BIT6                  
#define RCC_IOPDEN              BIT5                    
#define RCC_IOPCEN              BIT4                 
#define RCC_IOPBEN              BIT3                 
#define RCC_IOPAEN              BIT2                   
#define RCC_AFIOEN              BIT0                  
//RCC_APB1ENR Peripheral Clock Enable
#define RCC_DACEN               BIT29                 
#define RCC_PWREN               BIT28                  
#define RCC_BKPEN               BIT27                  
#define RCC_CANEN               BIT25                   
#define RCC_I2C3EN              BIT23                 
#define RCC_I2C2EN              BIT22                  
#define RCC_I2C1EN              BIT21                  
#define RCC_USART5EN            BIT20                 
#define RCC_USART4EN            BIT19                 
#define RCC_USART3EN            BIT18                  
#define RCC_USART2EN            BIT17                  
#define RCC_SPI3EN              BIT15                  
#define RCC_SPI2EN              BIT14                  
#define RCC_WWDGEN              BIT11  
#define RCC_TIM14EN             BIT8
#define RCC_TIM13EN             BIT7
#define RCC_TIM12EN             BIT6
#define RCC_TIM7EN              BIT5                  
#define RCC_TIM6EN              BIT4                  
#define RCC_TIM5EN              BIT3                 
#define RCC_TIM4EN              BIT2                  
#define RCC_TIM3EN              BIT1                 
#define RCC_TIM2EN              BIT0                  
//RCC_BDCR Backup Domain Control Register
#define RCC_BDRST               BIT16                
#define RCC_RTCEN               BIT15                 
#define RCC_RTCSEL1             BIT9                  
#define RCC_RTCSEL0             BIT8                  
#define RCC_LSEBYP              BIT2                   
#define RCC_LSERDY              BIT1                 
#define RCC_LSEON               BIT0                   
//RCC_CSR Control Status Register
#define RCC_LPWRRSTF            BIT31                
#define RCC_WWDGRSTF            BIT30              
#define RCC_IWDGRSTF            BIT29                 
#define RCC_SFTRSTF             BIT28                  
#define RCC_PORRSTF             BIT27                 
#define RCC_PINRSTF             BIT26                
#define RCC_RMVF                BIT24                  
#define RCC_LSIRDY              BIT1                  
#define RCC_LSION               BIT0 

#define RCC_HSE_OFF                      ((u32)0x00000000)
#define RCC_HSE_ON                       ((u32)0x00010000)
#define RCC_HSE_Bypass                   ((u32)0x00040000)
#define RCC_PLLSOURCE_HSI_Div2           ((u32)0x00000000)
#define RCC_PLLSOURCE_HSE_Div1           ((u32)0x00010000)
#define RCC_PLLSOURCE_HSE_Div2           ((u32)0x00030000)

#define RCC_PLL_X2                     ((u32)0x00000000)
#define RCC_PLL_X3                     ((u32)0x00040000)
#define RCC_PLL_X4                     ((u32)0x00080000)
#define RCC_PLL_X5                     ((u32)0x000C0000)
#define RCC_PLL_X6                     ((u32)0x00100000)
#define RCC_PLL_X7                     ((u32)0x00140000)
#define RCC_PLL_X8                     ((u32)0x00180000)
#define RCC_PLL_X9                     ((u32)0x001C0000)
#define RCC_PLL_X10                    ((u32)0x00200000)
#define RCC_PLL_X11                    ((u32)0x00240000)
#define RCC_PLL_X12                    ((u32)0x00280000)
#define RCC_PLL_X13                    ((u32)0x002C0000)
#define RCC_PLL_X14                    ((u32)0x00300000)
#define RCC_PLL_X15                    ((u32)0x00340000)
#define RCC_PLL_X16                    ((u32)0x00380000)

#define RCC_SYSCLKSOURCE_HSI             ((u32)0x00000000)
#define RCC_SYSCLKSOURCE_HSE             ((u32)0x00000001)
#define RCC_SYSCLKSOURCE_PLLCLK          ((u32)0x00000002)

#define RCC_SYSCLK_DIV1                  ((u32)0x00000000)
#define RCC_SYSCLK_DIV2                  ((u32)0x00000080)
#define RCC_SYSCLK_DIV4                  ((u32)0x00000090)
#define RCC_SYSCLK_DIV8                  ((u32)0x000000A0)
#define RCC_SYSCLK_DIV16                 ((u32)0x000000B0)
#define RCC_SYSCLK_DIV64                 ((u32)0x000000C0)
#define RCC_SYSCLK_DIV128                ((u32)0x000000D0)
#define RCC_SYSCLK_DIV256                ((u32)0x000000E0)
#define RCC_SYSCLK_DIV512                ((u32)0x000000F0)

#define RCC_HCLK_DIV1                    ((u32)0x00000000)
#define RCC_HCLK_DIV2                    ((u32)0x00000400)
#define RCC_HCLK_DIV4                    ((u32)0x00000500)
#define RCC_HCLK_DIV8                    ((u32)0x00000600)
#define RCC_HCLK_DIV16                   ((u32)0x00000700)

#define RCC_IT_LSIRDY                    ((u8)0x01)
#define RCC_IT_LSERDY                    ((u8)0x02)
#define RCC_IT_HSIRDY                    ((u8)0x04)
#define RCC_IT_HSERDY                    ((u8)0x08)
#define RCC_IT_PLLRDY                    ((u8)0x10)
#define RCC_IT_CSS                       ((u8)0x80)

#define RCC_USBCLKSOURCE_PLLCLK_1DIV5    ((u8)0x00)
#define RCC_USBCLKSOURCE_PLLCLK_DIV1     ((u8)0x01)

#define RCC_PCLK2_Div2                   ((u32)0x00000000)
#define RCC_PCLK2_Div4                   ((u32)0x00004000)
#define RCC_PCLK2_Div6                   ((u32)0x00008000)
#define RCC_PCLK2_Div8                   ((u32)0x0000C000)

#define RCC_LSE_OFF                      ((u8)0x00)
#define RCC_LSE_ON                       ((u8)0x01)
#define RCC_LSE_BYPASS                   ((u8)0x04)

#define RCC_RTCCLKSOURCE_LSE             ((u32)0x00000100)
#define RCC_RTCCLKSOURCE_LSI             ((u32)0x00000200)
#define RCC_RTCCLKSOURCE_HSE_Div128      ((u32)0x00000300)

#define RCC_AHB_DMA                ((u32)0x00000001)
#define RCC_AHB_SRAM               ((u32)0x00000004)
#define RCC_AHB_FLITF              ((u32)0x00000010)

#define RCC_APB2_AFIO              ((u32)0x00000001)
#define RCC_APB2_GPIOA             ((u32)0x00000004)
#define RCC_APB2_GPIOB             ((u32)0x00000008)
#define RCC_APB2_GPIOC             ((u32)0x00000010)
#define RCC_APB2_GPIOD             ((u32)0x00000020)
#define RCC_APB2_GPIOE             ((u32)0x00000040)
#define RCC_APB2_ADC1              ((u32)0x00000200)
#define RCC_APB2_ADC2              ((u32)0x00000400)
#define RCC_APB2_TIM1              ((u32)0x00000800)
#define RCC_APB2_SPI1              ((u32)0x00001000)
#define RCC_APB2_USART1            ((u32)0x00004000)
#define RCC_APB2_ALL               ((u32)0x00005E7D)

#define RCC_APB1_TIM2              ((u32)0x00000001)
#define RCC_APB1_TIM3              ((u32)0x00000002)
#define RCC_APB1_TIM4              ((u32)0x00000004)
#define RCC_APB1_WWDG              ((u32)0x00000800)
#define RCC_APB1_SPI2              ((u32)0x00004000)
#define RCC_APB1_USART2            ((u32)0x00020000)
#define RCC_APB1_USART3            ((u32)0x00040000)
#define RCC_APB1_I2C1              ((u32)0x00200000)
#define RCC_APB1_I2C2              ((u32)0x00400000)
#define RCC_APB1_USB               ((u32)0x00800000)
#define RCC_APB1_CAN               ((u32)0x02000000)
#define RCC_APB1_BKP               ((u32)0x08000000)
#define RCC_APB1_PWR               ((u32)0x10000000)
#define RCC_APB1_ALL               ((u32)0x1AE64807)
#define RCC_MCO_NOCLOCK                  ((u8)0x00)
#define RCC_MCO_SYSCLK                   ((u8)0x04)
#define RCC_MCO_HSI                      ((u8)0x05)
#define RCC_MCO_HSE                      ((u8)0x06)
#define RCC_MCO_PLLCLK_Div2              ((u8)0x07)
#define RCC_FLAG_HSIRDY                  ((u8)0x20)
#define RCC_FLAG_HSERDY                  ((u8)0x31)
#define RCC_FLAG_PLLRDY                  ((u8)0x39)
#define RCC_FLAG_LSERDY                  ((u8)0x41)
#define RCC_FLAG_LSIRDY                  ((u8)0x61)
#define RCC_FLAG_PINRST                  ((u8)0x7A)
#define RCC_FLAG_PORRST                  ((u8)0x7B)
#define RCC_FLAG_SFTRST                  ((u8)0x7C)
#define RCC_FLAG_IWDGRST                 ((u8)0x7D)
#define RCC_FLAG_WWDGRST                 ((u8)0x7E)
#define RCC_FLAG_LPWRRST                 ((u8)0x7F)

#define RCC_OFFSET                (RCC_BASE - PERIPH_BASE)
#define CR_OFFSET                 (RCC_OFFSET + 0x00)
#define HSION_BITNUMBER           0x00
#define CR_HSION_BB               (PERIPH_BB_BASE + (CR_OFFSET * 32) + (HSION_BitNumber * 4))
#define PLLON_BITNUMBER           0x18
#define CR_PLLON_BB               (PERIPH_BB_BASE + (CR_OFFSET * 32) + (PLLON_BitNumber * 4))
#define CSSON_BITNUMBER           0x13
#define CR_CSSON_BB               (PERIPH_BB_BASE + (CR_OFFSET * 32) + (CSSON_BitNumber * 4))
#define CFGR_OFFSET               (RCC_OFFSET + 0x04)
#define USBPRE_BITNUMBER          0x16
#define CFGR_USBPRE_BB            (PERIPH_BB_BASE + (CFGR_OFFSET * 32) + (USBPRE_BitNumber * 4))
#define BDRST_BITNUMBER           0x10
#define BDCR_BDRST_BB             (PERIPH_BB_BASE + (BDCR_OFFSET * 32) + (BDRST_BitNumber * 4))
#define BDCR_OFFSET               (RCC_OFFSET + 0x20)
#define RTCEN_BITNUMBER           0x0F
#define BDCR_RTCEN_BB             (PERIPH_BB_BASE + (BDCR_OFFSET * 32) + (RTCEN_BitNumber * 4))

#define CR1_SPE_SET          ((u16)0x0040)
#define CR1_SPE_RESET        ((u16)0xFFBF)
#define CR1_CRCNEXT_SET      ((u16)0x1000)
#define CR1_CRCEN_SET        ((u16)0x2000)
#define CR1_CRCEN_RESET      ((u16)0xDFFF)
#define CR2_SSOE_SET        ((u16)0x0004)
#define CR2_SSOE_RESET      ((u16)0xFFFB)
#define CR1_CLEAR_Mask       ((u16)0x3040)

#ifndef BKP
    #define BKP                 ((BKP_TYPEDEF *) BKP_BASE)
#endif
#ifndef PWR
    #define PWR                 ((PWR_TYPEDEF *) PWR_BASE)
#endif

#define OB                      ((OB_TYPEDEF *) OB_BASE) 
#define RCC                     ((RCC_TYPEDEF *) RCC_BASE)

#define PYGMY_RCC_HSE_READY                 ( RCC->CR & RCC_HSERDY )
#define PYGMY_RCC_HSI_READY                 ( RCC->CR & RCC_HSIRDY )
#define PYGMY_RCC_PLL_READY                 ( RCC->CR & RCC_PLLRDY )
#define PYGMY_RCC_PLL_ENABLE                RCC->CR |= RCC_PLLON;
#define PYGMY_RCC_PLL_DISABLE               RCC->CR &= ~RCC_PLLON;
#define PYGMY_RCC_LSE_ENABLE                RCC->CR |= RCC_LSEON;
#define PYGMY_RCC_LSE_DISABLE               RCC->CR &= ~RCC_LSEON;
#define PYGMY_RCC_HSE_ENABLE                RCC->CR |= RCC_HSEON;
#define PYGMY_RCC_HSE_DISABLE               RCC->CR &= ~( RCC_HSEON | RCC_HSEBYP );
#define PYGMY_RCC_HSE_BYPASS                RCC->CR &= ~RCC_HSEON; RCC->CR |= ( RCC_HSEBYP|RCC_HSEON );
#define PYGMY_RCC_HSI_ENABLE                RCC->CFGR |= RCC_HSION;
#define PYGMY_RCC_HSI_DISABLE               RCC->CFGR &= ~RCC_HSION;
#define PYGMY_RCC_PWR_ENABLE                RCC->APB1ENR |= RCC_PWREN;
#define PYGMY_RCC_PWR_DISABLE               RCC->APB1ENR &= ~RCC_PWREN;
#define PYGMY_RCC_BKP_ENABLE                RCC->APB1ENR |= RCC_BKPEN;
#define PYGMY_RCC_BKP_DISABLE               RCC->APB1ENR &= ~RCC_BKPEN;
#define PYGMY_RCC_DMA1_ENABLE               RCC->AHBENR |= RCC_DMA1EN;
#define PYGMY_RCC_DMA1_DISABLE              RCC->AHBENR &= ~RCC_DMA1EN;
#define PYGMY_RCC_DMA2_ENABLE               RCC->AHBENR |= RCC_DMA2EN;
#define PYGMY_RCC_DMA2_DISABLE              RCC->AHBENR &= ~RCC_DMA2EN;
#define PYGMY_RCC_ADC1_ENABLE               RCC->APB2ENR |= RCC_ADC1EN;
#define PYGMY_RCC_ADC1_DISABLE              RCC->APB2ENR &= ~RCC_ADC1EN;
#define PYGMY_RCC_ADC2_ENABLE               RCC->APB2ENR |= RCC_ADC2EN;
#define PYGMY_RCC_ADC2_DISABLE              RCC->APB2ENR &= ~RCC_ADC2EN;
#define PYGMY_RCC_ADC3_ENABLE               RCC->APB2ENR |= RCC_ADC3EN;
#define PYGMY_RCC_ADC3_DISABLE              RCC->APB2ENR &= ~RCC_ADC3EN;
#define PYGMY_RCC_AFIO_ENABLE               RCC->APB2ENR |= RCC_AFIOEN;
#define PYGMY_RCC_AFIO_DISABLE              RCC->APB2ENR &= ~RCC_AFIOEN;
#define PYGMY_RCC_GPIOA_ENABLE              RCC->APB2ENR |= RCC_IOPAEN;
#define PYGMY_RCC_GPIOB_ENABLE              RCC->APB2ENR |= RCC_IOPBEN;
#define PYGMY_RCC_GPIOC_ENABLE              RCC->APB2ENR |= RCC_IOPCEN;
#define PYGMY_RCC_GPIOD_ENABLE              RCC->APB2ENR |= RCC_IOPDEN;
#define PYGMY_RCC_GPIOE_ENABLE              RCC->APB2ENR |= RCC_IOPEEN;
#define PYGMY_RCC_GPIOF_ENABLE              RCC->APB2ENR |= RCC_IOPFEN;
#define PYGMY_RCC_GPIOG_ENABLE              RCC->APB2ENR |= RCC_IOPGEN;
#define PYGMY_RCC_GPIOA_DISABLE             RCC->APB2ENR &= ~RCC_IOPAEN;
#define PYGMY_RCC_GPIOB_DISABLE             RCC->APB2ENR &= ~RCC_IOPBEN;
#define PYGMY_RCC_GPIOC_DISABLE             RCC->APB2ENR &= ~RCC_IOPCEN;
#define PYGMY_RCC_GPIOD_DISABLE             RCC->APB2ENR &= ~RCC_IOPDEN;
#define PYGMY_RCC_GPIOE_DISABLE             RCC->APB2ENR &= ~RCC_IOPEEN;
#define PYGMY_RCC_GPIOF_DISABLE             RCC->APB2ENR &= ~RCC_IOPFEN;
#define PYGMY_RCC_GPIOG_DISABLE             RCC->APB2ENR &= ~RCC_IOPGEN;
#define PYGMY_RCC_TIMER1_ENABLE             RCC->APB2ENR |= RCC_TIM1EN;
#define PYGMY_RCC_TIMER1_DISABLE            RCC->APB2ENR &= ~RCC_TIM1EN;
#define PYGMY_RCC_TIMER2_ENABLE             RCC->APB1ENR |= RCC_TIM2EN;
#define PYGMY_RCC_TIMER2_DISABLE            RCC->APB1ENR &= ~RCC_TIM2EN;
#define PYGMY_RCC_TIMER3_ENABLE             RCC->APB1ENR |= RCC_TIM3EN;
#define PYGMY_RCC_TIMER3_DISABLE            RCC->APB1ENR &= ~RCC_TIM3EN;
#define PYGMY_RCC_TIMER4_ENABLE             RCC->APB1ENR |= RCC_TIM4EN;
#define PYGMY_RCC_TIMER4_DISABLE            RCC->APB1ENR &= ~RCC_TIM4EN;
#define PYGMY_RCC_TIMER5_ENABLE             RCC->APB1ENR |= RCC_TIM5EN;
#define PYGMY_RCC_TIMER5_DISABLE            RCC->APB1ENR &= ~RCC_TIM5EN;
#define PYGMY_RCC_TIMER6_ENABLE             RCC->APB1ENR |= RCC_TIM6EN;
#define PYGMY_RCC_TIMER6_DISABLE            RCC->APB1ENR &= ~RCC_TIM6EN;
#define PYGMY_RCC_TIMER7_ENABLE             RCC->APB1ENR |= RCC_TIM7EN;
#define PYGMY_RCC_TIMER7_DISABLE            RCC->APB1ENR &= ~RCC_TIM7EN;
#define PYGMY_RCC_TIMER8_ENABLE             RCC->APB2ENR |= RCC_TIM8EN;
#define PYGMY_RCC_TIMER8_DISABLE            RCC->APB2ENR &= ~RCC_TIM8EN;                                            
#define PYGMY_RCC_TIMER9_ENABLE             RCC->APB2ENR |= RCC_TIM9EN;
#define PYGMY_RCC_TIMER9_DISABLE            RCC->APB2ENR &= ~RCC_TIM9EN;
#define PYGMY_RCC_TIMER10_ENABLE            RCC->APB2ENR |= RCC_TIM10EN;
#define PYGMY_RCC_TIMER10_DISABLE           RCC->APB2ENR &= ~RCC_TIM10EN;
#define PYGMY_RCC_TIMER11_ENABLE            RCC->APB2ENR |= RCC_TIM11EN;
#define PYGMY_RCC_TIMER11_DISABLE           RCC->APB2ENR &= ~RCC_TIM11EN;
#define PYGMY_RCC_TIMER12_ENABLE            RCC->APB1ENR |= RCC_TIM12EN;
#define PYGMY_RCC_TIMER12_DISABLE           RCC->APB1ENR &= ~RCC_TIM12EN;
#define PYGMY_RCC_TIMER13_ENABLE            RCC->APB1ENR |= RCC_TIM13EN;
#define PYGMY_RCC_TIMER13_DISABLE           RCC->APB1ENR &= ~RCC_TIM13EN;
#define PYGMY_RCC_TIMER14_ENABLE            RCC->APB1ENR |= RCC_TIM14EN;
#define PYGMY_RCC_TIMER14_DISABLE           RCC->APB1ENR &= ~RCC_TIM14EN;
#define PYGMY_RCC_TIMER15_ENABLE            RCC->APB2ENR |= RCC_TIM15EN;
#define PYGMY_RCC_TIMER15_DISABLE           RCC->APB2ENR &= ~RCC_TIM15EN;
#define PYGMY_RCC_TIMER16_ENABLE            RCC->APB2ENR |= RCC_TIM16EN;
#define PYGMY_RCC_TIMER16_DISABLE           RCC->APB2ENR &= ~RCC_TIM16EN;
#define PYGMY_RCC_TIMER17_ENABLE            RCC->APB2ENR |= RCC_TIM17EN;
#define PYGMY_RCC_TIMER17_DISABLE           RCC->APB2ENR &= ~RCC_TIM17EN;
#define PYGMY_RCC_SPI1_ENABLE               RCC->APB2ENR |= RCC_SPI1EN;
#define PYGMY_RCC_SPI1_DISABLE              RCC->APB2ENR &= ~RCC_SPI1EN;
#define PYGMY_RCC_SPI2_ENABLE               RCC->APB1ENR |= RCC_SPI2EN;
#define PYGMY_RCC_SPI2_DISABLE              RCC->APB1ENR &= ~RCC_SPI2EN;
#define PYGMY_RCC_SPI3_ENABLE               RCC->APB1ENR |= RCC_SPI3EN;
#define PYGMY_RCC_SPI3_DISABLE              RCC->APB1ENR &= ~RCC_SPI3EN;
#define PYGMY_RCC_DAC_ENABLE                RCC->APB1ENR |= RCC_DACEN;
#define PYGMY_RCC_DAC_DISABLE               RCC->APB1ENR &= ~RCC_DACEN;
#define PYGMY_RCC_USB_ENABLE                RCC->APB1ENR |= RCC_USBEN;
#define PYGMY_RCC_USB_DISABLE               RCC->APB1ENR &= ~RCC_USBEN;
#define PYGMY_RCC_USART5_ENABLE             RCC->APB1ENR |= RCC_USART5EN;
#define PYGMY_RCC_USART5_DISABLE            RCC->APB1ENR &= ~RCC_USART5EN;
#define PYGMY_RCC_USART4_ENABLE             RCC->APB1ENR |= RCC_USART4EN;
#define PYGMY_RCC_USART4_DISABLE            RCC->APB1ENR &= ~RCC_USART4EN;
#define PYGMY_RCC_USART3_ENABLE             RCC->APB1ENR |= RCC_USART3EN;
#define PYGMY_RCC_USART3_DISABLE            RCC->APB1ENR &= ~RCC_USART3EN;
#define PYGMY_RCC_USART2_ENABLE             RCC->APB1ENR |= RCC_USART2EN;
#define PYGMY_RCC_USART2_DISABLE            RCC->APB1ENR &= ~RCC_USART2EN;
#define PYGMY_RCC_USART1_ENABLE             RCC->APB2ENR |= RCC_USART1EN;
#define PYGMY_RCC_USART1_DISABLE            RCC->APB2ENR &= ~RCC_USART1EN;
#define PYGMY_RCC_I2C1_ENABLE               RCC->APB1ENR |= RCC_I2C1EN;
#define PYGMY_RCC_I2C1_DISABLE              RCC->APB1ENR &= ~RCC_I2C1EN;
#define PYGMY_RCC_I2C2_ENABLE               RCC->APB1ENR |= RCC_I2C2EN;
#define PYGMY_RCC_I2C2_DISABLE              RCC->APB1ENR &= ~RCC_I2C2EN;
#define PYGMY_RCC_I2C3_ENABLE               RCC->APB1ENR |= RCC_I2C3EN;
#define PYGMY_RCC_I2C3_DISABLE              RCC->APB1ENR &= ~RCC_I2C3EN;

#define PYGMY_RCC_WWDGEN_ENABLE             RCC->APB1ENR |= RCC_WWDGEN;
#define PYGMY_RCC_WWDGEN_DISABLE            RCC->APB1ENR &= ~RCC_WWDGEN;
/**************************************************************************
                                Timers
***************************************************************************/
// TIMX CR1
#define TIM_CKD_CLEAR               (BIT9|BIT8)                      
#define TIM_CKD_M0DE0               0                       
#define TIM_CKD_MODE1               BIT8                      
#define TIM_CKD_MODE2               BIT9                        
#define TIM_ARPE                    BIT7                      
#define TIM_CMS_CLEAR               (BIT6|BIT5)                
#define TIM_CMS_MODE0               0                      
#define TIM_CMS_MODE1               BIT5                     
#define TIM_CMS_MODE2               BIT6                     
#define TIM_CMS_MODE3               (BIT6|BIT5)             
#define TIM_DIR                     BIT4                        
#define TIM_OPM                     BIT3                        
#define TIM_URS                     BIT2                          
#define TIM_UDIS                    BIT1                         
#define TIM_CEN                     BIT0                        
// TIMX CR2
#define TIM_TI1S                    BIT7                      
#define TIM_MMS_CLEAR               (BIT6|BIT5|BIT4)               
#define TIM_MMS_MODE0               0                            
#define TIM_MMS_MODE1               BIT4                       
#define TIM_MMS_MODE2               BIT5                         
#define TIM_MMS_MODE3               (BIT5|BIT4)              
#define TIM_MMS_MODE4               BIT6                    
#define TIM_MMS_MODE5               (BIT6|BIT4)                
#define TIM_MMS_MODE6               (BIT6|BIT5)               
#define TIM_MMS_MODE7               (BIT6|BIT5|BIT4)        
#define TIM_CCDS                    BIT3                     
// TIMX_SMCR Slave Mode Control       
#define TIM_ETP                     BIT15                      
#define TIM_ECE                     BIT14                    
#define TIM_ETPS_CLEAR              (BIT13|BIT12)            
#define TIM_ETPS_DIV1               0                             
#define TIM_ETPS_DIV2               BIT12                         
#define TIM_ETPS_DIV4               BIT13                          
#define TIM_ETPS_DIV8               (BIT13|BIT12)                 
#define TIM_ETF_CLEAR               (BIT11|BIT10|BIT9|BIT8)     
#define TIM_ETF_MODE0               0                               
#define TIM_ETF_MODE1               BIT8                           
#define TIM_ETF_MODE2               BIT9                          
#define TIM_ETF_MODE3               (BIT9|BIT8)                  
#define TIM_ETF_MODE4               BIT10                      
#define TIM_ETF_MODE5               (BIT10|BIT8)                 
#define TIM_ETF_MODE6               (BIT10|BIT9)                
#define TIM_ETF_MODE7               (BIT10|BIT9|BIT8)           
#define TIM_ETF_MODE8               BIT11                      
#define TIM_ETF_MODE9               (BIT11|BIT8)               
#define TIM_ETF_MODE10              (BIT11|BIT9)               
#define TIM_ETF_MODE11              (BIT11|BIT9|BIT8)          
#define TIM_ETF_MODE12              (BIT11|BIT10)                
#define TIM_ETF_MODE13              (BIT11|BIT10|BIT8)          
#define TIM_ETF_MODE14              (BIT11|BIT10|BIT9)           
#define TIM_ETF_MODE15              (BIT11|BIT10|BIT9|BIT8)    
#define TIM_MSM                     BIT7                        
#define TIM_TS_CLEAR                (BIT6|BIT5|BIT4)              
#define TIM_TS_MODE0                0                            
#define TIM_TS_MODE1                BIT4                         
#define TIM_TS_MODE2                BIT5                        
#define TIM_TS_MODE3                (BIT5|BIT4)                  
#define TIM_TS_MODE4                BIT6                           
#define TIM_TS_MODE5                (BIT6|BIT4)                 
#define TIM_TS_MODE6                (BIT6|BIT5)                   
#define TIM_TS_MODE7                (BIT6|BIT5|BIT4)              
#define TIM_SMS_CLEAR               (BIT2|BIT1|BIT0)                
#define TIM_SMS_MODE0               0                               
#define TIM_SMS_MODE1               BIT0                          
#define TIM_SMS_MODE2               BIT1                           
#define TIM_SMS_MODE3               (BIT1|BIT0)                    
#define TIM_SMS_MODE4               BIT2                          
#define TIM_SMS_MODE5               (BIT2|BIT0)                 
#define TIM_SMS_MODE6               (BIT2|BIT1)                    
#define TIM_SMS_MODE7               (BIT2|BIT1|BIT0)                
// TIMX_DIER DMA Interrupt Enable Register
#define TIM_TDE                     BIT14                         
#define TIM_CC4DE                   BIT12                       
#define TIM_CC3DE                   BIT11                          
#define TIM_CC2DE                   BIT10                          
#define TIM_CC1DE                   BIT9                          
#define TIM_UDE                     BIT8                         
#define TIM_TIE                     BIT6                           
#define TIM_CC4IE                   BIT4                         
#define TIM_CC3IE                   BIT3                           
#define TIM_CC2IE                   BIT2                           
#define TIM_CC1IE                   BIT1                            
#define TIM_UIE                     BIT0                          
// TIMX_SR Status Register
#define TIM_CC4OF                   BIT12                      
#define TIM_CC3OF                   BIT11                        
#define TIM_CC2OF                   BIT10                         
#define TIM_CC1OF                   BIT9                         
#define TIM_TIF                     BIT6                          
#define TIM_CC4IF                   BIT4                        
#define TIM_CC3IF                   BIT3                         
#define TIM_CC2IF                   BIT2                           
#define TIM_CC1IF                   BIT1                           
#define TIM_UIF                     BIT0                           
// TIMX_EGR Event Generation Register
#define TIM_TG                      BIT6                          
#define TIM_CC4G                    BIT4                            
#define TIM_CC3G                    BIT3                           
#define TIM_CC2G                    BIT2                           
#define TIM_CC1G                    BIT1                           
#define TIM_UG                      BIT0                           
// TIMX_CCMR1 Capture Compare Mode Register 1
#define TIM_OC2CE                   BIT15                         
#define TIM_OC2M_CLEAR              (BIT14|BIT13|BIT12)
#define TIM_OC2M_MODE0              0
#define TIM_OC2M_MODE1              BIT12
#define TIM_OC2M_MODE2              BIT13
#define TIM_OC2M_MODE3              (BIT13|BIT12)
#define TIM_OC2M_MODE4              BIT14
#define TIM_OC2M_MODE5              (BIT14|BIT12)
#define TIM_OC2M_MODE6              (BIT14|BIT13)
#define TIM_OC2M_MODE7              (BIT14|BIT13|BIT12)
#define TIM_OC2M_PWM1               TIM_OC2M_MODE6
#define TIM_OC2M_PWM2               TIM_OC2M_MODE7
#define TIM_OC2PE                   BIT11                          
#define TIM_OC2FE                   BIT10                           
#define TIM_CC2S_CLEAR              (BIT9|BIT8)                     
#define TIM_CC2S_MODE0              0                                
#define TIM_CC2S_MODE1              BIT8                           
#define TIM_CC2S_MODE2              BIT9                            
#define TIM_CC2S_MODE3              (BIT9|BIT8)                      
#define TIM_OC1CE                   BIT7                            
#define TIM_OC1M_CLEAR              (BIT6|BIT5|BIT4)               
#define TIM_OC1M_MODE0              0                             
#define TIM_OC1M_MODE1              BIT4                            
#define TIM_OC1M_MODE2              BIT5                            
#define TIM_OC1M_MODE3              (BIT5|BIT4)                   
#define TIM_OC1M_MODE4              BIT6                             
#define TIM_OC1M_MODE5              (BIT6|BIT4)                     
#define TIM_OC1M_MODE6              (BIT6|BIT5)                     
#define TIM_OC1M_MODE7              (BIT6|BIT5|BIT4)
#define TIM_OC1M_PWM1               TIM_OC1M_MODE6
#define TIM_OC1M_PWM2               TIM_OC1M_MODE7
#define TIM_OC1PE                   BIT3                           
#define TIM_OC1FE                   BIT2                            
#define TIM_CC1S_CLEAR              (BIT1|BIT0)                     
#define TIM_CC1S_MODE0              0                               
#define TIM_CC1S_MODE1              BIT0                           
#define TIM_CC1S_MODE2              BIT1                             
#define TIM_CC1S_MODE3              BIT2                            
// This register has dual purpose bits, the following are the alternate defs 
#define TIM_IC2F_CLEAR              (BIT15|BIT14|BIT13|BIT12)         
#define TIM_IC2PSC_CLEAR            (BIT11|BIT10)                    
#define TIM_IC1F_CLEAR              (BIT7|BIT6|BIT5|BIT4)            
#define TIM_IC1F_MODE0              0                                 
#define TIM_IC1F_MODE1              BIT4                            
#define TIM_IC1F_MODE2              BIT5                             
#define TIM_IC1F_MODE3              (BIT5|BIT4)                       
#define TIM_IC1F_MODE4              BIT6                             
#define TIM_IC1F_MODE5              (BIT6|BIT4)                       
#define TIM_IC1F_MODE6              (BIT6|BIT5)                       
#define TIM_IC1F_MODE7              (BIT6|BIT5|BIT4)                 
#define TIM_IC1F_MODE8              BIT7                              
#define TIM_IC1F_MODE9              (BIT7|BIT4)                       
#define TIM_IC1F_MODE10             (BIT7|BIT5)                       
#define TIM_IC1F_MODE11             (BIT7|BIT5|BIT4)                  
#define TIM_IC1F_MODE12             (BIT7|BIT6)                     
#define TIM_IC1F_MODE13             (BIT7|BIT6|BIT4)                  
#define TIM_IC1F_MODE14             (BIT7|BIT6|BIT5)                  
#define TIM_IC1F_MODE15             (BIT7|BIT6|BIT5|BIT4)             
#define TIM_IC1PSC_CLEAR            (BIT3|BIT2)                      
#define TIM_IC1PSC_MODE0            0                                
#define TIM_IC1PSC_MODE1            BIT2                             
#define TIM_IC1PSC_MODE2            BIT3                             
#define TIM_IC1PSC_MODE3            (BIT3|BIT2)                       
// TIMX_CCMR2
#define TIM_OC4CE                   BIT15                             
#define TIM_OC4M_CLEAR              (BIT14|BIT13|BIT12)  
#define TIM_OC4M_MODE0              0
#define TIM_OC4M_MODE1              BIT12
#define TIM_OC4M_MODE2              BIT13
#define TIM_OC4M_MODE3              (BIT13|BIT12)
#define TIM_OC4M_MODE4              BIT14
#define TIM_OC4M_MODE5              (BIT14|BIT12)
#define TIM_OC4M_MODE6              (BIT14|BIT13)
#define TIM_OC4M_MODE7              (BIT14|BIT13|BIT12)
#define TIM_OC4M_PWM1               TIM_OC4M_MODE6
#define TIM_OC4M_PWM2               TIM_OC4M_MODE7
#define TIM_OC4PE                   BIT11                             
#define TIM_OC4FE                   BIT10                             
#define TIM_CC4S_CLEAR              (BIT9|BIT8)                      
#define TIM_CC4S_MODE0              0                                
#define TIM_CC4S_MODE1              BIT8                            
#define TIM_CC4S_MODE2              BIT9                           
#define TIM_CC4S_MODE3              (BIT9|BIT8)                       
#define TIM_OC3CE                   BIT7                              
#define TIM_OC3M_CLEAR              (BIT6|BIT5|BIT4)   
#define TIM_OC3M_MODE0              0
#define TIM_OC3M_MODE1              BIT4
#define TIM_OC3M_MODE2              BIT5
#define TIM_OC3M_MODE3              (BIT5|BIT4)
#define TIM_OC3M_MODE4              BIT6
#define TIM_OC3M_MODE5              (BIT6|BIT4)
#define TIM_OC3M_MODE6              (BIT6|BIT5)
#define TIM_OC3M_MODE7              (BIT6|BIT5|BIT4)
#define TIM_OC3M_PWM1               TIM_OC3M_MODE6
#define TIM_OC3M_PWM2               TIM_OC3M_MODE7
#define TIM_OC3PE                   BIT3                            
#define TIM_OC3FE                   BIT2                            
#define TIM_CC3S_CLEAR              (BIT1|BIT0)                    
#define TIM_CC3S_MODE0              0                                 
#define TIM_CC3S_MODE1              BIT0                             
#define TIM_CC3S_MODE2              BIT1                             
#define TIM_CC3S_MODE3              (BIT1|BIT0)                     
// This register has dual purpose bits, the following are the alternate defs for Input Capture
#define TIM_IC4F_CLEAR              (BIT15|BIT14|BIT13|BIT12)         
#define TIM_IC4PSC_CLEAR            (BIT11|BIT10)                     
#define TIM_IC3F_CLEAR              (BIT7|BIT6|BIT5|BIT4)             
#define TIM_IC3PSC_CLEAR            (BIT3|BIT2)                       
// TIMX_CCER Capture Compare Enable Register
#define TIM_CC4P                    BIT13                          
#define TIM_CC4E                    BIT12                          
#define TIM_CC3P                    BIT9                            
#define TIM_CC3E                    BIT8                              
#define TIM_CC2P                    BIT5                              
#define TIM_CC2E                    BIT4                             
#define TIM_CC1P                    BIT1                             
#define TIM_CC1E                    BIT0                          
// TIMX_DCR DMA Control Register
#define TIM_DBL_CLEAR               (BIT12|BIT11|BIT10|BIT9|BIT8)     
#define TIM_DBL_0                   0                                
#define TIM_DBL_1                   0                                 
#define TIM_DBL_2                   BIT8                              
#define TIM_DBL_3                   BIT9                              
#define TIM_DBL_4                   (BIT9|BIT8)                       
#define TIM_DBL_5                   BIT10                           
#define TIM_DBL_6                   (BIT10|BIT8)                     
#define TIM_DBL_7                   (BIT10|BIT9)                     
#define TIM_DBL_8                   (BIT10|BIT9|BIT8)                
#define TIM_DBL_9                   BIT11                             
#define TIM_DBL_10                  (BIT11|BIT8)                      
#define TIM_DBL_11                  (BIT11|BIT9)                      
#define TIM_DBL_12                  (BIT11|BIT9|BIT8)                 
#define TIM_DBL_13                  (BIT11|BIT10)                     
#define TIM_DBL_14                  (BIT11|BIT10|BIT8)                
#define TIM_DBL_15                  (BIT11|BIT10|BIT9)                
#define TIM_DBL_16                  (BIT11|BIT10|BIT9|BIT8)           
#define TIM_DBL_17                  BIT12                             
#define TIM_DBL_18                  (BIT12|BIT8)                      
#define TIM_DBA_CLEAR               (BIT4|BIT3|BIT2|BIT1|BIT0)        
#define TIM_DBA_0                   0 // Default is TIM_CR1, all values are bytes offset from TIM_CR1
#define TIM_DBA_1                   BIT0                            
#define TIM_DBA_2                   BIT1                             
#define TIM_DBA_3                   (BIT1|BIT0)                       
#define TIM_DBA_4                   BIT2                            
#define TIM_DBA_5                   (BIT2|BIT0)                       
#define TIM_DBA_6                   (BIT2|BIT1)                       
#define TIM_DBA_7                   (BIT2|BIT1|BIT0)                  
#define TIM_DBA_8                   BIT3                             
#define TIM_DBA_9                   (BIT3|BIT0)                      
#define TIM_DBA_10                  (BIT3|BIT1)                       
#define TIM_DBA_11                  (BIT3|BIT1|BIT0)                  
#define TIM_DBA_12                  (BIT3|BIT2)                      
#define TIM_DBA_13                  (BIT3|BIT2|BIT0)                 
#define TIM_DBA_14                  (BIT3|BIT2|BIT1)                 
#define TIM_DBA_15                  (BIT3|BIT2|BIT1|BIT0)            
#define TIM_DBA_16                  BIT4                            
#define TIM_DBA_17                  (BIT4|BIT0)                       
#define TIM_DBA_18                  (BIT4|BIT1)                       
#define TIM_DBA_19                  (BIT4|BIT1|BIT0)                
#define TIM_DBA_20                  (BIT4|BIT2)                       
#define TIM_DBA_21                  (BIT4|BIT2|BIT0)                 
#define TIM_DBA_22                  (BIT4|BIT2|BIT1)                 
#define TIM_DBA_23                  (BIT4|BIT2|BIT1|BIT0)             
#define TIM_DBA_24                  (BIT4|BIT3)                      
#define TIM_DBA_25                  (BIT4|BIT3|BIT0)                 
#define TIM_DBA_26                  (BIT4|BIT3|BIT1)                 
#define TIM_DBA_27                  (BIT4|BIT3|BIT1|BIT0)            
#define TIM_DBA_28                  (BIT4|BIT3|BIT2)                 
#define TIM_DBA_29                  (BIT4|BIT3|BIT2|BIT0)             
#define TIM_DBA_30                  (BIT4|BIT3|BIT2|BIT1)             
#define TIM_DBA_31                  (BIT4|BIT3|BIT2|BIT1|BIT0)        
// Timer1 BDTR Register
#define TIM_MOE                     BIT15 // !!! Master Output Enable !!!
#define TIM_AOE                     BIT14 // !!! Automatic Update Enable !!!
#define TIM_BKP                     BIT13 // Break Polarity
#define TIM_BKE                     BIT12 // Break Enable
#define TIM_OSSR                    BIT11 // Off state selection, run
#define TIM_OSSI                    BIT10 // Off state selection, idle
#define TIM_LOCK_CLEAR              (BIT9|BIT8)
#define TIM_LOCK_OFF                0

typedef struct {
                vu32 POWER;
                vu32 CLKCR;
                vu32 ARG;
                vu32 CMD;
                vuc32 RESPCMD;
                vuc32 RESP1;
                vuc32 RESP2;
                vuc32 RESP3;
                vuc32 RESP4;
                vu32 DTIMER;
                vu32 DLEN;
                vu32 DCTRL;
                vuc32 DCOUNT;
                vuc32 STA;
                vu32 ICR;
                vu32 MASK;
                u32  RESERVED0[2];
                vuc32 FIFOCNT;
                u32  RESERVED1[13];
                vu32 FIFO;
                } SDIO_TYPEDEF;

typedef struct {
                volatile u16 CR1;
                volatile u16 RESERVED0;
                volatile u16 CR2;
                volatile u16 RESERVED1;
                volatile u16 SMCR;
                volatile u16 RESERVED2;
                volatile u16 DIER;
                volatile u16 RESERVED3;
                volatile u16 SR;
                volatile u16 RESERVED4;
                volatile u16 EGR;
                volatile u16 RESERVED5;
                volatile u16 CCMR1;
                volatile u16 RESERVED6;
                volatile u16 CCMR2;
                volatile u16 RESERVED7;
                volatile u16 CCER;
                volatile u16 RESERVED8;
                volatile u16 CNT;
                volatile u16 RESERVED9;
                volatile u16 PSC;
                volatile u16 RESERVED10;
                volatile u16 ARR;
                volatile u16 RESERVED11;
                volatile u16 RCR;
                volatile u16 RESERVED12;
                volatile u16 CCR1;
                volatile u16 RESERVED13;
                volatile u16 CCR2;
                volatile u16 RESERVED14;
                volatile u16 CCR3;
                volatile u16 RESERVED15;
                volatile u16 CCR4;
                volatile u16 RESERVED16;
                volatile u16 BDTR;
                volatile u16 RESERVED17;
                volatile u16 DCR;
                volatile u16 RESERVED18;
                volatile vu16 DMAR;
                volatile u16 RESERVED19;
                } TIM1_TYPEDEF;

typedef struct {
                volatile u16 CR1;
                volatile u16 RESERVED0;
                volatile u16 CR2;
                volatile u16 RESERVED1;
                volatile u16 SMCR;
                volatile u16 RESERVED2;
                volatile u16 DIER;
                volatile u16 RESERVED3;
                volatile u16 SR;
                volatile u16 RESERVED4;
                volatile u16 EGR;
                volatile u16 RESERVED5;
                volatile u16 CCMR1;
                volatile u16 RESERVED6;
                volatile u16 CCMR2;
                volatile u16 RESERVED7;
                volatile u16 CCER;
                volatile u16 RESERVED8;
                volatile u16 CNT;
                volatile u16 RESERVED9;
                volatile u16 PSC;
                volatile u16 RESERVED10;
                volatile u16 ARR;
                volatile u16 RESERVED11[3];
                volatile u16 CCR1;
                volatile u16 RESERVED12;
                volatile u16 CCR2;
                volatile u16 RESERVED13;
                volatile u16 CCR3;
                volatile u16 RESERVED14;
                volatile u16 CCR4;
                volatile u16 RESERVED15[3];
                volatile u16 DCR;
                volatile u16 RESERVED16;
                volatile u16 DMAR;
                volatile u16 RESERVED17;
                } TIM_TYPEDEF;

#define TIMER   TIM_TYPEDEF
#define TIMER1  TIM1_TYPEDEF

#define TIM1_BASE                   (APB2PERIPH_BASE + 0x2C00)
#define TIM2_BASE                   (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE                   (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE                   (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE                   (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE                   (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE                   (APB1PERIPH_BASE + 0x1400)
#define TIM8_BASE                   (APB2PERIPH_BASE + 0x3400)
#define TIM9_BASE                   (APB2PERIPH_BASE + 0x4C00)
#define TIM10_BASE                  (APB2PERIPH_BASE + 0x5000)
#define TIM11_BASE                  (APB2PERIPH_BASE + 0x5400)
#define TIM12_BASE                  (APB1PERIPH_BASE + 0x1800)
#define TIM13_BASE                  (APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASE                  (APB1PERIPH_BASE + 0x2000)
#define TIM15_BASE                  (APB2PERIPH_BASE + 0x4000)
#define TIM16_BASE                  (APB2PERIPH_BASE + 0x4400)
#define TIM17_BASE                  (APB2PERIPH_BASE + 0x4800)

#define TIM1                        ((TIM1_TYPEDEF *) TIM1_BASE)
#define TIM2                        ((TIM_TYPEDEF *) TIM2_BASE)
#define TIM3                        ((TIM_TYPEDEF *) TIM3_BASE)
#define TIM4                        ((TIM_TYPEDEF *) TIM4_BASE)
#define TIM5                        ((TIM_TYPEDEF *) TIM5_BASE)
#define TIM6                        ((TIM_TYPEDEF *) TIM6_BASE)
#define TIM7                        ((TIM_TYPEDEF *) TIM7_BASE)
#define TIM8                        ((TIM_TYPEDEF *) TIM8_BASE)
#define TIM9                        ((TIM_TYPEDEF *) TIM9_BASE)
#define TIM10                       ((TIM_TYPEDEF *) TIM10_BASE)
#define TIM11                       ((TIM_TYPEDEF *) TIM11_BASE)
#define TIM12                       ((TIM_TYPEDEF *) TIM12_BASE)
#define TIM13                       ((TIM_TYPEDEF *) TIM13_BASE)
#define TIM14                       ((TIM_TYPEDEF *) TIM14_BASE)
#define TIM15                       ((TIM_TYPEDEF *) TIM15_BASE)
#define TIM16                       ((TIM_TYPEDEF *) TIM16_BASE)
#define TIM17                       ((TIM_TYPEDEF *) TIM17_BASE)

// Warning! there is no Timer0, it is included as a place holder
enum {  PYGMY_TIMER0,   PYGMY_TIMER1,   PYGMY_TIMER2,   PYGMY_TIMER3,   PYGMY_TIMER4,   PYGMY_TIMER5, 
        PYGMY_TIMER6,   PYGMY_TIMER7,   PYGMY_TIMER8,   PYGMY_TIMER9,   PYGMY_TIMER10,  PYGMY_TIMER11, 
        PYGMY_TIMER12,  PYGMY_TIMER13,  PYGMY_TIMER14,  PYGMY_TIMER15,  PYGMY_TIMER16,  PYGMY_TIMER17
        };

/**************************************************************************
                               Interrupt
***************************************************************************/
typedef struct
{
  volatile u32 ISER[2];
  u32 RESERVED0[30];
  volatile u32 ICER[2];
  u32 RESERVED1[30];
  volatile u32 ISPR[2];
  u32 RESERVED2[30];
  volatile u32 ICPR[2];
  u32 RESERVED3[30];
  volatile u32 IABR[2];
  u32 RESERVED4[62];
  volatile u32 IPR[11];
} NVIC_TYPEDEF;

typedef struct
{
  volatile u32 EVCR;
  volatile u32 MAPR;
  volatile u32 EXTICR[4];
} AFIO_TYPEDEF;

typedef struct
{
  volatile u32 IMR;
  volatile u32 EMR;
  volatile u32 RTSR;
  volatile u32 FTSR;
  volatile u32 SWIER;
  volatile u32 PR;
} EXTI_TYPEDEF;

typedef struct
{
  volatile u32 CTRL;
  volatile u32 LOAD;
  volatile u32 VAL;
  volatile const u32 CALIB;
} SYSTICK_TYPEDEF;

typedef struct
{
  volatile const u32 CPUID;
  volatile u32 ICSR;
  volatile u32 VTOR;
  volatile u32 AIRCR;
  volatile u32 SCR;
  volatile u32 CCR;
  volatile u32 SHPR[3];
  volatile u32 SHCSR;
  volatile u32 CFSR;
  volatile u32 HFSR;
  volatile u32 DFSR;
  volatile u32 MMFAR;
  volatile u32 BFAR;
  volatile u32 AFSR;
} SCB_TYPEDEF;

//#define SRAM_BASE             ((u32)0x20000000)
//#define PERIPH_BASE           ((u32)0x40000000)
//#define PERIPH_BB_BASE        ((u32)0x42000000) // Alias
//#define SRAM_BB_BASE          ((u32)0x22000000) // Alias
//#define FLASH_BASE            ((u32)0x40022000) // Flash Registers Starting Address
//#define OB_BASE               ((u32)0x1FFFF800) // Flash Option Registers Starting Address
//#define APB1PERIPH_BASE       PERIPH_BASE
//#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
//#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)
//#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000)
//#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)

#ifndef AFIO_BASE
    #define AFIO_BASE             ( ( (u32)0x40000000 ) + 0x10000 )
#endif
#ifndef EXTI_BASE
    #define EXTI_BASE             ( ( (u32)0x40000000 ) + 0x10400 )
#endif

#define AIRCR_VECTKEY_MASK          ((u32)0x05FA0000)

// Naming retains compatibility with ST Reference Material
// A notable exception being the L15X Low Power series which have a selection
// of specially allocated vectors marked L15X_IRQ instead of IRQ
// All other Vectors are compatible through F10X, F2 and F4 series
// Note that not all vectors are used in all MCUs, refer to Reference manual
// for specific part number for more information.
#define WWDG_IRQ                    ((u8)0x00)  // ( 0 ) Window WatchDog 
#define PVD_IRQ                     ((u8)0x01)  // ( 1 )  PVD through EXTI Line detection
#define TAMPER_IRQ                  ((u8)0x02)  // ( 2 )  Tamper Pin
#define RTC_IRQ                     ((u8)0x03)  // ( 3 )  RTC wakeup through EXTI
#define FLASH_IRQ                   ((u8)0x04)  // ( 4 )  FLASH General
#define RCC_IRQ                     ((u8)0x05)  // ( 5 )  RCC General
#define EXTI0_IRQ                   ((u8)0x06)  // ( 6 )  EXTI Line0  
#define EXTI1_IRQ                   ((u8)0x07)  // ( 7 )  EXTI Line1 
#define EXTI2_IRQ                   ((u8)0x08)  // ( 8 )  EXTI Line2  
#define EXTI3_IRQ                   ((u8)0x09)  // ( 9 )  EXTI Line3
#define EXTI4_IRQ                   ((u8)0x0A)  // ( 10 ) EXTI Line4 
#define DMA1_CH1_IRQ                ((u8)0x0B)  // ( 11 ) DMA1 Channel1 
#define DMA1_CH2_IRQ                ((u8)0x0C)  // ( 12 ) DMA1 Channel2 
#define DMA1_CH3_IRQ                ((u8)0x0D)  // ( 13 ) DMA1 Channel3 
#define DMA1_CH4_IRQ                ((u8)0x0E)  // ( 14 ) DMA1 Channel4 
#define DMA1_CH5_IRQ                ((u8)0x0F)  // ( 15 ) DMA1 Channel5 
#define DMA1_CH6_IRQ                ((u8)0x10)  // ( 16 ) DMA1 Channel6 
#define DMA1_CH7_IRQ                ((u8)0x11)  // ( 17 ) DMA1 Channel7 
// Some of the Vectors following have aliases to account for varying allocations across MCUs 
#define ADC_IRQ                     ((u8)0x12)  // ( 18 ) ADC1 && ( ADC2, ADC3 )
#define USB_HP_CAN_TX_IRQ           ((u8)0x13)  // ( 19 ) USB High Priority or CAN TX 
    #define USB_HP_IRQ              ((u8)0x13)  // ( 19 ) USB High Priority
    #define CAN_TX_IRQ              ((u8)0x13)  // ( 19 ) CAN TX
#define USB_LP_CAN_RX0_IRQ          ((u8)0x14)  // ( 20 ) USB Low Priority or CAN RX0 
    #define USB_LP_IRQ              ((u8)0x14)  // ( 20 ) USB Low Priority
    #define CAN_RX0_IRQ             ((u8)0x14)  // ( 20 ) CAN RX0
#define CAN_RX1_IRQ                 ((u8)0x15)  // ( 21 ) CAN RX1
    #define DAC_L15X_IRQ            ((u8)0x15)  // ( 21 ) DAC Underrun ( STM32L15X )
#define CAN_SCE_IRQ                 ((u8)0x16)  // ( 22 ) CAN SCE 
    #define COMP_L15X_IRQ           ((u8)0x16)  // ( 23 ) Comparator EXTI ( STM32L15X )
#define EXTI9_5_IRQ                 ((u8)0x17)  // ( 23 ) External Line[9:5]
#define TIM1_BRK_IRQ                ((u8)0x18)  // ( 24 ) Timer1 Break 
    #define TIM9_IRQ                ((u8)0x18)  // ( 24 ) Timer9 General  ( F103XL )
    #define TIM15_IRQ               ((u8)0x18)  // ( 24 ) Timer15 General ( F100 )
    #define LCD_L15X_IRQ            ((u8)0x18)  // ( 24 ) LCD ( STM32L15X )
#define TIM1_UP_IRQ                 ((u8)0x19)  // ( 25 ) Timer1 Update
    #define TIM10_IRQ               ((u8)0x19)  // ( 25 ) Timer10 General ( F103XL )
    #define TIM16_IRQ               ((u8)0x19)  // ( 25 ) Timer16 General ( F100 )
    #define TIM9_L15X_IRQ           ((u8)0x19)  // ( 25 ) Timer9 General ( STM32L15X )
#define TIM1_TRG_COM_IRQ            ((u8)0x1A)  // ( 26 ) Timer1 Trigger and Commutation
    #define TIM11_IRQ               ((u8)0x1A)  // ( 26 ) Timer11 Global ( F103XL )
    #define TIM10_L15X_IRQ          ((u8)0x1A)  // ( 26 ) Timer10 General ( STM32L15X )
#define TIM1_CC_IRQ                 ((u8)0x1B)  // ( 27 ) Timer1 Capture Compare
    #define TIM17_IRQ               ((u8)0x1B)  // ( 27 ) Timer17 General ( F100 )
    #define TIM11_L15X_IRQ          ((u8)0x1B)  // ( 27 ) Timer11 General ( STM32L15X )
#define TIM2_IRQ                    ((u8)0x1C)  // ( 28 ) Timer2 General
#define TIM3_IRQ                    ((u8)0x1D)  // ( 29 ) Timer3 General
#define TIM4_IRQ                    ((u8)0x1E)  // ( 30 ) Timer4 General
#define I2C1_EV_IRQ                 ((u8)0x1F)  // ( 31 ) I2C1 Event
#define I2C1_ER_IRQ                 ((u8)0x20)  // ( 32 ) I2C1 Error
#define I2C2_EV_IRQ                 ((u8)0x21)  // ( 33 ) I2C2 Event
#define I2C2_ER_IRQ                 ((u8)0x22)  // ( 34 ) I2C2 Error
#define SPI1_IRQ                    ((u8)0x23)  // ( 35 ) SPI1 General
#define SPI2_IRQ                    ((u8)0x24)  // ( 36 ) SPI2 General
#define USART1_IRQ                  ((u8)0x25)  // ( 37 ) USART1 General
#define USART2_IRQ                  ((u8)0x26)  // ( 38 ) USART2 General
#define USART3_IRQ                  ((u8)0x27)  // ( 39 ) USART3 General
#define EXTI15_10_IRQ               ((u8)0x28)  // ( 40 ) External Line[15:10]
#define RTC_ALARM_IRQ               ((u8)0x29)  // ( 41 ) RTC Alarm through EXTI Line
#define USB_WAKEUP_IRQ              ((u8)0x2A)  // ( 42 ) USB WakeUp from suspend through EXTI Line
    #define CEC_IRQ                 ((u8)0x2A)  // ( 42 ) HDMI CEC
    #define OTG_FS_WKUP_IRQ         ((u8)0x2A)  // ( 42 ) USB OTG FS EXTI Wakeup
    #define USB_FS_WKUP_L15X_IRQ    ((u8)0x2A)  // ( 42 ) USB FS WakeUp ( STM32L15X )
#define TIM8_BRK_IRQ                ((u8)0x2B)  // ( 43 ) Timer8 Break
    #define TIM12_IRQ               ((u8)0x2B)  // ( 43 ) Timer12 General
    #define TIM6_L15X_IRQ           ((u8)0x2B)  // ( 43 ) Timer6 General ( STM32L15X )
#define TIM8_UP_IRQ                 ((u8)0x2C)  // ( 44 ) Timer8 Up
    #define TIM13_IRQ               ((u8)0x2C)  // ( 44 ) Timer13 General
    #define TIM7_L15X_IRQ           ((u8)0x2C)  // ( 44 ) Timer7 General ( STM32L15X )
#define TIM8_TRG_COM_IRQ            ((u8)0x2D)  // ( 45 ) Timer8 Trigger
    #define TIM14_IRQ               ((u8)0x2D)  // ( 45 ) Timer14 General
#define TIM8_CC_IRQ                 ((u8)0x2E)  // ( 46 ) Timer8 Capture Compare
#define ADC3_IRQ                    ((u8)0x2F)  // ( 47 ) ADC3 General
    #define DMA1_CH8_IRQ            ((u8)0x2F)  // ( 47 ) DMA1 Channel8 
#define FSMC_IRQ                    ((u8)0x30)  // ( 48 ) FSMC General
#define SDIO_IRQ                    ((u8)0x31)  // ( 49 ) SDIO General
#define TIM5_IRQ                    ((u8)0x32)  // ( 50 ) Timer5 General
#define SPI3_IRQ                    ((u8)0x33)  // ( 51 ) SPI3 General
#define UART4_IRQ                   ((u8)0x34)  // ( 52 ) UART4 General
#define UART5_IRQ                   ((u8)0x35)  // ( 53 ) UART5 General
#define TIM6_IRQ                    ((u8)0x36)  // ( 54 ) Timer6 General
    #define DAC_IRQ                 ((u8)0x36)  // ( 54 ) DAC Underrun
#define TIM7_IRQ                    ((u8)0x37)  // ( 55 ) Timer7 General
#define DMA2_CH1_IRQ                ((u8)0x38)  // ( 56 ) DMA2 Channel1
#define DMA2_CH2_IRQ                ((u8)0x39)  // ( 57 ) DMA2 Channel2
#define DMA2_CH3_IRQ                ((u8)0x3A)  // ( 58 ) DMA2 Channel3 
#define DMA2_CH4_5_IRQ              ((u8)0x3B)  // ( 59 ) DMA2 Channel4 and Channel5
#define DMA2_CH5_IRQ                ((u8)0x3C)  // ( 60 ) DMA2 Channel5 ( if MISC_REMAP set in AFIO_MAPR2 )
#define ETH_IRQ                     ((u8)0x3D)  // ( 61 ) Ethernet General
#define ETH_WKUP_IRQ                ((u8)0x3E)  // ( 62 ) Ethernet Wakeup
#define CAN2_TX                     ((u8)0x3F)  // ( 63 ) CAN2 TX
#define CAN2_RX0_IRQ                ((u8)0x40)  // ( 64 ) CAN2 RX0
#define CAN2_RX1_IRQ                ((u8)0x41)  // ( 65 ) CAN2 RX1
#define CAN2_SCE_IRQ                ((u8)0x42)  // ( 66 ) CAN2 SCE
#define OTG_FS_IRQ                  ((u8)0x43)  // ( 67 ) USB OTG FS 
#define DMA2_CH6_IRQ                ((u8)0x44)  // ( 68 ) DMA2 Channel6 
#define DMA2_CH7_IRQ                ((u8)0x45)  // ( 69 ) DMA2 Channel7
#define DMA2_CH8_IRQ                ((u8)0x46)  // ( 70 ) DMA2 Channel8
#define USART6_IRQ                  ((u8)0x47)  // ( 71 ) USART6 General
#define I2C3_EV_IRQ                 ((u8)0x48)  // ( 72 ) I2C3 Event Interrupt
#define I2C3_ER_IRQ                 ((u8)0x49)  // ( 73 ) I2C3 Error Interrupt
#define OTG_HS_EP1_OUT_IRQ          ((u8)0x4A)  // ( 74 ) USB OTG HS EndPoint 1 Out
#define OTG_HS_EP1_IN_IRQ           ((u8)0x4B)  // ( 75 ) USB OTG HS EndPoint 1 In
#define OTG_HS_WKUP_IRQ             ((u8)0x4C)  // ( 76 ) USB OTG HS WakeUp
#define OTG_HS_IRQ                  ((u8)0x4D)  // ( 77 ) USB OTG HS 
#define DCMI_IRQ                    ((u8)0x4E)  // ( 78 ) DCMI ( Camera Bus )
#define CRYP_IRQ                    ((u8)0x4F)  // ( 79 ) Cryptographic Cell
#define HASH_IRQ                    ((u8)0x50)  // ( 80 ) Hash and Ring
// The following are intentionally defined out-of-range for special handling
#define MEMFAULT_IRQ                ((u8)0xF0)  // System Register Memory Fault
#define BUSFAULT_IRQ                ((u8)0xF1)  // System Register Bus Fault
#define USAGEFAULT_IRQ              ((u8)0xF2)  // System Register Usage Fault
#define SVCALL_IRQ                  ((u8)0xF3)  // System Register SV Call Interrupt
#define PENDSVCALL_IRQ              ((u8)0xF4)  // System Register Pending SV Interrupt
#define SYSTICK_IRQ                 ((u8)0xF5)  // System Register SysTick Interrupt

#define NVIC_VECTORTABLE_RAM        ((u32)0x20000000)
#define NVIC_VECTORTABLE_FLASH      ((u32)0x08000000)
#define NVIC_LP_SEVONPEND           ((u8)0x10)
#define NVIC_LP_SLEEPDEEP           ((u8)0x04)
#define NVIC_LP_SLEEPONEXIT         ((u8)0x02)
/* Deprecated
#define NVIC_PRIORITY0             ((u32)0x700) // 0 bits for pre-emption priority 4 bits for subpriority 
#define NVIC_PRIORITY1              ((u32)0x600) // 1 bits for pre-emption priority 3 bits for subpriority 
#define NVIC_PRIORITY2              ((u32)0x500) // 2 bits for pre-emption priority 2 bits for subpriority 
#define NVIC_PRIORITY3              ((u32)0x400) // 3 bits for pre-emption priority 1 bits for subpriority 
#define NVIC_PRIORITY4              ((u32)0x300) // 4 bits for pre-emption priority 0 bits for subpriority 
*/

#define EXTI_0                  ((u32)0x00001)  // External interrupt line 0 
#define EXTI_1                  ((u32)0x00002)  // External interrupt line 1 
#define EXTI_2                  ((u32)0x00004)  // External interrupt line 2 
#define EXTI_3                  ((u32)0x00008)  // External interrupt line 3 
#define EXTI_4                  ((u32)0x00010)  // External interrupt line 4 
#define EXTI_5                  ((u32)0x00020)  // External interrupt line 5 
#define EXTI_6                  ((u32)0x00040)  // External interrupt line 6 
#define EXTI_7                  ((u32)0x00080)  // External interrupt line 7 
#define EXTI_8                  ((u32)0x00100)  // External interrupt line 8 
#define EXTI_9                  ((u32)0x00200)  // External interrupt line 9 
#define EXTI_10                 ((u32)0x00400)  // External interrupt line 10 
#define EXTI_11                 ((u32)0x00800)  // External interrupt line 11 
#define EXTI_12                 ((u32)0x01000)  // External interrupt line 12 
#define EXTI_13                 ((u32)0x02000)  // External interrupt line 13 
#define EXTI_14                 ((u32)0x04000)  // External interrupt line 14 
#define EXTI_15                 ((u32)0x08000)  // External interrupt line 15 
#define EXTI_16                 ((u32)0x10000)  // External interrupt line 16
                                                    // Connected to the PVD Output
#define EXTI_17                 ((u32)0x20000)  // External interrupt line 17 
                                                    // Connected to the RTC Alarm event 
#define EXTI_18                 ((u32)0x40000)  // External interrupt line 18 
/* Deprecated                                                   
#define EXTI_GPIOA                  0x00
#define EXTI_GPIOB                  0x01
#define EXTI_GPIOC                  0x02
#define EXTI_GPIOD                  0x03
#define EXTI_GPIOE                  0x04
#define EXTI_GPIOF                  0x05
#define EXTI_GPIOG                  0x06
#define EXTI_FALLING                0x01
#define EXTI_RISING                 0x02
#define EXTI_WAKE                   0x04
*/      

#define TRIGGER_FALLING             0x01
#define TRIGGER_RISING              0x02
#define TRIGGER_WAKE                0x04

#define SCS_BASE                    ((u32)0xE000E000)         // System Control
#define SYSTICK_BASE                (SCS_BASE + 0x0010)
#define NVIC_BASE                   (SCS_BASE + 0x0100)
#define SCB_BASE                    (SCS_BASE + 0x0D00)       // System Control
#define SYSTICK                     ((SYSTICK_TYPEDEF *) SYSTICK_BASE)
#define NVIC                        ((NVIC_TYPEDEF *) NVIC_BASE)
#define SCB                         ((SCB_TYPEDEF *) SCB_BASE)  
#ifndef AFIO
    #define AFIO                ((AFIO_TYPEDEF *) AFIO_BASE)
#endif
#ifndef EXTI
    #define EXTI                ((EXTI_TYPEDEF *) EXTI_BASE)
#endif
#define PRIORITY0        0x00    // Reserved for Pygmy System
#define PRIORITY1        0x10    // Highest for normal use, 15 is lowest
#define PRIORITY2        0x20    // Always use the lowest feasible to avoid
#define PRIORITY3        0x30    // conflicts with service related interrupts
#define PRIORITY4        0x40
#define PRIORITY5        0x50
#define PRIORITY6        0x60
#define PRIORITY7        0x70
#define PRIORITY8        0x80
#define PRIORITY9        0x90
#define PRIORITY10       0xA0
#define PRIORITY11       0xB0
#define PRIORITY12       0xC0
#define PRIORITY13       0xD0
#define PRIORITY14       0xE0
#define PRIORITY15       0xF0

// The following clears pending bits to prevent recursive access
// must be used at beginning of interrupt handler
#define PYGMY_EXTICLEAR         EXTI->PR = 0x00000000; EXTI->SWIER = 0x00000000
#ifndef PYGMY_RESET
    #define PYGMY_RESET             SCB->AIRCR = (AIRCR_VECTKEY_MASK | BIT2);
#endif

/**************************************************************************
                                Watchdog
***************************************************************************/
typedef struct
{
  volatile u32 KR;
  volatile u32 PR;
  volatile u32 RLR;
  volatile u32 SR;
} IWDG_TYPEDEF;

typedef struct
{
  volatile u32 CR;
  volatile u32 CFR;
  volatile u32 SR;
} WWDG_TYPEDEF;

#ifndef WWDG_BASE
    #define WWDG_BASE           (APB1PERIPH_BASE + 0x2C00)
#endif
#ifndef IWDG_BASE
    #define IWDG_BASE           (APB1PERIPH_BASE + 0x3000)
#endif
#ifndef WWDG
    #define WWDG                ((WWDG_TYPEDEF *) WWDG_BASE)
#endif
#ifndef IWDG
    #define IWDG                ((IWDG_TYPEDEF *) IWDG_BASE)
#endif

#define IWDT_PREDIV4                            0
#define IWDT_PREDIV8                            BIT0
#define IWDT_PREDIV16                           BIT1
#define IWDT_PREDIV32                           ( BIT1 | BIT0 )
#define IWDT_PREDIV64                           BIT2
#define IWDT_PREDIV128                          ( BIT2 | BIT0 )
#define IWDT_PREDIV256                          ( BIT2 | BIT1 )
#define PYGMY_WATCHDOG_UNLOCK                   IWDG->KR = 0x5555 // register access key
#define PYGMY_WATCHDOG_PRESCALER( __IWDT_PRE )  IWDG->PR = __IWDT_PRE // WD Timer is 12 bit
#define PYGMY_WATCHDOG_TIMER( __IWDT_RELOAD )   IWDG->RLR = __IWDT_RELOAD
#define PYGMY_WATCHDOG_START                    IWDG->KR = 0xCCCC // WD start key
#define PYGMY_WATCHDOG_STOP                     IWDG->KR = 0x0000
#define PYGMY_WATCHDOG_REFRESH                  IWDG->KR = 0xAAAA // WD update key
//------------------------------------------End WatchDog---------------------------------------
//---------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------
//---------------------------------------Clock Macro Defs---------------------------------------


/**************************************************************************
                            Port and Pin
***************************************************************************/
typedef struct {
                volatile u32 CRL;
                volatile u32 CRH;
                volatile u32 IDR;
                volatile u32 ODR;
                volatile u32 BSRR;
                volatile u32 BRR;
                volatile u32 LCKR;
                } GPIO;

#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800)

#define GPIOA               ((GPIO *) GPIOA_BASE)
#define GPIOB               ((GPIO *) GPIOB_BASE)
#define GPIOC               ((GPIO *) GPIOC_BASE)
#define GPIOD               ((GPIO *) GPIOD_BASE)
#define GPIOE               ((GPIO *) GPIOE_BASE)
    
#define ALLPINS_CLEAR                       0xFFFFFFFF                // Only use before setting bits individually
#define ALLPINS_IN_ANALOG                   0x00000000
#define ALLPINS_IN_FLOAT                    0x44444444
#define ALLPINS_IN_PULL                     0x88888888
#define ALLPINS_OUT10_GPPUSHPULL            0x11111111
#define ALLPINS_OUT10_OPENDRAIN             0x55555555
#define ALLPINS_OUT10_ALTPUSHPULL           0x99999999
#define ALLPINS_OUT2_GPPUSHPULL             0x22222222
#define ALLPINS_OUT2_GPOPENDRAIN            0x66666666
#define ALLPINS_OUT2_ALTPUSHPULL            0xAAAAAAAA
#define ALLPINS_OUT2_ALTOPENDRAIN           0xEEEEEEEE
#define ALLPINS_OUT50_GPPUSHPULL            0x33333333
#define ALLPINS_OUT50_GPOPENDRAIN           0x77777777
#define ALLPINS_OUT50_ALTPUSHPULL           0xBBBBBBBB
#define ALLPINS_OUT50_ALTOPENDRAIN          0xFFFFFFFF
#define PIN0_CLEAR                          (BIT3|BIT2|BIT1|BIT0)     // Clear before setting
#define PIN0_IN_ANALOG                      0                         // Clear for Analog
#define PIN0_IN_FLOAT                       (BIT2)
#define PIN0_IN_PULL                        (BIT3)
#define PIN0_OUT10_GPPUSHPULL               (BIT0)                    // Mode bit
#define PIN0_OUT10_GPOPENDRAIN              (BIT2|BIT0)
#define PIN0_OUT10_ALTPUSHPULL              (BIT3|BIT0)
#define PIN0_OUT10_ALTOPENDRAIN             (BIT3|BIT2|BIT0)
#define PIN0_OUT2_GPPUSHPULL                (BIT1)                    // Mode bit
#define PIN0_OUT2_GPOPENDRAIN               (BIT2|BIT1)
#define PIN0_OUT2_ALTPUSHPULL               (BIT3|BIT1)
#define PIN0_OUT2_ALTOPENDRAIN              (BIT3|BIT2|BIT1)
#define PIN0_OUT50_GPPUSHPULL               (BIT1|BIT0)               // Mode Bits
#define PIN0_OUT50_GPOPENDRAIN              (BIT2|BIT1|BIT0)
#define PIN0_OUT50_ALTPUSHPULL              (BIT3|BIT1|BIT0)
#define PIN0_OUT50_ALTOPENDRAIN             (BIT3|BIT2|BIT1|BIT0)
// Pin1
#define PIN1_CLEAR                          (BIT7|BIT6|BIT5|BIT4)     // Clear before setting
#define PIN1_IN_ANALOG                      0                         // Clear for Analog
#define PIN1_IN_FLOAT                       (BIT6)
#define PIN1_IN_PULL                        (BIT7)
#define PIN1_OUT10_GPPUSHPULL               (BIT4)                    // Mode bit
#define PIN1_OUT10_GPOPENDRAIN              (BIT6|BIT4)
#define PIN1_OUT10_ALTPUSHPULL              (BIT7|BIT4)
#define PIN1_OUT10_ALTOPENDRAIN             (BIT7|BIT6|BIT4)
#define PIN1_OUT2_GPPUSHPULL                (BIT5)                    // Mode bit
#define PIN1_OUT2_GPOPENDRAIN               (BIT6|BIT5)
#define PIN1_OUT2_ALTPUSHPULL               (BIT7|BIT5)
#define PIN1_OUT2_ALTOPENDRAIN              (BIT7|BIT6|BIT5)
#define PIN1_OUT50_GPPUSHPULL               (BIT5|BIT4)               // Mode Bits
#define PIN1_OUT50_GPOPENDRAIN              (BIT6|BIT5|BIT4)
#define PIN1_OUT50_ALTPUSHPULL              (BIT7|BIT5|BIT4)
#define PIN1_OUT50_ALTOPENDRAIN             (BIT7|BIT6|BIT5|BIT4)
// Pin2                                   
#define PIN2_CLEAR                          (BIT11|BIT10|BIT9|BIT8)      // Clear before setting
#define PIN2_IN_ANALOG                      0                       // Clear for Analog
#define PIN2_IN_FLOAT                       (BIT10)
#define PIN2_IN_PULL                        (BIT11)
#define PIN2_OUT10_GPPUSHPULL               (BIT8)                    // Mode bit
#define PIN2_OUT10_GPOPENDRAIN              (BIT10|BIT8)
#define PIN2_OUT10_ALTPUSHPULL              (BIT11|BIT8)
#define PIN2_OUT10_ALTOPENDRAIN             (BIT11|BIT10|BIT8)
#define PIN2_OUT2_GPPUSHPULL                (BIT9)                    // Mode bit
#define PIN2_OUT2_GPOPENDRAIN               (BIT10|BIT9)
#define PIN2_OUT2_ALTPUSHPULL               (BIT11|BIT9)
#define PIN2_OUT2_ALTOPENDRAIN              (BIT11|BIT10|BIT9)
#define PIN2_OUT50_GPPUSHPULL               (BIT9|BIT8)               // Mode Bits
#define PIN2_OUT50_GPOPENDRAIN              (BIT10|BIT9|BIT8)
#define PIN2_OUT50_ALTPUSHPULL              (BIT11|BIT9|BIT8)
#define PIN2_OUT50_ALTOPENDRAIN             (BIT11|BIT10|BIT9|BIT8)
// Pin3                                   
#define PIN3_CLEAR                          (BIT15|BIT14|BIT13|BIT12)      // Clear before setting
#define PIN3_IN_ANALOG                      0                       // Clear for Analog
#define PIN3_IN_FLOAT                       (BIT14)
#define PIN3_IN_PULL                        (BIT15)
#define PIN3_OUT10_GPPUSHPULL               (BIT12)                    // Mode bit
#define PIN3_OUT10_GPOPENDRAIN              (BIT14|BIT12)
#define PIN3_OUT10_ALTPUSHPULL              (BIT15|BIT12)
#define PIN3_OUT10_ALTOPENDRAIN             (BIT15|BIT14|BIT12)
#define PIN3_OUT2_GPPUSHPULL                (BIT13)                    // Mode bit
#define PIN3_OUT2_GPOPENDRAIN               (BIT14|BIT13)
#define PIN3_OUT2_ALTPUSHPULL               (BIT15|BIT13)
#define PIN3_OUT2_ALTOPENDRAIN              (BIT15|BIT14|BIT13)
#define PIN3_OUT50_GPPUSHPULL               (BIT13|BIT12)               // Mode Bits
#define PIN3_OUT50_GPOPENDRAIN              (BIT14|BIT13|BIT12)
#define PIN3_OUT50_ALTPUSHPULL              (BIT15|BIT13|BIT12)
#define PIN3_OUT50_ALTOPENDRAIN             (BIT15|BIT14|BIT1|BIT12)
// Pin4                                   
#define PIN4_CLEAR                          (BIT19|BIT18|BIT17|BIT16)      // Clear before setting
#define PIN4_IN_ANALOG                      0                       // Clear for Analog
#define PIN4_IN_FLOAT                       (BIT18)
#define PIN4_IN_PULL                        (BIT19)
#define PIN4_OUT10_GPPUSHPULL               (BIT16)                    // Mode bit
#define PIN4_OUT10_GPOPENDRAIN              (BIT18|BIT16)
#define PIN4_OUT10_ALTPUSHPULL              (BIT19|BIT16)
#define PIN4_OUT10_ALTOPENDRAIN             (BIT19|BIT18|BIT16)
#define PIN4_OUT2_GPPUSHPULL                (BIT17)                    // Mode bit
#define PIN4_OUT2_GPOPENDRAIN               (BIT18|BIT17)
#define PIN4_OUT2_ALTPUSHPULL               (BIT19|BIT17)
#define PIN4_OUT2_ALTOPENDRAIN              (BIT19|BIT18|BIT17)
#define PIN4_OUT50_GPPUSHPULL               (BIT17|BIT16)               // Mode Bits
#define PIN4_OUT50_GPOPENDRAIN              (BIT18|BIT17|BIT16)
#define PIN4_OUT50_ALTPUSHPULL              (BIT19|BIT17|BIT16)
#define PIN4_OUT50_ALTOPENDRAIN             (BIT19|BIT18|BIT17|BIT16)
// Pin5                                  
#define PIN5_CLEAR                          (BIT23|BIT22|BIT21|BIT20)      // Clear before setting
#define PIN5_IN_ANALOG                      0                       // Clear for Analog
#define PIN5_IN_FLOAT                       (BIT22)
#define PIN5_IN_PULL                        (BIT23)
#define PIN5_OUT10_GPPUSHPULL               (BIT20)                    // Mode bit
#define PIN5_OUT10_GPOPENDRAIN              (BIT22|BIT20)
#define PIN5_OUT10_ALTPUSHPULL              (BIT23|BIT20)
#define PIN5_OUT10_ALTOPENDRAIN             (BIT23|BIT2|BIT20)
#define PIN5_OUT2_GPPUSHPULL                (BIT21)                    // Mode bit
#define PIN5_OUT2_GPOPENDRAIN               (BIT22|BIT21)
#define PIN5_OUT2_ALTPUSHPULL               (BIT23|BIT21)
#define PIN5_OUT2_ALTOPENDRAIN              (BIT23|BIT22|BIT1)
#define PIN5_OUT50_GPPUSHPULL               (BIT21|BIT20)               // Mode Bits
#define PIN5_OUT50_GPOPENDRAIN              (BIT22|BIT21|BIT20)
#define PIN5_OUT50_ALTPUSHPULL              (BIT23|BIT21|BIT20)
#define PIN5_OUT50_ALTOPENDRAIN             (BIT23|BIT22|BIT21|BIT20)
// Pin6                                   
#define PIN6_CLEAR                          (BIT27|BIT26|BIT25|BIT24)      // Clear before setting
#define PIN6_IN_ANALOG                      0                       // Clear for Analog
#define PIN6_IN_FLOAT                       (BIT26)
#define PIN6_IN_PULL                        (BIT27)
#define PIN6_OUT10_GPPUSHPULL               (BIT24)                    // Mode bit
#define PIN6_OUT10_GPOPENDRAIN              (BIT26|BIT24)
#define PIN6_OUT10_ALTPUSHPULL              (BIT27|BIT24)
#define PIN6_OUT10_ALTOPENDRAIN             (BIT27|BIT26|BIT24)
#define PIN6_OUT2_GPPUSHPULL                (BIT25)                    // Mode bit
#define PIN6_OUT2_GPOPENDRAIN               (BIT26|BIT25)
#define PIN6_OUT2_ALTPUSHPULL               (BIT27|BIT25)
#define PIN6_OUT2_ALTOPENDRAIN              (BIT27|BIT26|BIT25)
#define PIN6_OUT50_GPPUSHPULL               (BIT25|BIT24)               // Mode Bits
#define PIN6_OUT50_GPOPENDRAIN              (BIT26|BIT25|BIT24)
#define PIN6_OUT50_ALTPUSHPULL              (BIT27|BIT25|BIT24)
#define PIN6_OUT50_ALTOPENDRAIN             (BIT27|BIT26|BIT25|BIT24)
// Pin7                                   
#define PIN7_CLEAR                          (BIT31|BIT30|BIT29|BIT28)      // Clear before setting
#define PIN7_IN_ANALOG                      0                       // Clear for Analog
#define PIN7_IN_FLOAT                       (BIT30)
#define PIN7_IN_PULL                        (BIT31)
#define PIN7_OUT10_GPPUSHPULL               (BIT28)                    // Mode bit
#define PIN7_OUT10_GPOPENDRAIN              (BIT30|BIT28)
#define PIN7_OUT10_ALTPUSHPULL              (BIT31|BIT28)
#define PIN7_OUT10_ALTOPENDRAIN             (BIT31|BIT30|BIT28)
#define PIN7_OUT2_GPPUSHPULL                (BIT29)                    // Mode bit
#define PIN7_OUT2_GPOPENDRAIN               (BIT30|BIT29)
#define PIN7_OUT2_ALTPUSHPULL               (BIT31|BIT29)
#define PIN7_OUT2_ALTOPENDRAIN              (BIT31|BIT30|BIT29)
#define PIN7_OUT50_GPPUSHPULL               (BIT29|BIT28)               // Mode Bits
#define PIN7_OUT50_GPOPENDRAIN              (BIT30|BIT29|BIT28)
#define PIN7_OUT50_ALTPUSHPULL              (BIT31|BIT29|BIT28)
#define PIN7_OUT50_ALTOPENDRAIN             (BIT31|BIT30|BIT29|BIT28)
// Pin8
#define PIN8_CLEAR                          PIN0_CLEAR
#define PIN8_IN_ANALOG                      PIN0_IN_ANALOG
#define PIN8_IN_FLOAT                       PIN0_IN_FLOAT
#define PIN8_IN_PULL                        PIN0_IN_PULL
#define PIN8_OUT10_GPPUSHPULL               PIN0_OUT10_GPPUSHPULL
#define PIN8_OUT10_GPOPENDRAIN              PIN0_OUT10_GPOPENDRAIN
#define PIN8_OUT10_ALTPUSHPULL              PIN0_OUT10_ALTPUSHPULL
#define PIN8_OUT10_ALTOPENDRAIN             PIN0_OUT10_ALTOPENDRAIN
#define PIN8_OUT2_GPPUSHPULL                PIN0_OUT2_GPPUSHPULL
#define PIN8_OUT2_GPOPENDRAIN               PIN0_OUT2_GPOPENDRAIN
#define PIN8_OUT2_ALTPUSHPULL               PIN0_OUT2_ALTPUSHPULL
#define PIN8_OUT2_ALTOPENDRAIN              PIN0_OUT2_ALTOPENDRAIN
#define PIN8_OUT50_GPPUSHPULL               PIN0_OUT50_GPPUSHPULL
#define PIN8_OUT50_GPOPENDRAIN              PIN0_OUT50_GPOPENDRAIN
#define PIN8_OUT50_ALTPUSHPULL              PIN0_OUT50_ALTPUSHPULL
#define PIN8_OUT50_ALTOPENDRAIN             PIN0_OUT50_ALTOPENDRAIN
// Pin9
#define PIN9_CLEAR                          PIN1_CLEAR
#define PIN9_IN_ANALOG                      PIN1_IN_ANALOG
#define PIN9_IN_FLOAT                       PIN1_IN_FLOAT
#define PIN9_IN_PULL                        PIN1_IN_PULL
#define PIN9_OUT10_GPPUSHPULL               PIN1_OUT10_GPPUSHPULL
#define PIN9_OUT10_GPOPENDRAIN              PIN1_OUT10_GPOPENDRAIN
#define PIN9_OUT10_ALTPUSHPULL              PIN1_OUT10_ALTPUSHPULL
#define PIN9_OUT10_ALTOPENDRAIN             PIN1_OUT10_ALTOPENDRAIN
#define PIN9_OUT2_GPPUSHPULL                PIN1_OUT2_GPPUSHPULL
#define PIN9_OUT2_GPOPENDRAIN               PIN1_OUT2_GPOPENDRAIN
#define PIN9_OUT2_ALTPUSHPULL               PIN1_OUT2_ALTPUSHPULL
#define PIN9_OUT2_ALTOPENDRAIN              PIN1_OUT2_ALTOPENDRAIN
#define PIN9_OUT50_GPPUSHPULL               PIN1_OUT50_GPPUSHPULL
#define PIN9_OUT50_GPOPENDRAIN              PIN1_OUT50_GPOPENDRAIN
#define PIN9_OUT50_ALTPUSHPULL              PIN1_OUT50_ALTPUSHPULL
#define PIN9_OUT50_ALTOPENDRAIN             PIN1_OUT50_ALTOPENDRAIN
// Pin10
#define PIN10_CLEAR                         PIN2_CLEAR
#define PIN10_IN_ANALOG                     PIN2_IN_ANALOG
#define PIN10_IN_FLOAT                      PIN2_IN_FLOAT
#define PIN10_IN_PULL                       PIN2_IN_PULL
#define PIN10_OUT10_GPPUSHPULL              PIN2_OUT10_GPPUSHPULL
#define PIN10_OUT10_GPOPENDRAIN             PIN2_OUT10_GPOPENDRAIN
#define PIN10_OUT10_ALTPUSHPULL             PIN2_OUT10_ALTPUSHPULL
#define PIN10_OUT10_ALTOPENDRAIN            PIN2_OUT10_ALTOPENDRAIN
#define PIN10_OUT2_GPPUSHPULL               PIN2_OUT2_GPPUSHPULL
#define PIN10_OUT2_GPOPENDRAIN              PIN2_OUT2_GPOPENDRAIN
#define PIN10_OUT2_ALTPUSHPULL              PIN2_OUT2_ALTPUSHPULL
#define PIN10_OUT2_ALTOPENDRAIN             PIN2_OUT2_ALTOPENDRAIN
#define PIN10_OUT50_GPPUSHPULL              PIN2_OUT50_GPPUSHPULL
#define PIN10_OUT50_GPOPENDRAIN             PIN2_OUT50_GPOPENDRAIN
#define PIN10_OUT50_ALTPUSHPULL             PIN2_OUT50_ALTPUSHPULL
#define PIN10_OUT50_ALTOPENDRAIN            PIN2_OUT50_ALTOPENDRAIN
// Pin11
#define PIN11_CLEAR                         PIN3_CLEAR
#define PIN11_IN_ANALOG                     PIN3_IN_ANALOG
#define PIN11_IN_FLOAT                      PIN3_IN_FLOAT
#define PIN11_IN_PULL                       PIN3_IN_PULL
#define PIN11_OUT10_GPPUSHPULL              PIN3_OUT10_GPPUSHPULL
#define PIN11_OUT10_GPOPENDRAIN             PIN3_OUT10_GPOPENDRAIN
#define PIN11_OUT10_ALTPUSHPULL             PIN3_OUT10_ALTPUSHPULL
#define PIN11_OUT10_ALTOPENDRAIN            PIN3_OUT10_ALTOPENDRAIN
#define PIN11_OUT2_GPPUSHPULL               PIN3_OUT2_GPPUSHPULL
#define PIN11_OUT2_GPOPENDRAIN              PIN3_OUT2_GPOPENDRAIN
#define PIN11_OUT2_ALTPUSHPULL              PIN3_OUT2_ALTPUSHPULL
#define PIN11_OUT2_ALTOPENDRAIN             PIN3_OUT2_ALTOPENDRAIN
#define PIN11_OUT50_GPPUSHPULL              PIN3_OUT50_GPPUSHPULL
#define PIN11_OUT50_GPOPENDRAIN             PIN3_OUT50_GPOPENDRAIN
#define PIN11_OUT50_ALTPUSHPULL             PIN3_OUT50_ALTPUSHPULL
#define PIN11_OUT50_ALTOPENDRAIN            PIN3_OUT50_ALTOPENDRAIN
// Pin12
#define PIN12_CLEAR                         PIN4_CLEAR
#define PIN12_IN_ANALOG                     PIN4_IN_ANALOG
#define PIN12_IN_FLOAT                      PIN4_IN_FLOAT
#define PIN12_IN_PULL                       PIN4_IN_PULL
#define PIN12_OUT10_GPPUSHPULL              PIN4_OUT10_GPPUSHPULL
#define PIN12_OUT10_GPOPENDRAIN             PIN4_OUT10_GPOPENDRAIN
#define PIN12_OUT10_ALTPUSHPULL             PIN4_OUT10_ALTPUSHPULL
#define PIN12_OUT10_ALTOPENDRAIN            PIN4_OUT10_ALTOPENDRAIN
#define PIN12_OUT2_GPPUSHPULL               PIN4_OUT2_GPPUSHPULL
#define PIN12_OUT2_GPOPENDRAIN              PIN4_OUT2_GPOPENDRAIN
#define PIN12_OUT2_ALTPUSHPULL              PIN4_OUT2_ALTPUSHPULL
#define PIN12_OUT2_ALTOPENDRAIN             PIN4_OUT2_ALTOPENDRAIN
#define PIN12_OUT50_GPPUSHPULL              PIN4_OUT50_GPPUSHPULL
#define PIN12_OUT50_GPOPENDRAIN             PIN4_OUT50_GPOPENDRAIN
#define PIN12_OUT50_ALTPUSHPULL             PIN4_OUT50_ALTPUSHPULL
#define PIN12_OUT50_ALTOPENDRAIN            PIN4_OUT50_ALTOPENDRAIN
// Pin13
#define PIN13_CLEAR                         PIN5_CLEAR
#define PIN13_IN_ANALOG                     PIN5_IN_ANALOG
#define PIN13_IN_FLOAT                      PIN5_IN_FLOAT
#define PIN13_IN_PULL                       PIN5_IN_PULL
#define PIN13_OUT10_GPPUSHPULL              PIN5_OUT10_GPPUSHPULL
#define PIN13_OUT10_GPOPENDRAIN             PIN5_OUT10_GPOPENDRAIN
#define PIN13_OUT10_ALTPUSHPULL             PIN5_OUT10_ALTPUSHPULL
#define PIN13_OUT10_ALTOPENDRAIN            PIN5_OUT10_ALTOPENDRAIN
#define PIN13_OUT2_GPPUSHPULL               PIN5_OUT2_GPPUSHPULL
#define PIN13_OUT2_GPOPENDRAIN              PIN5_OUT2_GPOPENDRAIN
#define PIN13_OUT2_ALTPUSHPULL              PIN5_OUT2_ALTPUSHPULL
#define PIN13_OUT2_ALTOPENDRAIN             PIN5_OUT2_ALTOPENDRAIN
#define PIN13_OUT50_GPPUSHPULL              PIN5_OUT50_GPPUSHPULL
#define PIN13_OUT50_GPOPENDRAIN             PIN5_OUT50_GPOPENDRAIN
#define PIN13_OUT50_ALTPUSHPULL             PIN5_OUT50_ALTPUSHPULL
#define PIN13_OUT50_ALTOPENDRAIN            PIN5_OUT50_ALTOPENDRAIN
// Pin14
#define PIN14_CLEAR                         PIN6_CLEAR
#define PIN14_IN_ANALOG                     PIN6_IN_ANALOG
#define PIN14_IN_FLOAT                      PIN6_IN_FLOAT
#define PIN14_IN_PULL                       PIN6_IN_PULL
#define PIN14_OUT10_GPPUSHPULL              PIN6_OUT10_GPPUSHPULL
#define PIN14_OUT10_GPOPENDRAIN             PIN6_OUT10_GPOPENDRAIN
#define PIN14_OUT10_ALTPUSHPULL             PIN6_OUT10_ALTPUSHPULL
#define PIN14_OUT10_ALTOPENDRAIN            PIN6_OUT10_ALTOPENDRAIN
#define PIN14_OUT2_GPPUSHPULL               PIN6_OUT2_GPPUSHPULL
#define PIN14_OUT2_GPOPENDRAIN              PIN6_OUT2_GPOPENDRAIN
#define PIN14_OUT2_ALTPUSHPULL              PIN6_OUT2_ALTPUSHPULL
#define PIN14_OUT2_ALTOPENDRAIN             PIN6_OUT2_ALTOPENDRAIN
#define PIN14_OUT50_GPPUSHPULL              PIN6_OUT50_GPPUSHPULL
#define PIN14_OUT50_GPOPENDRAIN             PIN6_OUT50_GPOPENDRAIN
#define PIN14_OUT50_ALTPUSHPULL             PIN6_OUT50_ALTPUSHPULL
#define PIN14_OUT50_ALTOPENDRAIN            PIN6_OUT50_ALTOPENDRAIN
// Pin15
#define PIN15_CLEAR                         PIN7_CLEAR
#define PIN15_IN_ANALOG                     PIN7_IN_ANALOG
#define PIN15_IN_FLOAT                      PIN7_IN_FLOAT
#define PIN15_IN_PULL                       PIN7_IN_PULL
#define PIN15_OUT10_GPPUSHPULL              PIN7_OUT10_GPPUSHPULL
#define PIN15_OUT10_GPOPENDRAIN             PIN7_OUT10_GPOPENDRAIN
#define PIN15_OUT10_ALTPUSHPULL             PIN7_OUT10_ALTPUSHPULL
#define PIN15_OUT10_ALTOPENDRAIN            PIN7_OUT10_ALTOPENDRAIN
#define PIN15_OUT2_GPPUSHPULL               PIN7_OUT2_GPPUSHPULL
#define PIN15_OUT2_GPOPENDRAIN              PIN7_OUT2_GPOPENDRAIN
#define PIN15_OUT2_ALTPUSHPULL              PIN7_OUT2_ALTPUSHPULL
#define PIN15_OUT2_ALTOPENDRAIN             PIN7_OUT2_ALTOPENDRAIN
#define PIN15_OUT50_GPPUSHPULL              PIN7_OUT50_GPPUSHPULL
#define PIN15_OUT50_GPOPENDRAIN             PIN7_OUT50_GPOPENDRAIN
#define PIN15_OUT50_ALTPUSHPULL             PIN7_OUT50_ALTPUSHPULL
#define PIN15_OUT50_ALTOPENDRAIN            PIN7_OUT50_ALTOPENDRAIN    
    
#define PIN_CLEAR                            (BIT3|BIT2|BIT1|BIT0)     // Clear before setting
#define PIN_IN_ANALOG                        0                         // Clear for Analog
#define PIN_IN_FLOAT                         (BIT2)
#define PIN_IN_PULL                          (BIT3)
#define PIN_OUT10_GPPUSHPULL                 (BIT0)                    // Mode bit
#define PIN_OUT10_GPOPENDRAIN                (BIT2|BIT0)
#define PIN_OUT10_ALTPUSHPULL                (BIT3|BIT0)
#define PIN_OUT10_ALTOPENDRAIN               (BIT3|BIT2|BIT0)
#define PIN_OUT2_GPPUSHPULL                  (BIT1)                    // Mode bit
#define PIN_OUT2_GPOPENDRAIN                 (BIT2|BIT1)
#define PIN_OUT2_ALTPUSHPULL                 (BIT3|BIT1)
#define PIN_OUT2_ALTOPENDRAIN                (BIT3|BIT2|BIT1)
#define PIN_OUT50_GPPUSHPULL                 (BIT1|BIT0)               // Mode Bits
#define PIN_OUT50_GPOPENDRAIN                (BIT2|BIT1|BIT0)
#define PIN_OUT50_ALTPUSHPULL                (BIT3|BIT1|BIT0)
#define PIN_OUT50_ALTOPENDRAIN               (BIT3|BIT2|BIT1|BIT0)

// GPIOA Base Pin Macros
#define PA0_ALT             GPIOA->CRL &= ~PIN0_CLEAR; GPIOA->CRL |= PIN0_OUT50_ALTPUSHPULL;
#define PA0_OUT             GPIOA->CRL &= ~PIN0_CLEAR; GPIOA->CRL |= PIN0_OUT50_GPPUSHPULL;
#define PA0_FLOAT           GPIOA->CRL &= ~PIN0_CLEAR; GPIOA->CRL |= PIN0_IN_FLOAT;
#define PA0_PULLUP          GPIOA->CRL &= ~PIN0_CLEAR; GPIOA->CRL |= PIN0_IN_PULL; GPIOA->BSRR = BIT0;
#define PA0_PULLDOWN        GPIOA->CRL &= ~PIN0_CLEAR; GPIOA->CRL |= PIN0_IN_PULL; GPIOA->BRR = BIT0;
#define PA0_ANALOG          GPIOA->CRL &= ~PIN0_CLEAR; GPIOA->CRL |= PIN0_IN_ANALOG;
#define PA0_GET             ( GPIOA->IDR & BIT0 )
#define PA0_STATE           PA0_GET
#define PA0_HIGH            GPIOA->BSRR = BIT0;
#define PA0_LOW             GPIOA->BRR = BIT0;

#define PA1_ALT             GPIOA->CRL &= ~PIN1_CLEAR; GPIOA->CRL |= PIN1_OUT50_ALTPUSHPULL;
#define PA1_OUT             GPIOA->CRL &= ~PIN1_CLEAR; GPIOA->CRL |= PIN1_OUT50_GPPUSHPULL;
#define PA1_FLOAT           GPIOA->CRL &= ~PIN1_CLEAR; GPIOA->CRL |= PIN1_IN_FLOAT;
#define PA1_PULLUP          GPIOA->CRL &= ~PIN1_CLEAR; GPIOA->CRL |= PIN1_IN_PULL; GPIOA->BSRR = BIT1;
#define PA1_PULLDOWN        GPIOA->CRL &= ~PIN1_CLEAR; GPIOA->CRL |= PIN1_IN_PULL; GPIOA->BRR = BIT1;
#define PA1_ANALOG          GPIOA->CRL &= ~PIN1_CLEAR; GPIOA->CRL |= PIN1_IN_ANALOG;
#define PA1_GET             ( GPIOA->IDR & BIT1 )
#define PA1_STATE           PA1_GET
#define PA1_HIGH            GPIOA->BSRR = BIT1;
#define PA1_LOW             GPIOA->BRR = BIT1;

#define PA2_ALT             GPIOA->CRL &= ~PIN2_CLEAR; GPIOA->CRL |= PIN2_OUT50_ALTPUSHPULL;
#define PA2_OUT             GPIOA->CRL &= ~PIN2_CLEAR; GPIOA->CRL |= PIN2_OUT50_GPPUSHPULL;
#define PA2_FLOAT           GPIOA->CRL &= ~PIN2_CLEAR; GPIOA->CRL |= PIN2_IN_FLOAT;
#define PA2_PULLUP          GPIOA->CRL &= ~PIN2_CLEAR; GPIOA->CRL |= PIN2_IN_PULL; GPIOA->BSRR = BIT2;
#define PA2_PULLDOWN        GPIOA->CRL &= ~PIN2_CLEAR; GPIOA->CRL |= PIN2_IN_PULL; GPIOA->BRR = BIT2;
#define PA2_ANALOG          GPIOA->CRL &= ~PIN2_CLEAR; GPIOA->CRL |= PIN2_IN_ANALOG;
#define PA2_GET             ( GPIOA->IDR & BIT2 )
#define PA2_STATE           PA2_GET
#define PA2_HIGH            GPIOA->BSRR = BIT2;
#define PA2_LOW             GPIOA->BRR = BIT2;

#define PA3_ALT             GPIOA->CRL &= ~PIN3_CLEAR; GPIOA->CRL |= PIN3_OUT50_ALTPUSHPULL;
#define PA3_OUT             GPIOA->CRL &= ~PIN3_CLEAR; GPIOA->CRL |= PIN3_OUT50_GPPUSHPULL;
#define PA3_FLOAT           GPIOA->CRL &= ~PIN3_CLEAR; GPIOA->CRL |= PIN3_IN_FLOAT;
#define PA3_PULLUP          GPIOA->CRL &= ~PIN3_CLEAR; GPIOA->CRL |= PIN3_IN_PULL; GPIOA->BSRR = BIT3;
#define PA3_PULLDOWN        GPIOA->CRL &= ~PIN3_CLEAR; GPIOA->CRL |= PIN3_IN_PULL; GPIOA->BRR = BIT3;
#define PA3_ANALOG          GPIOA->CRL &= ~PIN3_CLEAR; GPIOA->CRL |= PIN3_IN_ANALOG;
#define PA3_GET             ( GPIOA->IDR & BIT3 )
#define PA3_STATE           PA3_GET
#define PA3_HIGH            GPIOA->BSRR = BIT3;
#define PA3_LOW             GPIOA->BRR = BIT3;

#define PA4_ALT             GPIOA->CRL &= ~PIN4_CLEAR; GPIOA->CRL |= PIN4_OUT50_ALTPUSHPULL;
#define PA4_OUT             GPIOA->CRL &= ~PIN4_CLEAR; GPIOA->CRL |= PIN4_OUT50_GPPUSHPULL;
#define PA4_FLOAT           GPIOA->CRL &= ~PIN4_CLEAR; GPIOA->CRL |= PIN4_IN_FLOAT;
#define PA4_PULLUP          GPIOA->CRL &= ~PIN4_CLEAR; GPIOA->CRL |= PIN4_IN_PULL; GPIOA->BSRR = BIT4;
#define PA4_PULLDOWN        GPIOA->CRL &= ~PIN4_CLEAR; GPIOA->CRL |= PIN4_IN_PULL; GPIOA->BRR = BIT4;
#define PA4_ANALOG          GPIOA->CRL &= ~PIN4_CLEAR; GPIOA->CRL |= PIN4_IN_ANALOG;
#define PA4_GET             ( GPIOA->IDR & BIT4 )
#define PA4_STATE           PA4_GET
#define PA4_HIGH            GPIOA->BSRR = BIT4;
#define PA4_LOW             GPIOA->BRR = BIT4;

#define PA5_ALT             GPIOA->CRL &= ~PIN5_CLEAR; GPIOA->CRL |= PIN5_OUT50_ALTPUSHPULL;
#define PA5_OUT             GPIOA->CRL &= ~PIN5_CLEAR; GPIOA->CRL |= PIN5_OUT50_GPPUSHPULL;
#define PA5_FLOAT           GPIOA->CRL &= ~PIN5_CLEAR; GPIOA->CRL |= PIN5_IN_FLOAT;
#define PA5_PULLUP          GPIOA->CRL &= ~PIN5_CLEAR; GPIOA->CRL |= PIN5_IN_PULL; GPIOA->BSRR = BIT5;
#define PA5_PULLDOWN        GPIOA->CRL &= ~PIN5_CLEAR; GPIOA->CRL |= PIN5_IN_PULL; GPIOA->BRR = BIT5;
#define PA5_ANALOG          GPIOA->CRL &= ~PIN5_CLEAR; GPIOA->CRL |= PIN5_IN_ANALOG;
#define PA5_GET             ( GPIOA->IDR & BIT5 )
#define PA5_STATE           PA5_GET
#define PA5_HIGH            GPIOA->BSRR = BIT5;
#define PA5_LOW             GPIOA->BRR = BIT5;

#define PA6_ALT             GPIOA->CRL &= ~PIN6_CLEAR; GPIOA->CRL |= PIN6_OUT50_ALTPUSHPULL;
#define PA6_OUT             GPIOA->CRL &= ~PIN6_CLEAR; GPIOA->CRL |= PIN6_OUT50_GPPUSHPULL;
#define PA6_FLOAT           GPIOA->CRL &= ~PIN6_CLEAR; GPIOA->CRL |= PIN6_IN_FLOAT;
#define PA6_PULLUP          GPIOA->CRL &= ~PIN6_CLEAR; GPIOA->CRL |= PIN6_IN_PULL; GPIOA->BSRR = BIT6;
#define PA6_PULLDOWN        GPIOA->CRL &= ~PIN6_CLEAR; GPIOA->CRL |= PIN6_IN_PULL; GPIOA->BRR = BIT6;
#define PA6_ANALOG          GPIOA->CRL &= ~PIN6_CLEAR; GPIOA->CRL |= PIN6_IN_ANALOG;
#define PA6_GET             ( GPIOA->IDR & BIT6 )
#define PA6_STATE           PA6_GET
#define PA6_HIGH            GPIOA->BSRR = BIT6;
#define PA6_LOW             GPIOA->BRR = BIT6;

#define PA7_ALT             GPIOA->CRL &= ~PIN7_CLEAR; GPIOA->CRL |= PIN7_OUT50_ALTPUSHPULL;
#define PA7_OUT             GPIOA->CRL &= ~PIN7_CLEAR; GPIOA->CRL |= PIN7_OUT50_GPPUSHPULL;
#define PA7_FLOAT           GPIOA->CRL &= ~PIN7_CLEAR; GPIOA->CRL |= PIN7_IN_FLOAT;
#define PA7_PULLUP          GPIOA->CRL &= ~PIN7_CLEAR; GPIOA->CRL |= PIN7_IN_PULL; GPIOA->BSRR = BIT7;
#define PA7_PULLDOWN        GPIOA->CRL &= ~PIN7_CLEAR; GPIOA->CRL |= PIN7_IN_PULL; GPIOA->BRR = BIT7;
#define PA7_ANALOG          GPIOA->CRL &= ~PIN7_CLEAR; GPIOA->CRL |= PIN7_IN_ANALOG;
#define PA7_GET             ( GPIOA->IDR & BIT7 )
#define PA7_STATE           PA7_GET
#define PA7_HIGH            GPIOA->BSRR = BIT7;
#define PA7_LOW             GPIOA->BRR = BIT7;

#define PA8_ALT             GPIOA->CRH &= ~PIN8_CLEAR; GPIOA->CRH |= PIN8_OUT50_ALTPUSHPULL;
#define PA8_OUT             GPIOA->CRH &= ~PIN8_CLEAR; GPIOA->CRH |= PIN8_OUT50_GPPUSHPULL;
#define PA8_FLOAT           GPIOA->CRH &= ~PIN8_CLEAR; GPIOA->CRH |= PIN8_IN_FLOAT;
#define PA8_PULLUP          GPIOA->CRH &= ~PIN8_CLEAR; GPIOA->CRH |= PIN8_IN_PULL; GPIOA->BSRR = BIT8;
#define PA8_PULLDOWN        GPIOA->CRH &= ~PIN8_CLEAR; GPIOA->CRH |= PIN8_IN_PULL; GPIOA->BRR = BIT8;
#define PA8_ANALOG          GPIOA->CRH &= ~PIN8_CLEAR; GPIOA->CRH |= PIN8_IN_ANALOG;
#define PA8_GET             ( GPIOA->IDR & BIT8 )
#define PA8_STATE           PA8_GET
#define PA8_HIGH            GPIOA->BSRR = BIT8;
#define PA8_LOW             GPIOA->BRR = BIT8;

#define PA9_ALT             GPIOA->CRH &= ~PIN9_CLEAR; GPIOA->CRH |= PIN9_OUT50_ALTPUSHPULL;
#define PA9_OUT             GPIOA->CRH &= ~PIN9_CLEAR; GPIOA->CRH |= PIN9_OUT50_GPPUSHPULL;
#define PA9_FLOAT           GPIOA->CRH &= ~PIN9_CLEAR; GPIOA->CRH |= PIN9_IN_FLOAT;
#define PA9_PULLUP          GPIOA->CRH &= ~PIN9_CLEAR; GPIOA->CRH |= PIN9_IN_PULL; GPIOA->BSRR = BIT9;
#define PA9_PULLDOWN        GPIOA->CRH &= ~PIN9_CLEAR; GPIOA->CRH |= PIN9_IN_PULL; GPIOA->BRR = BIT9;
#define PA9_ANALOG          GPIOA->CRH &= ~PIN9_CLEAR; GPIOA->CRH |= PIN9_IN_ANALOG;
#define PA9_GET             ( GPIOA->IDR & BIT9 )
#define PA9_STATE           PA9_GET
#define PA9_HIGH            GPIOA->BSRR = BIT9;
#define PA9_LOW             GPIOA->BRR = BIT9;

#define PA10_ALT             GPIOA->CRH &= ~PIN10_CLEAR; GPIOA->CRH |= PIN10_OUT50_ALTPUSHPULL;
#define PA10_OUT             GPIOA->CRH &= ~PIN10_CLEAR; GPIOA->CRH |= PIN10_OUT50_GPPUSHPULL;
#define PA10_FLOAT           GPIOA->CRH &= ~PIN10_CLEAR; GPIOA->CRH |= PIN10_IN_FLOAT;
#define PA10_PULLUP          GPIOA->CRH &= ~PIN10_CLEAR; GPIOA->CRH |= PIN10_IN_PULL; GPIOA->BSRR = BIT10;
#define PA10_PULLDOWN        GPIOA->CRH &= ~PIN10_CLEAR; GPIOA->CRH |= PIN10_IN_PULL; GPIOA->BRR = BIT10;
#define PA10_ANALOG          GPIOA->CRH &= ~PIN10_CLEAR; GPIOA->CRH |= PIN10_IN_ANALOG;
#define PA10_GET             ( GPIOA->IDR & BIT10 )
#define PA10_STATE           PA10_GET
#define PA10_HIGH            GPIOA->BSRR = BIT10;
#define PA10_LOW             GPIOA->BRR = BIT10;

#define PA11_ALT             GPIOA->CRH &= ~PIN11_CLEAR; GPIOA->CRH |= PIN11_OUT50_ALTPUSHPULL;
#define PA11_OUT             GPIOA->CRH &= ~PIN11_CLEAR; GPIOA->CRH |= PIN11_OUT50_GPPUSHPULL;
#define PA11_FLOAT           GPIOA->CRH &= ~PIN11_CLEAR; GPIOA->CRH |= PIN11_IN_FLOAT;
#define PA11_PULLUP          GPIOA->CRH &= ~PIN11_CLEAR; GPIOA->CRH |= PIN11_IN_PULL; GPIOA->BSRR = BIT11;
#define PA11_PULLDOWN        GPIOA->CRH &= ~PIN11_CLEAR; GPIOA->CRH |= PIN11_IN_PULL; GPIOA->BRR = BIT11;
#define PA11_ANALOG          GPIOA->CRH &= ~PIN11_CLEAR; GPIOA->CRH |= PIN11_IN_ANALOG;
#define PA11_GET             ( GPIOA->IDR & BIT11 )
#define PA11_STATE           PA11_GET
#define PA11_HIGH            GPIOA->BSRR = BIT11;
#define PA11_LOW             GPIOA->BRR = BIT11;

#define PA12_ALT             GPIOA->CRH &= ~PIN12_CLEAR; GPIOA->CRH |= PIN12_OUT50_ALTPUSHPULL;
#define PA12_OUT             GPIOA->CRH &= ~PIN12_CLEAR; GPIOA->CRH |= PIN12_OUT50_GPPUSHPULL;
#define PA12_FLOAT           GPIOA->CRH &= ~PIN12_CLEAR; GPIOA->CRH |= PIN12_IN_FLOAT;
#define PA12_PULLUP          GPIOA->CRH &= ~PIN12_CLEAR; GPIOA->CRH |= PIN12_IN_PULL; GPIOA->BSRR = BIT12;
#define PA12_PULLDOWN        GPIOA->CRH &= ~PIN12_CLEAR; GPIOA->CRH |= PIN12_IN_PULL; GPIOA->BRR = BIT12;
#define PA12_ANALOG          GPIOA->CRH &= ~PIN12_CLEAR; GPIOA->CRH |= PIN12_IN_ANALOG;
#define PA12_GET             ( GPIOA->IDR & BIT12 )
#define PA12_STATE           PA12_GET
#define PA12_HIGH            GPIOA->BSRR = BIT12;
#define PA12_LOW             GPIOA->BRR = BIT12;

#define PA13_ALT             GPIOA->CRH &= ~PIN13_CLEAR; GPIOA->CRH |= PIN13_OUT50_ALTPUSHPULL;
#define PA13_OUT             GPIOA->CRH &= ~PIN13_CLEAR; GPIOA->CRH |= PIN13_OUT50_GPPUSHPULL;
#define PA13_FLOAT           GPIOA->CRH &= ~PIN13_CLEAR; GPIOA->CRH |= PIN13_IN_FLOAT;
#define PA13_PULLUP          GPIOA->CRH &= ~PIN13_CLEAR; GPIOA->CRH |= PIN13_IN_PULL; GPIOA->BSRR = BIT13;
#define PA13_PULLDOWN        GPIOA->CRH &= ~PIN13_CLEAR; GPIOA->CRH |= PIN13_IN_PULL; GPIOA->BRR = BIT13;
#define PA13_ANALOG          GPIOA->CRH &= ~PIN13_CLEAR; GPIOA->CRH |= PIN13_IN_ANALOG;
#define PA13_GET             ( GPIOA->IDR & BIT13 )
#define PA13_STATE           PA13_GET
#define PA13_HIGH            GPIOA->BSRR = BIT13;
#define PA13_LOW             GPIOA->BRR = BIT13;

#define PA14_ALT             GPIOA->CRH &= ~PIN14_CLEAR; GPIOA->CRH |= PIN14_OUT50_ALTPUSHPULL;
#define PA14_OUT             GPIOA->CRH &= ~PIN14_CLEAR; GPIOA->CRH |= PIN14_OUT50_GPPUSHPULL;
#define PA14_FLOAT           GPIOA->CRH &= ~PIN14_CLEAR; GPIOA->CRH |= PIN14_IN_FLOAT;
#define PA14_PULLUP          GPIOA->CRH &= ~PIN14_CLEAR; GPIOA->CRH |= PIN14_IN_PULL; GPIOA->BSRR = BIT14;
#define PA14_PULLDOWN        GPIOA->CRH &= ~PIN14_CLEAR; GPIOA->CRH |= PIN14_IN_PULL; GPIOA->BRR = BIT14;
#define PA14_ANALOG          GPIOA->CRH &= ~PIN14_CLEAR; GPIOA->CRH |= PIN14_IN_ANALOG;
#define PA14_GET             ( GPIOA->IDR & BIT14 )
#define PA14_STATE           PA14_GET
#define PA14_HIGH            GPIOA->BSRR = BIT14;
#define PA14_LOW             GPIOA->BRR = BIT14;

#define PA15_ALT             GPIOA->CRH &= ~PIN15_CLEAR; GPIOA->CRH |= PIN15_OUT50_ALTPUSHPULL;
#define PA15_OUT             GPIOA->CRH &= ~PIN15_CLEAR; GPIOA->CRH |= PIN15_OUT50_GPPUSHPULL;
#define PA15_FLOAT           GPIOA->CRH &= ~PIN15_CLEAR; GPIOA->CRH |= PIN15_IN_FLOAT;
#define PA15_PULLUP          GPIOA->CRH &= ~PIN15_CLEAR; GPIOA->CRH |= PIN15_IN_PULL; GPIOA->BSRR = BIT15;
#define PA15_PULLDOWN        GPIOA->CRH &= ~PIN15_CLEAR; GPIOA->CRH |= PIN15_IN_PULL; GPIOA->BRR = BIT15;
#define PA15_ANALOG          GPIOA->CRH &= ~PIN15_CLEAR; GPIOA->CRH |= PIN15_IN_ANALOG;
#define PA15_GET             ( GPIOA->IDR & BIT15 )
#define PA15_STATE           PA15_GET
#define PA15_HIGH            GPIOA->BSRR = BIT15;
#define PA15_LOW             GPIOA->BRR = BIT15;

#define PB0_ALT             GPIOB->CRL &= ~PIN0_CLEAR; GPIOB->CRL |= PIN0_OUT50_ALTPUSHPULL;
#define PB0_OUT             GPIOB->CRL &= ~PIN0_CLEAR; GPIOB->CRL |= PIN0_OUT50_GPPUSHPULL;
#define PB0_FLOAT           GPIOB->CRL &= ~PIN0_CLEAR; GPIOB->CRL |= PIN0_IN_FLOAT;
#define PB0_PULLUP          GPIOB->CRL &= ~PIN0_CLEAR; GPIOB->CRL |= PIN0_IN_PULL; GPIOB->BSRR = BIT0;
#define PB0_PULLDOWN        GPIOB->CRL &= ~PIN0_CLEAR; GPIOB->CRL |= PIN0_IN_PULL; GPIOB->BRR = BIT0;
#define PB0_ANALOG          GPIOB->CRL &= ~PIN0_CLEAR; GPIOB->CRL |= PIN0_IN_ANALOG;
#define PB0_GET             ( GPIOB->IDR & BIT0 )
#define PB0_STATE           PB0_GET
#define PB0_HIGH            GPIOB->BSRR = BIT0;
#define PB0_LOW             GPIOB->BRR = BIT0;

#define PB1_ALT             GPIOB->CRL &= ~PIN1_CLEAR; GPIOB->CRL |= PIN1_OUT50_ALTPUSHPULL;
#define PB1_OUT             GPIOB->CRL &= ~PIN1_CLEAR; GPIOB->CRL |= PIN1_OUT50_GPPUSHPULL;
#define PB1_FLOAT           GPIOB->CRL &= ~PIN1_CLEAR; GPIOB->CRL |= PIN1_IN_FLOAT;
#define PB1_PULLUP          GPIOB->CRL &= ~PIN1_CLEAR; GPIOB->CRL |= PIN1_IN_PULL; GPIOB->BSRR = BIT1;
#define PB1_PULLDOWN        GPIOB->CRL &= ~PIN1_CLEAR; GPIOB->CRL |= PIN1_IN_PULL; GPIOB->BRR = BIT1;
#define PB1_ANALOG          GPIOB->CRL &= ~PIN1_CLEAR; GPIOB->CRL |= PIN1_IN_ANALOG;
#define PB1_GET             ( GPIOB->IDR & BIT1 )
#define PB1_HIGH            GPIOB->BSRR = BIT1;
#define PB1_LOW             GPIOB->BRR = BIT1;

#define PB2_ALT             GPIOB->CRL &= ~PIN2_CLEAR; GPIOB->CRL |= PIN2_OUT50_ALTPUSHPULL;
#define PB2_OUT             GPIOB->CRL &= ~PIN2_CLEAR; GPIOB->CRL |= PIN2_OUT50_GPPUSHPULL;
#define PB2_FLOAT           GPIOB->CRL &= ~PIN2_CLEAR; GPIOB->CRL |= PIN2_IN_FLOAT;
#define PB2_PULLUP          GPIOB->CRL &= ~PIN2_CLEAR; GPIOB->CRL |= PIN2_IN_PULL; GPIOB->BSRR = BIT2;
#define PB2_PULLDOWN        GPIOB->CRL &= ~PIN2_CLEAR; GPIOB->CRL |= PIN2_IN_PULL; GPIOB->BRR = BIT2;
#define PB2_ANALOG          GPIOB->CRL &= ~PIN2_CLEAR; GPIOB->CRL |= PIN2_IN_ANALOG;
#define PB2_GET             ( GPIOB->IDR & BIT2 )
#define PB2_STATE           PB2_GET
#define PB2_HIGH            GPIOB->BSRR = BIT2;
#define PB2_LOW             GPIOB->BRR = BIT2;

#define PB3_ALT             GPIOB->CRL &= ~PIN3_CLEAR; GPIOB->CRL |= PIN3_OUT50_ALTPUSHPULL;
#define PB3_OUT             GPIOB->CRL &= ~PIN3_CLEAR; GPIOB->CRL |= PIN3_OUT50_GPPUSHPULL;
#define PB3_FLOAT           GPIOB->CRL &= ~PIN3_CLEAR; GPIOB->CRL |= PIN3_IN_FLOAT;
#define PB3_PULLUP          GPIOB->CRL &= ~PIN3_CLEAR; GPIOB->CRL |= PIN3_IN_PULL; GPIOB->BSRR = BIT3;
#define PB3_PULLDOWN        GPIOB->CRL &= ~PIN3_CLEAR; GPIOB->CRL |= PIN3_IN_PULL; GPIOB->BRR = BIT3;
#define PB3_ANALOG          GPIOB->CRL &= ~PIN3_CLEAR; GPIOB->CRL |= PIN3_IN_ANALOG;
#define PB3_GET             ( GPIOB->IDR & BIT3 )
#define PB3_STATE           PB3_GET
#define PB3_HIGH            GPIOB->BSRR = BIT3;
#define PB3_LOW             GPIOB->BRR = BIT3;

#define PB4_ALT             GPIOB->CRL &= ~PIN4_CLEAR; GPIOB->CRL |= PIN4_OUT50_ALTPUSHPULL;
#define PB4_OUT             GPIOB->CRL &= ~PIN4_CLEAR; GPIOB->CRL |= PIN4_OUT50_GPPUSHPULL;
#define PB4_FLOAT           GPIOB->CRL &= ~PIN4_CLEAR; GPIOB->CRL |= PIN4_IN_FLOAT;
#define PB4_PULLUP          GPIOB->CRL &= ~PIN4_CLEAR; GPIOB->CRL |= PIN4_IN_PULL; GPIOB->BSRR = BIT4;
#define PB4_PULLDOWN        GPIOB->CRL &= ~PIN4_CLEAR; GPIOB->CRL |= PIN4_IN_PULL; GPIOB->BRR = BIT4;
#define PB4_ANALOG          GPIOB->CRL &= ~PIN4_CLEAR; GPIOB->CRL |= PIN4_IN_ANALOG;
#define PB4_GET             ( GPIOB->IDR & BIT4 )
#define PB4_STATE           PB4_GET
#define PB4_HIGH            GPIOB->BSRR = BIT4;
#define PB4_LOW             GPIOB->BRR = BIT4;

#define PB5_ALT             GPIOB->CRL &= ~PIN5_CLEAR; GPIOB->CRL |= PIN5_OUT50_ALTPUSHPULL;
#define PB5_OUT             GPIOB->CRL &= ~PIN5_CLEAR; GPIOB->CRL |= PIN5_OUT50_GPPUSHPULL;
#define PB5_FLOAT           GPIOB->CRL &= ~PIN5_CLEAR; GPIOB->CRL |= PIN5_IN_FLOAT;
#define PB5_PULLUP          GPIOB->CRL &= ~PIN5_CLEAR; GPIOB->CRL |= PIN5_IN_PULL; GPIOB->BSRR = BIT5;
#define PB5_PULLDOWN        GPIOB->CRL &= ~PIN5_CLEAR; GPIOB->CRL |= PIN5_IN_PULL; GPIOB->BRR = BIT5;
#define PB5_ANALOG          GPIOB->CRL &= ~PIN5_CLEAR; GPIOB->CRL |= PIN5_IN_ANALOG;
#define PB5_GET             ( GPIOB->IDR & BIT5 )
#define PB5_STATE           PB5_GET
#define PB5_HIGH            GPIOB->BSRR = BIT5;
#define PB5_LOW             GPIOB->BRR = BIT5;

#define PB6_ALT             GPIOB->CRL &= ~PIN6_CLEAR; GPIOB->CRL |= PIN6_OUT50_ALTPUSHPULL;
#define PB6_OUT             GPIOB->CRL &= ~PIN6_CLEAR; GPIOB->CRL |= PIN6_OUT50_GPPUSHPULL;
#define PB6_FLOAT           GPIOB->CRL &= ~PIN6_CLEAR; GPIOB->CRL |= PIN6_IN_FLOAT;
#define PB6_PULLUP          GPIOB->CRL &= ~PIN6_CLEAR; GPIOB->CRL |= PIN6_IN_PULL; GPIOB->BSRR = BIT6;
#define PB6_PULLDOWN        GPIOB->CRL &= ~PIN6_CLEAR; GPIOB->CRL |= PIN6_IN_PULL; GPIOB->BRR = BIT6;
#define PB6_ANALOG          GPIOB->CRL &= ~PIN6_CLEAR; GPIOB->CRL |= PIN6_IN_ANALOG;
#define PB6_GET             ( GPIOB->IDR & BIT6 )
#define PB6_STATE           PB6_GET
#define PB6_HIGH            GPIOB->BSRR = BIT6;
#define PB6_LOW             GPIOB->BRR = BIT6;

#define PB7_ALT             GPIOB->CRL &= ~PIN7_CLEAR; GPIOB->CRL |= PIN7_OUT50_ALTPUSHPULL;
#define PB7_OUT             GPIOB->CRL &= ~PIN7_CLEAR; GPIOB->CRL |= PIN7_OUT50_GPPUSHPULL;
#define PB7_FLOAT           GPIOB->CRL &= ~PIN7_CLEAR; GPIOB->CRL |= PIN7_IN_FLOAT;
#define PB7_PULLUP          GPIOB->CRL &= ~PIN7_CLEAR; GPIOB->CRL |= PIN7_IN_PULL; GPIOB->BSRR = BIT7;
#define PB7_PULLDOWN        GPIOB->CRL &= ~PIN7_CLEAR; GPIOB->CRL |= PIN7_IN_PULL; GPIOB->BRR = BIT7;
#define PB7_ANALOG          GPIOB->CRL &= ~PIN7_CLEAR; GPIOB->CRL |= PIN7_IN_ANALOG;
#define PB7_GET             ( GPIOB->IDR & BIT7 )
#define PB7_STATE           PB7_GET
#define PB7_HIGH            GPIOB->BSRR = BIT7;
#define PB7_LOW             GPIOB->BRR = BIT7;

#define PB8_ALT             GPIOB->CRH &= ~PIN8_CLEAR; GPIOB->CRH |= PIN8_OUT50_ALTPUSHPULL;
#define PB8_OUT             GPIOB->CRH &= ~PIN8_CLEAR; GPIOB->CRH |= PIN8_OUT50_GPPUSHPULL;
#define PB8_FLOAT           GPIOB->CRH &= ~PIN8_CLEAR; GPIOB->CRH |= PIN8_IN_FLOAT;
#define PB8_PULLUP          GPIOB->CRH &= ~PIN8_CLEAR; GPIOB->CRH |= PIN8_IN_PULL; GPIOB->BSRR = BIT8;
#define PB8_PULLDOWN        GPIOB->CRH &= ~PIN8_CLEAR; GPIOB->CRH |= PIN8_IN_PULL; GPIOB->BRR = BIT8;
#define PB8_ANALOG          GPIOB->CRH &= ~PIN8_CLEAR; GPIOB->CRH |= PIN8_IN_ANALOG;
#define PB8_GET             ( GPIOB->IDR & BIT8 )
#define PB8_STATE           PB8_GET
#define PB8_HIGH            GPIOB->BSRR = BIT8;
#define PB8_LOW             GPIOB->BRR = BIT8;

#define PB9_ALT             GPIOB->CRH &= ~PIN9_CLEAR; GPIOB->CRH |= PIN9_OUT50_ALTPUSHPULL;
#define PB9_OUT             GPIOB->CRH &= ~PIN9_CLEAR; GPIOB->CRH |= PIN9_OUT50_GPPUSHPULL;
#define PB9_FLOAT           GPIOB->CRH &= ~PIN9_CLEAR; GPIOB->CRH |= PIN9_IN_FLOAT;
#define PB9_PULLUP          GPIOB->CRH &= ~PIN9_CLEAR; GPIOB->CRH |= PIN9_IN_PULL; GPIOB->BSRR = BIT9;
#define PB9_PULLDOWN        GPIOB->CRH &= ~PIN9_CLEAR; GPIOB->CRH |= PIN9_IN_PULL; GPIOB->BRR = BIT9;
#define PB9_ANALOG          GPIOB->CRH &= ~PIN9_CLEAR; GPIOB->CRH |= PIN9_IN_ANALOG;
#define PB9_GET             ( GPIOB->IDR & BIT9 )
#define PB9_STATE           PB9_GET
#define PB9_HIGH            GPIOB->BSRR = BIT9;
#define PB9_LOW             GPIOB->BRR = BIT9;
                            
#define PB10_ALT            GPIOB->CRH &= ~PIN10_CLEAR; GPIOB->CRH |= PIN10_OUT50_ALTPUSHPULL;
#define PB10_OUT            GPIOB->CRH &= ~PIN10_CLEAR; GPIOB->CRH |= PIN10_OUT50_GPPUSHPULL;
#define PB10_FLOAT          GPIOB->CRH &= ~PIN10_CLEAR; GPIOB->CRH |= PIN10_IN_FLOAT;
#define PB10_PULLUP         GPIOB->CRH &= ~PIN10_CLEAR; GPIOB->CRH |= PIN10_IN_PULL; GPIOB->BSRR = BIT10;
#define PB10_PULLDOWN       GPIOB->CRH &= ~PIN10_CLEAR; GPIOB->CRH |= PIN10_IN_PULL; GPIOB->BRR = BIT10;
#define PB10_ANALOG         GPIOB->CRH &= ~PIN10_CLEAR; GPIOB->CRH |= PIN10_IN_ANALOG;
#define PB10_GET            ( GPIOB->IDR & BIT10 )
#define PB10_STATE          PB10_GET
#define PB10_HIGH           GPIOB->BSRR = BIT10;
#define PB10_LOW            GPIOB->BRR = BIT10;

#define PB11_ALT            GPIOB->CRH &= ~PIN11_CLEAR; GPIOB->CRH |= PIN11_OUT50_ALTPUSHPULL;
#define PB11_OUT            GPIOB->CRH &= ~PIN11_CLEAR; GPIOB->CRH |= PIN11_OUT50_GPPUSHPULL;
#define PB11_FLOAT          GPIOB->CRH &= ~PIN11_CLEAR; GPIOB->CRH |= PIN11_IN_FLOAT;
#define PB11_PULLUP         GPIOB->CRH &= ~PIN11_CLEAR; GPIOB->CRH |= PIN11_IN_PULL; GPIOB->BSRR = BIT11;
#define PB11_PULLDOWN       GPIOB->CRH &= ~PIN11_CLEAR; GPIOB->CRH |= PIN11_IN_PULL; GPIOB->BRR = BIT11;
#define PB11_ANALOG         GPIOB->CRH &= ~PIN11_CLEAR; GPIOB->CRH |= PIN11_IN_ANALOG;
#define PB11_GET            ( GPIOB->IDR & BIT11 )
#define PB11_STATE          PB11_GET
#define PB11_HIGH           GPIOB->BSRR = BIT11;
#define PB11_LOW            GPIOB->BRR = BIT11;

#define PB12_ALT            GPIOB->CRH &= ~PIN12_CLEAR; GPIOB->CRH |= PIN12_OUT50_ALTPUSHPULL;
#define PB12_OUT            GPIOB->CRH &= ~PIN12_CLEAR; GPIOB->CRH |= PIN12_OUT50_GPPUSHPULL;
#define PB12_FLOAT          GPIOB->CRH &= ~PIN12_CLEAR; GPIOB->CRH |= PIN12_IN_FLOAT;
#define PB12_PULLUP         GPIOB->CRH &= ~PIN12_CLEAR; GPIOB->CRH |= PIN12_IN_PULL; GPIOB->BSRR = BIT12;
#define PB12_PULLDOWN       GPIOB->CRH &= ~PIN12_CLEAR; GPIOB->CRH |= PIN12_IN_PULL; GPIOB->BRR = BIT12;
#define PB12_ANALOG         GPIOB->CRH &= ~PIN12_CLEAR; GPIOB->CRH |= PIN12_IN_ANALOG;
#define PB12_GET            ( GPIOB->IDR & BIT12 )
#define PB12_STATE          PB12_GET
#define PB12_HIGH           GPIOB->BSRR = BIT12;
#define PB12_LOW            GPIOB->BRR = BIT12;

#define PB13_ALT            GPIOB->CRH &= ~PIN13_CLEAR; GPIOB->CRH |= PIN13_OUT50_ALTPUSHPULL;
#define PB13_OUT            GPIOB->CRH &= ~PIN13_CLEAR; GPIOB->CRH |= PIN13_OUT50_GPPUSHPULL;
#define PB13_FLOAT          GPIOB->CRH &= ~PIN13_CLEAR; GPIOB->CRH |= PIN13_IN_FLOAT;
#define PB13_PULLUP         GPIOB->CRH &= ~PIN13_CLEAR; GPIOB->CRH |= PIN13_IN_PULL; GPIOB->BSRR = BIT13;
#define PB13_PULLDOWN       GPIOB->CRH &= ~PIN13_CLEAR; GPIOB->CRH |= PIN13_IN_PULL; GPIOB->BRR = BIT13;
#define PB13_ANALOG         GPIOB->CRH &= ~PIN13_CLEAR; GPIOB->CRH |= PIN13_IN_ANALOG;
#define PB13_GET            ( GPIOB->IDR & BIT13 )
#define PB13_STATE          PB13_GET
#define PB13_HIGH           GPIOB->BSRR = BIT13;
#define PB13_LOW            GPIOB->BRR = BIT13;

#define PB14_ALT            GPIOB->CRH &= ~PIN14_CLEAR; GPIOB->CRH |= PIN14_OUT50_ALTPUSHPULL;
#define PB14_OUT            GPIOB->CRH &= ~PIN14_CLEAR; GPIOB->CRH |= PIN14_OUT50_GPPUSHPULL;
#define PB14_FLOAT          GPIOB->CRH &= ~PIN14_CLEAR; GPIOB->CRH |= PIN14_IN_FLOAT;
#define PB14_PULLUP         GPIOB->CRH &= ~PIN14_CLEAR; GPIOB->CRH |= PIN14_IN_PULL; GPIOB->BSRR = BIT14;
#define PB14_PULLDOWN       GPIOB->CRH &= ~PIN14_CLEAR; GPIOB->CRH |= PIN14_IN_PULL; GPIOB->BRR = BIT14;
#define PB14_ANALOG         GPIOB->CRH &= ~PIN14_CLEAR; GPIOB->CRH |= PIN14_IN_ANALOG;
#define PB14_GET            ( GPIOB->IDR & BIT14 )
#define PB14_STATE          PB14_GET
#define PB14_HIGH           GPIOB->BSRR = BIT14;
#define PB14_LOW            GPIOB->BRR = BIT14;

#define PB15_ALT            GPIOB->CRH &= ~PIN15_CLEAR; GPIOB->CRH |= PIN15_OUT50_ALTPUSHPULL;
#define PB15_OUT            GPIOB->CRH &= ~PIN15_CLEAR; GPIOB->CRH |= PIN15_OUT50_GPPUSHPULL;
#define PB15_FLOAT          GPIOB->CRH &= ~PIN15_CLEAR; GPIOB->CRH |= PIN15_IN_FLOAT;
#define PB15_PULLUP         GPIOB->CRH &= ~PIN15_CLEAR; GPIOB->CRH |= PIN15_IN_PULL; GPIOB->BSRR = BIT15;
#define PB15_PULLDOWN       GPIOB->CRH &= ~PIN15_CLEAR; GPIOB->CRH |= PIN15_IN_PULL; GPIOB->BRR = BIT15;
#define PB15_ANALOG         GPIOB->CRH &= ~PIN15_CLEAR; GPIOB->CRH |= PIN15_IN_ANALOG;
#define PB15_GET            ( GPIOB->IDR & BIT15 )
#define PB15_STATE          PB15_GET
#define PB15_HIGH           GPIOB->BSRR = BIT15;
#define PB15_LOW            GPIOB->BRR = BIT15;

#define PC0_ALT             GPIOC->CRL &= ~PIN0_CLEAR; GPIOC->CRL |= PIN0_OUT50_ALTPUSHPULL;
#define PC0_OUT             GPIOC->CRL &= ~PIN0_CLEAR; GPIOC->CRL |= PIN0_OUT50_GPPUSHPULL;
#define PC0_FLOAT           GPIOC->CRL &= ~PIN0_CLEAR; GPIOC->CRL |= PIN0_IN_FLOAT;
#define PC0_PULLUP          GPIOC->CRL &= ~PIN0_CLEAR; GPIOC->CRL |= PIN0_IN_PULL; GPIOC->BSRR = BIT0;
#define PC0_PULLDOWN        GPIOC->CRL &= ~PIN0_CLEAR; GPIOC->CRL |= PIN0_IN_PULL; GPIOC->BRR = BIT0;
#define PC0_ANALOG          GPIOC->CRL &= ~PIN0_CLEAR; GPIOC->CRL |= PIN0_IN_ANALOG;
#define PC0_GET             ( GPIOC->IDR & BIT0 )
#define PC0_STATE           PC0_STATE
#define PC0_HIGH            GPIOC->BSRR = BIT0;
#define PC0_LOW             GPIOC->BRR = BIT0;

#define PC1_ALT             GPIOC->CRL &= ~PIN1_CLEAR; GPIOC->CRL |= PIN1_OUT50_ALTPUSHPULL;
#define PC1_OUT             GPIOC->CRL &= ~PIN1_CLEAR; GPIOC->CRL |= PIN1_OUT50_GPPUSHPULL;
#define PC1_FLOAT           GPIOC->CRL &= ~PIN1_CLEAR; GPIOC->CRL |= PIN1_IN_FLOAT;
#define PC1_PULLUP          GPIOC->CRL &= ~PIN1_CLEAR; GPIOC->CRL |= PIN1_IN_PULL; GPIOC->BSRR = BIT1;
#define PC1_PULLDOWN        GPIOC->CRL &= ~PIN1_CLEAR; GPIOC->CRL |= PIN1_IN_PULL; GPIOC->BRR = BIT1;
#define PC1_ANALOG          GPIOC->CRL &= ~PIN1_CLEAR; GPIOC->CRL |= PIN1_IN_ANALOG;
#define PC1_GET             ( GPIOC->IDR & BIT1 )
#define PC1_STATE           PC1_GET
#define PC1_HIGH            GPIOC->BSRR = BIT1;
#define PC1_LOW             GPIOC->BRR = BIT1;

#define PC2_ALT             GPIOC->CRL &= ~PIN2_CLEAR; GPIOC->CRL |= PIN2_OUT50_ALTPUSHPULL;
#define PC2_OUT             GPIOC->CRL &= ~PIN2_CLEAR; GPIOC->CRL |= PIN2_OUT50_GPPUSHPULL;
#define PC2_FLOAT           GPIOC->CRL &= ~PIN2_CLEAR; GPIOC->CRL |= PIN2_IN_FLOAT;
#define PC2_PULLUP          GPIOC->CRL &= ~PIN2_CLEAR; GPIOC->CRL |= PIN2_IN_PULL; GPIOC->BSRR = BIT2;
#define PC2_PULLDOWN        GPIOC->CRL &= ~PIN2_CLEAR; GPIOC->CRL |= PIN2_IN_PULL; GPIOC->BRR = BIT2;
#define PC2_ANALOG          GPIOC->CRL &= ~PIN2_CLEAR; GPIOC->CRL |= PIN2_IN_ANALOG;
#define PC2_GET             ( GPIOC->IDR & BIT2 )
#define PC2_STATE           PC2_GET
#define PC2_HIGH            GPIOC->BSRR = BIT2;
#define PC2_LOW             GPIOC->BRR = BIT2;

#define PC3_ALT             GPIOC->CRL &= ~PIN3_CLEAR; GPIOC->CRL |= PIN3_OUT50_ALTPUSHPULL;
#define PC3_OUT             GPIOC->CRL &= ~PIN3_CLEAR; GPIOC->CRL |= PIN3_OUT50_GPPUSHPULL;
#define PC3_FLOAT           GPIOC->CRL &= ~PIN3_CLEAR; GPIOC->CRL |= PIN3_IN_FLOAT;
#define PC3_PULLUP          GPIOC->CRL &= ~PIN3_CLEAR; GPIOC->CRL |= PIN3_IN_PULL; GPIOC->BSRR = BIT3;
#define PC3_PULLDOWN        GPIOC->CRL &= ~PIN3_CLEAR; GPIOC->CRL |= PIN3_IN_PULL; GPIOC->BRR = BIT3;
#define PC3_ANALOG          GPIOC->CRL &= ~PIN3_CLEAR; GPIOC->CRL |= PIN3_IN_ANALOG;
#define PC3_GET             ( GPIOC->IDR & BIT3 )
#define PC3_STATE           PC3_GET
#define PC3_HIGH            GPIOC->BSRR = BIT3;
#define PC3_LOW             GPIOC->BRR = BIT3;

#define PC4_ALT             GPIOC->CRL &= ~PIN4_CLEAR; GPIOC->CRL |= PIN4_OUT50_ALTPUSHPULL;
#define PC4_OUT             GPIOC->CRL &= ~PIN4_CLEAR; GPIOC->CRL |= PIN4_OUT50_GPPUSHPULL;
#define PC4_FLOAT           GPIOC->CRL &= ~PIN4_CLEAR; GPIOC->CRL |= PIN4_IN_FLOAT;
#define PC4_PULLUP          GPIOC->CRL &= ~PIN4_CLEAR; GPIOC->CRL |= PIN4_IN_PULL; GPIOC->BSRR = BIT4;
#define PC4_PULLDOWN        GPIOC->CRL &= ~PIN4_CLEAR; GPIOC->CRL |= PIN4_IN_PULL; GPIOC->BRR = BIT4;
#define PC4_ANALOG          GPIOC->CRL &= ~PIN4_CLEAR; GPIOC->CRL |= PIN4_IN_ANALOG;
#define PC4_GET             ( GPIOC->IDR & BIT4 )
#define PC4_STATE           PC4_GET
#define PC4_HIGH            GPIOC->BSRR = BIT4;
#define PC4_LOW             GPIOC->BRR = BIT4;

#define PC5_ALT             GPIOC->CRL &= ~PIN5_CLEAR; GPIOC->CRL |= PIN5_OUT50_ALTPUSHPULL;
#define PC5_OUT             GPIOC->CRL &= ~PIN5_CLEAR; GPIOC->CRL |= PIN5_OUT50_GPPUSHPULL;
#define PC5_FLOAT           GPIOC->CRL &= ~PIN5_CLEAR; GPIOC->CRL |= PIN5_IN_FLOAT;
#define PC5_PULLUP          GPIOC->CRL &= ~PIN5_CLEAR; GPIOC->CRL |= PIN5_IN_PULL; GPIOC->BSRR = BIT5;
#define PC5_PULLDOWN        GPIOC->CRL &= ~PIN5_CLEAR; GPIOC->CRL |= PIN5_IN_PULL; GPIOC->BRR = BIT5;
#define PC5_ANALOG          GPIOC->CRL &= ~PIN5_CLEAR; GPIOC->CRL |= PIN5_IN_ANALOG;
#define PC5_GET             ( GPIOC->IDR & BIT5 )
#define PC5_STATE           PC5_GET
#define PC5_HIGH            GPIOC->BSRR = BIT5;
#define PC5_LOW             GPIOC->BRR = BIT5;

#define PC6_ALT             GPIOC->CRL &= ~PIN6_CLEAR; GPIOC->CRL |= PIN6_OUT50_ALTPUSHPULL;
#define PC6_OUT             GPIOC->CRL &= ~PIN6_CLEAR; GPIOC->CRL |= PIN6_OUT50_GPPUSHPULL;
#define PC6_FLOAT           GPIOC->CRL &= ~PIN6_CLEAR; GPIOC->CRL |= PIN6_IN_FLOAT;
#define PC6_PULLUP          GPIOC->CRL &= ~PIN6_CLEAR; GPIOC->CRL |= PIN6_IN_PULL; GPIOC->BSRR = BIT6;
#define PC6_PULLDOWN        GPIOC->CRL &= ~PIN6_CLEAR; GPIOC->CRL |= PIN6_IN_PULL; GPIOC->BRR = BIT6;
#define PC6_ANALOG          GPIOC->CRL &= ~PIN6_CLEAR; GPIOC->CRL |= PIN6_IN_ANALOG;
#define PC6_GET             ( GPIOC->IDR & BIT6 )
#define PC6_STATE           PC6_GET
#define PC6_HIGH            GPIOC->BSRR = BIT6;
#define PC6_LOW             GPIOC->BRR = BIT6;

#define PC7_ALT             GPIOC->CRL &= ~PIN7_CLEAR; GPIOC->CRL |= PIN7_OUT50_ALTPUSHPULL;
#define PC7_OUT             GPIOC->CRL &= ~PIN7_CLEAR; GPIOC->CRL |= PIN7_OUT50_GPPUSHPULL;
#define PC7_FLOAT           GPIOC->CRL &= ~PIN7_CLEAR; GPIOC->CRL |= PIN7_IN_FLOAT;
#define PC7_PULLUP          GPIOC->CRL &= ~PIN7_CLEAR; GPIOC->CRL |= PIN7_IN_PULL; GPIOC->BSRR = BIT7;
#define PC7_PULLDOWN        GPIOC->CRL &= ~PIN7_CLEAR; GPIOC->CRL |= PIN7_IN_PULL; GPIOC->BRR = BIT7;
#define PC7_ANALOG          GPIOC->CRL &= ~PIN7_CLEAR; GPIOC->CRL |= PIN7_IN_ANALOG;
#define PC7_GET             ( GPIOC->IDR & BIT7 )
#define PC7_STATE           PC7_GET
#define PC7_HIGH            GPIOC->BSRR = BIT7;
#define PC7_LOW             GPIOC->BRR = BIT7;

#define PC8_ALT             GPIOC->CRH &= ~PIN8_CLEAR; GPIOC->CRH |= PIN8_OUT50_ALTPUSHPULL;
#define PC8_OUT             GPIOC->CRH &= ~PIN8_CLEAR; GPIOC->CRH |= PIN8_OUT50_GPPUSHPULL;
#define PC8_FLOAT           GPIOC->CRH &= ~PIN8_CLEAR; GPIOC->CRH |= PIN8_IN_FLOAT;
#define PC8_PULLUP          GPIOC->CRH &= ~PIN8_CLEAR; GPIOC->CRH |= PIN8_IN_PULL; GPIOC->BSRR = BIT8;
#define PC8_PULLDOWN        GPIOC->CRH &= ~PIN8_CLEAR; GPIOC->CRH |= PIN8_IN_PULL; GPIOC->BRR = BIT8;
#define PC8_ANALOG          GPIOC->CRH &= ~PIN8_CLEAR; GPIOC->CRH |= PIN8_IN_ANALOG;
#define PC8_GET             ( GPIOC->IDR & BIT8 )
#define PC8_STATE           PC8_GET
#define PC8_HIGH            GPIOC->BSRR = BIT8;
#define PC8_LOW             GPIOC->BRR = BIT8;

#define PC9_ALT             GPIOC->CRH &= ~PIN9_CLEAR; GPIOC->CRH |= PIN9_OUT50_ALTPUSHPULL;
#define PC9_OUT             GPIOC->CRH &= ~PIN9_CLEAR; GPIOC->CRH |= PIN9_OUT50_GPPUSHPULL;
#define PC9_FLOAT           GPIOC->CRH &= ~PIN9_CLEAR; GPIOC->CRH |= PIN9_IN_FLOAT;
#define PC9_PULLUP          GPIOC->CRH &= ~PIN9_CLEAR; GPIOC->CRH |= PIN9_IN_PULL; GPIOC->BSRR = BIT9;
#define PC9_PULLDOWN        GPIOC->CRH &= ~PIN9_CLEAR; GPIOC->CRH |= PIN9_IN_PULL; GPIOC->BRR = BIT9;
#define PC9_ANALOG          GPIOC->CRH &= ~PIN9_CLEAR; GPIOC->CRH |= PIN9_IN_ANALOG;
#define PC9_GET             ( GPIOC->IDR & BIT9 )
#define PC9_STATE           PC9_GET
#define PC9_HIGH            GPIOC->BSRR = BIT9;
#define PC9_LOW             GPIOC->BRR = BIT9;

#define PC10_ALT            GPIOC->CRH &= ~PIN10_CLEAR; GPIOC->CRH |= PIN10_OUT50_ALTPUSHPULL;
#define PC10_OUT            GPIOC->CRH &= ~PIN10_CLEAR; GPIOC->CRH |= PIN10_OUT50_GPPUSHPULL;
#define PC10_FLOAT          GPIOC->CRH &= ~PIN10_CLEAR; GPIOC->CRH |= PIN10_IN_FLOAT;
#define PC10_PULLUP         GPIOC->CRH &= ~PIN10_CLEAR; GPIOC->CRH |= PIN10_IN_PULL; GPIOC->BSRR = BIT10;
#define PC10_PULLDOWN       GPIOC->CRH &= ~PIN10_CLEAR; GPIOC->CRH |= PIN10_IN_PULL; GPIOC->BRR = BIT10;
#define PC10_ANALOG         GPIOC->CRH &= ~PIN10_CLEAR; GPIOC->CRH |= PIN10_IN_ANALOG;
#define PC10_GET            ( GPIOC->IDR & BIT10 )
#define PC10_STATE          PC10_GET
#define PC10_HIGH           GPIOC->BSRR = BIT10;
#define PC10_LOW            GPIOC->BRR = BIT10;

#define PC11_ALT            GPIOC->CRH &= ~PIN11_CLEAR; GPIOC->CRH |= PIN11_OUT50_ALTPUSHPULL;
#define PC11_OUT            GPIOC->CRH &= ~PIN11_CLEAR; GPIOC->CRH |= PIN11_OUT50_GPPUSHPULL;
#define PC11_FLOAT          GPIOC->CRH &= ~PIN11_CLEAR; GPIOC->CRH |= PIN11_IN_FLOAT;
#define PC11_PULLUP         GPIOC->CRH &= ~PIN11_CLEAR; GPIOC->CRH |= PIN11_IN_PULL; GPIOC->BSRR = BIT11;
#define PC11_PULLDOWN       GPIOC->CRH &= ~PIN11_CLEAR; GPIOC->CRH |= PIN11_IN_PULL; GPIOC->BRR = BIT11;
#define PC11_ANALOG         GPIOC->CRH &= ~PIN11_CLEAR; GPIOC->CRH |= PIN11_IN_ANALOG;
#define PC11_GET            ( GPIOC->IDR & BIT11 )
#define PC11_STATE          PC11_GET
#define PC11_HIGH           GPIOC->BSRR = BIT11;
#define PC11_LOW            GPIOC->BRR = BIT11;

#define PC12_ALT            GPIOC->CRH &= ~PIN12_CLEAR; GPIOC->CRH |= PIN12_OUT50_ALTPUSHPULL;
#define PC12_OUT            GPIOC->CRH &= ~PIN12_CLEAR; GPIOC->CRH |= PIN12_OUT50_GPPUSHPULL;
#define PC12_FLOAT          GPIOC->CRH &= ~PIN12_CLEAR; GPIOC->CRH |= PIN12_IN_FLOAT;
#define PC12_PULLUP         GPIOC->CRH &= ~PIN12_CLEAR; GPIOC->CRH |= PIN12_IN_PULL; GPIOC->BSRR = BIT12;
#define PC12_PULLDOWN       GPIOC->CRH &= ~PIN12_CLEAR; GPIOC->CRH |= PIN12_IN_PULL; GPIOC->BRR = BIT12;
#define PC12_ANALOG         GPIOC->CRH &= ~PIN12_CLEAR; GPIOC->CRH |= PIN12_IN_ANALOG;
#define PC12_GET            ( GPIOC->IDR & BIT12 )
#define PC12_STATE          PC12_GET
#define PC12_HIGH           GPIOC->BSRR = BIT12;
#define PC12_LOW            GPIOC->BRR = BIT12;

#define PC13_ALT            GPIOC->CRH &= ~PIN13_CLEAR; GPIOC->CRH |= PIN13_OUT50_ALTPUSHPULL;
#define PC13_OUT            GPIOC->CRH &= ~PIN13_CLEAR; GPIOC->CRH |= PIN13_OUT50_GPPUSHPULL;
#define PC13_FLOAT          GPIOC->CRH &= ~PIN13_CLEAR; GPIOC->CRH |= PIN13_IN_FLOAT;
#define PC13_PULLUP         GPIOC->CRH &= ~PIN13_CLEAR; GPIOC->CRH |= PIN13_IN_PULL; GPIOC->BSRR = BIT13;
#define PC13_PULLDOWN       GPIOC->CRH &= ~PIN13_CLEAR; GPIOC->CRH |= PIN13_IN_PULL; GPIOC->BRR = BIT13;
#define PC13_ANALOG         GPIOC->CRH &= ~PIN13_CLEAR; GPIOC->CRH |= PIN13_IN_ANALOG;
#define PC13_GET            ( GPIOC->IDR & BIT13 )
#define PC13_STATE          PC13_GET
#define PC13_HIGH           GPIOC->BSRR = BIT13;
#define PC13_LOW            GPIOC->BRR = BIT13;

#define PC14_ALT            GPIOC->CRH &= ~PIN14_CLEAR; GPIOC->CRH |= PIN14_OUT50_ALTPUSHPULL;
#define PC14_OUT            GPIOC->CRH &= ~PIN14_CLEAR; GPIOC->CRH |= PIN14_OUT50_GPPUSHPULL;
#define PC14_FLOAT          GPIOC->CRH &= ~PIN14_CLEAR; GPIOC->CRH |= PIN14_IN_FLOAT;
#define PC14_PULLUP         GPIOC->CRH &= ~PIN14_CLEAR; GPIOC->CRH |= PIN14_IN_PULL; GPIOC->BSRR = BIT14;
#define PC14_PULLDOWN       GPIOC->CRH &= ~PIN14_CLEAR; GPIOC->CRH |= PIN14_IN_PULL; GPIOC->BRR = BIT14;
#define PC14_ANALOG         GPIOC->CRH &= ~PIN14_CLEAR; GPIOC->CRH |= PIN14_IN_ANALOG;
#define PC14_GET            ( GPIOC->IDR & BIT14 )
#define PC14_STATE          PC14_GET
#define PC14_HIGH           GPIOC->BSRR = BIT14;
#define PC14_LOW            GPIOC->BRR = BIT14;

#define PC15_ALT            GPIOC->CRH &= ~PIN15_CLEAR; GPIOC->CRH |= PIN15_OUT50_ALTPUSHPULL;
#define PC15_OUT            GPIOC->CRH &= ~PIN15_CLEAR; GPIOC->CRH |= PIN15_OUT50_GPPUSHPULL;
#define PC15_FLOAT          GPIOC->CRH &= ~PIN15_CLEAR; GPIOC->CRH |= PIN15_IN_FLOAT;
#define PC15_PULLUP         GPIOC->CRH &= ~PIN15_CLEAR; GPIOC->CRH |= PIN15_IN_PULL; GPIOC->BSRR = BIT15;
#define PC15_PULLDOWN       GPIOC->CRH &= ~PIN15_CLEAR; GPIOC->CRH |= PIN15_IN_PULL; GPIOC->BRR = BIT15;
#define PC15_ANALOG         GPIOC->CRH &= ~PIN15_CLEAR; GPIOC->CRH |= PIN15_IN_ANALOG;
#define PC15_GET            ( GPIOC->IDR & BIT15 )
#define PC15_STATE          PC15_GET
#define PC15_HIGH           GPIOC->BSRR = BIT15;
#define PC15_LOW            GPIOC->BRR = BIT15;

#define PD0_ALT             GPIOD->CRL &= ~PIN0_CLEAR; GPIOD->CRL |= PIN0_OUT50_ALTPUSHPULL;
#define PD0_OUT             GPIOD->CRL &= ~PIN0_CLEAR; GPIOD->CRL |= PIN0_OUT50_GPPUSHPULL;
#define PD0_FLOAT           GPIOD->CRL &= ~PIN0_CLEAR; GPIOD->CRL |= PIN0_IN_FLOAT;
#define PD0_PULLUP          GPIOD->CRL &= ~PIN0_CLEAR; GPIOD->CRL |= PIN0_IN_PULL; GPIOD->BSRR = BIT0;
#define PD0_PULLDOWN        GPIOD->CRL &= ~PIN0_CLEAR; GPIOD->CRL |= PIN0_IN_PULL; GPIOD->BRR = BIT0;
#define PD0_ANALOG          GPIOD->CRL &= ~PIN0_CLEAR; GPIOD->CRL |= PIN0_IN_ANALOG;
#define PD0_GET             ( GPIOD->IDR & BIT0 )
#define PD0_STATE           PD0_GET
#define PD0_HIGH            GPIOD->BSRR = BIT0;
#define PD0_LOW             GPIOD->BRR = BIT0;

#define PD1_ALT             GPIOD->CRL &= ~PIN1_CLEAR; GPIOD->CRL |= PIN1_OUT50_ALTPUSHPULL;
#define PD1_OUT             GPIOD->CRL &= ~PIN1_CLEAR; GPIOD->CRL |= PIN1_OUT50_GPPUSHPULL;
#define PD1_FLOAT           GPIOD->CRL &= ~PIN1_CLEAR; GPIOD->CRL |= PIN1_IN_FLOAT;
#define PD1_PULLUP          GPIOD->CRL &= ~PIN1_CLEAR; GPIOD->CRL |= PIN1_IN_PULL; GPIOC->BSRR = BIT1;
#define PD1_PULLDOWN        GPIOD->CRL &= ~PIN1_CLEAR; GPIOD->CRL |= PIN1_IN_PULL; GPIOC->BRR = BIT1;
#define PD1_ANALOG          GPIOD->CRL &= ~PIN1_CLEAR; GPIOD->CRL |= PIN1_IN_ANALOG;
#define PD1_GET             ( GPIOD->IDR & BIT1 )
#define PD1_STATE           PD1_GET
#define PD1_HIGH            GPIOD->BSRR = BIT1;
#define PD1_LOW             GPIOD->BRR = BIT1;

#define PD2_ALT             GPIOD->CRL &= ~PIN2_CLEAR; GPIOD->CRL |= PIN2_OUT50_ALTPUSHPULL;
#define PD2_OUT             GPIOD->CRL &= ~PIN2_CLEAR; GPIOD->CRL |= PIN2_OUT50_GPPUSHPULL;
#define PD2_FLOAT           GPIOD->CRL &= ~PIN2_CLEAR; GPIOD->CRL |= PIN2_IN_FLOAT;
#define PD2_PULLUP          GPIOD->CRL &= ~PIN2_CLEAR; GPIOD->CRL |= PIN2_IN_PULL; GPIOD->BSRR = BIT2;
#define PD2_PULLDOWN        GPIOD->CRL &= ~PIN2_CLEAR; GPIOD->CRL |= PIN2_IN_PULL; GPIOD->BRR = BIT2;
#define PD2_ANALOG          GPIOD->CRL &= ~PIN2_CLEAR; GPIOD->CRL |= PIN2_IN_ANALOG;
#define PD2_GET             ( GPIOD->IDR & BIT2 )
#define PD2_STATE           PD2_GET
#define PD2_HIGH            GPIOD->BSRR = BIT2;
#define PD2_LOW             GPIOD->BRR = BIT2;

#define PD3_ALT             GPIOD->CRL &= ~PIN3_CLEAR; GPIOD->CRL |= PIN3_OUT50_ALTPUSHPULL;
#define PD3_OUT             GPIOD->CRL &= ~PIN3_CLEAR; GPIOD->CRL |= PIN3_OUT50_GPPUSHPULL;
#define PD3_FLOAT           GPIOD->CRL &= ~PIN3_CLEAR; GPIOD->CRL |= PIN3_IN_FLOAT;
#define PD3_PULLUP          GPIOD->CRL &= ~PIN3_CLEAR; GPIOD->CRL |= PIN3_IN_PULL; GPIOD->BSRR = BIT3;
#define PD3_PULLDOWN        GPIOD->CRL &= ~PIN3_CLEAR; GPIOD->CRL |= PIN3_IN_PULL; GPIOD->BRR = BIT3;
#define PD3_ANALOG          GPIOD->CRL &= ~PIN3_CLEAR; GPIOD->CRL |= PIN3_IN_ANALOG;
#define PD3_GET             ( GPIOD->IDR & BIT3 )
#define PD3_STATE           PD3_GET
#define PD3_HIGH            GPIOD->BSRR = BIT3;
#define PD3_LOW             GPIOD->BRR = BIT3;

#define PD4_ALT             GPIOD->CRL &= ~PIN4_CLEAR; GPIOD->CRL |= PIN4_OUT50_ALTPUSHPULL;
#define PD4_OUT             GPIOD->CRL &= ~PIN4_CLEAR; GPIOD->CRL |= PIN4_OUT50_GPPUSHPULL;
#define PD4_FLOAT           GPIOD->CRL &= ~PIN4_CLEAR; GPIOD->CRL |= PIN4_IN_FLOAT;
#define PD4_PULLUP          GPIOD->CRL &= ~PIN4_CLEAR; GPIOD->CRL |= PIN4_IN_PULL; GPIOD->BSRR = BIT4;
#define PD4_PULLDOWN        GPIOD->CRL &= ~PIN4_CLEAR; GPIOD->CRL |= PIN4_IN_PULL; GPIOD->BRR = BIT4;
#define PD4_ANALOG          GPIOD->CRL &= ~PIN4_CLEAR; GPIOD->CRL |= PIN4_IN_ANALOG;
#define PD4_GET             ( GPIOD->IDR & BIT4 )
#define PD4_STATE           PD4_GET
#define PD4_HIGH            GPIOD->BSRR = BIT4;
#define PD4_LOW             GPIOD->BRR = BIT4;

#define PD5_ALT             GPIOD->CRL &= ~PIN5_CLEAR; GPIOD->CRL |= PIN5_OUT50_ALTPUSHPULL;
#define PD5_OUT             GPIOD->CRL &= ~PIN5_CLEAR; GPIOD->CRL |= PIN5_OUT50_GPPUSHPULL;
#define PD5_FLOAT           GPIOD->CRL &= ~PIN5_CLEAR; GPIOD->CRL |= PIN5_IN_FLOAT;
#define PD5_PULLUP          GPIOD->CRL &= ~PIN5_CLEAR; GPIOD->CRL |= PIN5_IN_PULL; GPIOD->BSRR = BIT5;
#define PD5_PULLDOWN        GPIOD->CRL &= ~PIN5_CLEAR; GPIOD->CRL |= PIN5_IN_PULL; GPIOD->BRR = BIT5;
#define PD5_ANALOG          GPIOD->CRL &= ~PIN5_CLEAR; GPIOD->CRL |= PIN5_IN_ANALOG;
#define PD5_GET             ( GPIOD->IDR & BIT5 )
#define PD5_STATE           PD5_GET
#define PD5_HIGH            GPIOD->BSRR = BIT5;
#define PD5_LOW             GPIOD->BRR = BIT5;

#define PD6_ALT             GPIOD->CRL &= ~PIN6_CLEAR; GPIOD->CRL |= PIN6_OUT50_ALTPUSHPULL;
#define PD6_OUT             GPIOD->CRL &= ~PIN6_CLEAR; GPIOD->CRL |= PIN6_OUT50_GPPUSHPULL;
#define PD6_FLOAT           GPIOD->CRL &= ~PIN6_CLEAR; GPIOD->CRL |= PIN6_IN_FLOAT;
#define PD6_PULLUP          GPIOD->CRL &= ~PIN6_CLEAR; GPIOD->CRL |= PIN6_IN_PULL; GPIOD->BSRR = BIT6;
#define PD6_PULLDOWN        GPIOD->CRL &= ~PIN6_CLEAR; GPIOD->CRL |= PIN6_IN_PULL; GPIOD->BRR = BIT6;
#define PD6_ANALOG          GPIOD->CRL &= ~PIN6_CLEAR; GPIOD->CRL |= PIN6_IN_ANALOG;
#define PD6_GET             ( GPIOD->IDR & BIT6 )
#define PD6_STATE           PD6_GET
#define PD6_HIGH            GPIOD->BSRR = BIT6;
#define PD6_LOW             GPIOD->BRR = BIT6;

#define PD7_ALT             GPIOD->CRL &= ~PIN7_CLEAR; GPIOD->CRL |= PIN7_OUT50_ALTPUSHPULL;
#define PD7_OUT             GPIOD->CRL &= ~PIN7_CLEAR; GPIOD->CRL |= PIN7_OUT50_GPPUSHPULL;
#define PD7_FLOAT           GPIOD->CRL &= ~PIN7_CLEAR; GPIOD->CRL |= PIN7_IN_FLOAT;
#define PD7_PULLUP          GPIOD->CRL &= ~PIN7_CLEAR; GPIOD->CRL |= PIN7_IN_PULL; GPIOD->BSRR = BIT7;
#define PD7_PULLDOWN        GPIOD->CRL &= ~PIN7_CLEAR; GPIOD->CRL |= PIN7_IN_PULL; GPIOD->BRR = BIT7;
#define PD7_ANALOG          GPIOD->CRL &= ~PIN7_CLEAR; GPIOD->CRL |= PIN7_IN_ANALOG;
#define PD7_GET             ( GPIOD->IDR & BIT7 )
#define PD7_STATE           PD7_GET
#define PD7_HIGH            GPIOD->BSRR = BIT7;
#define PD7_LOW             GPIOD->BRR = BIT7;

#define PD8_ALT             GPIOD->CRH &= ~PIN8_CLEAR; GPIOD->CRH |= PIN8_OUT50_ALTPUSHPULL;
#define PD8_OUT             GPIOD->CRH &= ~PIN8_CLEAR; GPIOD->CRH |= PIN8_OUT50_GPPUSHPULL;
#define PD8_FLOAT           GPIOD->CRH &= ~PIN8_CLEAR; GPIOD->CRH |= PIN8_IN_FLOAT;
#define PD8_PULLUP          GPIOD->CRH &= ~PIN8_CLEAR; GPIOD->CRH |= PIN8_IN_PULL; GPIOD->BSRR = BIT8;
#define PD8_PULLDOWN        GPIOD->CRH &= ~PIN8_CLEAR; GPIOD->CRH |= PIN8_IN_PULL; GPIOD->BRR = BIT8;
#define PD8_ANALOG          GPIOD->CRH &= ~PIN8_CLEAR; GPIOD->CRH |= PIN8_IN_ANALOG;
#define PD8_GET             ( GPIOD->IDR & BIT8 )
#define PD8_STATE           PD8_GET
#define PD8_HIGH            GPIOD->BSRR = BIT8;
#define PD8_LOW             GPIOD->BRR = BIT8;

#define PD9_ALT             GPIOD->CRH &= ~PIN9_CLEAR; GPIOD->CRH |= PIN9_OUT50_ALTPUSHPULL;
#define PD9_OUT             GPIOD->CRH &= ~PIN9_CLEAR; GPIOD->CRH |= PIN9_OUT50_GPPUSHPULL;
#define PD9_FLOAT           GPIOD->CRH &= ~PIN9_CLEAR; GPIOD->CRH |= PIN9_IN_FLOAT;
#define PD9_PULLUP          GPIOD->CRH &= ~PIN9_CLEAR; GPIOD->CRH |= PIN9_IN_PULL; GPIOD->BSRR = BIT9;
#define PD9_PULLDOWN        GPIOD->CRH &= ~PIN9_CLEAR; GPIOD->CRH |= PIN9_IN_PULL; GPIOD->BRR = BIT9;
#define PD9_ANALOG          GPIOD->CRH &= ~PIN9_CLEAR; GPIOD->CRH |= PIN9_IN_ANALOG;
#define PD9_GET             ( GPIOD->IDR & BIT9 )
#define PD9_STATE           PD9_GET
#define PD9_HIGH            GPIOD->BSRR = BIT9;
#define PD9_LOW             GPIOD->BRR = BIT9;

#define PD10_ALT            GPIOD->CRH &= ~PIN10_CLEAR; GPIOD->CRH |= PIN10_OUT50_ALTPUSHPULL;
#define PD10_OUT            GPIOD->CRH &= ~PIN10_CLEAR; GPIOD->CRH |= PIN10_OUT50_GPPUSHPULL;
#define PD10_FLOAT          GPIOD->CRH &= ~PIN10_CLEAR; GPIOD->CRH |= PIN10_IN_FLOAT;
#define PD10_PULLUP         GPIOD->CRH &= ~PIN10_CLEAR; GPIOD->CRH |= PIN10_IN_PULL; GPIOD->BSRR = BIT10;
#define PD10_PULLDOWN       GPIOD->CRH &= ~PIN10_CLEAR; GPIOD->CRH |= PIN10_IN_PULL; GPIOD->BRR = BIT10;
#define PD10_ANALOG         GPIOD->CRH &= ~PIN10_CLEAR; GPIOD->CRH |= PIN10_IN_ANALOG;
#define PD10_GET            ( GPIOD->IDR & BIT10 )
#define PD10_HIGH           GPIOD->BSRR = BIT10;
#define PD10_LOW            GPIOD->BRR = BIT10;
                            
#define PD11_ALT            GPIOD->CRH &= ~PIN11_CLEAR; GPIOD->CRH |= PIN11_OUT50_ALTPUSHPULL;
#define PD11_OUT            GPIOD->CRH &= ~PIN11_CLEAR; GPIOD->CRH |= PIN11_OUT50_GPPUSHPULL;
#define PD11_FLOAT          GPIOD->CRH &= ~PIN11_CLEAR; GPIOD->CRH |= PIN11_IN_FLOAT;
#define PD11_PULLUP         GPIOD->CRH &= ~PIN11_CLEAR; GPIOD->CRH |= PIN11_IN_PULL; GPIOD->BSRR = BIT11;
#define PD11_PULLDOWN       GPIOD->CRH &= ~PIN11_CLEAR; GPIOD->CRH |= PIN11_IN_PULL; GPIOD->BRR = BIT11;
#define PD11_ANALOG         GPIOD->CRH &= ~PIN11_CLEAR; GPIOD->CRH |= PIN11_IN_ANALOG;
#define PD11_GET            ( GPIOD->IDR & BIT11 )
#define PD11_STATE          PD11_GET
#define PD11_HIGH           GPIOD->BSRR = BIT11;
#define PD11_LOW            GPIOD->BRR = BIT11;

#define PD12_ALT            GPIOD->CRH &= ~PIN12_CLEAR; GPIOD->CRH |= PIN12_OUT50_ALTPUSHPULL;
#define PD12_OUT            GPIOD->CRH &= ~PIN12_CLEAR; GPIOD->CRH |= PIN12_OUT50_GPPUSHPULL;
#define PD12_FLOAT          GPIOD->CRH &= ~PIN12_CLEAR; GPIOD->CRH |= PIN12_IN_FLOAT;
#define PD12_PULLUP         GPIOD->CRH &= ~PIN12_CLEAR; GPIOD->CRH |= PIN12_IN_PULL; GPIOD->BSRR = BIT12;
#define PD12_PULLDOWN       GPIOD->CRH &= ~PIN12_CLEAR; GPIOD->CRH |= PIN12_IN_PULL; GPIOD->BRR = BIT12;
#define PD12_ANALOG         GPIOD->CRH &= ~PIN12_CLEAR; GPIOD->CRH |= PIN12_IN_ANALOG;
#define PD12_GET            ( GPIOD->IDR & BIT12 )
#define PD12_STATE          PD12_GET
#define PD12_HIGH           GPIOD->BSRR = BIT12;
#define PD12_LOW            GPIOD->BRR = BIT12;

#define PD13_ALT            GPIOD->CRH &= ~PIN13_CLEAR; GPIOD->CRH |= PIN13_OUT50_ALTPUSHPULL;
#define PD13_OUT            GPIOD->CRH &= ~PIN13_CLEAR; GPIOD->CRH |= PIN13_OUT50_GPPUSHPULL;
#define PD13_FLOAT          GPIOD->CRH &= ~PIN13_CLEAR; GPIOD->CRH |= PIN13_IN_FLOAT;
#define PD13_PULLUP         GPIOD->CRH &= ~PIN13_CLEAR; GPIOD->CRH |= PIN13_IN_PULL; GPIOD->BSRR = BIT13;
#define PD13_PULLDOWN       GPIOD->CRH &= ~PIN13_CLEAR; GPIOD->CRH |= PIN13_IN_PULL; GPIOD->BRR = BIT13;
#define PD13_ANALOG         GPIOD->CRH &= ~PIN13_CLEAR; GPIOD->CRH |= PIN13_IN_ANALOG;
#define PD13_GET            ( GPIOD->IDR & BIT13 )
#define PD13_STATE          PD13_GET
#define PD13_HIGH           GPIOD->BSRR = BIT13;
#define PD13_LOW            GPIOD->BRR = BIT13;

#define PD14_ALT            GPIOD->CRH &= ~PIN14_CLEAR; GPIOD->CRH |= PIN14_OUT50_ALTPUSHPULL;
#define PD14_OUT            GPIOD->CRH &= ~PIN14_CLEAR; GPIOD->CRH |= PIN14_OUT50_GPPUSHPULL;
#define PD14_FLOAT          GPIOD->CRH &= ~PIN14_CLEAR; GPIOD->CRH |= PIN14_IN_FLOAT;
#define PD14_PULLUP         GPIOD->CRH &= ~PIN14_CLEAR; GPIOD->CRH |= PIN14_IN_PULL; GPIOD->BSRR = BIT14;
#define PD14_PULLDOWN       GPIOD->CRH &= ~PIN14_CLEAR; GPIOD->CRH |= PIN14_IN_PULL; GPIOD->BRR = BIT14;
#define PD14_ANALOG         GPIOD->CRH &= ~PIN14_CLEAR; GPIOD->CRH |= PIN14_IN_ANALOG;
#define PD14_GET            ( GPIOD->IDR & BIT14 )
#define PD14_STATE          PD14_GET
#define PD14_HIGH           GPIOD->BSRR = BIT14;
#define PD14_LOW            GPIOD->BRR = BIT14;

#define PD15_ALT            GPIOD->CRH &= ~PIN15_CLEAR; GPIOD->CRH |= PIN15_OUT50_ALTPUSHPULL;
#define PD15_OUT            GPIOD->CRH &= ~PIN15_CLEAR; GPIOD->CRH |= PIN15_OUT50_GPPUSHPULL;
#define PD15_FLOAT          GPIOD->CRH &= ~PIN15_CLEAR; GPIOD->CRH |= PIN15_IN_FLOAT;
#define PD15_PULLUP         GPIOD->CRH &= ~PIN15_CLEAR; GPIOD->CRH |= PIN15_IN_PULL; GPIOD->BSRR = BIT15;
#define PD15_PULLDOWN       GPIOD->CRH &= ~PIN15_CLEAR; GPIOD->CRH |= PIN15_IN_PULL; GPIOD->BRR = BIT15;
#define PD15_ANALOG         GPIOD->CRH &= ~PIN15_CLEAR; GPIOD->CRH |= PIN15_IN_ANALOG;
#define PD15_GET            ( GPIOD->IDR & BIT15 )
#define PD15_STATE          PD15_GET
#define PD15_HIGH           GPIOD->BSRR = BIT15;
#define PD15_LOW            GPIOD->BRR = BIT15;

#ifdef GPIOE
#define PE0_ALT             GPIOE->CRL &= ~PIN0_CLEAR; GPIOE->CRL |= PIN0_OUT50_ALTPUSHPULL;
#define PE0_OUT             GPIOE->CRL &= ~PIN0_CLEAR; GPIOE->CRL |= PIN0_OUT50_GPPUSHPULL;
#define PE0_FLOAT           GPIOE->CRL &= ~PIN0_CLEAR; GPIOE->CRL |= PIN0_IN_FLOAT;
#define PE0_PULLUP          GPIOE->CRL &= ~PIN0_CLEAR; GPIOE->CRL |= PIN0_IN_PULL; GPIOE->BSRR = BIT0;
#define PE0_PULLDOWN        GPIOE->CRL &= ~PIN0_CLEAR; GPIOE->CRL |= PIN0_IN_PULL; GPIOE->BRR = BIT0;
#define PE0_ANALOG          GPIOE->CRL &= ~PIN0_CLEAR; GPIOE->CRL |= PIN0_IN_ANALOG;
#define PE0_GET             ( GPIOE->IDR & BIT0 )
#define PE0_STATE           PE0_GET
#define PE0_HIGH            GPIOE->BSRR = BIT0;
#define PE0_LOW             GPIOE->BRR = BIT0;

#define PE1_ALT             GPIOE->CRL &= ~PIN1_CLEAR; GPIOE->CRL |= PIN1_OUT50_ALTPUSHPULL;
#define PE1_OUT             GPIOE->CRL &= ~PIN1_CLEAR; GPIOE->CRL |= PIN1_OUT50_GPPUSHPULL;
#define PE1_FLOAT           GPIOE->CRL &= ~PIN1_CLEAR; GPIOE->CRL |= PIN1_IN_FLOAT;
#define PE1_PULLUP          GPIOE->CRL &= ~PIN1_CLEAR; GPIOE->CRL |= PIN1_IN_PULL; GPIOE->BSRR = BIT1;
#define PE1_PULLDOWN        GPIOE->CRL &= ~PIN1_CLEAR; GPIOE->CRL |= PIN1_IN_PULL; GPIOE->BRR = BIT1;
#define PE1_ANALOG          GPIOE->CRL &= ~PIN1_CLEAR; GPIOE->CRL |= PIN1_IN_ANALOG;
#define PE1_GET             ( GPIOE->IDR & BIT1 )
#define PE1_STATE           PE1_GET
#define PE1_HIGH            GPIOE->BSRR = BIT1;
#define PE1_LOW             GPIOE->BRR = BIT1;

#define PE2_ALT             GPIOE->CRL &= ~PIN2_CLEAR; GPIOE->CRL |= PIN2_OUT50_ALTPUSHPULL;
#define PE2_OUT             GPIOE->CRL &= ~PIN2_CLEAR; GPIOE->CRL |= PIN2_OUT50_GPPUSHPULL;
#define PE2_FLOAT           GPIOE->CRL &= ~PIN2_CLEAR; GPIOE->CRL |= PIN2_IN_FLOAT;
#define PE2_PULLUP          GPIOE->CRL &= ~PIN2_CLEAR; GPIOE->CRL |= PIN2_IN_PULL; GPIOE->BSRR = BIT2;
#define PE2_PULLDOWN        GPIOE->CRL &= ~PIN2_CLEAR; GPIOE->CRL |= PIN2_IN_PULL; GPIOE->BRR = BIT2;
#define PE2_ANALOG          GPIOE->CRL &= ~PIN2_CLEAR; GPIOE->CRL |= PIN2_IN_ANALOG;
#define PE2_GET             ( GPIOE->IDR & BIT2 )
#define PE2_STATE           PE2_GET
#define PE2_HIGH            GPIOE->BSRR = BIT2;
#define PE2_LOW             GPIOE->BRR = BIT2;

#define PE3_ALT             GPIOE->CRL &= ~PIN3_CLEAR; GPIOE->CRL |= PIN3_OUT50_ALTPUSHPULL;
#define PE3_OUT             GPIOE->CRL &= ~PIN3_CLEAR; GPIOE->CRL |= PIN3_OUT50_GPPUSHPULL;
#define PE3_FLOAT           GPIOE->CRL &= ~PIN3_CLEAR; GPIOE->CRL |= PIN3_IN_FLOAT;
#define PE3_PULLUP          GPIOE->CRL &= ~PIN3_CLEAR; GPIOE->CRL |= PIN3_IN_PULL; GPIOE->BSRR = BIT3;
#define PE3_PULLDOWN        GPIOE->CRL &= ~PIN3_CLEAR; GPIOE->CRL |= PIN3_IN_PULL; GPIOE->BRR = BIT3;
#define PE3_ANALOG          GPIOE->CRL &= ~PIN3_CLEAR; GPIOE->CRL |= PIN3_IN_ANALOG;
#define PE3_GET             ( GPIOE->IDR & BIT3 )
#define PE3_STATE           PE3_GET
#define PE3_HIGH            GPIOE->BSRR = BIT3;
#define PE3_LOW             GPIOE->BRR = BIT3;

#define PE4_ALT             GPIOE->CRL &= ~PIN4_CLEAR; GPIOE->CRL |= PIN4_OUT50_ALTPUSHPULL;
#define PE4_OUT             GPIOE->CRL &= ~PIN4_CLEAR; GPIOE->CRL |= PIN4_OUT50_GPPUSHPULL;
#define PE4_FLOAT           GPIOE->CRL &= ~PIN4_CLEAR; GPIOE->CRL |= PIN4_IN_FLOAT;
#define PE4_PULLUP          GPIOE->CRL &= ~PIN4_CLEAR; GPIOE->CRL |= PIN4_IN_PULL; GPIOE->BSRR = BIT4;
#define PE4_PULLDOWN        GPIOE->CRL &= ~PIN4_CLEAR; GPIOE->CRL |= PIN4_IN_PULL; GPIOE->BRR = BIT4;
#define PE4_ANALOG          GPIOE->CRL &= ~PIN4_CLEAR; GPIOE->CRL |= PIN4_IN_ANALOG;
#define PE4_GET             ( GPIOE->IDR & BIT4 )
#define PE4_STATE           PE4_GET
#define PE4_HIGH            GPIOE->BSRR = BIT4;
#define PE4_LOW             GPIOE->BRR = BIT4;

#define PE5_ALT             GPIOE->CRL &= ~PIN5_CLEAR; GPIOE->CRL |= PIN5_OUT50_ALTPUSHPULL;
#define PE5_OUT             GPIOE->CRL &= ~PIN5_CLEAR; GPIOE->CRL |= PIN5_OUT50_GPPUSHPULL;
#define PE5_FLOAT           GPIOE->CRL &= ~PIN5_CLEAR; GPIOE->CRL |= PIN5_IN_FLOAT;
#define PE5_PULLUP          GPIOE->CRL &= ~PIN5_CLEAR; GPIOE->CRL |= PIN5_IN_PULL; GPIOE->BSRR = BIT5;
#define PE5_PULLDOWN        GPIOE->CRL &= ~PIN5_CLEAR; GPIOE->CRL |= PIN5_IN_PULL; GPIOE->BRR = BIT5;
#define PE5_ANALOG          GPIOE->CRL &= ~PIN5_CLEAR; GPIOE->CRL |= PIN5_IN_ANALOG;
#define PE5_GET             ( GPIOE->IDR & BIT5 )
#define PE5_STATE           PE5_GET
#define PE5_HIGH            GPIOE->BSRR = BIT5;
#define PE5_LOW             GPIOE->BRR = BIT5;

#define PE6_ALT             GPIOE->CRL &= ~PIN6_CLEAR; GPIOE->CRL |= PIN6_OUT50_ALTPUSHPULL;
#define PE6_OUT             GPIOE->CRL &= ~PIN6_CLEAR; GPIOE->CRL |= PIN6_OUT50_GPPUSHPULL;
#define PE6_FLOAT           GPIOE->CRL &= ~PIN6_CLEAR; GPIOE->CRL |= PIN6_IN_FLOAT;
#define PE6_PULLUP          GPIOE->CRL &= ~PIN6_CLEAR; GPIOE->CRL |= PIN6_IN_PULL; GPIOE->BSRR = BIT6;
#define PE6_PULLDOWN        GPIOE->CRL &= ~PIN6_CLEAR; GPIOE->CRL |= PIN6_IN_PULL; GPIOE->BRR = BIT6;
#define PE6_ANALOG          GPIOE->CRL &= ~PIN6_CLEAR; GPIOE->CRL |= PIN6_IN_ANALOG;
#define PE6_GET             ( GPIOE->IDR & BIT6 )
#define PE6_STATE           PE6_GET
#define PE6_HIGH            GPIOE->BSRR = BIT6;
#define PE6_LOW             GPIOE->BRR = BIT6;

#define PE7_ALT             GPIOE->CRL &= ~PIN7_CLEAR; GPIOE->CRL |= PIN7_OUT50_ALTPUSHPULL;
#define PE7_OUT             GPIOE->CRL &= ~PIN7_CLEAR; GPIOE->CRL |= PIN7_OUT50_GPPUSHPULL;
#define PE7_FLOAT           GPIOE->CRL &= ~PIN7_CLEAR; GPIOE->CRL |= PIN7_IN_FLOAT;
#define PE7_PULLUP          GPIOE->CRL &= ~PIN7_CLEAR; GPIOE->CRL |= PIN7_IN_PULL; GPIOE->BSRR = BIT7;
#define PE7_PULLDOWN        GPIOE->CRL &= ~PIN7_CLEAR; GPIOE->CRL |= PIN7_IN_PULL; GPIOE->BRR = BIT7;
#define PE7_ANALOG          GPIOE->CRL &= ~PIN7_CLEAR; GPIOE->CRL |= PIN7_IN_ANALOG;
#define PE7_GET             ( GPIOE->IDR & BIT7 )
#define PE7_STATE           PE7_GET
#define PE7_HIGH            GPIOE->BSRR = BIT7;
#define PE7_LOW             GPIOE->BRR = BIT7;

#define PE8_ALT             GPIOE->CRH &= ~PIN8_CLEAR; GPIOE->CRH |= PIN8_OUT50_ALTPUSHPULL;
#define PE8_OUT             GPIOE->CRH &= ~PIN8_CLEAR; GPIOE->CRH |= PIN8_OUT50_GPPUSHPULL;
#define PE8_FLOAT           GPIOE->CRH &= ~PIN8_CLEAR; GPIOE->CRH |= PIN8_IN_FLOAT;
#define PE8_PULLUP          GPIOE->CRH &= ~PIN8_CLEAR; GPIOE->CRH |= PIN8_IN_PULL; GPIOE->BSRR = BIT8;
#define PE8_PULLDOWN        GPIOE->CRH &= ~PIN8_CLEAR; GPIOE->CRH |= PIN8_IN_PULL; GPIOE->BRR = BIT8;
#define PE8_ANALOG          GPIOE->CRH &= ~PIN8_CLEAR; GPIOE->CRH |= PIN8_IN_ANALOG;
#define PE8_GET             ( GPIOE->IDR & BIT8 )
#define PE8_STATE           PE8_GET
#define PE8_HIGH            GPIOE->BSRR = BIT8;
#define PE8_LOW             GPIOE->BRR = BIT8;

#define PE9_ALT             GPIOE->CRH &= ~PIN9_CLEAR; GPIOE->CRH |= PIN9_OUT50_ALTPUSHPULL;
#define PE9_OUT             GPIOE->CRH &= ~PIN9_CLEAR; GPIOE->CRH |= PIN9_OUT50_GPPUSHPULL;
#define PE9_FLOAT           GPIOE->CRH &= ~PIN9_CLEAR; GPIOE->CRH |= PIN9_IN_FLOAT;
#define PE9_PULLUP          GPIOE->CRH &= ~PIN9_CLEAR; GPIOE->CRH |= PIN9_IN_PULL; GPIOE->BSRR = BIT9;
#define PE9_PULLDOWN        GPIOE->CRH &= ~PIN9_CLEAR; GPIOE->CRH |= PIN9_IN_PULL; GPIOE->BRR = BIT9;
#define PE9_ANALOG          GPIOE->CRH &= ~PIN9_CLEAR; GPIOE->CRH |= PIN9_IN_ANALOG;
#define PE9_GET             ( GPIOE->IDR & BIT9 )
#define PE9_STATE           PE9_GET
#define PE9_HIGH            GPIOE->BSRR = BIT9;
#define PE9_LOW             GPIOE->BRR = BIT9;

#define PE10_ALT            GPIOE->CRH &= ~PIN10_CLEAR; GPIOE->CRH |= PIN10_OUT50_ALTPUSHPULL;
#define PE10_OUT            GPIOE->CRH &= ~PIN10_CLEAR; GPIOE->CRH |= PIN10_OUT50_GPPUSHPULL;
#define PE10_FLOAT          GPIOE->CRH &= ~PIN10_CLEAR; GPIOE->CRH |= PIN10_IN_FLOAT;
#define PE10_PULLUP         GPIOE->CRH &= ~PIN10_CLEAR; GPIOE->CRH |= PIN10_IN_PULL; GPIOE->BSRR = BIT10;
#define PE10_PULLDOWN       GPIOE->CRH &= ~PIN10_CLEAR; GPIOE->CRH |= PIN10_IN_PULL; GPIOE->BRR = BIT10;
#define PE10_ANALOG         GPIOE->CRH &= ~PIN10_CLEAR; GPIOE->CRH |= PIN10_IN_ANALOG;
#define PE10_GET            ( GPIOE->IDR & BIT10 )
#define PE10_STATE          PE10_GET
#define PE10_HIGH           GPIOE->BSRR = BIT10;
#define PE10_LOW            GPIOE->BRR = BIT10;

#define PE11_ALT            GPIOE->CRH &= ~PIN11_CLEAR; GPIOE->CRH |= PIN11_OUT50_ALTPUSHPULL;
#define PE11_OUT            GPIOE->CRH &= ~PIN11_CLEAR; GPIOE->CRH |= PIN11_OUT50_GPPUSHPULL;
#define PE11_FLOAT          GPIOE->CRH &= ~PIN11_CLEAR; GPIOE->CRH |= PIN11_IN_FLOAT;
#define PE11_PULLUP         GPIOE->CRH &= ~PIN11_CLEAR; GPIOE->CRH |= PIN11_IN_PULL; GPIOE->BSRR = BIT11;
#define PE11_PULLDOWN       GPIOE->CRH &= ~PIN11_CLEAR; GPIOE->CRH |= PIN11_IN_PULL; GPIOE->BRR = BIT11;
#define PE11_ANALOG         GPIOE->CRH &= ~PIN11_CLEAR; GPIOE->CRH |= PIN11_IN_ANALOG;
#define PE11_GET            ( GPIOE->IDR & BIT11 )
#define PE11_STATE          PE11_GET
#define PE11_HIGH           GPIOE->BSRR = BIT11;
#define PE11_LOW            GPIOE->BRR = BIT11;

#define PE12_ALT            GPIOE->CRH &= ~PIN12_CLEAR; GPIOE->CRH |= PIN12_OUT50_ALTPUSHPULL;
#define PE12_OUT            GPIOE->CRH &= ~PIN12_CLEAR; GPIOE->CRH |= PIN12_OUT50_GPPUSHPULL;
#define PE12_FLOAT          GPIOE->CRH &= ~PIN12_CLEAR; GPIOE->CRH |= PIN12_IN_FLOAT;
#define PE12_PULLUP         GPIOE->CRH &= ~PIN12_CLEAR; GPIOE->CRH |= PIN12_IN_PULL; GPIOE->BSRR = BIT12;
#define PE12_PULLDOWN       GPIOE->CRH &= ~PIN12_CLEAR; GPIOE->CRH |= PIN12_IN_PULL; GPIOE->BRR = BIT12;
#define PE12_ANALOG         GPIOE->CRH &= ~PIN12_CLEAR; GPIOE->CRH |= PIN12_IN_ANALOG;
#define PE12_GET            ( GPIOE->IDR & BIT12 )
#define PE12_STATE          PE12_GET
#define PE12_HIGH           GPIOE->BSRR = BIT12;
#define PE12_LOW            GPIOE->BRR = BIT12;

#define PE13_ALT            GPIOE->CRH &= ~PIN13_CLEAR; GPIOE->CRH |= PIN13_OUT50_ALTPUSHPULL;
#define PE13_OUT            GPIOE->CRH &= ~PIN13_CLEAR; GPIOE->CRH |= PIN13_OUT50_GPPUSHPULL;
#define PE13_FLOAT          GPIOE->CRH &= ~PIN13_CLEAR; GPIOE->CRH |= PIN13_IN_FLOAT;
#define PE13_PULLUP         GPIOE->CRH &= ~PIN13_CLEAR; GPIOE->CRH |= PIN13_IN_PULL; GPIOE->BSRR = BIT13;
#define PE13_PULLDOWN       GPIOE->CRH &= ~PIN13_CLEAR; GPIOE->CRH |= PIN13_IN_PULL; GPIOE->BRR = BIT13;
#define PE13_ANALOG         GPIOE->CRH &= ~PIN13_CLEAR; GPIOE->CRH |= PIN13_IN_ANALOG;
#define PE13_GET            ( GPIOE->IDR & BIT13 )
#define PE13_HIGH           GPIOE->BSRR = BIT13;
#define PE13_LOW            GPIOE->BRR = BIT13;

#define PE14_ALT            GPIOE->CRH &= ~PIN14_CLEAR; GPIOE->CRH |= PIN14_OUT50_ALTPUSHPULL;
#define PE14_OUT            GPIOE->CRH &= ~PIN14_CLEAR; GPIOE->CRH |= PIN14_OUT50_GPPUSHPULL;
#define PE14_FLOAT          GPIOE->CRH &= ~PIN14_CLEAR; GPIOE->CRH |= PIN14_IN_FLOAT;
#define PE14_PULLUP         GPIOE->CRH &= ~PIN14_CLEAR; GPIOE->CRH |= PIN14_IN_PULL; GPIOE->BSRR = BIT14;
#define PE14_PULLDOWN       GPIOE->CRH &= ~PIN14_CLEAR; GPIOE->CRH |= PIN14_IN_PULL; GPIOE->BRR = BIT14;
#define PE14_ANALOG         GPIOE->CRH &= ~PIN14_CLEAR; GPIOE->CRH |= PIN14_IN_ANALOG;
#define PE14_GET            ( GPIOE->IDR & BIT14 )
#define PE14_STATE          PE14_GET
#define PE14_HIGH           GPIOE->BSRR = BIT14;
#define PE14_LOW            GPIOE->BRR = BIT14;

#define PE15_ALT            GPIOE->CRH &= ~PIN15_CLEAR; GPIOE->CRH |= PIN15_OUT50_ALTPUSHPULL;
#define PE15_OUT            GPIOE->CRH &= ~PIN15_CLEAR; GPIOE->CRH |= PIN15_OUT50_GPPUSHPULL;
#define PE15_FLOAT          GPIOE->CRH &= ~PIN15_CLEAR; GPIOE->CRH |= PIN15_IN_FLOAT;
#define PE15_PULLUP         GPIOE->CRH &= ~PIN15_CLEAR; GPIOE->CRH |= PIN15_IN_PULL; GPIOE->BSRR = BIT15;
#define PE15_PULLDOWN       GPIOE->CRH &= ~PIN15_CLEAR; GPIOE->CRH |= PIN15_IN_PULL; GPIOE->BRR = BIT15;
#define PE15_ANALOG         GPIOE->CRH &= ~PIN15_CLEAR; GPIOE->CRH |= PIN15_IN_ANALOG;
#define PE15_GET            ( GPIOE->IDR & BIT15 )
#define PE15_STATE          PE15_GET
#define PE15_HIGH           GPIOE->BSRR = BIT15;
#define PE15_LOW            GPIOE->BRR = BIT15;
#endif // GPIOE

#ifdef GPIOF
#define PF0_ALT             GPIOF->CRL &= ~PIN0_CLEAR; GPIOF->CRL |= PIN0_OUT50_ALTPUSHPULL;
#define PF0_OUT             GPIOF->CRL &= ~PIN0_CLEAR; GPIOF->CRL |= PIN0_OUT50_GPPUSHPULL;
#define PF0_FLOAT           GPIOF->CRL &= ~PIN0_CLEAR; GPIOF->CRL |= PIN0_IN_FLOAT;
#define PF0_PULLUP          GPIOF->CRL &= ~PIN0_CLEAR; GPIOF->CRL |= PIN0_IN_PULL; GPIOF->BSRR = BIT0;
#define PF0_PULLDOWN        GPIOF->CRL &= ~PIN0_CLEAR; GPIOF->CRL |= PIN0_IN_PULL; GPIOF->BRR = BIT0;
#define PF0_ANALOG          GPIOF->CRL &= ~PIN0_CLEAR; GPIOF->CRL |= PIN0_IN_ANALOG;
#define PF0_GET             ( GPIOF->IDR & BIT0 )
#define PF0_STATE           PF0_GET
#define PF0_HIGH            GPIOF->BSRR = BIT0;
#define PF0_LOW             GPIOF->BRR = BIT0;

#define PF1_ALT             GPIOF->CRL &= ~PIN1_CLEAR; GPIOF->CRL |= PIN1_OUT50_ALTPUSHPULL;
#define PF1_OUT             GPIOF->CRL &= ~PIN1_CLEAR; GPIOF->CRL |= PIN1_OUT50_GPPUSHPULL;
#define PF1_FLOAT           GPIOF->CRL &= ~PIN1_CLEAR; GPIOF->CRL |= PIN1_IN_FLOAT;
#define PF1_PULLUP          GPIOF->CRL &= ~PIN1_CLEAR; GPIOF->CRL |= PIN1_IN_PULL; GPIOF->BSRR = BIT1;
#define PF1_PULLDOWN        GPIOF->CRL &= ~PIN1_CLEAR; GPIOF->CRL |= PIN1_IN_PULL; GPIOF->BRR = BIT1;
#define PF1_ANALOG          GPIOF->CRL &= ~PIN1_CLEAR; GPIOF->CRL |= PIN1_IN_ANALOG;
#define PF1_GET             ( GPIOF->IDR & BIT1 )
#define PF1_STATE           PF1_GET
#define PF1_HIGH            GPIOF->BSRR = BIT1;
#define PF1_LOW             GPIOF->BRR = BIT1;

#define PF2_ALT             GPIOF->CRL &= ~PIN2_CLEAR; GPIOF->CRL |= PIN2_OUT50_ALTPUSHPULL;
#define PF2_OUT             GPIOF->CRL &= ~PIN2_CLEAR; GPIOF->CRL |= PIN2_OUT50_GPPUSHPULL;
#define PF2_FLOAT           GPIOF->CRL &= ~PIN2_CLEAR; GPIOF->CRL |= PIN2_IN_FLOAT;
#define PF2_PULLUP          GPIOF->CRL &= ~PIN2_CLEAR; GPIOF->CRL |= PIN2_IN_PULL; GPIOF->BSRR = BIT2;
#define PF2_PULLDOWN        GPIOF->CRL &= ~PIN2_CLEAR; GPIOF->CRL |= PIN2_IN_PULL; GPIOF->BRR = BIT2;
#define PF2_ANALOG          GPIOF->CRL &= ~PIN2_CLEAR; GPIOF->CRL |= PIN2_IN_ANALOG;
#define PF2_GET             ( GPIOF->IDR & BIT2 )
#define PF2_STATE           PF2_GET
#define PF2_HIGH            GPIOF->BSRR = BIT2;
#define PF2_LOW             GPIOF->BRR = BIT2;

#define PF3_ALT             GPIOF->CRL &= ~PIN3_CLEAR; GPIOF->CRL |= PIN3_OUT50_ALTPUSHPULL;
#define PF3_OUT             GPIOF->CRL &= ~PIN3_CLEAR; GPIOF->CRL |= PIN3_OUT50_GPPUSHPULL;
#define PF3_FLOAT           GPIOF->CRL &= ~PIN3_CLEAR; GPIOF->CRL |= PIN3_IN_FLOAT;
#define PF3_PULLUP          GPIOF->CRL &= ~PIN3_CLEAR; GPIOF->CRL |= PIN3_IN_PULL; GPIOF->BSRR = BIT3;
#define PF3_PULLDOWN        GPIOF->CRL &= ~PIN3_CLEAR; GPIOF->CRL |= PIN3_IN_PULL; GPIOF->BRR = BIT3;
#define PF3_ANALOG          GPIOF->CRL &= ~PIN3_CLEAR; GPIOF->CRL |= PIN3_IN_ANALOG;
#define PF3_GET             ( GPIOF->IDR & BIT3 )
#define PF3_STATE           PF3_GET
#define PF3_HIGH            GPIOF->BSRR = BIT3;
#define PF3_LOW             GPIOF->BRR = BIT3;

#define PF4_ALT             GPIOF->CRL &= ~PIN4_CLEAR; GPIOF->CRL |= PIN4_OUT50_ALTPUSHPULL;
#define PF4_OUT             GPIOF->CRL &= ~PIN4_CLEAR; GPIOF->CRL |= PIN4_OUT50_GPPUSHPULL;
#define PF4_FLOAT           GPIOF->CRL &= ~PIN4_CLEAR; GPIOF->CRL |= PIN4_IN_FLOAT;
#define PF4_PULLUP          GPIOF->CRL &= ~PIN4_CLEAR; GPIOF->CRL |= PIN4_IN_PULL; GPIOF->BSRR = BIT4;
#define PF4_PULLDOWN        GPIOF->CRL &= ~PIN4_CLEAR; GPIOF->CRL |= PIN4_IN_PULL; GPIOF->BRR = BIT4;
#define PF4_ANALOG          GPIOF->CRL &= ~PIN4_CLEAR; GPIOF->CRL |= PIN4_IN_ANALOG;
#define PF4_GET             ( GPIOF->IDR & BIT4 )
#define PF4_STATE           PF4_GET
#define PF4_HIGH            GPIOF->BSRR = BIT4;
#define PF4_LOW             GPIOF->BRR = BIT4;

#define PF5_ALT             GPIOF->CRL &= ~PIN5_CLEAR; GPIOF->CRL |= PIN5_OUT50_ALTPUSHPULL;
#define PF5_OUT             GPIOF->CRL &= ~PIN5_CLEAR; GPIOF->CRL |= PIN5_OUT50_GPPUSHPULL;
#define PF5_FLOAT           GPIOF->CRL &= ~PIN5_CLEAR; GPIOF->CRL |= PIN5_IN_FLOAT;
#define PF5_PULLUP          GPIOF->CRL &= ~PIN5_CLEAR; GPIOF->CRL |= PIN5_IN_PULL; GPIOF->BSRR = BIT5;
#define PF5_PULLDOWN        GPIOF->CRL &= ~PIN5_CLEAR; GPIOF->CRL |= PIN5_IN_PULL; GPIOF->BRR = BIT5;
#define PF5_ANALOG          GPIOF->CRL &= ~PIN5_CLEAR; GPIOF->CRL |= PIN5_IN_ANALOG;
#define PF5_GET             ( GPIOF->IDR & BIT5 )
#define PF5_STATE           PF5_GET
#define PF5_HIGH            GPIOF->BSRR = BIT5;
#define PF5_LOW             GPIOF->BRR = BIT5;

#define PF6_ALT             GPIOF->CRL &= ~PIN6_CLEAR; GPIOF->CRL |= PIN6_OUT50_ALTPUSHPULL;
#define PF6_OUT             GPIOF->CRL &= ~PIN6_CLEAR; GPIOF->CRL |= PIN6_OUT50_GPPUSHPULL;
#define PF6_FLOAT           GPIOF->CRL &= ~PIN6_CLEAR; GPIOF->CRL |= PIN6_IN_FLOAT;
#define PF6_PULLUP          GPIOF->CRL &= ~PIN6_CLEAR; GPIOF->CRL |= PIN6_IN_PULL; GPIOF->BSRR = BIT6;
#define PF6_PULLDOWN        GPIOF->CRL &= ~PIN6_CLEAR; GPIOF->CRL |= PIN6_IN_PULL; GPIOF->BRR = BIT6;
#define PF6_ANALOG          GPIOF->CRL &= ~PIN6_CLEAR; GPIOF->CRL |= PIN6_IN_ANALOG;
#define PF6_GET             ( GPIOF->IDR & BIT6 )
#define PF6_STATE           PF6_GET
#define PF6_HIGH            GPIOF->BSRR = BIT6;
#define PF6_LOW             GPIOF->BRR = BIT6;

#define PF7_ALT             GPIOF->CRL &= ~PIN7_CLEAR; GPIOF->CRL |= PIN7_OUT50_ALTPUSHPULL;
#define PF7_OUT             GPIOF->CRL &= ~PIN7_CLEAR; GPIOF->CRL |= PIN7_OUT50_GPPUSHPULL;
#define PF7_FLOAT           GPIOF->CRL &= ~PIN7_CLEAR; GPIOF->CRL |= PIN7_IN_FLOAT;
#define PF7_PULLUP          GPIOF->CRL &= ~PIN7_CLEAR; GPIOF->CRL |= PIN7_IN_PULL; GPIOF->BSRR = BIT7;
#define PF7_PULLDOWN        GPIOF->CRL &= ~PIN7_CLEAR; GPIOF->CRL |= PIN7_IN_PULL; GPIOF->BRR = BIT7;
#define PF7_ANALOG          GPIOF->CRL &= ~PIN7_CLEAR; GPIOF->CRL |= PIN7_IN_ANALOG;
#define PF7_GET             ( GPIOF->IDR & BIT7 )
#define PF7_STATE           PF7_GET
#define PF7_HIGH            GPIOF->BSRR = BIT7;
#define PF7_LOW             GPIOF->BRR = BIT7;

#define PF8_ALT             GPIOF->CRH &= ~PIN8_CLEAR; GPIOF->CRH |= PIN8_OUT50_ALTPUSHPULL;
#define PF8_OUT             GPIOF->CRH &= ~PIN8_CLEAR; GPIOF->CRH |= PIN8_OUT50_GPPUSHPULL;
#define PF8_FLOAT           GPIOF->CRH &= ~PIN8_CLEAR; GPIOF->CRH |= PIN8_IN_FLOAT;
#define PF8_PULLUP          GPIOF->CRH &= ~PIN8_CLEAR; GPIOF->CRH |= PIN8_IN_PULL; GPIOF->BSRR = BIT8;
#define PF8_PULLDOWN        GPIOF->CRH &= ~PIN8_CLEAR; GPIOF->CRH |= PIN8_IN_PULL; GPIOF->BRR = BIT8;
#define PF8_ANALOG          GPIOF->CRH &= ~PIN8_CLEAR; GPIOF->CRH |= PIN8_IN_ANALOG;
#define PF8_GET             ( GPIOF->IDR & BIT8 )
#define PF8_STATE           PF8_GET
#define PF8_HIGH            GPIOF->BSRR = BIT8;
#define PF8_LOW             GPIOF->BRR = BIT8;

#define PF9_ALT             GPIOF->CRH &= ~PIN9_CLEAR; GPIOF->CRH |= PIN9_OUT50_ALTPUSHPULL;
#define PF9_OUT             GPIOF->CRH &= ~PIN9_CLEAR; GPIOF->CRH |= PIN9_OUT50_GPPUSHPULL;
#define PF9_FLOAT           GPIOF->CRH &= ~PIN9_CLEAR; GPIOF->CRH |= PIN9_IN_FLOAT;
#define PF9_PULLUP          GPIOF->CRH &= ~PIN9_CLEAR; GPIOF->CRH |= PIN9_IN_PULL; GPIOF->BSRR = BIT9;
#define PF9_PULLDOWN        GPIOF->CRH &= ~PIN9_CLEAR; GPIOF->CRH |= PIN9_IN_PULL; GPIOF->BRR = BIT9;
#define PF9_ANALOG          GPIOF->CRH &= ~PIN9_CLEAR; GPIOF->CRH |= PIN9_IN_ANALOG;
#define PF9_GET             ( GPIOF->IDR & BIT9 )
#define PF9_STATE           PF9_GET
#define PF9_HIGH            GPIOF->BSRR = BIT9;
#define PF9_LOW             GPIOF->BRR = BIT9;

#define PF10_ALT            GPIOF->CRH &= ~PIN10_CLEAR; GPIOF->CRH |= PIN10_OUT50_ALTPUSHPULL;
#define PF10_OUT            GPIOF->CRH &= ~PIN10_CLEAR; GPIOF->CRH |= PIN10_OUT50_GPPUSHPULL;
#define PF10_FLOAT          GPIOF->CRH &= ~PIN10_CLEAR; GPIOF->CRH |= PIN10_IN_FLOAT;
#define PF10_PULLUP         GPIOF->CRH &= ~PIN10_CLEAR; GPIOF->CRH |= PIN10_IN_PULL; GPIOF->BSRR = BIT10;
#define PF10_PULLDOWN       GPIOF->CRH &= ~PIN10_CLEAR; GPIOF->CRH |= PIN10_IN_PULL; GPIOF->BRR = BIT10;
#define PF10_ANALOG         GPIOF->CRH &= ~PIN10_CLEAR; GPIOF->CRH |= PIN10_IN_ANALOG;
#define PF10_GET            ( GPIOF->IDR & BIT10 )
#define PF10_STATE          PF10_GET
#define PF10_HIGH           GPIOF->BSRR = BIT10;
#define PF10_LOW            GPIOF->BRR = BIT10;

#define PF11_ALT            GPIOF->CRH &= ~PIN11_CLEAR; GPIOF->CRH |= PIN11_OUT50_ALTPUSHPULL;
#define PF11_OUT            GPIOF->CRH &= ~PIN11_CLEAR; GPIOF->CRH |= PIN11_OUT50_GPPUSHPULL;
#define PF11_FLOAT          GPIOF->CRH &= ~PIN11_CLEAR; GPIOF->CRH |= PIN11_IN_FLOAT;
#define PF11_PULLUP         GPIOF->CRH &= ~PIN11_CLEAR; GPIOF->CRH |= PIN11_IN_PULL; GPIOF->BSRR = BIT11;
#define PF11_PULLDOWN       GPIOF->CRH &= ~PIN11_CLEAR; GPIOF->CRH |= PIN11_IN_PULL; GPIOF->BRR = BIT11;
#define PF11_ANALOG         GPIOF->CRH &= ~PIN11_CLEAR; GPIOF->CRH |= PIN11_IN_ANALOG;
#define PF11_GET            ( GPIOF->IDR & BIT11 )
#define PF11_STATE          PF11_GET
#define PF11_HIGH           GPIOF->BSRR = BIT11;
#define PF11_LOW            GPIOF->BRR = BIT11;

#define PF12_ALT            GPIOF->CRH &= ~PIN12_CLEAR; GPIOF->CRH |= PIN12_OUT50_ALTPUSHPULL;
#define PF12_OUT            GPIOF->CRH &= ~PIN12_CLEAR; GPIOF->CRH |= PIN12_OUT50_GPPUSHPULL;
#define PF12_FLOAT          GPIOF->CRH &= ~PIN12_CLEAR; GPIOF->CRH |= PIN12_IN_FLOAT;
#define PF12_PULLUP         GPIOF->CRH &= ~PIN12_CLEAR; GPIOF->CRH |= PIN12_IN_PULL; GPIOF->BSRR = BIT12;
#define PF12_PULLDOWN       GPIOF->CRH &= ~PIN12_CLEAR; GPIOF->CRH |= PIN12_IN_PULL; GPIOF->BRR = BIT12;
#define PF12_ANALOG         GPIOF->CRH &= ~PIN12_CLEAR; GPIOF->CRH |= PIN12_IN_ANALOG;
#define PF12_GET            ( GPIOF->IDR & BIT12 )
#define PF12_STATE          PF12_GET
#define PF12_HIGH           GPIOF->BSRR = BIT12;
#define PF12_LOW            GPIOF->BRR = BIT12;

#define PF13_ALT            GPIOF->CRH &= ~PIN13_CLEAR; GPIOF->CRH |= PIN13_OUT50_ALTPUSHPULL;
#define PF13_OUT            GPIOF->CRH &= ~PIN13_CLEAR; GPIOF->CRH |= PIN13_OUT50_GPPUSHPULL;
#define PF13_FLOAT          GPIOF->CRH &= ~PIN13_CLEAR; GPIOF->CRH |= PIN13_IN_FLOAT;
#define PF13_PULLUP         GPIOF->CRH &= ~PIN13_CLEAR; GPIOF->CRH |= PIN13_IN_PULL; GPIOF->BSRR = BIT13;
#define PF13_PULLDOWN       GPIOF->CRH &= ~PIN13_CLEAR; GPIOF->CRH |= PIN13_IN_PULL; GPIOF->BRR = BIT13;
#define PF13_ANALOG         GPIOF->CRH &= ~PIN13_CLEAR; GPIOF->CRH |= PIN13_IN_ANALOG;
#define PF13_GET            ( GPIOF->IDR & BIT13 )
#define PF13_STATE          PF13_GET
#define PF13_HIGH           GPIOF->BSRR = BIT13;
#define PF13_LOW            GPIOF->BRR = BIT13;

#define PF14_ALT            GPIOF->CRH &= ~PIN14_CLEAR; GPIOF->CRH |= PIN14_OUT50_ALTPUSHPULL;
#define PF14_OUT            GPIOF->CRH &= ~PIN14_CLEAR; GPIOF->CRH |= PIN14_OUT50_GPPUSHPULL;
#define PF14_FLOAT          GPIOF->CRH &= ~PIN14_CLEAR; GPIOF->CRH |= PIN14_IN_FLOAT;
#define PF14_PULLUP         GPIOF->CRH &= ~PIN14_CLEAR; GPIOF->CRH |= PIN14_IN_PULL; GPIOF->BSRR = BIT14;
#define PF14_PULLDOWN       GPIOF->CRH &= ~PIN14_CLEAR; GPIOF->CRH |= PIN14_IN_PULL; GPIOF->BRR = BIT14;
#define PF14_ANALOG         GPIOF->CRH &= ~PIN14_CLEAR; GPIOF->CRH |= PIN14_IN_ANALOG;
#define PF14_GET            ( GPIOF->IDR & BIT14 )
#define PF14_STATE          PF14_GET
#define PF14_HIGH           GPIOF->BSRR = BIT14;
#define PF14_LOW            GPIOF->BRR = BIT14;

#define PF15_ALT            GPIOF->CRH &= ~PIN15_CLEAR; GPIOF->CRH |= PIN15_OUT50_ALTPUSHPULL;
#define PF15_OUT            GPIOF->CRH &= ~PIN15_CLEAR; GPIOF->CRH |= PIN15_OUT50_GPPUSHPULL;
#define PF15_FLOAT          GPIOF->CRH &= ~PIN15_CLEAR; GPIOF->CRH |= PIN15_IN_FLOAT;
#define PF15_PULLUP         GPIOF->CRH &= ~PIN15_CLEAR; GPIOF->CRH |= PIN15_IN_PULL; GPIOF->BSRR = BIT15;
#define PF15_PULLDOWN       GPIOF->CRH &= ~PIN15_CLEAR; GPIOF->CRH |= PIN15_IN_PULL; GPIOF->BRR = BIT15;
#define PF15_ANALOG         GPIOF->CRH &= ~PIN15_CLEAR; GPIOF->CRH |= PIN15_IN_ANALOG;
#define PF15_GET            ( GPIOF->IDR & BIT15 )
#define PF15_STATE          PF15_GET
#define PF15_HIGH           GPIOF->BSRR = BIT15;
#define PF15_LOW            GPIOF->BRR = BIT15;
                            
#endif // GPIOF

#define PORTA_ALT            PORTA_MSB_ALT; PORTA_LSB_ALT;       
#define PORTA_OUT            PORTA_MSB_OUT; PORTA_LSB_OUT;                       
#define PORTA_FLOAT          PORTA_MSB_FLOAT; PORTA_LSB_FLOAT;
#define PORTA_PULLUP         PORTA_MSB_PULLUP; PORTA_LSB_PULLUP;
#define PORTA_PULLDOWN       PORTA_MSB_PULLDOWN; PORTA_LSB_PULLDOWN;
#define PORTA_ANALOG         PORTA_MSB_ANALOG; PORTA_LSB_ANALOG;
#define PORTA_GET            ( GPIOA->IDR )
#define PORTA_HIGH           GPIOA->ODR = 0xFFFF;
#define PORTA_LOW            GPIOA->ODR = 0x0000;
#define PORTA_SET( pVal )    GPIOA->ODR = pVal;

#define PORTA_LSB_ALT         GPIOA->CRL = ALLPINS_OUT50_ALTPUSHPULL;          
#define PORTA_LSB_OUT         GPIOA->CRL = ALLPINS_OUT50_GPPUSHPULL;
#define PORTA_LSB_FLOAT       GPIOA->CRL = ALLPINS_IN_FLOAT;  
#define PORTA_LSB_PULLUP      GPIOA->CRL = ALLPINS_IN_PULL; GPIOA->BSRR = 0x00FF;
#define PORTA_LSB_PULLDOWN    GPIOA->CRL = ALLPINS_IN_PULL; GPIOA->BRR = 0x00FF;
#define PORTA_LSB_ANALOG      GPIOA->CRL = ALLPINS_IN_ANALOG;  
#define PORTA_LSB_GET         ( GPIOA->IDR & 0x00FF )  
#define PORTA_LSB_HIGH        GPIOA->BSRR = 0x00FF;  
#define PORTA_LSB_LOW         GPIOA->BRR = 0x00FF;   
#define PORTA_LSB_SET( pVal ) GPIOA->BRR = 0x00FF; GPIOA->BSRR = pVal;

#define PORTA_MSB_ALT         GPIOA->CRH = ALLPINS_OUT50_ALTPUSHPULL;          
#define PORTA_MSB_OUT         GPIOA->CRH = ALLPINS_OUT50_GPPUSHPULL;
#define PORTA_MSB_FLOAT       GPIOA->CRH = ALLPINS_IN_FLOAT;  
#define PORTA_MSB_PULLUP      GPIOA->CRH = ALLPINS_IN_PULL; GPIOA->BSRR = 0xFF00;
#define PORTA_MSB_PULLDOWN    GPIOA->CRH = ALLPINS_IN_PULL; GPIOA->BRR = 0xFF00;  
#define PORTA_MSB_ANALOG      GPIOA->CRH = ALLPINS_IN_ANALOG;  
#define PORTA_MSB_GET         ( GPIOA->IDR >> 8 )
#define PORTA_MSB_HIGH        GPIOA->BSRR = 0xFF00;  
#define PORTA_MSB_LOW         GPIOA->BRR = 0xFF00;  
#define PORTA_MSB_SET( pVal ) GPIOA->BRR = 0xFF00; GPIOA->BSRR = ( pVal << 8 );        

#define PORTB_ALT            PORTB_MSB_ALT; PORTB_LSB_ALT;       
#define PORTB_OUT            PORTB_MSB_OUT; PORTB_LSB_OUT;                       
#define PORTB_FLOAT          PORTB_MSB_FLOAT; PORTB_LSB_FLOAT;
#define PORTB_PULLUP         PORTB_MSB_PULLUP; PORTB_LSB_PULLUP;
#define PORTB_PULLDOWN       PORTB_MSB_PULLDOWN; PORTB_LSB_PULLDOWN;
#define PORTB_ANALOG         PORTB_MSB_ANALOG; PORTB_LSB_ANALOG;
#define PORTB_GET            ( GPIOB->IDR )
#define PORTB_HIGH           GPIOB->ODR = 0xFFFF;
#define PORTB_LOW            GPIOB->ODR = 0x0000;
#define PORTB_SET( pVal )    GPIOB->ODR = pVal;

#define PORTB_LSB_ALT         GPIOB->CRL = ALLPINS_OUT50_ALTPUSHPULL;          
#define PORTB_LSB_OUT         GPIOB->CRL = ALLPINS_OUT50_GPPUSHPULL;
#define PORTB_LSB_FLOAT       GPIOB->CRL = ALLPINS_IN_FLOAT;  
#define PORTB_LSB_PULLUP      GPIOB->CRL = ALLPINS_IN_PULL; GPIOA->BSRR = 0x00FF;
#define PORTB_LSB_PULLDOWN    GPIOB->CRL = ALLPINS_IN_PULL; GPIOA->BRR = 0x00FF;
#define PORTB_LSB_ANALOG      GPIOB->CRL = ALLPINS_IN_ANALOG;  
#define PORTB_LSB_GET         ( GPIOB->IDR & 0x00FF )  
#define PORTB_LSB_HIGH        GPIOB->BSRR = 0x00FF;  
#define PORTB_LSB_LOW         GPIOB->BRR = 0x00FF;   
#define PORTB_LSB_SET( pVal ) GPIOB->BRR = 0x00FF; GPIOB->BSRR = pVal;

#define PORTB_MSB_ALT         GPIOB->CRH = ALLPINS_OUT50_ALTPUSHPULL;          
#define PORTB_MSB_OUT         GPIOB->CRH = ALLPINS_OUT50_GPPUSHPULL;
#define PORTB_MSB_FLOAT       GPIOB->CRH = ALLPINS_IN_FLOAT;  
#define PORTB_MSB_PULLUP      GPIOB->CRH = ALLPINS_IN_PULL; GPIOB->BSRR = 0xFF00;
#define PORTB_MSB_PULLDOWN    GPIOB->CRH = ALLPINS_IN_PULL; GPIOB->BRR = 0xFF00;  
#define PORTB_MSB_ANALOG      GPIOB->CRH = ALLPINS_IN_ANALOG;  
#define PORTB_MSB_GET         ( GPIOB->IDR >> 8 )
#define PORTB_MSB_HIGH        GPIOB->BSRR = 0xFF00;  
#define PORTB_MSB_LOW         GPIOB->BRR = 0xFF00;  
#define PORTB_MSB_SET( pVal ) GPIOB->BRR = 0xFF00; GPIOB->BSRR = ( pVal << 8 );

#define PORTC_ALT            PORTC_MSB_ALT; PORTC_LSB_ALT;       
#define PORTC_OUT            PORTC_MSB_OUT; PORTC_LSB_OUT;                       
#define PORTC_FLOAT          PORTC_MSB_FLOAT; PORTC_LSB_FLOAT;
#define PORTC_PULLUP         PORTC_MSB_PULLUP; PORTC_LSB_PULLUP;
#define PORTC_PULLDOWN       PORTC_MSB_PULLDOWN; PORTC_LSB_PULLDOWN;
#define PORTC_ANALOG         PORTC_MSB_ANALOG; PORTC_LSB_ANALOG;
#define PORTC_GET            ( GPIOC->IDR )
#define PORTC_HIGH           GPIOC->ODR = 0xFFFF;
#define PORTC_LOW            GPIOC->ODR = 0x0000;
#define PORTC_SET( pVal )    GPIOC->ODR = pVal;

#define PORTC_LSB_ALT         GPIOC->CRL = ALLPINS_OUT50_ALTPUSHPULL;          
#define PORTC_LSB_OUT         GPIOC->CRL = ALLPINS_OUT50_GPPUSHPULL;
#define PORTC_LSB_FLOAT       GPIOC->CRL = ALLPINS_IN_FLOAT;  
#define PORTC_LSB_PULLUP      GPIOC->CRL = ALLPINS_IN_PULL; GPIOC->BSRR = 0x00FF;
#define PORTC_LSB_PULLDOWN    GPIOC->CRL = ALLPINS_IN_PULL; GPIOC->BRR = 0x00FF;
#define PORTC_LSB_ANALOG      GPIOC->CRL = ALLPINS_IN_ANALOG;  
#define PORTC_LSB_GET         ( GPIOC->IDR & 0x00FF )  
#define PORTC_LSB_HIGH        GPIOC->BSRR = 0x00FF;  
#define PORTC_LSB_LOW         GPIOC->BRR = 0x00FF;   
#define PORTC_LSB_SET( pVal ) GPIOC->BRR = 0x00FF; GPIOC->BSRR = pVal;

#define PORTC_MSB_ALT         GPIOC->CRH = ALLPINS_OUT50_ALTPUSHPULL;          
#define PORTC_MSB_OUT         GPIOC->CRH = ALLPINS_OUT50_GPPUSHPULL;
#define PORTC_MSB_FLOAT       GPIOC->CRH = ALLPINS_IN_FLOAT;  
#define PORTC_MSB_PULLUP      GPIOC->CRH = ALLPINS_IN_PULL; GPIOC->BSRR = 0xFF00;
#define PORTC_MSB_PULLDOWN    GPIOC->CRH = ALLPINS_IN_PULL; GPIOC->BRR = 0xFF00;  
#define PORTC_MSB_ANALOG      GPIOC->CRH = ALLPINS_IN_ANALOG;  
#define PORTC_MSB_GET         ( GPIOC->IDR >> 8 )
#define PORTC_MSB_HIGH        GPIOC->BSRR = 0xFF00;  
#define PORTC_MSB_LOW         GPIOC->BRR = 0xFF00;  
#define PORTC_MSB_SET( pVal ) GPIOC->BRR = 0xFF00; GPIOC->BSRR = ( pVal << 8 );

#define PORTD_ALT            PORTD_MSB_ALT; PORTD_LSB_ALT;       
#define PORTD_OUT            PORTD_MSB_OUT; PORTD_LSB_OUT;                       
#define PORTD_FLOAT          PORTD_MSB_FLOAT; PORTD_LSB_FLOAT;
#define PORTD_PULLUP         PORTD_MSB_PULLUP; PORTD_LSB_PULLUP;
#define PORTD_PULLDOWN       PORTD_MSB_PULLDOWN; PORTD_LSB_PULLDOWN;
#define PORTD_ANALOG         PORTD_MSB_ANALOG; PORTD_LSB_ANALOG;
#define PORTD_GET            ( GPIOD->IDR )
#define PORTD_HIGH           GPIOD->ODR = 0xFFFF;
#define PORTD_LOW            GPIOD->ODR = 0x0000;
#define PORTD_SET( pVal )    GPIOD->ODR = pVal;

#define PORTD_LSB_ALT         GPIOD->CRL = ALLPINS_OUT50_ALTPUSHPULL;          
#define PORTD_LSB_OUT         GPIOD->CRL = ALLPINS_OUT50_GPPUSHPULL;
#define PORTD_LSB_FLOAT       GPIOD->CRL = ALLPINS_IN_FLOAT;  
#define PORTD_LSB_PULLUP      GPIOD->CRL = ALLPINS_IN_PULL; GPIOD->BSRR = 0x00FF;
#define PORTD_LSB_PULLDOWN    GPIOD->CRL = ALLPINS_IN_PULL; GPIOD->BRR = 0x00FF;
#define PORTD_LSB_ANALOG      GPIOD->CRL = ALLPINS_IN_ANALOG;  
#define PORTD_LSB_GET         ( GPIOD->IDR & 0x00FF )  
#define PORTD_LSB_HIGH        GPIOD->BSRR = 0x00FF;  
#define PORTD_LSB_LOW         GPIOD->BRR = 0x00FF;   
#define PORTD_LSB_SET( pVal ) GPIOD->BRR = 0x00FF; GPIOD->BSRR = pVal;

#define PORTD_MSB_ALT         GPIOD->CRH = ALLPINS_OUT50_ALTPUSHPULL;          
#define PORTD_MSB_OUT         GPIOD->CRH = ALLPINS_OUT50_GPPUSHPULL;
#define PORTD_MSB_FLOAT       GPIOD->CRH = ALLPINS_IN_FLOAT;  
#define PORTD_MSB_PULLUP      GPIOD->CRH = ALLPINS_IN_PULL; GPIOD->BSRR = 0xFF00;
#define PORTD_MSB_PULLDOWN    GPIOD->CRH = ALLPINS_IN_PULL; GPIOD->BRR = 0xFF00;  
#define PORTD_MSB_ANALOG      GPIOD->CRH = ALLPINS_IN_ANALOG;  
#define PORTD_MSB_GET         ( GPIOD->IDR >> 8 )
#define PORTD_MSB_HIGH        GPIOD->BSRR = 0xFF00;  
#define PORTD_MSB_LOW         GPIOD->BRR = 0xFF00;  
#define PORTD_MSB_SET( pVal ) GPIOD->BRR = 0xFF00; GPIOD->BSRR = ( pVal << 8 );

#ifdef GPIOE
#define PORTE_ALT            PORTE_MSB_ALT; PORTE_LSB_ALT;       
#define PORTE_OUT            PORTE_MSB_OUT; PORTE_LSB_OUT;                       
#define PORTE_FLOAT          PORTE_MSB_FLOAT; PORTE_LSB_FLOAT;
#define PORTE_PULLUP         PORTE_MSB_PULLUP; PORTE_LSB_PULLUP;
#define PORTE_PULLDOWN       PORTE_MSB_PULLDOWN; PORTE_LSB_PULLDOWN;
#define PORTE_ANALOG         PORTE_MSB_ANALOG; PORTE_LSB_ANALOG;
#define PORTE_GET            ( GPIOE->IDR )
#define PORTE_HIGH           GPIOE->ODR = 0xFFFF;
#define PORTE_LOW            GPIOE->ODR = 0x0000;
#define PORTE_SET( pVal )    GPIOE->ODR = pVal;

#define PORTE_LSB_ALT         GPIOE->CRL = ALLPINS_OUT50_ALTPUSHPULL;          
#define PORTE_LSB_OUT         GPIOE->CRL = ALLPINS_OUT50_GPPUSHPULL;
#define PORTE_LSB_FLOAT       GPIOE->CRL = ALLPINS_IN_FLOAT;  
#define PORTE_LSB_PULLUP      GPIOE->CRL = ALLPINS_IN_PULL; GPIOE->BSRR = 0x00FF;
#define PORTE_LSB_PULLDOWN    GPIOE->CRL = ALLPINS_IN_PULL; GPIOE->BRR = 0x00FF;
#define PORTE_LSB_ANALOG      GPIOE->CRL = ALLPINS_IN_ANALOG;  
#define PORTE_LSB_GET         ( GPIOE->IDR & 0x00FF )  
#define PORTE_LSB_HIGH        GPIOE->BSRR = 0x00FF;  
#define PORTE_LSB_LOW         GPIOE->BRR = 0x00FF;   
#define PORTE_LSB_SET( pVal ) GPIOE->BRR = 0x00FF; GPIOE->BSRR = pVal;

#define PORTE_MSB_ALT         GPIOE->CRH = ALLPINS_OUT50_ALTPUSHPULL;          
#define PORTE_MSB_OUT         GPIOE->CRH = ALLPINS_OUT50_GPPUSHPULL;
#define PORTE_MSB_FLOAT       GPIOE->CRH = ALLPINS_IN_FLOAT;  
#define PORTE_MSB_PULLUP      GPIOE->CRH = ALLPINS_IN_PULL; GPIOE->BSRR = 0xFF00;
#define PORTE_MSB_PULLDOWN    GPIOE->CRH = ALLPINS_IN_PULL; GPIOE->BRR = 0xFF00;  
#define PORTE_MSB_ANALOG      GPIOE->CRH = ALLPINS_IN_ANALOG;  
#define PORTE_MSB_GET         ( GPIOE->IDR >> 8 )
#define PORTE_MSB_HIGH        GPIOE->BSRR = 0xFF00;  
#define PORTE_MSB_LOW         GPIOE->BRR = 0xFF00;  
#define PORTE_MSB_SET( pVal ) GPIOE->BRR = 0xFF00; GPIOE->BSRR = ( pVal << 8 );
#endif // GPIOE

#ifdef GPIOF
#define PORTF_ALT            PORTF_MSB_ALT; PORTF_LSB_ALT;       
#define PORTF_OUT            PORTF_MSB_OUT; PORTF_LSB_OUT;                       
#define PORTF_FLOAT          PORTF_MSB_FLOAT; PORTF_LSB_FLOAT;
#define PORTF_PULLUP         PORTF_MSB_PULLUP; PORTF_LSB_PULLUP;
#define PORTF_PULLDOWN       PORTF_MSB_PULLDOWN; PORTF_LSB_PULLDOWN;
#define PORTF_ANALOG         PORTF_MSB_ANALOG; PORTF_LSB_ANALOG;
#define PORTF_GET            ( GPIOF->IDR )
#define PORTF_HIGH           GPIOF->ODR = 0xFFFF;
#define PORTF_LOW            GPIOF->ODR = 0x0000;
#define PORTF_SET( pVal )    GPIOF->ODR = pVal;

#define PORTF_LSB_ALT         GPIOF->CRL = ALLPINS_OUT50_ALTPUSHPULL;          
#define PORTF_LSB_OUT         GPIOF->CRL = ALLPINS_OUT50_GPPUSHPULL;
#define PORTF_LSB_FLOAT       GPIOF->CRL = ALLPINS_IN_FLOAT;  
#define PORTF_LSB_PULLUP      GPIOF->CRL = ALLPINS_IN_PULL; GPIOF->BSRR = 0x00FF;
#define PORTF_LSB_PULLDOWN    GPIOF->CRL = ALLPINS_IN_PULL; GPIOF->BRR = 0x00FF;
#define PORTF_LSB_ANALOG      GPIOF->CRL = ALLPINS_IN_ANALOG;  
#define PORTF_LSB_GET         ( GPIOF->IDR & 0x00FF )  
#define PORTF_LSB_HIGH        GPIOF->BSRR = 0x00FF;  
#define PORTF_LSB_LOW         GPIOF->BRR = 0x00FF;   
#define PORTF_LSB_SET( pVal ) GPIOF->BRR = 0x00FF; GPIOF->BSRR = pVal;

#define PORTF_MSB_ALT         GPIOF->CRH = ALLPINS_OUT50_ALTPUSHPULL;          
#define PORTF_MSB_OUT         GPIOF->CRH = ALLPINS_OUT50_GPPUSHPULL;
#define PORTF_MSB_FLOAT       GPIOF->CRH = ALLPINS_IN_FLOAT;  
#define PORTF_MSB_PULLUP      GPIOF->CRH = ALLPINS_IN_PULL; GPIOF->BSRR = 0xFF00;
#define PORTF_MSB_PULLDOWN    GPIOF->CRH = ALLPINS_IN_PULL; GPIOF->BRR = 0xFF00;  
#define PORTF_MSB_ANALOG      GPIOF->CRH = ALLPINS_IN_ANALOG;  
#define PORTF_MSB_GET         ( GPIOF->IDR >> 8 )
#define PORTF_MSB_HIGH        GPIOF->BSRR = 0xFF00;  
#define PORTF_MSB_LOW         GPIOF->BRR = 0xFF00;  
#define PORTF_MSB_SET( pVal ) GPIOF->BRR = 0xFF00; GPIOF->BSRR = ( pVal << 8 );
#endif

#define PIN_PULLUP                              BIT4
#define PIN_PULLDOWN                            BIT5
#define PIN_OUT2                                BIT6    // deprecated
#define PIN_OUT10                               BIT7    // deprecated

#define OUT                                     PIN_OUT50_GPPUSHPULL
#define IN                                      PIN_IN_FLOAT
#define ANALOG                                  PIN_IN_ANALOG
#define PULLUP                                  (PIN_IN_PULL|PIN_PULLUP)
#define PULLDOWN                                (PIN_IN_PULL|PIN_PULLDOWN)
#define ALT                                     PIN_OUT50_ALTPUSHPULL

#define HIGH                                    1
#define LOW                                     0
#define NONE                                    0xFF

enum {
        PA0,PA1,PA2,PA3,PA4,PA5,PA6,PA7,PA8,PA9,PA10,PA11,PA12,PA13,PA14,PA15,
        PB0,PB1,PB2,PB3,PB4,PB5,PB6,PB7,PB8,PB9,PB10,PB11,PB12,PB13,PB14,PB15,
        PC0,PC1,PC2,PC3,PC4,PC5,PC6,PC7,PC8,PC9,PC10,PC11,PC12,PC13,PC14,PC15,
        PD0,PD1,PD2,PD3,PD4,PD5,PD6,PD7,PD8,PD9,PD10,PD11,PD12,PD13,PD14,PD15,
        PE0,PE1,PE2,PE3,PE4,PE5,PE6,PE7,PE8,PE9,PE10,PE11,PE12,PE13,PE14,PE15,
        PF0,PF1,PF2,PF3,PF4,PF5,PF6,PF7,PF8,PF9,PF10,PF11,PF12,PF13,PF14,PF15,
        ADCTEMP,ADCREF};


#define PYGMY_PWM_EN                  BIT0
#define PYGMY_PWM_DIR                 BIT1           
    
typedef struct {    
                    u32 UpCount;
                    u32 DownCount;
                    u32 Count;
                    u8 Pin;
                    u8 CR;
                } PYGMYPWM;

/**************************************************************************
                                Analog
***************************************************************************/
//CR1
#define ADC_AWDEN                   BIT23
#define ADC_JAWDEN                  BIT22
#define ADC_JDISCEN                 BIT12
#define ADC_DISCEN                  BIT11
#define ADC_JAUTO                   BIT10
#define ADC_AWDSGL                  BIT9
#define ADC_SCAN                    BIT8
#define ADC_JEOCIE                  BIT7
#define ADC_AWDIE                   BIT6
#define ADC_EOCIE                   BIT5
//SR

#define ADC_STRT                    BIT4    // Conversion started flag
#define ADC_JSTRT                   BIT3    // Injected conversion started flag
#define ADC_JEOC                    BIT2    // Injected End Of Conversion flag
#define ADC_EOC                     BIT1    // End Of Conversion flag
#define ADC_AWD                     BIT0    // Analog Watch Dog flag

//CR2
#define ADC_TSVREFE                 BIT23   // Temp Sense and VREF Enable
#define ADC_SWSTART                 BIT22   // Software Start Conversion
#define ADC_JSWSTART                BIT21   // Injected Conversion Software Start
#define ADC_EXTTRIG                 BIT20
#define ADC_EXTSEL_CLEAR            (BIT19|BIT18|BIT17)
#define ADC_EXTSEL_TIMER1CC1        0        // All bits clear, default
#define ADC_EXTSEL_TIMER1CC2        BIT17
#define ADC_EXTSEL_TIMER1CC3        BIT18
#define ADC_EXTSEL_TIMER2CC2        (BIT18|BIT17)
#define ADC_EXTSEL_TIMER3TRGO       BIT19
#define ADC_EXTSEL_TIMER4CC4        (BIT19|BIT17)
#define ADC_EXTSEL_EXTI11           (BIT19|BIT18)
#define ADC_EXTSEL_SWSTART          (BIT19|BIT18|BIT17)
#define ADC_JEXTTRIG                BIT15
#define ADC_JEXTSEL_TIMER1TRGO      0         // All bits clear, default
#define ADC_JEXTSEL_ALIGN           BIT11
#define ADC_DMA                     BIT8
#define ADC_RSTCAL                  BIT3
#define ADC_CAL                     BIT2
#define ADC_CONT                    BIT1
#define ADC_ADON                    BIT0
#define ADC_SMP_CLEAR               (BIT0|BIT1|BIT3) // All SMP bits must be shifted into place
#define ADC_SMP_1P5                 0        // All bits clear
#define ADC_SMP_7P5                 BIT0
#define ADC_SMP_13P5                BIT1
#define ADC_SMP_28P5                (BIT1|BIT0)
#define ADC_SMP_41P5                BIT2
#define ADC_SMP_55P5                (BIT2|BIT0)
#define ADC_SMP_71P5                (BIT3|BIT1)
#define ADC_SMP_239P5               (BIT2|BIT1|BIT0)

//STM32 ADC

typedef struct{
    volatile u32 SR;
    volatile u32 CR1;
    volatile u32 CR2;
    volatile u32 SMPR1;
    volatile u32 SMPR2;
    volatile u32 JOFR1;
    volatile u32 JOFR2;
    volatile u32 JOFR3;
    volatile u32 JOFR4;
    volatile u32 HTR;
    volatile u32 LTR;
    volatile u32 SQR1;
    volatile u32 SQR2;
    volatile u32 SQR3;
    volatile u32 JSQR;
    volatile u32 JDR1;
    volatile u32 JDR2;
    volatile u32 JDR3;
    volatile u32 JDR4;
    volatile u32 DR;
    } PYGMYADC;

#define ADC1_BASE               (APB2PERIPH_BASE + 0x2400)
#define ADC2_BASE               (APB2PERIPH_BASE + 0x2800)
#define ADC1                    ((PYGMYADC *) ADC1_BASE)
#define ADC2                    ((PYGMYADC *) ADC2_BASE)

//--------------------------------------------------------------------------------------------
//------------------------------------------I2C-----------------------------------------------
typedef struct
{
  vu16 CR1;
  u16 RESERVED0;
  vu16 CR2;
  u16 RESERVED1;
  vu16 OAR1;
  u16 RESERVED2;
  vu16 OAR2;
  u16 RESERVED3;
  vu16 DR;
  u16 RESERVED4;
  vu16 SR1;
  u16 RESERVED5;
  vu16 SR2;
  u16 RESERVED6;
  vu16 CCR;
  u16 RESERVED7;
  vu16 TRISE;
  u16 RESERVED8;
} I2C_TYPEDEF;

// Register CR1
#define I2C_SWRST               BIT15
#define I2C_ALERT               BIT13
#define I2C_CR1_PEC                 BIT12
#define I2C_POS                 BIT11
#define I2C_ACK                 BIT10
#define I2C_STOP                BIT9
#define I2C_START               BIT8
#define I2C_NOSTRETCH           BIT7
#define I2C_ENGC                BIT6
#define I2C_ENPEC               BIT5
#define I2C_ENARP               BIT4
#define I2C_SMBTYPE             BIT3
#define I2C_SMBUS               BIT1
#define I2C_PE                  BIT0
// Register CR2
#define I2C_LAST                BIT12
#define I2C_DMAEN               BIT11
#define I2C_ITBUFEN             BIT10
#define I2C_ITEVTEN             BIT9
#define I2C_ITERREN             BIT8
// Register OAR1
#define I2C_ADDMODE             BIT15 // 7Bit addr must be left shifted 1
// Register OAR2
#define I2C_ENDUAL              BIT0 // 7Bit addr must be left shifted 1 and ENDUAL set for dual addr
// Register SR1
#define I2C_SMBALERT            BIT15
#define I2C_TIMEOUT             BIT14
#define I2C_PECERR              BIT12
#define I2C_OVR                 BIT11
#define I2C_AF                  BIT10
#define I2C_ARLO                BIT9
#define I2C_BERR                BIT8
#define I2C_TXE                 BIT7
#define I2C_RXNE                BIT6
#define I2C_STOPF               BIT4
#define I2C_ADD10               BIT3
#define I2C_BTF                 BIT2
#define I2C_ADDR                BIT1
#define I2C_SB                  BIT0
// Register SR2
#define I2C_SR2_PEC                 ( BIT15|BIT14|BIT13|BIT12|BIT11|BIT10|BIT9|BIT8 )
#define I2C_DUALF               BIT7
#define I2C_SMBHOST             BIT6
#define I2C_SMBDEFAULT          BIT5
#define I2C_GENCALL             BIT4
#define I2C_TRA                 BIT2
#define I2C_BUSY                BIT1
#define I2C_MSL                 BIT0
// Register CCR
#define I2C_FS                  BIT15
#define I2C_DUTY                BIT14


//------------------------------------------END I2C-----------------------------------------------
//--------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------
//------------------------------------------SPI-----------------------------------------------
typedef struct
{
  vu16 CR1;
  u16 RESERVED0;
  vu16 CR2;
  u16 RESERVED1;
  vu16 SR;
  u16  RESERVED2;
  vu16 DR;
  u16  RESERVED3;
  vu16 CRCPR;
  u16 RESERVED4;
  vu16 RXCRCR;
  u16  RESERVED5;
  vu16 TXCRCR;
  u16  RESERVED6;
} SPI_TYPEDEF;

// Deprecated
#define PYGMY_SPI2_RESET                 PYGMY_RCC_SPI2_DISABLE; PYGMY_RCC_SPI2_ENABLE
#define PYGMY_SPI1_RESET                 PYGMY_RCC_SPI1_DISABLE; PYGMY_RCC_SPI1_ENABLE
#define PYGMY_SPI1_SETMASTER             SPI1->CR1 |= CR1_SPE_Set
#define PYGMY_SPI1_CLEARMASTER           SPI1->CR1 &= ~CR1_SPE_Set
#define PYGMY_SPI2_SETMASTER             SPI2->CR1 |= CR1_SPE_Set
#define PYGMY_SPI2_CLEARMASTER           SPI2->CR1 &= ~CR1_SPE_Set
#define PYGMY_SPI1_SEND( u16Data )       SPI1->DR = u16Data
#define PYGMY_SPI2_SEND( u16Data )       SPI2->DR = u16Data
#define PYGMY_SPI1_RECEIVE               SPI1->DR
#define PYGMY_SPI2_RECEIVE               SPI2->DR
#define PYGMY_SPI1_WAIT                  while( SPI1->SR & BSY  )
#define PYGMY_SPI2_WAIT                  while( SPI2->SR & BSY  )
#define PYGMY_SPI1_ENABLE                SPI1->CR1 |= SPE
#define PYGMY_SPI1_DISABLE               SPI1->CR1 &= ~SPE
#define PYGMY_SPI2_ENABLE                SPI2->CR1 |= SPE
#define PYGMY_SPI2_DISABLE               SPI2->CR1 &= ~SPE



#define SPI_DIRECTION_2Lines_FullDuplex     0x0000
#define SPI_DIRECTION_2Lines_RxOnly         0x0400
#define SPI_DIRECTION_1Line_Rx              0x8000
#define SPI_DIRECTION_1Line_Tx              0xC000
#define SPI_MODE_MASTER                     0x0104
#define SPI_MODE_SLAVE                      0x0000
#define SPI_DATASIZE_16b                    0x0800
#define SPI_DATASIZE_8b                     0x0000
#define SPI_CPOL_LOW                        0x0000
#define SPI_CPOL_HIGH                       0x0002
#define SPI_CPHA_1EDGE                      0x0000
#define SPI_CPHA_2EDGE                      0x0001
#define SPI_NSS_SOFT                        0x0200
#define SPI_NSS_HARD                        0x0000
#define SPI_BAUDRATEPRESCALER_2             0x0000
#define SPI_BAUDRATEPRESCALER_4             0x0008
#define SPI_BAUDRATEPRESCALER_8             0x0010
#define SPI_BAUDRATEPRESCALER_16            0x0018
#define SPI_BAUDRATEPRESCALER_32            0x0020
#define SPI_BAUDRATEPRESCALER_64            0x0028
#define SPI_BAUDRATEPRESCALER_128           0x0030
#define SPI_BAUDRATEPRESCALER_256           0x0038
#define SPI_FIRSTBIT_MSB                    0x0000
#define SPI_FIRSTBIt_LSB                    0x0080
#define SPI_DMAREQ_Tx                       0x0002
#define SPI_DMAREQ_Rx                       0x0001
#define SPI_NSSINTERNALSOFT_SET             0x0100
#define SPI_NSSINTERNALSOFT_RESET           0xFEFF
#define SPI_CRC_TX                          0x00
#define SPI_CRC_RX                          0x01
#define SPI_DIRECTION_RX                    0xBFFF
#define SPI_DIRECTION_TX                    0x4000
#define SPI_IT_TXE                          0x71
#define SPI_IT_RXNE                         0x60
#define SPI_IT_ERR                          0x50
#define SPI_IT_OVR                          0x56
#define SPI_IT_MODF                         0x55
#define SPI_IT_CRCERR                       0x54
#define SPI_FLAG_RXNE                       0x0001
#define SPI_FLAG_TXE                        0x0002
#define SPI_FLAG_CRCERR                     0x0010
#define SPI_FLAG_MODF                       0x0020
#define SPI_FLAG_OVR                        0x0040
#define SPI_FLAG_BSY                        0x0080


//--------------------------------------------------------------------------------------------
//-------------------------------------------SPI Bits-----------------------------------------
// SPI_CR1
#define SPI_BIDIMODE            BIT15               
#define SPI_BIDIOE              BIT14                 
#define SPI_CRCEN               BIT13              
#define SPI_CRCNEXT             BIT12              
#define SPI_DFF                 BIT11                
#define SPI_RXONLY              BIT10              
#define SPI_SSM                 BIT9                   
#define SPI_SSI                 BIT8                  
#define SPI_LSBFIRST            BIT7                   
#define SPI_SPE                 BIT6                  
#define SPI_BR2                 0
#define SPI_BR4                 BIT3                  
#define SPI_BR8                 BIT4                   
#define SPI_BR16                BIT4|BIT3             
#define SPI_BR32                BIT5                
#define SPI_BR64                BIT5|BIT3          
#define SPI_BR128               BIT5|BIT4              
#define SPI_BR256               BIT5|BIT4|BIT3        
#define SPI_MSTR                BIT2                
#define SPI_CPOL                BIT1                 
#define SPI_CPHA                BIT0                  
//SPI_CR2
#define SPI_TXEIE               BIT7                
#define SPI_RXNEIE              BIT6                
#define SPI_ERRIE               BIT5               
#define SPI_SSOE                BIT2                 
#define SPI_TXDMAEN             BIT1             
#define SPI_RXDMAEN
//SPI_SR Status Register
#define SPI_BSY                 BIT7              
#define SPI_OVR                 BIT6           
#define SPI_MODF                BIT5             
#define SPI_CRCERR              BIT4              
#define SPI_UDR                 BIT3             
#define SPI_CHSIDE              BIT2              
#define SPI_TXE                 BIT1               
#define SPI_RXNE                BIT0                

//----------------------------------------End SPI---------------------------------------------
//--------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------
//------------------------------------Hardware USARTS-----------------------------------------
typedef struct
{
  vu16 SR;
  u16 RESERVED0;
  vu16 DR;
  u16 RESERVED1;
  vu16 BRR;
  u16 RESERVED2;
  vu16 CR1;
  u16 RESERVED3;
  vu16 CR2;
  u16 RESERVED4;
  vu16 CR3;
  u16 RESERVED5;
  vu16 GTPR;
  u16 RESERVED6;
} USART_TYPEDEF;

// Status Register SR
#define USART_CTS                   BIT9
#define USART_LBD                   BIT8
#define USART_TXE                   BIT7
#define USART_TC                    BIT6
#define USART_RXNE                  BIT5
#define USART_IDLE                  BIT4
#define USART_ORE                   BIT3
#define USART_NF                    BIT2
#define USART_FE                    BIT1
#define USART_PE                    BIT0
// Baud Rate Register BRR
#define USART_MANTISSA_MASK         0xFFF0
#define USART_FRACTION_MASK         0x000F
// Control Register CR1
#define USART_OVER8                 BIT15 // default is 0 ( OVER16 )
#define USART_UE                    BIT13 // USART Enable
#define USART_M                     BIT12 // Data bits
#define USART_WAKE                  BIT11 
#define USART_PCE                   BIT10 // Parity Control Enable
#define USART_PS                    BIT9  // 0 = Even Parity, 1 = Odd Parity
#define USART_PEIE                  BIT8  // 1 = Parity Interrupt Enabled
#define USART_TXEIE                 BIT7  // 1 = Transmit Interrupt Enabled
#define USART_TCIE                  BIT6  // 1 = Transmit Complete Interrupt Enabled
#define USART_RXNEIE                BIT5  // 1 = RXNE Interrupt Enabled
#define USART_IDLEIE                BIT4  // 1 = IDLE Interrupt Enabled
#define USART_TE                    BIT3  // Transmitter Enable
#define USART_RE                    BIT2  // Receiver Enabled
#define USART_RWU                   BIT1  // Wake-up Mute
#define USART_SBK                   BIT0  // Send Break
// Control Register CR2
#define USART_LINEN                 BIT14 // 1 = Enable Line-End Synch Breaks
#define USART_STOP1                 BIT13 | BIT12 // ( Both bits must be cleared )
#define USART_STOPP5                BIT12
#define USART_STOP2                 BIT13
#define USART_STOP2P5               BIT13 | BIT12 // ( Both Bits must be set )
#define USART_CLKEN                 BIT11 // 1 = Enable SCLK for Synchronous mode
#define USART_CPOL                  BIT10 // 1 = Steady High Clock Polarity
#define USART_CPHA                  BIT9  
#define USART_LBCL                  BIT8
#define USART_LBDIE                 BIT6 // LINE Break Interrupt Enable
#define USART_LBDL                  BIT5
// CR BIT3-BIT0 are Multiprocessor Node Address
// Control Register CR3
#define USART_ONEBITE               BIT11 // 1 = One bit sample, 0 = three bit
#define USART_CTSIE                 BIT10 // 1 = CTS Interrupt Enabled
#define USART_CTSE                  BIT9  // 1 = CTS Enabled
#define USART_RTSE                  BIT8  // 1 = RTS Enabled
#define USART_DMAT                  BIT7  // 1 = DMA is Enabled for Transmission
#define USART_DMAR                  BIT6  // 1 = DMA is Enabled for Receive
#define USART_SCEN                  BIT5  // 1 = Smartcard Mode Enabled
#define USART_NACK                  BIT4  // 1 = SmartCard NACK Enabled
#define USART_HDSEL                 BIT3  // 1 = Half Duplex Mode
#define USART_IRLP                  BIT2  // 1 = IrDA Low Power Mode Enabled
#define USART_IREN                  BIT1  // 1 = IrDA Enabled
#define USART_EIE                   BIT0  // 1 = Error Interrupt Enabled


#define SPI2_BASE                   (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE                   (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASE                 (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE                 (APB1PERIPH_BASE + 0x4800)
#define USART4_BASE                 (APB1PERIPH_BASE + 0x4C00)
#define USART5_BASE                 (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE                   (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE                   (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASE                   (APB1PERIPH_BASE + 0x5C00)
#define CAN_BASE                    (APB1PERIPH_BASE + 0x6400)

#define SPI1_BASE                   (APB2PERIPH_BASE + 0x3000)
#define USART1_BASE                 (APB2PERIPH_BASE + 0x3800)

#define SDIO_BASE                   (AHBPERIPH_BASE + 0x8000)

#define SDIO                        ((SDIO_TYPEDEF *) SDIO_BASE)
#define SPI1                        ((SPI_TYPEDEF *) SPI1_BASE)
#define SPI2                        ((SPI_TYPEDEF *) SPI2_BASE)
#define SPI3                        ((SPI_TYPEDEF *) SPI3_BASE)
#define USART1                      ((USART_TYPEDEF *) USART1_BASE)
#define USART2                      ((USART_TYPEDEF *) USART2_BASE)
#define USART3                      ((USART_TYPEDEF *) USART3_BASE)
#define USART4                      ((USART_TYPEDEF *) USART4_BASE)
#define USART5                      ((USART_TYPEDEF *) USART5_BASE)
#define I2C1                        ((I2C_TYPEDEF *) I2C1_BASE)
#define I2C2                        ((I2C_TYPEDEF *) I2C2_BASE)
#define I2C3                        ((I2C_TYPEDEF *) I2C3_BASE)
#define CAN                         ((CAN_TYPEDEF *) CAN_BASE)

//----------------------------------End Hardware USARTS---------------------------------------
//--------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------
//-----------------------------------Pygmy OS Comports----------------------------------------
/*#define DATA        1
#define COMMAND     0
#define PYGMY_STREAMS_ECHO          BIT1
#define PYGMY_STREAMS_BACKSPACE     BIT2
#define PYGMY_STREAMS_ACTIONCHARS   BIT3
#define PYGMY_STREAMS_CR            BIT4
#define PYGMY_STREAMS_USERHANDLER   BIT5*/

/*typedef struct {
                u16 RXBufferLen;
                u16 RXIndex;
                u16 RXLen;
                u16 TXBufferLen;
                u16 TXIndex;
                u16 TXLen;
                PYGMYCMDPTR Put;
                PYGMYCMDPTR Putc;
                PYGMYVOIDPTR Get;
                u8 *RXBuffer;
                u8 *TXBuffer;
                u8 *ActionChars;
                u8 Pipe;
                u8 CR;
                } PYGMYFIFO;    */

/*enum {
        RS232,
        RS422,
        RS485,
        SPI,
        I2C,
        BITBANG,
        };*/
 
#define RTS                 BIT0
#define CTS                 BIT1
#define STOP1               BIT2
#define STOP15              BIT3
#define TXIE                BIT4
#define TXFIFO              TXIE
   
#define I2CBUSERROR         BIT0     
#define I2CWORDADDRESS      BIT1
#define I2CSPEEDSTANDARD    BIT2 // 100kbps, All speed bits clear = 10kbps
#define I2CSPEEDFAST        BIT3 // 400kbps
#define I2CSPEEDFASTPLUS    BIT4 // 1Mbps
#define I2CSPEEDHIGH        BIT5 // 3.4Mbps
#define I2CPOLLFORACK       BIT6 // Poll for an Ack on Address to show busy status cleared
 
#define SPIWORDADDRESS      BIT1
#define SPILONGADDRESS      BIT2
#define SPIDUMMYONREAD      BIT3   
#define SPIDUMMYONWRITE     BIT4

typedef struct{
                u8 SCL;
                u8 SDA;
                u8 Address;
                GPIO *PortSCL;
                GPIO *PortSDA;
                u16 PinSCL;
                u16 PinSDA;
                u16 Speed;
                u16 CR;
                //u8 Status;
                } PYGMYI2CPORT;
            
typedef struct{
                GPIO *PortCS;
                GPIO *PortSCK;
                GPIO *PortMOSI;
                GPIO *PortMISO;
                u16 PinCS;
                u16 PinSCK;
                u16 PinMOSI;
                u16 PinMISO;
                u16 CR;
                } PYGMYSPIPORT;
            
typedef struct{
                GPIO *PortCS;
                GPIO *PortA0;
                GPIO *PortWR;
                GPIO *PortRD;
                GPIO *PortDATA;
                u16 PinCS;
                u16 PinA0;
                u16 PinWR;
                u16 PinRD;
                u16 PinD0;
                u16 Mask;
                u8 Width;
                } PYGMYPARALLELPORT;

typedef struct {
                u32 Baud;
                u32 Clock;
                u8 COM;
                u8 Protocol;
                u8 Databits;
                u8 Parity;
                u8 Stopbits;
                u8 Flowcontrol;

                u8 (*GetBaud)           ( void );
                u8 (*SetBaud)           ( u32 );
                u8 (*GetProtocol)       ( void );
                u8 (*SetProtocol)       ( u8 );
                u8 (*GetDatabits)       ( void );
                u8 (*SetDatabits)       ( u8 );
                u8 (*GetParity)         ( void );
                u8 (*SetParity)         ( u8 );
                u8 (*GetStopbits)       ( void );
                u8 (*SetStopbits)       ( u8 );
                u8 (*GetFlowcontrol)    ( void );
                u8 (*SetFlowcontrol)    ( u8 );
                void (*Open)            ( void );
                } PYGMYCOMPORT;

typedef struct{
                u8 SPI;
                u8 I2C;
                //u8 Type;
                } PYGMYPORTINDEX;

//STM32 DMA
typedef struct
{
  vu32 CCR;
  vu32 CNDTR;
  vu32 CPAR;
  vu32 CMAR;
} DMA_CHANNEL_TYPEDEF;

typedef struct
{
  vu32 ISR;
  vu32 IFCR;
} DMA_TYPEDEF;

#define DMA_MEM2MEM             BIT14
#define DMA_PRIORITY_MASK       ( BIT13|BIT12 )
#define DMA_PRIORITY_LOW        0
#define DMA_PRIORITY_MEDIUM     BIT12
#define DMA_PRIORITY_HIGH       BIT13
#define DMA_PRIORITY_VERYHIGH   ( BIT13|BIT12 )
#define DMA_MSIZE_MASK          ( BIT11|BIT10 )
#define DMA_MSIZE8              0
#define DMA_MSIZE16             BIT10
#define DMA_MSIZE32             BIT11
#define DMA_PSIZE_MASK          ( BIT9|BIT8 )
#define DMA_PSIZE8              0
#define DMA_PSIZE16             BIT8
#define DMA_PSIZE32             BIT9
#define DMA_MINC                BIT7
#define DMA_PINC                BIT6
#define DMA_CIRC                BIT5
#define DMA_DIR                 BIT4
#define DMA_TEIE                BIT3
#define DMA_HTIE                BIT2
#define DMA_TCIE                BIT1
#define DMA_EN                  BIT0  


#define DMA1_BASE (AHBPERIPH_BASE + 0x0000)
#define DMA2_BASE (AHBPERIPH_BASE + 0x0400)
#define DMA1 ((DMA_TYPEDEF *) DMA1_BASE)
#define DMA2 ((DMA_TYPEDEF *) DMA2_BASE)
#define DMA1_CH1_BASE (AHBPERIPH_BASE + 0x0008)
#define DMA1_CH2_BASE (AHBPERIPH_BASE + 0x001C)
#define DMA1_CH3_BASE (AHBPERIPH_BASE + 0x0030)
#define DMA1_CH4_BASE (AHBPERIPH_BASE + 0x0044)
#define DMA1_CH5_BASE (AHBPERIPH_BASE + 0x0058)
#define DMA1_CH6_BASE (AHBPERIPH_BASE + 0x006C)
#define DMA1_CH7_BASE (AHBPERIPH_BASE + 0x0080)
#define DMA2_CH1_BASE (AHBPERIPH_BASE + 0x0408)
#define DMA2_CH2_BASE (AHBPERIPH_BASE + 0x041C)
#define DMA2_CH3_BASE (AHBPERIPH_BASE + 0x0430)
#define DMA2_CH4_BASE (AHBPERIPH_BASE + 0x0444)
#define DMA2_CH5_BASE (AHBPERIPH_BASE + 0x0458)

#define DMA1_CH1 ((DMA_CHANNEL_TYPEDEF *)DMA1_CH1_BASE)
#define DMA1_CH2 ((DMA_CHANNEL_TYPEDEF *)DMA1_CH2_BASE)
#define DMA1_CH3 ((DMA_CHANNEL_TYPEDEF *)DMA1_CH3_BASE)
#define DMA1_CH4 ((DMA_CHANNEL_TYPEDEF *)DMA1_CH4_BASE)
#define DMA1_CH5 ((DMA_CHANNEL_TYPEDEF *)DMA1_CH5_BASE)
#define DMA1_CH6 ((DMA_CHANNEL_TYPEDEF *)DMA1_CH6_BASE)
#define DMA1_CH7 ((DMA_CHANNEL_TYPEDEF *)DMA1_CH7_BASE)
#define DMA2_CH1 ((DMA_CHANNEL_TYPEDEF *)DMA2_CH1_BASE)
#define DMA2_CH2 ((DMA_CHANNEL_TYPEDEF *)DMA2_CH2_BASE)
#define DMA2_CH3 ((DMA_CHANNEL_TYPEDEF *)DMA2_CH3_BASE)
#define DMA2_CH4 ((DMA_CHANNEL_TYPEDEF *)DMA2_CH4_BASE)
#define DMA2_CH5 ((DMA_CHANNEL_TYPEDEF *)DMA2_CH5_BASE)

#define PYGMY_DMA1_CH1_DISABLE      DMA1_CH1->CCR &= ~DMA_EN;
#define PYGMY_DMA1_CH2_DISABLE      DMA1_CH2->CCR &= ~DMA_EN;
#define PYGMY_DMA1_CH3_DISABLE      DMA1_CH3->CCR &= ~DMA_EN;
#define PYGMY_DMA1_CH4_DISABLE      DMA1_CH1->CCR &= ~DMA_EN;
#define PYGMY_DMA1_CH5_DISABLE      DMA1_CH2->CCR &= ~DMA_EN;
#define PYGMY_DMA1_CH6_DISABLE      DMA1_CH3->CCR &= ~DMA_EN;
#define PYGMY_DMA1_CH7_DISABLE      DMA1_CH1->CCR &= ~DMA_EN;
#define PYGMY_DMA2_CH1_DISABLE      DMA2_CH2->CCR &= ~DMA_EN;
#define PYGMY_DMA2_CH2_DISABLE      DMA2_CH3->CCR &= ~DMA_EN;
#define PYGMY_DMA2_CH3_DISABLE      DMA2_CH1->CCR &= ~DMA_EN;
#define PYGMY_DMA2_CH4_DISABLE      DMA2_CH2->CCR &= ~DMA_EN;
#define PYGMY_DMA2_CH5_DISABLE      DMA2_CH3->CCR &= ~DMA_EN;

#define PYGMY_DMA1_CH1_ENABLE       DMA1_CH1->CCR |= DMA_EN;
#define PYGMY_DMA1_CH2_ENABLE       DMA1_CH2->CCR |= DMA_EN;
#define PYGMY_DMA1_CH3_ENABLE       DMA1_CH3->CCR |= DMA_EN;
#define PYGMY_DMA1_CH4_ENABLE       DMA1_CH1->CCR |= DMA_EN;
#define PYGMY_DMA1_CH5_ENABLE       DMA1_CH2->CCR |= DMA_EN;
#define PYGMY_DMA1_CH6_ENABLE       DMA1_CH3->CCR |= DMA_EN;
#define PYGMY_DMA1_CH7_ENABLE       DMA1_CH1->CCR |= DMA_EN;
#define PYGMY_DMA2_CH1_ENABLE       DMA2_CH2->CCR |= DMA_EN;
#define PYGMY_DMA2_CH2_ENABLE       DMA2_CH3->CCR |= DMA_EN;
#define PYGMY_DMA2_CH3_ENABLE       DMA2_CH1->CCR |= DMA_EN;
#define PYGMY_DMA2_CH4_ENABLE       DMA2_CH2->CCR |= DMA_EN;
#define PYGMY_DMA2_CH5_ENABLE       DMA2_CH3->CCR |= DMA_EN;

/**************************************************************************
                                 FPEC
***************************************************************************/
typedef struct{
    volatile u32 ACR;
    volatile u32 KEYR;
    volatile u32 OPTKEYR;
    volatile u32 SR;
    volatile u32 CR;
    volatile u32 AR;
    volatile u32 RESERVED;
    volatile u32 OBR;
    volatile u32 WRPR;
    volatile u32 RESERVED2[ 8 ]; // XL
    volatile u32 KEYR2; // XL
    volatile u32 RESERVED3; // XL
    volatile u32 SR2; // XL
    volatile u32 CR2; // XL
    volatile u32 AR2; // XL
} FPEC_TYPEDEF;    

typedef struct{
    volatile u16 RDP;
    volatile u16 USER;
    volatile u16 Data0;
    volatile u16 Data1;   
    volatile u16 WRP0;
    volatile u16 WRP1;
    volatile u16 WRP2;
    volatile u16 WRP3;
} OB_TYPEDEF; // Option Regs

typedef struct{
    volatile u16 Pages;
} SIZEREG_TYPEDEF;

#define SIZEREG_BASE                ((u32)0x1FFFF7E0) // Number of 1KB Flash Pages in device
#define FPEC_BASE                   ((u32)0x40022000) // Flash Registers Starting Address
#define OB_BASE                     ((u32)0x1FFFF800) // Flash Option Registers Starting Address
#define FPEC                        ((FPEC_TYPEDEF *) FPEC_BASE)
//#define OB                          ((OB_TypeDef *) OB_BASE) 
#define SIZEREG                     ((SIZEREG_TYPEDEF *) SIZEREG_BASE )

#define FPEC_BASEADDRESS            0x08000000

#define FPEC_RDPRT                  0x00A5
#define FPEC_KEY1                   0x45670123
#define FPEC_KEY2                   0xCDEF89AB
#define FPEC_ACR_HLFCYA             BIT3

#define FPEC_SR_EOP                 BIT5
#define FPEC_SR_WRPRTERR            BIT4
#define FPEC_SR_PGERR               BIT2
#define FPEC_SR_BSY                 BIT0

#define FPEC_CR_EOPIE               BIT12
#define FPEC_CR_ERRIE               BIT10
#define FPEC_CR_OPTWRE              BIT9
#define FPEC_CR_LOCK                BIT7
#define FPEC_CR_STRT                BIT6
#define FPEC_CR_OPTER               BIT5
#define FPEC_CR_OPTPG               BIT4
#define FPEC_CR_MER                 BIT2
#define FPEC_CR_PER                 BIT1
#define FPEC_CR_PG                  BIT0

#define FPEC_OBR_RST_STDBY          BIT4
#define FPEC_OBR_RST_STOP           BIT3
#define FPEC_OBR_WDG_SW             BIT2
#define FPEC_OBR_RDPRT              BIT1
#define FPEC_OBR_OPTERR             BIT0

#define FPEC_ACR_PRFTBS             BIT5
#define FPEC_ACR_PRFTBE             BIT4
#define FPEC_ACR_HLFCYA             BIT3
#define FPEC_ACR_LATENCY0           0
#define FPEC_ACR_LATENCY1           BIT0
#define FPEC_ACR_LATENCY2           BIT1

#define FPEC_MAXBANK1               0x0807FFFF
#define FPEC_DENSITY_XL             BIT0
#define FPEC_DENSITY_ODDPAGE        BIT1

#define IHEX_DATA                   0x00
#define IHEX_EOF                    0x01
#define IHEX_EXTENDEDSEGMENTADDR    0x02
#define IHEX_STARTSEGMENTADDR       0x03
#define IHEX_EXTENDEDLINEARADDR     0x04
#define IHEX_STARTLINEARADDR        0x05
#define IHEX_BYTECOUNT_OFFSET       1 // 2 char field
#define IHEX_ADDR_OFFSET            3 // 4 char field
#define IHEX_TYPE_OFFSET            8 // 2 char field from 7-8, 7 always 0
#define IHEX_DATA_OFFSET            9 // ( BYTECOUNT * 2 ) long field 
/**************************************************************************
                            Public Functions
***************************************************************************/
u32 cpuGetMainClock( void );
void cpuSetMainClock( u32 freq );
u8 pinConfig( u16 ucPin, u8 ucMode );
void pinSet( u16 pin, bool state );
bool pinGet( u16 pin );

u16 pinAnalog( u8 ucPin );
u16 adcGetStatus( void );
void adcInit( void );
void adcStart( void );
u8 adcSetSampleTime( u8 ucChannel, u8 ucSampleTime );
u8 adcEnableChannel( u8 ucPin );
u8 adcGetSQR( u8 ucIndex );
u8 adcDisableChannel( u8 ucChannel );
void adcDisableAll( u8 ucChannel );
void adcSingleSampleInit( void );
u16 adcSingleSample( u8 ucPin );
u8 adcGetChannelFromPin( u8 ucPin );
u16 adcGet( u8 ucChannel );
float adcConvertRawToFloat( u16 raw );

u8 interruptGetTimerVector( u8 ucTimer );
void extiConfig( u8 ucPin, u16 uiMode );
void interruptEnable( u32 ulVector );
void interruptDisable( u32 ulVector );
void interruptSetPriority( u32 ulVector, u8 ucPriority );

void mcoEnable( u8 ucSource );
void enableTimerClock( u8 ucTimer );
void disableTimerClock( u8 ucTimer );
u8 enableTimerInterrupt( u8 ucTimer );
void *getTimer( u8 ucTimer );

void DMAChannel1_IRQHandler( void );
void ADC1_2_IRQHandler( void );

void *getCom1Uart( void );
void setCom1Uart( void* );
void setCom1TXHandler( void* );
void *getCom1TXHandler( void );
void setCom1RXHandler( void* );
void *getCom1RXHandler( void );

void *getCom2Uart( void );
void setCom2Uart( void* );
void setCom2TXHandler( void* );
void *getCom2TXHandler( void );
void setCom2RXHandler( void* );
void *getCom2RXHandler( void );

void *getCom3Uart( void );
void setCom3Uart( void* );
void setCom3TXHandler( void* );
void *getCom3TXHandler( void );
void setCom3RXHandler( void* );
void *getCom3RXHandler( void );

void *getCom4Uart( void );
void setCom4Uart( void* );
void setCom4TXHandler( void* );
void *getCom4TXHandler( void );
void setCom4RXHandler( void* );
void *getCom4RXHandler( void );

void setCom5Uart( void* );
void setCom5TXHandler( void* );
void *getCom5TXHandler( void );
void setCom5RXHandler(void* );
void *getCom5RXHandler( void );

void setCom6Uart( void* );
void setCom6TXHandler( void* );
void *getCom6TXHandler( void );
void setCom6RXHandler( void* );
void *getCom6RXHandler( void );

void setCom7Uart( void* );
void setCom7TXHandler( void* );
void *getCom7TXHandler( void );
void setCom7RXHandler( void* );
void *getCom7RXHandler( void );

void setCom8Uart( void* );
void setCom8TXHandler( void* );
void *getCom8TXHandler( void );
void setCom8RXHandler( void* );
void *getCom8RXHandler( void );

/**************************************************************************
                            Private Functions
***************************************************************************/
GPIO *pinGetPort( u16 pin );
void pinEnablePortClock( u16 pin );
void *pinGetTimer( u16 pin );
u8 pinGetChannel( u16 pin );
bool pinPWM( u16 pin, u32 frequency, u8 dutyCycle );
void pinSetSoftPWMFreq( u32 ulFreq );
u32 pinGetSoftPWMFreq( void );
void pinInitSoftPWM( void );
u8 pinRemoveSoftPWM( u8 ucPin );
u8 pinAddSoftPWM( u8 ucPin, u32 ulFreq, u8 ucDutyCycle );
void pinProcessSoftPWM( void );
u8 getDelayTimer( void );
u8 getPWMTimer( void );

void cpuComOpen( u8 port, u16 tx, u16 rx, u16 cts, u16 rts, u32 baudRate, u8 flowControl );


void putcUSART1( u8 Byte );
void putsUSART1( u8 *ucBuffer );

void putcUSART2( u8 Byte );
void putsUSART2( u8 *ucBuffer );

void putcUSART3( u8 Byte );
void putsUSART3( u8 *ucBuffer );

void putcUSART4( u8 Byte );
void putsUSART4( u8 *ucBuffer );