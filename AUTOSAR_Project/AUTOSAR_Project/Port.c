 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Youssef Samy Hassan
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"
#include "Port_Typedefs.h"
#include "SchM_Port.h"
#include "MemMap.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Dio Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif
/*******************************************************************************
 *                       Static Variables                                    *
 *******************************************************************************/
STATIC const Port_PinConfigType * Port_PinSetting = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

/*******************************************************************************
 *                      Function Definitions                                   *
 *******************************************************************************/
/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initializes the Port Driver module
************************************************************************************/
void Port_Init( const Port_ConfigType* ConfigPtr )
{
  
  Port_PinSetting =  ConfigPtr->Pins;           /*pointer to pins[0]*/
  volatile uint32 * PortGpio_Ptr = NULL_PTR;  /* point to the required Port Registers base address */
  volatile uint32 delay = 0;
  volatile uint8 count;
  
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
	if (NULL_PTR == ConfigPtr)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_CONFIG);
	}
	else
#endif
	{
          for(count = 0;count < PORT_numOfPins; count++)
          {
            /*setting the base address*/
            if ((count >= PORTA_ONSET_INDEX) && (count <= PORTA_END_INDEX))
           {
             PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;  /* PORTA Base Address */
             /* Enable clock for PORTA and allow time for clock to start*/
            SYSCTL_REGCGC2_REG |= (1<<PORTA_ID);
            delay = SYSCTL_REGCGC2_REG;
           }
           else if ((count >= PORTB_ONSET_INDEX) && (count <= PORTB_END_INDEX))
           {
             PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
             /* Enable clock for PORTB and allow time for clock to start*/
            SYSCTL_REGCGC2_REG |= (1<<PORTB_ID);
            delay = SYSCTL_REGCGC2_REG;
           }
           else if ((count >= PORTC_ONSET_INDEX) && (count <= PORTC_END_INDEX))
           {
             PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
             /*Protection for JTAG pins*/
             switch(count)
             {
             case PORTC_PIN0_ID_INDEX: continue;
                                       break;
             case PORTC_PIN1_ID_INDEX: continue;
                                       break;
             case PORTC_PIN2_ID_INDEX: continue;
                                       break;
             case PORTC_PIN3_ID_INDEX: continue;
                                       break;
             }
             /* Enable clock for PORTC and allow time for clock to start*/
             SYSCTL_REGCGC2_REG |= (1<<PORTC_ID);
             delay = SYSCTL_REGCGC2_REG;
           }
           else if ((count >= PORTD_ONSET_INDEX) && (count <= PORTD_END_INDEX))
           {
             PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
             /* Enable clock for PORTD and allow time for clock to start*/
             SYSCTL_REGCGC2_REG |= (1<<PORTD_ID);
             delay = SYSCTL_REGCGC2_REG;
           }
           else if ((count >= PORTE_ONSET_INDEX) && (count <= PORTE_END_INDEX))
           {
             PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
             /* Enable clock for PORTE and allow time for clock to start*/
            SYSCTL_REGCGC2_REG |= (1<<PORTE_ID);
            delay = SYSCTL_REGCGC2_REG;
           }
           else if ((count >= PORTF_ONSET_INDEX) && (count <= PORTF_END_INDEX))
           {
             PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
             /* Enable clock for PORTF and allow time for clock to start*/
            SYSCTL_REGCGC2_REG |= (1<<PORTF_ID);
            delay = SYSCTL_REGCGC2_REG;
           }
           else
           {
             /* Do Nothing*/
           }
           
           if((count == PORTD_PIN7_ID_INDEX)||(count == PORTF_PIN0_ID_INDEX))    /*PD7,PF0*/
           {
             /* Unlock the GPIOCR register */ 
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = PORT_UNLOCK_PASS;
            /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PinSetting[count].Pin_Num);  
           }
           else
           {
             /* Do Nothing*/
           }
           
           /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
           CLEAR_BIT((*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET)) , Port_PinSetting[count].Pin_Num);
           /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
           
           switch (Port_PinSetting[count].Pin_Mode)
           {
           case PORT_PIN_DIO_MODE:
                       /*clear the AFSEL REG FOR SELECTED PIN*/
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                      /*clear the pmc value*/
                   *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  break;
           case PORT_PIN_ADC_MODE:
             #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if given pin is from the pins supporting ADC_MODE*/
             
	 if (!(count == PORTB_PIN4_ID_INDEX)  /*PB4*/
           &&!(count == PORTB_PIN5_ID_INDEX)  /*PB5*/
           &&!(count == PORTD_PIN0_ID_INDEX)  /*PD0*/
           &&!(count == PORTD_PIN1_ID_INDEX)  /*PD1*/
           &&!(count == PORTD_PIN2_ID_INDEX)  /*PD2*/
           &&!(count == PORTD_PIN3_ID_INDEX)  /*PD3*/
           &&!(count == PORTE_PIN0_ID_INDEX)  /*PE0*/
           &&!(count == PORTE_PIN1_ID_INDEX)  /*PE1*/
           &&!(count == PORTE_PIN2_ID_INDEX)  /*PE2*/
           &&!(count == PORTE_PIN3_ID_INDEX)  /*PE3*/
           &&!(count == PORTE_PIN4_ID_INDEX)  /*PE4*/
           &&!(count == PORTE_PIN5_ID_INDEX)  /*PE5*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
                   
        { 
                 /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
                  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
        }
                  break;
         
           case PORT_PIN_UART_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if given pin is from the pins supporting UART_MODE,some UART pins could be configured for 2 UART_models like
            PC4,PC5,supports two models U4 & U1 in that case if pc4 or pc5 configured as UART this driver sets as U1*/
             
	 if (!(count == PORTB_PIN0_ID_INDEX)  /*PB0->U1RX*/
           &&!(count == PORTB_PIN1_ID_INDEX)  /*PB1->U1TX*/
           &&!(count == PORTA_PIN0_ID_INDEX)  /*PA0->U0RX*/
           &&!(count == PORTA_PIN1_ID_INDEX)  /*PA1->U0TX*/
           &&!(count == PORTC_PIN4_ID_INDEX)  /*PC4->U1RX,U4RX*/
           &&!(count == PORTC_PIN5_ID_INDEX)  /*PC5->U1TX,U4TX*/
           &&!(count == PORTC_PIN6_ID_INDEX)  /*PC6->U3RX*/
           &&!(count == PORTC_PIN7_ID_INDEX)  /*PC7->U3TX*/
           &&!(count == PORTD_PIN4_ID_INDEX)  /*PD4->U6RX*/
           &&!(count == PORTD_PIN5_ID_INDEX)  /*PD5->U6TX*/
           &&!(count == PORTD_PIN6_ID_INDEX)  /*PD6->U2RX*/
           &&!(count == PORTD_PIN7_ID_INDEX)  /*PD7->U2TX*/
           &&!(count == PORTE_PIN0_ID_INDEX)  /*PE0->U7RX*/
           &&!(count == PORTE_PIN1_ID_INDEX)  /*PE1->U7TX*/
           &&!(count == PORTE_PIN4_ID_INDEX)  /*PE4->U5RX*/
           &&!(count == PORTE_PIN5_ID_INDEX)  /*PE5->U5TX*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        {   
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  if((count == PORTC_PIN4_ID_INDEX) || ((count == PORTC_PIN5_ID_INDEX)))
                  {
                     /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0010*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_2<< (Port_PinSetting[count].Pin_Num * 4));
                  }
                  else
                  {
                    /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0001*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_1<< (Port_PinSetting[count].Pin_Num * 4));
                  }
        }
                  break;
           case PORT_PIN_SSI_MODE:
             #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if given pin is from the pins supporting SSI_MODE*/
	 if (!(count == PORTA_PIN2_ID_INDEX)  /*PA2->SSI0CLK*/
           &&!(count == PORTA_PIN3_ID_INDEX)  /*PA3->SSI0FSS*/
           &&!(count == PORTA_PIN4_ID_INDEX)  /*PA4->SSI0RX*/
           &&!(count == PORTA_PIN5_ID_INDEX)  /*PA5->SSI0TX*/
           &&!(count == PORTB_PIN4_ID_INDEX)  /*PB4->SSI2CLK*/
           &&!(count == PORTB_PIN5_ID_INDEX)  /*PB5->SSI2FSS*/
           &&!(count == PORTB_PIN6_ID_INDEX)  /*PB6->SSI2RX*/
           &&!(count == PORTB_PIN7_ID_INDEX)  /*PB7->SSI2TX*/
           &&!(count == PORTD_PIN0_ID_INDEX)  /*PD0->SSI1CLK,SSI3CLK,the driver make the default SSI3*/
           &&!(count == PORTD_PIN1_ID_INDEX)  /*PD1->SSI1FSS,SSI3FSS*/
           &&!(count == PORTD_PIN2_ID_INDEX)  /*PD2->SSI1RX,SSI3RX*/
           &&!(count == PORTD_PIN3_ID_INDEX)  /*PD3->SSI1TX,SSI3TX*/
           &&!(count == PORTF_PIN0_ID_INDEX)  /*PF0->SSI1RX*/
           &&!(count == PORTF_PIN1_ID_INDEX)  /*PF1->SSI1TX*/
           &&!(count == PORTF_PIN2_ID_INDEX)  /*PF2->SSI1CLK*/
           &&!(count == PORTF_PIN3_ID_INDEX)  /*PF3->SSI1FSS*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  if((count >= PORTD_PIN0_ID_INDEX) && ((count <= PORTD_PIN3_ID_INDEX)))
                  {/*as from PD0->PD3 SUPPORTS two SSI models(1&3) so as we set pmc=1 at pctl register we select model_3*/
                    /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0001*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_1<< (Port_PinSetting[count].Pin_Num * 4));
                  }
                  else
                  {
                   /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0001*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_2<< (Port_PinSetting[count].Pin_Num * 4));
                  }
        }
                  break;
           case PORT_PIN_I2C_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if given pin is from the pins supporting I2C_MODE*/
	 if (!(count == PORTA_PIN6_ID_INDEX)  /*PA6->I2C1SCL*/
           &&!(count == PORTA_PIN7_ID_INDEX)  /*PA7->I2C1SDA*/
           &&!(count == PORTB_PIN2_ID_INDEX)  /*PB2->I2C0SCL*/
           &&!(count == PORTB_PIN3_ID_INDEX)  /*PB3->I2C0SDA*/
           &&!(count == PORTD_PIN0_ID_INDEX)  /*PD0->I2C3SCL*/
           &&!(count == PORTD_PIN1_ID_INDEX)  /*PD1->I2C3SDA*/
           &&!(count == PORTE_PIN4_ID_INDEX)  /*PE4->I2C2SCL*/
           &&!(count == PORTE_PIN5_ID_INDEX)  /*PE5->I2C2SDA*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  
                    /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0011*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_3<< (Port_PinSetting[count].Pin_Num * 4));
        }
                  break;
           case PORT_PIN_PWM_M0_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if given pin is from the pins supporting PWM_M0_MODE*/
	 if (!(count == PORTB_PIN4_ID_INDEX)  /*PB4->M0PWM2*/
           &&!(count == PORTB_PIN5_ID_INDEX)  /*PB5->M0PWM3*/
           &&!(count == PORTB_PIN6_ID_INDEX)  /*PB6->M0PWM0*/
           &&!(count == PORTB_PIN7_ID_INDEX)  /*PB7->M0PWM1*/
           &&!(count == PORTC_PIN4_ID_INDEX)  /*PC4->M0PWM6*/
           &&!(count == PORTC_PIN5_ID_INDEX)  /*PC5->M0PWM7*/
           &&!(count == PORTD_PIN0_ID_INDEX)  /*PD0->M0PWM6*/
           &&!(count == PORTD_PIN1_ID_INDEX)  /*PD1->M0PWM7*/
           &&!(count == PORTE_PIN4_ID_INDEX)  /*PE4->M0PWM4*/
           &&!(count == PORTE_PIN5_ID_INDEX)  /*PE5->M0PWM5*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  
                  /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0100*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_4<< (Port_PinSetting[count].Pin_Num * 4));
        }
                  break;
          case PORT_PIN_PWM_M1_MODE:
             #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if given pin is from the pins supporting PWM_M1_MODE*/
	 if (!(count == PORTA_PIN6_ID_INDEX)  /*PA6->M1PWM2*/
           &&!(count == PORTA_PIN7_ID_INDEX)  /*PA7->M1PWM3*/
           &&!(count == PORTD_PIN1_ID_INDEX)  /*PD1->M1PWM1*/
           &&!(count == PORTE_PIN4_ID_INDEX)  /*PE4->M1PWM2*/
           &&!(count == PORTE_PIN5_ID_INDEX)  /*PE5->M1PWM3*/
           &&!(count == PORTF_PIN0_ID_INDEX)  /*PF0->M1PWM4*/
           &&!(count == PORTF_PIN1_ID_INDEX)  /*PF1->M1PWM5*/
           &&!(count == PORTF_PIN2_ID_INDEX)  /*PF2->M1PWM6*/
           &&!(count == PORTF_PIN3_ID_INDEX)  /*PF3->M1PWM7*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  
                 /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0101*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_5<< (Port_PinSetting[count].Pin_Num * 4));
        }
                  break;
         
           case PORT_PIN_FAULT_MODE:
               #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting FAULT_MODE*/
	 if (!(count == PORTD_PIN2_ID_INDEX)  /*PD2->M0FAULT0*/
           &&!(count == PORTD_PIN6_ID_INDEX)  /*PD6->M0FAULT0*/
           &&!(count == PORTF_PIN2_ID_INDEX)  /*PF2->M0FAULT0*/
           &&!(count == PORTF_PIN4_ID_INDEX)  /*PF4->M1FAULT1*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        {              /*SET Corresponding bit in AFSEL REG, to choose alternate function,for this bit*/
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  
                  if(count == PORTF_PIN4_ID_INDEX)
                  {
                     /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0101,,M1FAULT1*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_5<< (Port_PinSetting[count].Pin_Num * 4));
                  }
                  else
                  {
                     /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0100,M0FAULT0*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_4<< (Port_PinSetting[count].Pin_Num * 4));
                  }
        }
                  break;
          
           case PORT_PIN_QEI_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting QEI_MODE*/
	 if (!(count == PORTC_PIN4_ID_INDEX)  /*PC4->IDX1*/
           &&!(count == PORTC_PIN5_ID_INDEX)  /*PC5->PHA1*/
           &&!(count == PORTC_PIN6_ID_INDEX)  /*PC6->PHB1*/
           &&!(count == PORTD_PIN3_ID_INDEX)  /*PD3->IDX0*/
           &&!(count == PORTD_PIN6_ID_INDEX)  /*PD6->PHA0*/
           &&!(count == PORTD_PIN7_ID_INDEX)  /*PD7->PHB0*/
           &&!(count == PORTF_PIN0_ID_INDEX)  /*PF0->PHA0*/
           &&!(count == PORTF_PIN1_ID_INDEX)  /*PF1->PHB0*/
           &&!(count == PORTF_PIN4_ID_INDEX)  /*PF4->IDX0*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
             
        {  
           /*SET Corresponding bit in AFSEL REG, to choose alternate function,for this bit*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
         /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0110*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_6<< (Port_PinSetting[count].Pin_Num * 4));
        }
                  break;
           case PORT_PIN_GPT_MODE:
             #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting GPT_MODE*/
	 if (!((count >= PORTB_ONSET_INDEX)&&(count <= PORTB_END_INDEX))         /*PB0->PB7*/
           &&!((count >= PORTC_PIN4_ID_INDEX)&&(count <= PORTC_PIN7_ID_INDEX))  /*PC4->PC7*/
           &&!((count >= PORTD_PIN1_ID_INDEX)&&(count <= PORTD_PIN7_ID_INDEX))  /*PD1->PD7*/
           &&!((count >= PORTF_ONSET_INDEX)&&(count <=PORTF_END_INDEX))        /*PF0->PF4*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        { 
          /*SET Corresponding bit in AFSEL REG, to choose alternate function,for this bit*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
         /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0111*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_7<< (Port_PinSetting[count].Pin_Num * 4));
           
        }
                  break;
           case PORT_PIN_CAN_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting CAN_MODE*/
             if(!(count == PORTA_PIN0_ID_INDEX)        /*PA0->CAN1RX*/
              &&!(count == PORTA_PIN1_ID_INDEX)         /*PA1->CAN1TX*/
              &&!(count == PORTB_PIN4_ID_INDEX)         /*PB4->CAN0RX*/
              &&!(count == PORTB_PIN5_ID_INDEX)         /*PB5->CAN0TX*/
              &&!(count == PORTE_PIN4_ID_INDEX)        /*PE4->CAN0RX*/
              &&!(count == PORTE_PIN5_ID_INDEX)        /*PE5->CAN0TX*/
              &&!(count == PORTF_PIN0_ID_INDEX)       /*PF0->CAN0RX*/
              &&!(count == PORTF_PIN3_ID_INDEX)       /*PF3->CAN0TX*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
             
        {  
          /*SET Corresponding bit in AFSEL REG, to choose alternate function,for this bit*/
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  if((count == PORTF_PIN0_ID_INDEX) || (count == PORTF_PIN3_ID_INDEX))
                  {
                     /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0011*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_3<< (Port_PinSetting[count].Pin_Num * 4));
                  }
                  else
                  {
                     /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 1000*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_8<< (Port_PinSetting[count].Pin_Num * 4));
                  }
        }    
                  break;
           case PORT_PIN_NMI_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting NMI_MODE*/
             if(!(count == PORTD_PIN7_ID_INDEX)&&!(count == PORTF_PIN0_ID_INDEX))
            
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
             
        {  
                  /*SET Corresponding bit in AFSEL REG, to choose alternate function,for this bit*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
         /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 1000*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_8<< (Port_PinSetting[count].Pin_Num * 4));
           
        }
                  break;
           case PORT_PIN_UART_CONTROL_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting CONTROL_MODE_MODE*/
               if(!(count == PORTF_PIN0_ID_INDEX)  /*PF0->U1RTS,PMC=1*/
                &&!(count == PORTF_PIN1_ID_INDEX)  /*PF1->U1CTS*/
                &&!(count == PORTC_PIN4_ID_INDEX)  /*PC4->U1RTS,PMC=8*/
                &&!(count == PORTC_PIN5_ID_INDEX)  /*PC5->U1CTS*/
                  )
            
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
             
        {         /*SET Corresponding bit in AFSEL REG, to choose alternate function,for this bit*/
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  if(count == PORTF_PIN0_ID_INDEX ||count == PORTF_PIN0_ID_INDEX) {
                    /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0001*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_1<< (Port_PinSetting[count].Pin_Num * 4));
                  }
                    else{
                      /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 1000*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_8<< (Port_PinSetting[count].Pin_Num * 4));
                    }
        }       
                  break;
           case PORT_PIN_USB_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting USB_MODE*/
               if(!(count == PORTC_PIN6_ID_INDEX)  /*PC6->USB0EPEN*/
                &&!(count == PORTC_PIN7_ID_INDEX)  /*PC7->USB0PFLT*/
                &&!(count == PORTD_PIN3_ID_INDEX)  /*PD3->USB0EPEN*/
                &&!(count == PORTD_PIN4_ID_INDEX)  /*PD4->USB0PFLT*/
                  )
            
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
             
        {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                   /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 1000*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_8<< (Port_PinSetting[count].Pin_Num * 4));
        } 
                  break;
           case PORT_PIN_Core_MODE:
             #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting CORE_MODE*/
               if(!(count == PORTF_PIN1_ID_INDEX)  /*PF1->TRD1*/
                &&!(count == PORTF_PIN2_ID_INDEX)  /*PF2->TRD0*/
                &&!(count == PORTF_PIN3_ID_INDEX)  /*PD3->TRCLK*/
                  )
            
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
             
        {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                   /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[count].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 1110*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_14<< (Port_PinSetting[count].Pin_Num * 4));
        }
                  break;
            default:
                  /*Do Nothing*/
                  break;
           }
           if(Port_PinSetting[count].Pin_Dir == PORT_PIN_OUT)
           {
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinSetting[count].Pin_Num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
             if(Port_PinSetting[count].Pin_initalValue == PORT_PIN_HIGH)
             {
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PinSetting[count].Pin_Num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
             }
             else
             {
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PinSetting[count].Pin_Num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
             }
           }
           else if (Port_PinSetting[count].Pin_Dir == PORT_PIN_IN)
           {
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinSetting[count].Pin_Num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
             
             if(Port_PinSetting[count].Pin_internalResistor == PULL_UP)
             {
                 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PinSetting[count].Pin_Num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
             }
             else if(Port_PinSetting[count].Pin_internalResistor == PULL_DOWN)
             {
                 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PinSetting[count].Pin_Num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
             }
             else
             {
                 CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PinSetting[count].Pin_Num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                 CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PinSetting[count].Pin_Num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
             }
           }
           else
           {
             /*Do Nothing*/
           }
          }       
          Port_Status = PORT_INITIALIZED;
	}
}

/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): - Pin - Port Pin ID number
*                  - Direction - Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction
************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(
                          Port_PinType Pin, 
                          Port_PinDirectionType Direction
                          )
{
  boolean error = FALSE;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  {
    /* Check if the Driver is initialized before using this function */
    if(Port_Status == PORT_NOT_INITIALIZED)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,Port_SetPinDirection_SID,PORT_E_UNINIT);
    error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
    if(Pin >= PORT_numOfPins)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_SetPinDirection_SID,PORT_E_PARAM_PIN);
     error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
    if(Port_PinSetting[Pin].Pin_dirChangeable == STD_OFF)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_SetPinDirection_SID,PORT_E_DIRECTION_UNCHANGEABLE);
     error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
  }
#endif
  volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
  if(error == FALSE)
  {
      /*setting the base address*/
      if ((Pin >= PORTA_ONSET_INDEX) && (Pin <= PORTA_END_INDEX))
     {
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;  /* PORTA Base Address */
     }
     else if ((Pin >= PORTB_ONSET_INDEX) && (Pin <= PORTB_END_INDEX))
     {
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
     }
     else if ((Pin >= PORTC_ONSET_INDEX) && (Pin <= PORTC_END_INDEX))
     {
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
     }
     else if ((Pin >= PORTD_ONSET_INDEX) && (Pin <= PORTD_END_INDEX))
     {
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
     }
     else if ((Pin >= PORTE_ONSET_INDEX) && (Pin <= PORTE_END_INDEX))
     {
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
     }
     else if ((Pin >= PORTF_ONSET_INDEX) && (Pin <= PORTF_END_INDEX))
     {
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
     }
     else
     {
       /* Do Nothing*/
     }
     if(Direction == PORT_PIN_OUT)
     {
       SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);     /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
     }
     else
     {
       CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);   /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
     }
  }
  else
  {
    /* Do Nothing */
  }
}
#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction
************************************************************************************/
void Port_RefreshPortDirection(
                               void
                               )
{
  boolean error = FALSE;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if(Port_Status == PORT_NOT_INITIALIZED)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_RefreshPortDirection_SID,PORT_E_UNINIT);
    error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
#endif
  volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
  volatile uint8 count;
  if(error == FALSE)
  {
    for(count = 0; count < PORT_numOfPins; count++)
    {
      /* PORT061: The function Port_RefreshPortDirection shall exclude those port pins from
			 * refreshing that are configured as ‘pin direction changeable during runtime‘ */
      if (Port_PinSetting[count].Pin_dirChangeable == STD_ON)
      {
        continue;/*escape this pin from this iteration*/
      }
      else
      {
        /*setting the base address*/
        if ((PORTA_ONSET_INDEX <= count ) && (count <= PORTA_END_INDEX))
        {
          PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;  /* PORTA Base Address */
        }
        else if ((count >= PORTB_ONSET_INDEX) && (count <= PORTB_END_INDEX))
        {
          PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
        }
        else if ((count >= PORTC_ONSET_INDEX) && (count <= PORTC_END_INDEX))
        {
          PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
        }
        else if ((count >= PORTD_ONSET_INDEX) && (count <= PORTD_END_INDEX))
        {
          PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
        }
        else if ((count >= PORTE_ONSET_INDEX) && (count <= PORTE_END_INDEX))
        {
          PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
        }
        else if ((count >= PORTF_ONSET_INDEX) && (count <= PORTF_END_INDEX))
        {
          PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
        }
        else
        {
          /* Do Nothing*/
        }
        /* Refresh Pins Direction */
        if(Port_PinSetting[count].Pin_Dir == PORT_PIN_OUT)
        {
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinSetting[count].Pin_Num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        }
        else
        {
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinSetting[count].Pin_Num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        }
      }
    }
  }
  else
  {
    /* Do Nothing */
  }
}

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in):None
* Parameters (inout): None
* Parameters (out):  VersionInfo - Pointer to where to store the version information of this module.
* Return value: None
* Description: Returns the version information of this module
************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(
                         Std_VersionInfoType* versioninfo
                         )
{
    boolean error = FALSE;
#if(PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if(Port_Status == Port_NOT_INITIALIZED)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_GetVersionInfo_SID,PORT_E_UNINIT);
    error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
    /* Check if the parameter versioninfo value is a NULL pointer */
    if(versioninfo == NULL_PTR)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_GetVersionInfo_SID,PORT_E_PARAM_POINTER);
    error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
#endif
    if(error == FALSE)
    {
      /* Copy the vendor Id */
      versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
      /* Copy the module Id */
      versioninfo->moduleID = (uint16)PORT_MODULE_ID;
      /* Copy Software Major Version */
      versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
      /* Copy Software Minor Version */
      versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
      /* Copy Software Patch Version */
      versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
    else
    {
      /*Do Nothing*/
    }
}
#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): - Pin - Port Pin ID number
*                  - Mode - New Port Pin mode to be set on port pin
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode
************************************************************************************/
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode(
                     Port_PinType Pin, 
                     Port_PinModeType Mode
                     )
{
  boolean error = FALSE;
#if(PORT_DEV_ERROR_DETECT == STD_ON)
  {
    /* Check if the Driver is initialized before using this function */
    if(Port_Status == PORT_NOT_INITIALIZED)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_SetPinMode_SID,PORT_E_UNINIT);
    error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
    if(Pin >= PORT_numOfPins)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_SetPinMode_SID,PORT_E_PARAM_PIN);
     error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
    if(Port_PinSetting[Pin].Pin_modeChangeable == STD_OFF)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_SetPinMode_SID,PORT_E_DIRECTION_UNCHANGEABLE);
     error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
    if(Mode >= PORT_numOfConfiguriedMode)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_SetPinMode_SID,PORT_E_PARAM_INVALID_MODE);
     error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
  }
#endif
  volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
  if(error == FALSE)
  {
      /*setting the base address*/
      if ((Pin >= PORTA_ONSET_INDEX) && (Pin <= PORTA_END_INDEX)) 
     { 
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;  /* PORTA Base Address */ 
     } 
     else if ((Pin >= PORTB_ONSET_INDEX) && (Pin <= PORTB_END_INDEX)) 
     { 
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */ 
     } 
     else if ((Pin >= PORTC_ONSET_INDEX) && (Pin <= PORTC_END_INDEX)) 
     { 
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */ 
     } 
     else if ((Pin >= PORTD_ONSET_INDEX) && (Pin <= PORTD_END_INDEX)) 
     { 
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */ 
     } 
     else if ((Pin >= PORTE_ONSET_INDEX) && (Pin <= PORTE_END_INDEX)) 
     { 
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */ 
     } 
     else if ((Pin >= PORTF_ONSET_INDEX) && (Pin <= PORTF_END_INDEX)) 
     { 
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */ 
     } 
     else 
     { 
       /* Do Nothing*/ 
     }
	 /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
     CLEAR_BIT((*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET)) , Port_PinSetting[Pin].Pin_Num);
     /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
     SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
      switch (Mode)
           {
           case PORT_PIN_DIO_MODE:
                       /*clear the AFSEL REG FOR SELECTED PIN*/
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                      /*clear the pmc value*/
                   *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  break;
           case PORT_PIN_ADC_MODE:
             #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if given pin is from the pins supporting ADC_MODE*/
             
	 if (!(Pin == PORTB_PIN4_ID_INDEX)  /*PB4*/
           &&!(Pin == PORTB_PIN5_ID_INDEX)  /*PB5*/
           &&!(Pin == PORTD_PIN0_ID_INDEX)  /*PD0*/
           &&!(Pin == PORTD_PIN1_ID_INDEX)  /*PD1*/
           &&!(Pin == PORTD_PIN2_ID_INDEX)  /*PD2*/
           &&!(Pin == PORTD_PIN3_ID_INDEX)  /*PD3*/
           &&!(Pin == PORTE_PIN0_ID_INDEX)  /*PE0*/
           &&!(Pin == PORTE_PIN1_ID_INDEX)  /*PE1*/
           &&!(Pin == PORTE_PIN2_ID_INDEX)  /*PE2*/
           &&!(Pin == PORTE_PIN3_ID_INDEX)  /*PE3*/
           &&!(Pin == PORTE_PIN4_ID_INDEX)  /*PE4*/
           &&!(Pin == PORTE_PIN5_ID_INDEX)  /*PE5*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
                   
        { 
                 /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                  /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                  /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
                  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                  /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
        }
                  break;
         
           case PORT_PIN_UART_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if given pin is from the pins supporting UART_MODE,some UART pins could be configured for 2 UART_models like
            PC4,PC5,supports two models U4 & U1 in that case if pc4 or pc5 configured as UART this driver sets as U1*/
             
	 if (!(Pin == PORTB_PIN0_ID_INDEX)  /*PB0->U1RX*/
           &&!(Pin == PORTB_PIN1_ID_INDEX)  /*PB1->U1TX*/
           &&!(Pin == PORTA_PIN0_ID_INDEX)  /*PA0->U0RX*/
           &&!(Pin == PORTA_PIN1_ID_INDEX)  /*PA1->U0TX*/
           &&!(Pin == PORTC_PIN4_ID_INDEX)  /*PC4->U1RX,U4RX*/
           &&!(Pin == PORTC_PIN5_ID_INDEX)  /*PC5->U1TX,U4TX*/
           &&!(Pin == PORTC_PIN6_ID_INDEX)  /*PC6->U3RX*/
           &&!(Pin == PORTC_PIN7_ID_INDEX)  /*PC7->U3TX*/
           &&!(Pin == PORTD_PIN4_ID_INDEX)  /*PD4->U6RX*/
           &&!(Pin == PORTD_PIN5_ID_INDEX)  /*PD5->U6TX*/
           &&!(Pin == PORTD_PIN6_ID_INDEX)  /*PD6->U2RX*/
           &&!(Pin == PORTD_PIN7_ID_INDEX)  /*PD7->U2TX*/
           &&!(Pin == PORTE_PIN0_ID_INDEX)  /*PE0->U7RX*/
           &&!(Pin == PORTE_PIN1_ID_INDEX)  /*PE1->U7TX*/
           &&!(Pin == PORTE_PIN4_ID_INDEX)  /*PE4->U5RX*/
           &&!(Pin == PORTE_PIN5_ID_INDEX)  /*PE5->U5TX*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        {   
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                  if((Pin == PORTC_PIN4_ID_INDEX) || ((Pin == PORTC_PIN5_ID_INDEX)))
                  {
                     /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0010*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_2<< (Port_PinSetting[Pin].Pin_Num * 4));
                  }
                  else
                  {
                    /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0001*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_1<< (Port_PinSetting[Pin].Pin_Num * 4));
                  }
        }
                  break;
           case PORT_PIN_SSI_MODE:
             #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if given pin is from the pins supporting SSI_MODE*/
	 if (!(Pin == PORTA_PIN2_ID_INDEX)  /*PA2->SSI0CLK*/
           &&!(Pin == PORTA_PIN3_ID_INDEX)  /*PA3->SSI0FSS*/
           &&!(Pin == PORTA_PIN4_ID_INDEX)  /*PA4->SSI0RX*/
           &&!(Pin == PORTA_PIN5_ID_INDEX)  /*PA5->SSI0TX*/
           &&!(Pin == PORTB_PIN4_ID_INDEX)  /*PB4->SSI2CLK*/
           &&!(Pin == PORTB_PIN5_ID_INDEX)  /*PB5->SSI2FSS*/
           &&!(Pin == PORTB_PIN6_ID_INDEX)  /*PB6->SSI2RX*/
           &&!(Pin == PORTB_PIN7_ID_INDEX)  /*PB7->SSI2TX*/
           &&!(Pin == PORTD_PIN0_ID_INDEX)  /*PD0->SSI1CLK,SSI3CLK,the driver make the default SSI3*/
           &&!(Pin == PORTD_PIN1_ID_INDEX)  /*PD1->SSI1FSS,SSI3FSS*/
           &&!(Pin == PORTD_PIN2_ID_INDEX)  /*PD2->SSI1RX,SSI3RX*/
           &&!(Pin == PORTD_PIN3_ID_INDEX)  /*PD3->SSI1TX,SSI3TX*/
           &&!(Pin == PORTF_PIN0_ID_INDEX)  /*PF0->SSI1RX*/
           &&!(Pin == PORTF_PIN1_ID_INDEX)  /*PF1->SSI1TX*/
           &&!(Pin == PORTF_PIN2_ID_INDEX)  /*PF2->SSI1CLK*/
           &&!(Pin == PORTF_PIN3_ID_INDEX)  /*PF3->SSI1FSS*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                  if((Pin >= PORTD_PIN0_ID_INDEX) && ((Pin <= PORTD_PIN3_ID_INDEX)))
                  {/*as from PD0->PD3 SUPPORTS two SSI models(1&3) so as we set pmc=1 at pctl register we select model_3*/
                    /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0001*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_1<< (Port_PinSetting[Pin].Pin_Num * 4));
                  }
                  else
                  {
                   /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0001*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_2<< (Port_PinSetting[Pin].Pin_Num * 4));
                  }
        }
                  break;
           case PORT_PIN_I2C_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if given pin is from the pins supporting I2C_MODE*/
	 if (!(Pin == PORTA_PIN6_ID_INDEX)  /*PA6->I2C1SCL*/
           &&!(Pin == PORTA_PIN7_ID_INDEX)  /*PA7->I2C1SDA*/
           &&!(Pin == PORTB_PIN2_ID_INDEX)  /*PB2->I2C0SCL*/
           &&!(Pin == PORTB_PIN3_ID_INDEX)  /*PB3->I2C0SDA*/
           &&!(Pin == PORTD_PIN0_ID_INDEX)  /*PD0->I2C3SCL*/
           &&!(Pin == PORTD_PIN1_ID_INDEX)  /*PD1->I2C3SDA*/
           &&!(Pin == PORTE_PIN4_ID_INDEX)  /*PE4->I2C2SCL*/
           &&!(Pin == PORTE_PIN5_ID_INDEX)  /*PE5->I2C2SDA*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                  
                    /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0011*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_3<< (Port_PinSetting[Pin].Pin_Num * 4));
        }
                  break;
           case PORT_PIN_PWM_M0_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if given pin is from the pins supporting PWM_M0_MODE*/
	 if (!(Pin == PORTB_PIN4_ID_INDEX)  /*PB4->M0PWM2*/
           &&!(Pin == PORTB_PIN5_ID_INDEX)  /*PB5->M0PWM3*/
           &&!(Pin == PORTB_PIN6_ID_INDEX)  /*PB6->M0PWM0*/
           &&!(Pin == PORTB_PIN7_ID_INDEX)  /*PB7->M0PWM1*/
           &&!(Pin == PORTC_PIN4_ID_INDEX)  /*PC4->M0PWM6*/
           &&!(Pin == PORTC_PIN5_ID_INDEX)  /*PC5->M0PWM7*/
           &&!(Pin == PORTD_PIN0_ID_INDEX)  /*PD0->M0PWM6*/
           &&!(Pin == PORTD_PIN1_ID_INDEX)  /*PD1->M0PWM7*/
           &&!(Pin == PORTE_PIN4_ID_INDEX)  /*PE4->M0PWM4*/
           &&!(Pin == PORTE_PIN5_ID_INDEX)  /*PE5->M0PWM5*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                  
                  /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0100*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_4<< (Port_PinSetting[Pin].Pin_Num * 4));
        }
                  break;
          case PORT_PIN_PWM_M1_MODE:
             #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if given pin is from the pins supporting PWM_M1_MODE*/
	 if (!(Pin == PORTA_PIN6_ID_INDEX)  /*PA6->M1PWM2*/
           &&!(Pin == PORTA_PIN7_ID_INDEX)  /*PA7->M1PWM3*/
           &&!(Pin == PORTD_PIN1_ID_INDEX)  /*PD1->M1PWM1*/
           &&!(Pin == PORTE_PIN4_ID_INDEX)  /*PE4->M1PWM2*/
           &&!(Pin == PORTE_PIN5_ID_INDEX)  /*PE5->M1PWM3*/
           &&!(Pin == PORTF_PIN0_ID_INDEX)  /*PF0->M1PWM4*/
           &&!(Pin == PORTF_PIN1_ID_INDEX)  /*PF1->M1PWM5*/
           &&!(Pin == PORTF_PIN2_ID_INDEX)  /*PF2->M1PWM6*/
           &&!(Pin == PORTF_PIN3_ID_INDEX)  /*PF3->M1PWM7*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                  
                 /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0101*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_5<< (Port_PinSetting[Pin].Pin_Num * 4));
        }
                  break;
         
           case PORT_PIN_FAULT_MODE:
               #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting FAULT_MODE*/
	 if (!(Pin == PORTD_PIN2_ID_INDEX)  /*PD2->M0FAULT0*/
           &&!(Pin == PORTD_PIN6_ID_INDEX)  /*PD6->M0FAULT0*/
           &&!(Pin == PORTF_PIN2_ID_INDEX)  /*PF2->M0FAULT0*/
           &&!(Pin == PORTF_PIN4_ID_INDEX)  /*PF4->M1FAULT1*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        {              /*SET Corresponding bit in AFSEL REG, to choose alternate function,for this bit*/
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                  
                  if(Pin == PORTF_PIN4_ID_INDEX)
                  {
                     /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0101,,M1FAULT1*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_5<< (Port_PinSetting[Pin].Pin_Num * 4));
                  }
                  else
                  {
                     /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0100,M0FAULT0*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_4<< (Port_PinSetting[Pin].Pin_Num * 4));
                  }
        }
                  break;
          
           case PORT_PIN_QEI_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting QEI_MODE*/
	 if (!(Pin == PORTC_PIN4_ID_INDEX)  /*PC4->IDX1*/
           &&!(Pin == PORTC_PIN5_ID_INDEX)  /*PC5->PHA1*/
           &&!(Pin == PORTC_PIN6_ID_INDEX)  /*PC6->PHB1*/
           &&!(Pin == PORTD_PIN3_ID_INDEX)  /*PD3->IDX0*/
           &&!(Pin == PORTD_PIN6_ID_INDEX)  /*PD6->PHA0*/
           &&!(Pin == PORTD_PIN7_ID_INDEX)  /*PD7->PHB0*/
           &&!(Pin == PORTF_PIN0_ID_INDEX)  /*PF0->PHA0*/
           &&!(Pin == PORTF_PIN1_ID_INDEX)  /*PF1->PHB0*/
           &&!(Pin == PORTF_PIN4_ID_INDEX)  /*PF4->IDX0*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
             
        {  
           /*SET Corresponding bit in AFSEL REG, to choose alternate function,for this bit*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
         /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0110*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_6<< (Port_PinSetting[Pin].Pin_Num * 4));
        }
                  break;
           case PORT_PIN_GPT_MODE:
             #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting GPT_MODE*/
	 if (!((Pin >= PORTB_ONSET_INDEX)&&(Pin <= PORTB_END_INDEX))         /*PB0->PB7*/
           &&!((Pin >= PORTC_PIN4_ID_INDEX)&&(Pin <= PORTC_PIN7_ID_INDEX))  /*PC4->PC7*/
           &&!((Pin >= PORTD_PIN1_ID_INDEX)&&(Pin <= PORTD_PIN7_ID_INDEX))  /*PD1->PD7*/
           &&!((Pin >= PORTF_ONSET_INDEX)&&(Pin <=PORTF_END_INDEX))        /*PF0->PF4*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
        { 
          /*SET Corresponding bit in AFSEL REG, to choose alternate function,for this bit*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
         /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0111*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_7<< (Port_PinSetting[Pin].Pin_Num * 4));
           
        }
                  break;
           case PORT_PIN_CAN_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting CAN_MODE*/
             if(!(Pin == PORTA_PIN0_ID_INDEX)        /*PA0->CAN1RX*/
              &&!(Pin == PORTA_PIN1_ID_INDEX)         /*PA1->CAN1TX*/
              &&!(Pin == PORTB_PIN4_ID_INDEX)         /*PB4->CAN0RX*/
              &&!(Pin == PORTB_PIN5_ID_INDEX)         /*PB5->CAN0TX*/
              &&!(Pin == PORTE_PIN4_ID_INDEX)        /*PE4->CAN0RX*/
              &&!(Pin == PORTE_PIN5_ID_INDEX)        /*PE5->CAN0TX*/
              &&!(Pin == PORTF_PIN0_ID_INDEX)       /*PF0->CAN0RX*/
              &&!(Pin == PORTF_PIN3_ID_INDEX)       /*PF3->CAN0TX*/
            )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
             
        {  
          /*SET Corresponding bit in AFSEL REG, to choose alternate function,for this bit*/
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                  if((Pin == PORTF_PIN0_ID_INDEX) || (Pin == PORTF_PIN3_ID_INDEX))
                  {
                     /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0011*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_3<< (Port_PinSetting[Pin].Pin_Num * 4));
                  }
                  else
                  {
                     /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 1000*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_8<< (Port_PinSetting[Pin].Pin_Num * 4));
                  }
        }    
                  break;
           case PORT_PIN_NMI_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting NMI_MODE*/
             if(!(Pin == PORTD_PIN7_ID_INDEX)&&!(Pin == PORTF_PIN0_ID_INDEX))
            
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
             
        {  
                  /*SET Corresponding bit in AFSEL REG, to choose alternate function,for this bit*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
         /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 1000*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_8<< (Port_PinSetting[Pin].Pin_Num * 4));
           
        }
                  break;
           case PORT_PIN_UART_CONTROL_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting CONTROL_MODE_MODE*/
               if(!(Pin == PORTF_PIN0_ID_INDEX)  /*PF0->U1RTS,PMC=1*/
                &&!(Pin == PORTF_PIN1_ID_INDEX)  /*PF1->U1CTS*/
                &&!(Pin == PORTC_PIN4_ID_INDEX)  /*PC4->U1RTS,PMC=8*/
                &&!(Pin == PORTC_PIN5_ID_INDEX)  /*PC5->U1CTS*/
                  )
            
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
             
        {         /*SET Corresponding bit in AFSEL REG, to choose alternate function,for this bit*/
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                  if(Pin == PORTF_PIN0_ID_INDEX ||Pin == PORTF_PIN0_ID_INDEX) {
                    /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 0001*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_1<< (Port_PinSetting[Pin].Pin_Num * 4));
                  }
                    else{
                      /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 1000*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_8<< (Port_PinSetting[Pin].Pin_Num * 4));
                    }
        }       
                  break;
           case PORT_PIN_USB_MODE:
              #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting USB_MODE*/
               if(!(Pin == PORTC_PIN6_ID_INDEX)  /*PC6->USB0EPEN*/
                &&!(Pin == PORTC_PIN7_ID_INDEX)  /*PC7->USB0PFLT*/
                &&!(Pin == PORTD_PIN3_ID_INDEX)  /*PD3->USB0EPEN*/
                &&!(Pin == PORTD_PIN4_ID_INDEX)  /*PD4->USB0PFLT*/
                  )
            
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
             
        {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                   /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 1000*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_8<< (Port_PinSetting[Pin].Pin_Num * 4));
        } 
                  break;
           case PORT_PIN_Core_MODE:
             #if (PORT_DEV_ERROR_DETECT == STD_ON)
             /* check if given pin is from the pins supporting CORE_MODE*/
               if(!(Pin == PORTF_PIN1_ID_INDEX)  /*PF1->TRD1*/
                &&!(Pin == PORTF_PIN2_ID_INDEX)  /*PF2->TRD0*/
                &&!(Pin == PORTF_PIN3_ID_INDEX)  /*PD3->TRCLK*/
                  )
            
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,Port_Init_SID, PORT_E_PARAM_INVALID_MODE);
	}
	else
#endif
             
        {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);
                   /* Clear the corresponding pmcx value in PORT_CTL_REG_OFFSET,just to make sure before setting the pmc_value
                   corresponds to the mode ,that this bit area is zeroes,for ensuring setting the right value */
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(CLEAR_Mask_PMCx_4_bits << (Port_PinSetting[Pin].Pin_Num * 4));
                  /*set the corresponding pmc_value to the bit field area to 1110*/
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |=(PMC_value_14<< (Port_PinSetting[Pin].Pin_Num * 4));
        }
                  break;
            default:
                  /*Do Nothing*/
                  break;
           }

  }
  else
  {
    /*Do Nothing*/
  }
}
#endif
