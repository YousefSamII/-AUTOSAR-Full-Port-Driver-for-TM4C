 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: PORT.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - PORT Driver
 *
 * Author: Youssef Samy Hassan
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

//* Id for the company in the AUTOSAR
 /*for example Mohamed Tarek's ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)

/* Port Module Id */
#define PORT_MODULE_ID    (124U)

/* Port Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/* Standard AUTOSAR types */
#include "Std_Types.h"
/*non autosar file*/
#include "Port_Typedefs.h"
/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for Port initialization*/
#define Port_Init_SID                       (uint8)0x00
/* Service ID for change Port direction during runtime */
#define Port_SetPinDirection_SID            (uint8)0x01
/* Service ID for refresh Port direction */
#define Port_RefreshPortDirection_SID       (uint8)0x02 
/* Service ID for Port get version info */
#define Port_GetVersionInfo_SID             (uint8)0x03
/* Service ID for change Port mode during runtime*/
#define Port_SetPinMode_SID                 (uint8)0x04
/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* DET code to report incorrect Port Pin ID passed */
#define PORT_E_PARAM_PIN                        (uint8)0x0A
/* DET code to report Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE           (uint8)0x0B
/* DET code to report Port_Init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG                     (uint8)0x0C
/* DET code to report Port Pin Mode passed not valid */
#define PORT_E_PARAM_INVALID_MODE               (uint8)0x0D
/* DET code to report Port_SetPinMode service called when the mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE                (uint8)0x0E
/* DET code to report API service called prior to module initialization */
#define PORT_E_UNINIT                           (uint8)0x0F
/* DET code to report Api called with a NULL Pointer Parameter */
#define PORT_E_PARAM_POINTER                    (uint8)0x10
/*******************************************************************************
 *                     Preprocessor Definations                                *
 *******************************************************************************/
/*PORT IDs used to provide the clock for each port */
#define PORTA_ID                               (uint8)0x00
#define PORTB_ID                               (uint8)0x01
#define PORTC_ID                               (uint8)0x02
#define PORTD_ID                               (uint8)0x03
#define PORTE_ID                               (uint8)0x04
#define PORTF_ID                               (uint8)0x05   
/* Magic number used in lock register to unlock specific pins */
#define PORT_UNLOCK_PASS                       (uint32)0x4C4F434B
/* Modes PMC_values used in CTL register to select specific mode */
#define CLEAR_Mask_PMCx_4_bits                  (uint32)0x0000000F
#define PMC_value_1                             (uint32)0x00000001
#define PMC_value_2                             (uint32)0x00000002
#define PMC_value_3                             (uint32)0x00000003
#define PMC_value_4                             (uint32)0x00000004 
#define PMC_value_5                             (uint32)0x00000005
#define PMC_value_6                             (uint32)0x00000006 
#define PMC_value_7                             (uint32)0x00000007
#define PMC_value_8                             (uint32)0x00000008
#define PMC_value_9                             (uint32)0x00000009
#define PMC_value_10                            (uint32)0x0000000A
#define PMC_value_11                            (uint32)0x0000000B  
#define PMC_value_12                            (uint32)0x0000000C
#define PMC_value_13                            (uint32)0x0000000D
#define PMC_value_14                            (uint32)0x0000000E

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): ConfigPtr - Pointer to configuration set
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Initialize the Port Driver module.
************************************************************************************/
   
void Port_Init(const Port_ConfigType* ConfigPtr);
/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin - Port Pin ID number , Direction - Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Sets the port pin direction.
************************************************************************************/

#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
/* Function to set pin direction during runtime */
void Port_SetPinDirection( Port_PinType Pin,Port_PinDirectionType Direction);
#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction.
************************************************************************************/

void Port_RefreshPortDirection(void);  

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo - Pointer to where to store the version information of this module.
* Return value: None
* Description: Returns the version information of this module.
************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
/* Function for Port Get Version Info API */
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo);
#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin - Port Pin ID number, Mode - New Port Pin mode to be set on port pin
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.
************************************************************************************/
#if (PORT_SET_PIN_MODE_API == STD_ON)
/* Function to set the port pin mode of the referenced pin during runtime */
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode);
#endif

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/
/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Port_configration;

#endif /* PORT_H */
