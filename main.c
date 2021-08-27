/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the USB HID Mouse Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_usb_dev.h"
#include "cycfg_usbdev.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define MOUSE_ENDPOINT      (1u)    /* Based on USB HID descriptor */
#define MOUSE_DATA_LEN      (3U)
#define CURSOR_STEP_PLUS    ((uint8_t)  (5))
#define CURSOR_STEP_MINUS   ((uint8_t) (-5))
#define CURSOR_STEP_POS     (1U)
#define CURSOR_DELAY        (128U)

/* Vddd threshold to enable internal regulators of USBFS block */ 
#define USB_REG_THRESHOLD   (3700U)

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void usb_high_isr(void);
static void usb_medium_isr(void);
static void usb_low_isr(void);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Mouse packet array: buttons (1st byte), X (2nd byte), Y (3rd byte) */
CY_USB_DEV_ALLOC_ENDPOINT_BUFFER(mouse_data, MOUSE_DATA_LEN);

/* USB Interrupt Configuration */
const cy_stc_sysint_t usb_high_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_hi_IRQn,
    .intrPriority = 0U,
};
const cy_stc_sysint_t usb_medium_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_med_IRQn,
    .intrPriority = 1U,
};
const cy_stc_sysint_t usb_low_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_lo_IRQn,
    .intrPriority = 2U,
};


/* USBDEV context variables */
cy_stc_usbfs_dev_drv_context_t  usb_drvContext;
cy_stc_usb_dev_context_t        usb_devContext;
cy_stc_usb_dev_hid_context_t    usb_hidContext;

/* PD Port Config */
cy_stc_usbpd_config_t PD_PORT0_config;

/* USBPD Context */
cy_stc_usbpd_context_t usbpd_context;

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function. It initializes the USB Device block
*  and enumerates as a HID mouse device. It moves the cursor from left to right
*  and vice-versa indefinitely. 
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint32_t counter = 0U;
    uint32_t vddd = 0U; 

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the USBPD driver to read Vddd value. 
     * This call uses the SAR ADC present in the USBPD block
     * to measure the Vddd value. Based on the Vddd measured,
     * the internal regulators are enabled in the USBFS block. 
     */

#if defined(CY_DEVICE_CCG3)
    Cy_USBPD_Init(&usbpd_context, 0, mtb_usbpd_port0_HW, NULL,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, usbpd_context.dpmGetConfig);
#else
    Cy_USBPD_Init(&usbpd_context, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, usbpd_context.dpmGetConfig);

#endif /* CY_DEVICE_CCG3 */

    vddd = usbpd_context.adcVdddMv[CY_USBPD_ADC_ID_0];
    
    /* Enable internal regulator in case the Vddd > 3.7V */ 
    if(vddd > USB_REG_THRESHOLD)
    {
        Cy_USBFS_Dev_Drv_RegEnable(CYBSP_USB_HW, &usb_drvContext);
    }

    else
    {
        Cy_USBFS_Dev_Drv_RegDisable(CYBSP_USB_HW, &usb_drvContext);
    }

    /* Initialize the USB device */
    Cy_USB_Dev_Init(CYBSP_USB_HW, &CYBSP_USB_config, &usb_drvContext,
                    &usb_devices[0], &usb_devConfig, &usb_devContext);

    /* Initialize the HID Class */
    Cy_USB_Dev_HID_Init(&usb_hidConfig, &usb_hidContext, &usb_devContext);

    /* Initialize the USB interrupts */
    Cy_SysInt_Init(&usb_high_interrupt_cfg,   &usb_high_isr);
    Cy_SysInt_Init(&usb_medium_interrupt_cfg, &usb_medium_isr);
    Cy_SysInt_Init(&usb_low_interrupt_cfg,    &usb_low_isr);   

    /* Enable the USB interrupts */
    NVIC_EnableIRQ(usb_high_interrupt_cfg.intrSrc);
    NVIC_EnableIRQ(usb_medium_interrupt_cfg.intrSrc);
    NVIC_EnableIRQ(usb_low_interrupt_cfg.intrSrc);

    /* Make device appear on the bus. This function call is blocking, 
     * it waits until the device enumerates 
     */
    Cy_USB_Dev_Connect(true, CY_USB_DEV_WAIT_FOREVER, &usb_devContext);

    for(;;)
    {
        counter++;
        
        /* Swap direction of the cursor (left/right) */
        if ((counter % CURSOR_DELAY) == 0)
        {
            /* Set the cursor to move to the right */    
            mouse_data[CURSOR_STEP_POS] = CURSOR_STEP_PLUS;
        }

        if ((counter % (2*CURSOR_DELAY)) == 0)
        {
            /* Set the cursor to move to the left */
            mouse_data[CURSOR_STEP_POS] = CURSOR_STEP_MINUS;
        }

        /* Update mouse position */
        Cy_USB_Dev_WriteEpBlocking(MOUSE_ENDPOINT, mouse_data, MOUSE_DATA_LEN,
                                   CY_USB_DEV_WAIT_FOREVER, &usb_devContext);

        /* Wait for 10 ms */
        cyhal_system_delay_ms(10UL);
    }
}


/***************************************************************************
* Function Name: usb_high_isr
********************************************************************************
* Summary:
*  This function processes the high priority USB interrupts.
*
***************************************************************************/
static void usb_high_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(CYBSP_USB_HW, 
                               Cy_USBFS_Dev_Drv_GetInterruptCauseHi(CYBSP_USB_HW), 
                               &usb_drvContext);
}


/***************************************************************************
* Function Name: usb_medium_isr
********************************************************************************
* Summary:
*  This function processes the medium priority USB interrupts.
*
***************************************************************************/
static void usb_medium_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(CYBSP_USB_HW, 
                               Cy_USBFS_Dev_Drv_GetInterruptCauseMed(CYBSP_USB_HW), 
                               &usb_drvContext);
}


/***************************************************************************
* Function Name: usb_low_isr
********************************************************************************
* Summary:
*  This function processes the low priority USB interrupts.
*
**************************************************************************/
static void usb_low_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(CYBSP_USB_HW, 
                               Cy_USBFS_Dev_Drv_GetInterruptCauseLo(CYBSP_USB_HW), 
                               &usb_drvContext);
}


/* [] END OF FILE */
