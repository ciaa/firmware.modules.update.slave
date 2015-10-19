/* Copyright 2015, Daniel Cohen
 * Copyright 2015, Esteban Volentini
 * Copyright 2015, Matias Giori
 * Copyright 2015, Franco Salinas
 * Copyright 2015, Pablo Alcorta
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief This file implements the UPDT unpacker functionality
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup UPDT CIAA UPDT Unpacker
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * DC           Daniel Cohen
 * EV           Esteban Volentini
 * MG           Matias Giori
 * FS           Franco Salinas
 * PA           Pablo Alcorta
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20150515 v0.0.1  FS  first initial version
 */

/*==================[inclusions]=============================================*/
#include "UPDT_config.h"
#include "ciaaLibs_Endianess.h"
#include "ciaaPOSIX_stdio.h"
#include "ciaaPOSIX_assert.h"
#include "ciaaLibs_format.h"
#include "UPDT_services.h"
/*==================[macros and definitions]=================================*/
/** \brief Flash memory address where the configuration is stored */
#define UPDT_CONFIG_ADDRESS      0x0

/** \brief Configuration size */
#define UPDT_CONFIG_SIZE         0x18

typedef struct
{
   uint32_t reserved1;
   uint32_t firmware_version;
   uint32_t bootloader_flags;
   uint32_t bootloader_version;
   uint32_t reserved2;
   uint32_t application_version;
   uint32_t vendor_id;
   uint32_t model_id;
   uint32_t unique_id_low;
   uint32_t unique_id_high;
   uint32_t data_size;
} UPDT_configType;
/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
static UPDT_configType UPDT_config_new;
static UPDT_configType UPDT_config_old;
static uint8_t UPDT_config_buffer[UPDT_CONFIG_SIZE];
static int32_t UPDT_config_fd;
static uint32_t UPDT_config_flags;
/*==================[external data definition]===============================*/

/* payload parsing */
/*==================[internal functions definition]==========================*/
void UPDT_configParse(const uint8_t *payload, size_t size, UPDT_configType *info)
{
   uint32_t word;
   const uint32_t *ptr;
   ciaaPOSIX_assert(NULL != payload);
   ciaaPOSIX_assert (NULL != info);

   ptr = (const uint32_t *) payload;

   word = ciaaLibs_utilsNtohl(*ptr);
   info->reserved1 = 0xFFu & (word >> 24);
   info->firmware_version = 0xFFFFFFu & (word);

   word = ciaaLibs_utilsNtohl(*(ptr + 1));
   info->bootloader_flags = 0xFFu & (word >> 24);
   info->bootloader_version = 0xFFFFFFu & (word);

   word = ciaaLibs_utilsNtohl(*(ptr + 2));
   info->reserved2 = 0xFFu & (word >> 24);
   info->application_version = 0xFFFFFFu & (word);

   word = ciaaLibs_utilsNtohl(*(ptr + 3));
   info->vendor_id = 0xFFu & (word >> 24);
   info->model_id = 0xFFFFFFu & (word);

   //Corregir esta parte.. no deberia ir el bigendian, deberia ser una funcion esta condicion
   #if CIAAPLATFORM_BIGENDIAN == 0
   info->unique_id_low = ciaaLibs_utilsNtohl(*(ptr + 4));
   info->unique_id_high = ciaaLibs_utilsNtohl(*(ptr + 5));
   #else
   info->unique_id_high = ciaaLibs_utilsNtohl(*(const uint32_t *) (ptr + 4));
   info->unique_id_low = ciaaLibs_utilsNtohl(*(const uint32_t *) (ptr + 5));
   #endif
   info->data_size = ciaaLibs_utilsNtohl(*(ptr + 6));
}

/** \brief UPDT_configFormat stores in a buffer, in compacted format, the parameters of to data structure
 ** \param config_buffer is the buffer wherein the content of the structure is stored
 ** \param data_size is the number of elements that will have the buffer
 ** \param info is a variable of the type structure used to assemble the buffer
 **/
static void UPDT_configFormat (uint8_t *config_buffer,size_t data_size, UPDT_configType *type)
{
   ciaaPOSIX_assert(data_size == 32);
   /*Set of the field "reserved1" in the buffer*/
   ciaaLibs_SetByte (&config_buffer,0,(type->reserved1));
   /*Set of the field "firmware_version" in the buffer*/
   ciaaLibs_SetUint24 (&config_buffer,1,(type->firmware_version));
   /*Set of the field "bootloader_flags" in the buffer*/
   ciaaLibs_SetByte (&config_buffer,4,(type->bootloader_flags));
   /*Set of the field "bootloader_version" in the buffer*/
   ciaaLibs_SetUint24 (&config_buffer,5,(type->bootloader_version));
   /*Set of the field "reserved2" in the buffer*/
   ciaaLibs_SetByte (&config_buffer,8,(type->reserved2));
   /*Set of the field "application_version" in the buffer*/
   ciaaLibs_SetUint24 (&config_buffer,9,(type->application_version));
   /*Set of the field "vendor_id" in the buffer*/
   ciaaLibs_SetByte (&config_buffer,12,(type->vendor_id));
   /*Set of the field "model_id" in the buffer*/
   ciaaLibs_SetUint24 (&config_buffer,13,(type->model_id));
   /*Set of the field "unique_id" in the buffer*/
   #if CIAAPLATFORM_BIGENDIAN == 0
   ciaaLibs_SetUint32 (&config_buffer,16,(type->unique_id_low));
   ciaaLibs_SetUint32 (&config_buffer,20,(type->unique_id_high));
   #else
   ciaaLibs_SetUint32 (&config_buffer,16,(type->unique_id_high));
   ciaaLibs_SetUint32 (&config_buffer,20,(type->unique_id_low));
   #endif // CIAAPLATFORM_BIGENDIAN
   /*Set of the field "data_size" in the buffer*/
   ciaaLibs_SetUint32 (&config_buffer,24,(type->data_size));
}
/*==================[external functions definition]==========================*/

/** \note THE BOOTLOADER FLAGS AND VERSIONS ARE NOT STORED WITH A STORE CALL.
 ** BOTH MUST BE CONSTANTS DEFINED IN THE SERVICE HEADER FILE. A FIRMWARE
 ** UPDATE CANNOT CHANGE THE BOOTLOADER ATTRIBUTES.
 **/
uint32_t UPDT_configSet(const uint8_t *config, size_t size)
{
   /* compare the expected configuration size with the received size */
   if(UPDT_CONFIG_SIZE != size)
   {
      UPDT_config_flags = UPDT_CONFIG_ERROR_INVALID_SIZE;
   }
   else
   {
      UPDT_configParse(config, UPDT_CONFIG_SIZE, &UPDT_config_new);
      UPDT_configParse(UPDT_CONFIG_ADDRESS, UPDT_CONFIG_SIZE, &UPDT_config_old);

      /* ERRORS */

      /* reserved fields must be zero */
      if(0 != (UPDT_config_new.reserved1 | UPDT_config_new.reserved2))
      {
         UPDT_config_flags |= UPDT_CONFIG_ERROR_RESERVED;
      }

      if (UPDT_config_new.bootloader_flags != 0)
      {
         UPDT_config_flags |= UPDT_CONFIG_ERROR_BOOTLOADER_FLAGS;
      }

      if (UPDT_config_old.bootloader_version > UPDT_config_new.application_version)
      {
         UPDT_config_flags |= UPDT_CONFIG_ERROR_BOOTLOADER_VERSION;
      }

      if (UPDT_config_old.vendor_id != UPDT_config_new.vendor_id)
      {
         UPDT_config_flags |= UPDT_CONFIG_ERROR_VENDOR_ID;
      }

      if (UPDT_config_old.model_id != UPDT_config_new.model_id)
      {
         UPDT_config_flags |= UPDT_CONFIG_ERROR_MODEL_ID;
      }

      if (UPDT_config_old.unique_id_low != UPDT_config_new.unique_id_low  || UPDT_config_old.unique_id_high != UPDT_config_new.unique_id_high)
      {
         UPDT_config_flags |= UPDT_CONFIG_ERROR_UNIQUE_ID;
      }

      /* WARNINGS */

      if (UPDT_config_old.firmware_version >= UPDT_config_new.firmware_version)
      {
         UPDT_config_flags |= UPDT_CONFIG_WARNING_FIRMWARE_VERSION;
      }

      if (UPDT_config_old.application_version >= UPDT_config_new.application_version)
      {
         UPDT_config_flags |= UPDT_CONFIG_WARNING_APPLICATION_VERSION;
      }
   }
   return UPDT_config_flags;
}



int32_t UPDT_configWrite(void)
{
   int32_t ret = -1;
   ciaaPOSIX_assert(UPDT_config_fd >= 0);

   /*To the previous configuration is assigned the new version of protocol and firmware, because
   it's the only change (the other parameters are kept)*/
   UPDT_config_old.firmware_version = (UPDT_config_new.firmware_version);
   UPDT_config_old.application_version = UPDT_config_new.application_version;

   /*Becomes the structure in an bytes sequence*/
   UPDT_configFormat(UPDT_config_buffer,28,&UPDT_config_old);

   /*function call to move the point to the memory address where it will be written*/
   if (ciaaPOSIX_lseek (UPDT_config_fd,UPDT_CONFIG_ADDRESS,SEEK_SET) == UPDT_CONFIG_ADDRESS);
   {
      ret = ciaaPOSIX_write (UPDT_config_fd,UPDT_config_buffer,28);
   }
   return ret;
}


int32_t UPDT_configInit (int32_t fd)
{
   UPDT_config_fd = fd;
   return 0;
}


void UPDT_configClear (void)
{
   UPDT_config_fd = -1;
}


ssize_t UPDT_configSetResponse(uint8_t *buffer, size_t size)
{
   UPDT_configFormat(buffer,size,&UPDT_config_old);
   UPDT_config_flags = ciaaLibs_utilsNtohl (UPDT_config_flags);

   /* Concatenated flags of warning and error */
   ciaaLibs_SetUint24 (&buffer,28,UPDT_config_flags);
   return 0;
}
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


