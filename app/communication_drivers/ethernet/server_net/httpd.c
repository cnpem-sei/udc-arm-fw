//*****************************************************************************
// HTTP server.
// Adam Dunkels <adam@dunkels.com>
// Copyright (c) 2001, Adam Dunkels.
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. The name of the author may not be used to endorse or promote
//    products derived from this software without specific prior
//    written permission.
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// This file is part of the uIP TCP/IP stack.
//*****************************************************************************
// This file has been modified from its original uIP distribution to add
// functionality needed for the enet_uip example.
//*****************************************************************************

#include "uip.h"
#include "httpd.h"

#include <stdint.h>
#include <stdarg.h>
#include <string.h>


//*****************************************************************************
// Macro for easy access to buffer data
//*****************************************************************************
#define BUF_APPDATA ((u8_t *)uip_appdata)

typedef struct
{
	uint8_t *response_data;
	uint16_t length;
}response;
response response_to_client;

uint16_t new_data_send = 0;

static uint8_t new_message = 0;

#pragma DATA_SECTION(rx_buffer, "ETHERNETBUFFER")

//*****************************************************************************
struct var_buffer
{
    uint8_t data[3+3+8192];
    uint16_t index;
};

static struct var_buffer rx_buffer = {.index = 0};


//*****************************************************************************


//*****************************************************************************
// Initialize the web server.
// Starts to listen for incoming connection requests on TCP port 80.
//*****************************************************************************
void
httpd_init(void)
{
    // Listen to port 80.
    uip_listen(HTONS(80));
    //uip_listen(HTONS(90));

}



//*****************************************************************************
// return the status along with the command
//*****************************************************************************
uint8_t
httpd_get_command(void)
{
    if(new_message)
    {
        return(1);
    }
    else
    {
        return(0);
    }
}

//*****************************************************************************
// clear the command, so that we won't execute it again
//*****************************************************************************
void
httpd_clear_command(void)
{
	new_message = 0;
}

//*****************************************************************************
// insert a response
//*****************************************************************************

void
httpd_insert_response(uint16_t data_length,uint8_t *data)
{
    response_to_client.response_data = data;
    response_to_client.length = data_length;
    new_data_send = 1;
}

uint8_t *
BufferAdress(void)
{
	return (rx_buffer.data);
}

uint16_t
BufferLen(void)
{
	return (rx_buffer.index);
}

void
BufferClear(void)
{
	rx_buffer.index = 0;
}

uint16_t count;
//*****************************************************************************
// HTTP Application Callback Function
//*****************************************************************************
void
httpd_appcall(void)
{
	unsigned short sCarga;

	switch(uip_conn->lport)
    {
    // This is the web server:
    case HTONS(80):
    {

    	if(uip_newdata())
    		{
    			//rx_buffer.index = uip_len;

    			if(rx_buffer.index < 65536)
    			{
    				for(count = 0; count < uip_len ; count++) //rx_buffer.index
    				{
    					rx_buffer.data[rx_buffer.index++] = BUF_APPDATA[count];
    				}

    		    	sCarga = (rx_buffer.data[1]<<8) | rx_buffer.data[2];
    		    	if(rx_buffer.index > sCarga +2)
    		    	{
    		    		new_message = 1;
    		    	}
    			}

    		}
    	/*
    	if(uip_newdata())
    		{
    		rx_buffer.index = uip_len;

    			if(rx_buffer.index < 65536)
    			{
    				for(count = 0; count < rx_buffer.index; count++)
    				{
    					rx_buffer.data[count] = BUF_APPDATA[count];
    				}
    				new_message = 1;
    			}

    		}
		*/
    	if(new_data_send)
    		{
    			uip_send(response_to_client.response_data, response_to_client.length);
    			new_data_send = 0;
    		}


        // Finally, return to uIP. Our outgoing packet will soon be on its
        // way...
        return;
    }


    default:
    {
        // Should never happen.
        uip_abort();
        break;
    }
    }

}



