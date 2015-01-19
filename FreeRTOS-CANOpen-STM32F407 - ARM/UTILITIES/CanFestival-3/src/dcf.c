/*
  This file is part of CanFestival, a library implementing CanOpen
  Stack.

  Copyright (C): Edouard TISSERANT and Francis DUPIN

  See COPYING file for copyrights details.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307
  USA
*/


/**
** @file   dcf.c
** @author Edouard TISSERANT and Francis DUPIN
** @date   Mon Jun  4 17:06:12 2007
**
** @brief EXEMPLE OF SOMMARY
**
**
*/


#include "data.h"
#include "sysdep.h"
#include "dcf.h"

typedef struct {
    UNS16 Index;
    UNS8 Subindex;
    UNS32 Size;
    UNS8 *Data;
} dcf_entry_t;

void SaveNode(CO_Data* d, UNS8 nodeId);
static UNS8 read_consise_dcf_next_entry(CO_Data* d, UNS8 nodeId);
static UNS8 write_consise_dcf_next_entry(CO_Data* d, UNS8 nodeId);
UNS8 init_consise_dcf(CO_Data* d,UNS8 nodeId);


__inline void start_node(CO_Data* d, UNS8 nodeId){
    /* Ask slave node to go in operational mode */
    masterSendNMTstateChange (d, nodeId, NMT_Start_Node);
    d->NMTable[nodeId] = Operational;
}

/**
** @brief Function to be called from post_SlaveBootup
**
** @param d
** @param nodeId
*/
UNS8 check_and_start_node(CO_Data* d, UNS8 nodeId)
{   
    if(d->dcf_status != DCF_STATUS_INIT)
        return 0;
    if((init_consise_dcf(d, nodeId) == 0) || (read_consise_dcf_next_entry(d, nodeId) == 0)){
        start_node(d, nodeId);
        return 1;
    }
    d->dcf_status = DCF_STATUS_READ_CHECK;
    return 2;
}

__inline void start_and_seek_node(CO_Data* d, UNS8 nodeId){
   UNS8 node;
   start_node(d,nodeId);
   /* Look for other nodes waiting to be started */
   for(node = 0 ; node<NMT_MAX_NODE_ID ; node++){
       if(d->NMTable[node] != Initialisation)
           continue;
       if(check_and_start_node(d, node) == 2)
           return;
   }
   /* No more node to start. Let's start our own node */
   setState(d, Operational);
}

/**
**
**
** @param d
** @param nodeId
*/
static void CheckSDOAndContinue(CO_Data* d, UNS8 nodeId)
{
    UNS32 abortCode = 0;
    UNS8 buf[4], match = 0, node;
    UNS32 size=4;
    if(d->dcf_status == DCF_STATUS_READ_CHECK){
        // printf("DCF_STATUS_READ_CHECK \n");
        if(getReadResultNetworkDict (d, nodeId, buf, &size, &abortCode) != SDO_FINISHED)
            goto dcferror;
        /* Check if data received match the DCF */
        if(size == d->dcf_size){
            match = 1;
            while(size--)
                if(buf[size] != d->dcf_data[size])
                    match = 0;
        }
        if(match) {
            if(read_consise_dcf_next_entry(d, nodeId) == 0){
                start_and_seek_node(d, nodeId);
            }
        }
        else { /* Data received does not match : start rewriting all */
            if((init_consise_dcf(d, nodeId) == 0) || (write_consise_dcf_next_entry(d, nodeId) == 0))
                goto dcferror;                
            d->dcf_status = DCF_STATUS_WRITE;
        }
    }
    else if(d->dcf_status == DCF_STATUS_WRITE){
        // printf("DCF_STATUS_WRITE \n");
        if(getWriteResultNetworkDict (d, nodeId, &abortCode) != SDO_FINISHED)
            goto dcferror;
        if(write_consise_dcf_next_entry(d, nodeId) == 0){
#ifdef DCF_SAVE_NODE
            SaveNode(d, nodeId);
            d->dcf_status = DCF_STATUS_SAVED;
#else //DCF_SAVE_NODE
           start_and_seek_node(d,nodeId);
#endif //DCF_SAVE_NODE
        }
    }
    else if(d->dcf_status == DCF_STATUS_SAVED){
        // printf("DCF_STATUS_SAVED \n");
        if(getWriteResultNetworkDict (d, nodeId, &abortCode) != SDO_FINISHED)
            goto dcferror;
        masterSendNMTstateChange (d, nodeId, NMT_Reset_Node);
        d->dcf_status = DCF_STATUS_INIT;
        d->NMTable[nodeId] = Unknown_state;
    }
    return;
dcferror:
    MSG_ERR(0x1A01, "SDO error in consise DCF", abortCode);
    MSG_WAR(0x2A02, "server node : ", nodeId);
    d->NMTable[nodeId] = Unknown_state;
}

/**
**
**
** @param d
** @param nodeId
**
** @return
*/
UNS8 init_consise_dcf(CO_Data* d,UNS8 nodeId)
{
    /* Fetch DCF OD entry */
    UNS32 errorCode;
    ODCallback_t *Callback;
    UNS8* dcf;
    d->dcf_odentry = (*d->scanIndexOD)(0x1F22, &errorCode, &Callback);
    /* If DCF entry do not exist... Nothing to do.*/
    if (errorCode != OD_SUCCESSFUL) goto DCF_finish;
    /* Fix DCF table overflow */
    if(nodeId > d->dcf_odentry->bSubCount) goto DCF_finish;
    /* If DCF empty... Nothing to do */
    if(! d->dcf_odentry->pSubindex[nodeId].size) goto DCF_finish;
    dcf = *(UNS8**)d->dcf_odentry->pSubindex[nodeId].pObject;
    // printf("%.2x %.2x %.2x %.2x\n",dcf[0],dcf[1],dcf[2],dcf[3]);
    d->dcf_cursor = dcf + 4;
    d->dcf_entries_count = 0;
    d->dcf_status = DCF_STATUS_INIT;
    return 1;
    DCF_finish:
    return 0;
}

UNS8 get_next_DCF_data(CO_Data* d, dcf_entry_t *dcf_entry, UNS8 nodeId)
{
  UNS8* dcfend;
  UNS32 nb_entries;
  UNS32 szData;
  UNS8* dcf;
  if(!d->dcf_odentry)
     return 0;
  if(nodeId > d->dcf_odentry->bSubCount)
     return 0;
  szData = d->dcf_odentry->pSubindex[nodeId].size;
  dcf = *(UNS8**)d->dcf_odentry->pSubindex[nodeId].pObject;
  nb_entries = UNS32_LE(*((UNS32*)dcf));
  dcfend = dcf + szData;
  if((UNS8*)d->dcf_cursor + 7 < (UNS8*)dcfend && d->dcf_entries_count < nb_entries){
    /* DCF data may not be 32/16b aligned, 
    * we cannot directly dereference d->dcf_cursor 
    * as UNS16 or UNS32 
    * Do it byte per byte taking care on endianess*/
#ifdef CANOPEN_BIG_ENDIAN
    dcf_entry->Index = *(d->dcf_cursor++) << 8 | 
     	               *(d->dcf_cursor++);
#else
    memcpy(&dcf_entry->Index, d->dcf_cursor,2);
       	d->dcf_cursor+=2;
#endif
    dcf_entry->Subindex = *(d->dcf_cursor++);
#ifdef CANOPEN_BIG_ENDIAN
    dcf_entry->Size = *(d->dcf_cursor++) << 24 | 
     	              *(d->dcf_cursor++) << 16 | 
        	          *(d->dcf_cursor++) << 8 | 
        	          *(d->dcf_cursor++);
#else
    memcpy(&dcf_entry->Size, d->dcf_cursor,4);
    d->dcf_cursor+=4;
#endif
    d->dcf_data = dcf_entry->Data = d->dcf_cursor;
    d->dcf_size = dcf_entry->Size;
    d->dcf_cursor += dcf_entry->Size;
    d->dcf_entries_count++;
    return 1;
  }
  return 0;
}

static UNS8 write_consise_dcf_next_entry(CO_Data* d, UNS8 nodeId)
{
    UNS8 Ret;
    dcf_entry_t dcf_entry;
    if(!get_next_DCF_data(d, &dcf_entry, nodeId))
        return 0;
    Ret = writeNetworkDictCallBackAI(d, /* CO_Data* d*/
                    nodeId, /* UNS8 nodeId*/
                    dcf_entry.Index, /* UNS16 index*/
                    dcf_entry.Subindex, /* UNS8 subindex*/
                    (UNS8)dcf_entry.Size, /* UNS8 count*/
                    0, /* UNS8 dataType*/
                    dcf_entry.Data,/* void *data*/
                    CheckSDOAndContinue,/* Callback*/
                    0,   /* no endianize*/
                    0); /* no block mode */
    if(Ret)
        MSG_ERR(0x1A02,"Erreur writeNetworkDictCallBackAI",Ret);
    return 1;
}

static UNS8 read_consise_dcf_next_entry(CO_Data* d, UNS8 nodeId)
{
    UNS8 Ret;
    dcf_entry_t dcf_entry;
    if(!get_next_DCF_data(d, &dcf_entry, nodeId))
        return 0;
    Ret = readNetworkDictCallbackAI(d, /* CO_Data* d*/
                   nodeId, /* UNS8 nodeId*/
                   dcf_entry.Index, /* UNS16 index*/
                   dcf_entry.Subindex, /* UNS8 subindex*/
                   0, /* UNS8 dataType*/
                   CheckSDOAndContinue,/* Callback*/
                   0); /* no block mode */
    if(Ret)
        MSG_ERR(0x1A03,"Erreur readNetworkDictCallbackAI",Ret);
    return 1;
}

void SaveNode(CO_Data* d, UNS8 nodeId)
{
    UNS8 Ret;
    UNS32 data=0x65766173;
    Ret = writeNetworkDictCallBackAI(d, /* CO_Data* d*/
                    nodeId, /* UNS8 nodeId*/
                    0x1010, /* UNS16 index*/
                    1, /* UNS8 subindex*/
                    4, /* UNS8 count*/
                    0, /* UNS8 dataType*/
                    (void *)&data,/* void *data*/
                    CheckSDOAndContinue,/* Callback*/
                    0,   /* no endianize*/
                    0); /* no block mode */
    if(Ret)
        MSG_ERR(0x1A04,"Erreur writeNetworkDictCallBackAI",Ret);
}
