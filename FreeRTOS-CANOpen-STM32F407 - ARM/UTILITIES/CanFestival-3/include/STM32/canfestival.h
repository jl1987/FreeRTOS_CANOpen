/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN
AT91 Port: Peter CHRISTEN

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
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#ifndef __CAN_CANFESTIVAL__
#define __CAN_CANFESTIVAL__

#include "applicfg.h"
#include "data.h"
// #include "objdict/ObjDict_CAN1.h"  //使用绝对路径的时候编译不成功，改为下面的“文件名.h”后正常，傻逼
// #include "objdict/ObjDict_CAN2.h"  //使用绝对路径的时候编译不成功，改为下面的“文件名.h”后正常，傻逼

#include "ARM_OD.h"

//#define DEBUG_WAR_CONSOLE_ON  //测试原装DEBUG是否可用


// ---------  to be called by user app ---------
void initTimer(void);
UNS8 canSend(CAN_PORT notused, Message *m);

#endif
