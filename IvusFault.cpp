///////////////////////////////////////////////////////////
//  IvusFault.cpp
//  Definitions of the IvusFault Class
//  Created on      : 5/11/2017 3:37 PM
//  Original author : sorin.dinu
///////////////////////////////////////////////////////////
//Copyright(c) 2019 Terumo Heart, Inc.All Rights Reserved.
//Any use of this file without the express written
//consent of Terumo Heart, Inc. is strictly prohibited.

#include "IvusFault.h"
#include "IvusComm.h"
#include "IvusControl.h"
#include "IVUS.h"

namespace IVUS
{
    /**
        Initialize _pFault to null.
    */
    IvusFault* IvusFault::_pFault = nullptr;

    /**
        Storage for the pointer to the Comm class object instance.
     */
    IvusComm* IvusFault::_pComm = nullptr;

    /**
        Initialize the IVUS Fault handling and lockout GPIO pins.

        Param pins:         Reference to the IVUS specific pins.
        Return:             Pointer to IVUS Fault handling singleton class object.
    */
    IvusFault* IvusFault::Init(IvusPins& pins)
    {
        static IvusFault ivusFault(pins);
        _pFault = &ivusFault;

        return _pFault;
    }


    /**
        Method to get the IVUS Fault singleton class object.

        Return:             Pointer to IVUS Fault handling singleton class object.
     */
    IvusFault* IvusFault::GetIvusFault()
    {
        if (!_pFault)
        {
            LOG_FAULT("_pFault is invalid", nullptr, 0);
        }
        return _pFault;
    }


    /**
        Set a pointer to the Communications class singleton object instance.

        Param Comm:         Pointer to Communication singleton class object.
     */
    void IvusFault::SetComm(IvusComm *comm)
    {
        _pComm = comm;
    }

    /**
        Constructor.

        Param pins:         Reference to the IVUS specific pins.
    */
    IvusFault::IvusFault(IvusPins& pins)
    :
    FaultMonitor(pins.ilockPins, pins.gpioEmDiagTest, pins.ledCard3)
    {
    }

    void IvusFault::_HandleMq(const IMqItem* id, const MsgObj& msg)
    {
        if (_IsFault(id))
        {
            //_pFault->_pCommQ->Send(Communication::msgFault, id->_val); //Update the Fault bit map
            _faultBitmap |= (1UL << id->_val);
            _pFault->_pCommQ->Send(Communication::msgFault, _faultBitmap); //Update the Fault bit map
        }
    }
}
