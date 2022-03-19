///////////////////////////////////////////////////////////
//  IvusFault.h
//  Declarations of the IvusFault Class
//  Created on      : 5/11/2017 3:37 PM
//  Original author : sorin.dinu
///////////////////////////////////////////////////////////
//Copyright(c) 2019 Terumo Heart, Inc.All Rights Reserved.
//Any use of this file without the express written
//consent of Terumo Heart, Inc. is strictly prohibited.

#ifndef _IVUS_FAULT_H_
#define _IVUS_FAULT_H_


#include "Fault.h"
#include "IvusComm.h"

#include "IvusFault_Test_Include.h"


namespace IVUS
{
    // Forward declare.
    struct IvusPins;


    /**
       IVUS Fault class.  This is the fault handler class for the IVUS.
    */
    class IvusFault : public FaultMonitor
    {
    public:
        /**
           Initialize the IVUS Fault handling and lockout GPIO pins.
           Param pins:         Reference to the IVUS specific pins.
           Return:             Pointer to IVUS Fault handling singleton class object.
         */
        static IvusFault* Init(IvusPins& pins);


        /**
           Method to get the IVUS Fault singleton class object pointer.
           Return:             Pointer to IVUS Fault handling singleton class object.
        */
        static IvusFault* GetIvusFault();

        /**
            Set a pointer to the Communication singleton class object instance.

            Param Comm:        Pointer to Communication singleton class object.
        */
        static void SetComm(IvusComm *comm);

    private:
        static IvusFault* _pFault;

        //Stored pointer to the Comm singleton class object.
        static IvusComm* _pComm;

        IvusFault(IvusPins& pins);

        void _HandleMq(const IMqItem* id, const MsgObj& msg) override;

        FRIEND_TEST_FIXTURE_IVUS_FAULT

    };
}

#endif