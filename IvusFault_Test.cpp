// Copyright (c) 2019-2022 Terumo Heart, Inc. All Rights Reserved.
// Any use of this software without the express written 
// consent of Terumo Heart, Inc. is strictly prohibited.

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "IvusFault.h"
#include "IvusFault_Test.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
// Global Test message definitions
///////////////////////////////////////////////////////////////////////////////////////////////////

const RTOS::Ev ControlMQ::evStartPwrOnTimer;
const RTOS::ShortMsg Communication::msgFault;

///////////////////////////////////////////////////////////////////////////////////////////////////
// Global Test variables
///////////////////////////////////////////////////////////////////////////////////////////////////

bool MQ_Send_executed = false;

///////////////////////////////////////////////////////////////////////////////////////////////////
// Log and print callbacks
///////////////////////////////////////////////////////////////////////////////////////////////////

// Print Buffers
#define BUFFER_SIZE 1000
uint32_t LogEntryCount = 1;
char tt1print[BUFFER_SIZE] = { 0 };
char* t1print = &tt1print[0];
char tt2print[BUFFER_SIZE] = { 0 };
char* t2print = &tt2print[0];
char tt3print[BUFFER_SIZE] = { 0 };
char* t3print = &tt3print[0];
char tt4print[BUFFER_SIZE] = { 0 };
char* t4print = &tt4print[0];
char tt5print[BUFFER_SIZE] = { 0 };
char* t5print = &tt5print[0];

// Log callback
namespace TLOG
{
    tlog_result_t TLogEntry(
        tlog_h hTLog,
        tlog_level_t level,
        tlog_literal_t logCode,
        void* logData,
        uint8_t logDataSize)
    {
        if (logCode == nullptr)
        {
            return tlog_result_t::FAILED;
        }

        int tsize = logDataSize;
        int tdata = int(logData);
        char array[BUFFER_SIZE] = { 0 };
        char* pStr = &array[0];
        va_list argptr;
        va_start(argptr, logCode);
        vsnprintf(pStr, RTOS_INDEFINITE_TIMEOUT, logCode, argptr);
        va_end(argptr);

        if (LogEntryCount == 1)
        {
            for (int i = 0; i < BUFFER_SIZE; i++)
            {
                *(t1print + i) = *(pStr + i);
            }
        }
        else if (LogEntryCount == 2)
        {
            for (int i = 0; i < BUFFER_SIZE; i++)
            {
                *(t2print + i) = *(pStr + i);
            }
        }
        else if (LogEntryCount == 3)
        {
            for (int i = 0; i < BUFFER_SIZE; i++)
            {
                *(t3print + i) = *(pStr + i);
            }
        }
        else if (LogEntryCount == 4)
        {
            for (int i = 0; i < BUFFER_SIZE; i++)
            {
                *(t4print + i) = *(pStr + i);
            }
        }
        else if (LogEntryCount == 5)
        {
            for (int i = 0; i < BUFFER_SIZE; i++)
            {
                *(t5print + i) = *(pStr + i);
            }
        }

        LogEntryCount++;
        return tlog_result_t::OK;
    }
}

void ClearAllLogEntries()
{
    memset(t1print, 0, BUFFER_SIZE);
    memset(t2print, 0, BUFFER_SIZE);
    memset(t3print, 0, BUFFER_SIZE);
    memset(t4print, 0, BUFFER_SIZE);
    memset(t5print, 0, BUFFER_SIZE);
    LogEntryCount = 1;
}

/// /////////////////////////////////////////////////////////////////////////////////////////////////
///     IvusFault Class Unit Tests
/// 
/// public:
///     1. static IvusFault* Init(IvusPins& pins);
///     2. static IvusFault* GetIvusFault();
///     3. static void SetComm(IvusComm *comm);
/// 
/// private:
///     1. IvusFault(IvusPins& pins); // Constructor
///     2. _HandleMq(const IMqItem* id, const MsgObj& msg);	// MQ handler
/// 
/// ////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Test definitions and pointer initializations
///////////////////////////////////////////////////////////////////////////////////////////////////

IVUS::IvusComm* IVUS::IvusComm::_pComm = nullptr;
IVUS::IvusFault* _pIvusFault = nullptr;
IVUS::IvusComm* _pIvusComm = nullptr;
Logger* Logger::_pLogger = nullptr;
IVUS::IvusPins _ivusPins;

///////////////////////////////////////////////////////////////////////////////////////////////////
// Unit Test harness setup and teardown
///////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    _pIvusComm = IVUS::IvusComm::Init();
    _pIvusFault = IVUS::IvusFault::Init(_ivusPins);
    ClearAllLogEntries();
}

void teardown()
{
    _pIvusComm = nullptr;
    _pIvusFault = nullptr;
}

namespace IVUS
{
    /// 
    /// ////////////////////////////////////////////////////////////////////////////////////////////////
    /// // Public Unit Test Methods
    /// ////////////////////////////////////////////////////////////////////////////////////////////////
    /// 

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    /// 1. Test Init Method
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    TEST(IvusFault_Test, Test_Init)
    {
        // ---------------------------------------------------------------------------------------------
        /// a. Test the Init method by calling it and verifying the pointer set
        // 
        // Create a new IvusPins object pointer called ivusPins
        IVUS::IvusPins ivusPins = IVUS::IvusPins();
        // Create a new IvusFault object pointer called pIvusFault using the ivusPins object as the parameter
        IVUS::IvusFault* pIvusFault = IVUS::IvusFault::Init(ivusPins);
        // Verify that the IvusFault pointer received was not null
        EXPECT_NE(pIvusFault, nullptr);
        // Verify that the IvusFault object _pFault is now set to the one sent in
        EXPECT_EQ(pIvusFault->_pFault, pIvusFault);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    /// 2. Test GetIvusFault Method
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    TEST(IvusFault_Test, Test_GetIvusFault)
    {
        setup();

        // ---------------------------------------------------------------------------------------------
        /// a. Test the GetIvusFault method by calling it with a valid PrwFault pointer and verifying it being set
        // 
        // Call the GetIvusFault Target Method and get a IvusFault object pointer back
        IVUS::IvusFault* pIvusFault = IVUS::IvusFault::GetIvusFault();
        // Verify that the IvusFault pointer _pFault member is set to the one created
        EXPECT_EQ(pIvusFault->_pFault, pIvusFault);
        // Verify that the pointer received is not null
        EXPECT_GT(pIvusFault, nullptr);

        // ---------------------------------------------------------------------------------------------
        /// b. Test the GetIvusFault method by calling it with a null pointer and verifying the pointer is null and message logged
        // 
        // Set the IvusFault object _pFault memember to null
        _pIvusFault->_pFault = nullptr;
        // Call the GetIvusFault Target Method and get back a IvusFault pointer
        pIvusFault = IVUS::IvusFault::GetIvusFault();
        // Verify that the pointer received is null
        EXPECT_EQ(pIvusFault, nullptr);
        // Verify that the IvusFault: _pFault is invalid message was logged
        EXPECT_STREQ(LastString(t1print), "IvusFault.cpp : 52 : _pFault is invalid");

        teardown();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    /// 3. Test SetComm Method
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    TEST(IvusFault_Test, Test_SetComm)
    {
        setup();

        // ---------------------------------------------------------------------------------------------
        /// a. Test the SetComm method by calling it and verifying the pointer set
        // 
        // Call the void return SetComm Target Method with the test IvusComm object pointer
        _pIvusFault->SetComm(_pIvusComm);
        // Verify that the IvusFault object _pComm pointer is now set to the test IvusComm object pointer
        EXPECT_EQ(_pIvusFault->_pComm, _pIvusComm);

        teardown();
    }

    /// 
    /// ////////////////////////////////////////////////////////////////////////////////////////////////
    /// // Private Unit Test Methods
    /// ////////////////////////////////////////////////////////////////////////////////////////////////
    /// 

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    /// 1. Test IvusFault Constructor
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    TEST(IvusFault_Test, Test_IvusFault)
    {
        setup();

        // ---------------------------------------------------------------------------------------------
        /// a. Test the IvusFault method by calling it and verifying the pointer set
        // 
        // Call the IvusFault constructor with the IvusPins object pointer and get back a IvusFault object pointer
        IvusFault* pIvusFault = new IvusFault(_ivusPins);
        // Verify that the IvusFault object pointer received is not null
        EXPECT_GT(pIvusFault, nullptr);

        delete pIvusFault;

        teardown();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    /// 2. Test _HandleMq Method
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    TEST(IvusFault_Test, Test_HandleMq)
    {
        setup();

        // ---------------------------------------------------------------------------------------------
        /// a. Test the _HandleMq method by calling it and verifying the pointer and execution flag are set
        // 
        // Clear the MQ::Send execution flag
        MQ_Send_executed = false;
        // Create a new IMqItem object pointer called id
        IMqItem* id = new IMqItem();
        // Create a new MsgObj object pointer called msg
        MsgObj* msg = new MsgObj();
        // Call the void return _HandleMq Target Method with the id and msg as parameters
        _pIvusFault->_HandleMq(id, *msg);
        // Verify that the IvusFault object _faultBitmap member is now set to 1
        EXPECT_EQ(_pIvusFault->_faultBitmap, 1);
        // Verify that the MQ::Send method execution flag is now set
        EXPECT_TRUE(MQ_Send_executed);

        delete id;
        delete msg;

        teardown();
    }
}

///////////////////////////////////////////////////////////////////////////////
//  main Function to run All Test
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
