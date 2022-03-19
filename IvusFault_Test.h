// Copyright (c) 2016-2022 Terumo Heart, Inc. All Rights Reserved.
// Any use of this software without the express written 
// consent of Terumo Heart, Inc. is strictly prohibited.

#ifndef _HEADER_BLOCKS_H_
#define _HEADER_BLOCKS_H_

#include <cstdint>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <limits.h>
#include "stdarg.h"
#include "stdint.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		INCLUDE Header Block definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define INCLUDE_ADDRESSMAP_H_
#define INCLUDE_BLDCCONTROL_H_
#define INCLUDE_CAN_DAQ_H_
#define INCLUDE_CAN_DAQ_VARS_H_
#define INCLUDE_CAN_ENGINE_H_
#define INCLUDE_CAN_ENGINE_INTERNAL_H_
#define INCLUDE_CAN_ENGINE_TYPES_H_
#define INCLUDE_CAN_HARDWARE_API_H_
#define INCLUDE_COMM_OBJ_H_
#define INCLUDE_COMMON_STATE_MACHINE_H_
#define INCLUDE_COMMUNICATION_H_
#define INCLUDE_CONTROL_H_
#define INCLUDE_CPP_CANENGINE_H_
#define INCLUDE_CPP_EVENTGENERATOR_H_
#define INCLUDE_CPP_EXGRESPONDER_H_
#define INCLUDE_CPP_REGCLIENT_H_
#define INCLUDE_CPP_REGOWNER_H_
#define INCLUDE_CPP_TLOG_H_
#define INCLUDE_DEBUG_H_
#define INCLUDE_EVENT_H_
#define INCLUDE_EXCHANGE_H_
#define INCLUDE_FAULT_MONITOR_H_
#define INCLUDE_HAL_DAC_H_
#define INCLUDE_HAL_FLASH_H_
#define INCLUDE_HAL_GPIO_H_
#define INCLUDE_HAL_H_
#define INCLUDE_HAL_I2C_H_
#define INCLUDE_HAL_PIN_H_
#define INCLUDE_HAL_PULSE_H_
#define INCLUDE_HAL_PWM_H_
#define INCLUDE_HAL_TIMER_H_
#define INCLUDE_ICANINTERFACE_H_
#define INCLUDE_ISERVICEABLE_H_
#define INCLUDE_IVUS_COMM_H_
#define INCLUDE_IVUS_PINS_H_
#define INCLUDE_LED_H_
#define INCLUDE_LOGGER_H_
#define INCLUDE_LPC_H_
#define INCLUDE_MAILBOX_H_
#define INCLUDE_MDU_FAULT_H_
#define INCLUDE_MDU_PINS_H_
#define INCLUDE_NXGEN_H_
#define INCLUDE_NXGEN_PIN_DEFS_H_
#define INCLUDE_OCT_PINS_H_
#define INCLUDE_OE_COMM_H_
#define INCLUDE_OE_FAULT_H_
#define INCLUDE_OE_PARAMETERS_H_
#define INCLUDE_OE_PINS_H_
#define INCLUDE_OE_RTOS_H_
#define INCLUDE_PARAMSTORE_H_
#define INCLUDE_PIN_DETECTOR_H_
#define INCLUDE_POWER_MANAGER_H_
#define INCLUDE_QEI_BUILTIN_H_
#define INCLUDE_QEI_H_
#define INCLUDE_REGISTER_H_
#define INCLUDE_RTOS_TASK_H
#define INCLUDE_RUN_LEDS_H_
#define INCLUDE_SERIAL_PORT_H_
#define INCLUDE_STEPPER_CONTROL_H_
#define INCLUDE_STREAM_H_
#define INCLUDE_TIMEKEEPER_H_
#define INCLUDE_TLOG_H_
#define INCLUDE_TSTATE_H_
#define INCLUDE_UTILITIES_H_
#define INCLUDE_VERSIONINFO_H_
#define INCLUDE_WATCHDOG_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		Global Variables and Definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern uint32_t* eeprom;
extern uint32_t Sent_MQ_EVno;
extern uint32_t idIndex;
extern uint8_t Sent_tst_no;
extern uint8_t setStartError;
extern uint8_t powerIndex;
extern uint8_t crcCheck;
extern bool setCrc;
extern bool Tsk_created;
extern bool Tsk_started;
extern bool mtex_created;
extern bool q_initialized;
extern bool Tmr_Setup_done;
extern bool En_EV_Sent;
extern bool Rst_EV_Sent;
extern bool CB_Param_Sent;
extern bool tmr_running;
extern bool qei_enable;
extern bool got_mutex;
extern bool mutex_release;
extern bool SetPinInt_called;
extern bool EnablePinInt_called;
extern bool stallDectectionCall;
extern bool reportCall;
extern bool MQ_Send_executed;

typedef struct
{						// DAC Structure
    uint32_t  CR;		// DAC register Holds the conversion data
    uint32_t  CTRL;		// DAC control register
    uint32_t  CNTVAL;	// DAC counter value register
}
LPC_DAC_T;

#define MUTEX_TIMEOUT   10
#define LPC_DAC_BASE	0x400E1000
#define LPC_DAC ((LPC_DAC_T*) LPC_DAC_BASE)

enum result { OK, TIMEOUT, ERROR, INVALID };

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_ADDRESSMAP_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_ADDRESSMAP_H_

#define BOOT_ADDR_START				(0x1A000000UL)
#define BOOT_ADDR_END				(0x1A007FF7UL)
#define BOOT_SIZE					(BOOT_CRC_ADDR - BOOT_ADDR_START)
#define BOOT_VER_ADDR				(0x1A007FF8UL)
#define BOOT_CRC_ADDR				(0x1A007FFCUL)
#define APP_A_ADDR_START			(0x1A008004UL)
#define APP_A_ADDR_END				(0x1A07FFFFUL)
#define APP_A_SIZE					(APP_A_ADDR_END - APP_A_ADDR_START + 1)
#define APP_B_ADDR_START			(0x1B000000UL)
#define APP_B_ADDR_END				(0x1B06FFFFUL)
#define APP_B_SIZE					(APP_B_ADDR_END - APP_B_ADDR_START + 1)
#define APP_CRC_ADDR				eeprom
#define APP_VER_ADDR				(0x1A008008UL)
#define APP_NAME_ADDR				(0x1A00800CUL)
#define APP_NAME_SIZE				(0x34UL)
#define EEPROM_SERIAL_NO_ADDR		(0x20040000UL)
#define EEPROM_SERIAL_NO_SIZE		(0x10UL)
#define EEPROM_CONFIG_ADDR			(0x20040010UL)
#define EEPROM_CONFIG_SIZE			(0x100UL)
#define EEPROM_SERIAL_STR_SIZE		(0x1000UL)
#define EEPROM_SERIAL_BYT_SIZE      (0x0CUL)         // Size of the buffer used by the serno string
#define EEPROM_SERIAL_CRC_OFFSET	(0x0FUL)         // Size of the buffer used by the CRC Offset
#define EEPROM_CONFIG_ADDR          (0x20040010UL)
#define EEPROM_LOG_CONFIG_ADDR		(0x20040110UL)
#define EEPROM_LOG_CONFIG_SIZE		(0x10UL)

#endif // INCLUDE_ADDRESSMAP_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_TSTATE_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_TSTATE_H_

#ifdef __cplusplus
extern "C"
{
    namespace TState
    {
        // forward declare
        struct _tstate_s;
        struct _tstate_transition_s;
#else
// forward declare
extern struct _tstate_s;
extern struct _tstate_transition_s;
#endif

#define TR_Size(TR_TABLE)  (sizeof(TR_TABLE)/sizeof(tstate_transition_t))

// State Machine result
typedef enum
{
    FAILED,
    OK
}
tstate_result_t;

// Event type
typedef uint32_t tstate_event_t;

// Guard function type, return true to take the transition
typedef tstate_result_t(*tstate_guard_t)();

// Transition Effect function type
typedef void(*tstate_effect_t)();

// Transition structure
typedef struct _tstate_transition_s
{
    struct _tstate_s* pDestState;		            // Destination of the transition
    const tstate_event_t* pEvent;		            // Event to trigger this transition
    tstate_guard_t guard;				            // Guard for this transition, optional
    tstate_effect_t effect;				            // Effect of this transition, optional
}
tstate_transition_t;

// State Definition, should be constant
typedef struct _tstate_def_s
{
    tstate_effect_t entry;							// Entry function, called when entering the state
    tstate_effect_t exit;							// Exit function, called when leaving the state
    const struct _tstate_transition_s* pTranArray;  // Array of the transitions beginning from the state
    uint32_t numOfTransitions;						// Number of transitions beginning from this state
}
tstate_def_t;

// State structure, include runtime information
typedef struct _tstate_s
{
    const struct _tstate_def_s* pDef;		        // State definition
    struct _tstate_s* pParentState;			        // Parent state in the hierarchy state machine, null if this is the top state
    const struct _tstate_s* pInitSubState;	        // Initial sub state, null if this state does not include sub state machine
    struct _tstate_s* pCurrentSubState;		        // Current sub state, null if this state does not include sub state machine
}
tstate_t;

// Check if the specified state is "current" in the state machine and its sub state machines
// param[in]   pState  :  the top state include the state machine to be executed
// param[in]   pStateToBeChecked   :  the state to be checked
tstate_result_t TState_IsCurrentState(tstate_t* pState, tstate_t* pStateToBeChecked);

extern tstate_event_t EV_NULL;

#ifdef __cplusplus
    }
}
#endif

#endif // INCLUDE_TSTATE_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_RTOS_TASK_H
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_RTOS_TASK_H

#define RTOS_INDEFINITE_TIMEOUT (0xFFFFFFFFUL)
#define TCB_SIZE				172
#define Q_SIZE					48
#define SEM_SIZE				40
#define TMR_SIZE				48
#define MUTEX_SIZE				48
#define MEM_SIZE				32
#define MIN_STK_SIZE			64  // Minimum task stack size

namespace RTOS
{
    enum { DEFAULT_TASK_PRIORITY = 10 };

    // forward declare
    class Rtos;
    struct ShortMsg;

    // Enumeration for status return from RTOS functions
    enum result_t { OK, TIMEOUT, ERROR, INVALID };

    // RTOS::TaskCore Class
    // Abstract base class for a RTOS Task.
    class TaskCore
    {
    public:

        TaskCore(char const* name, uint8_t pri, uint32_t* stk, uint32_t stkSize) :
            _name(name),
            _pri(pri),
            _stk(stk),
            _tcb(),
            _stkSize(stkSize),
            _isCreated(false)
        { }

        virtual ~TaskCore(void) { }

        result_t Start(void) { return OK; }
        result_t Suspend(void) { return OK; }
        result_t Resume(void) { return OK; }
        result_t Delete(void) { return OK; }
        result_t PostSemaphore();
        void SetName(const char* pName);
        virtual bool Send(RTOS::ShortMsg msg, int num) = 0;

    protected:
        virtual void Execute(void) = 0;
        static void ExecuteTask(void* task);
        result_t PendSemaphore(uint32_t const timeout, uint32_t* count);

    private:
        char const* _name;
        uint8_t _pri;
        uint8_t _tcb[TCB_SIZE];
        uint32_t* _stk;
        uint32_t _stkSize;
        bool _isCreated;   // flag to indicated if the task has been created already
    };

    // RTOS::Task Class
    template <size_t stkSize>
    class Task : public TaskCore
    {
    public:
        Task(char const* name, uint8_t pri) : TaskCore(name, pri, _stkMemory, stkSize), _keepRun(true) { }
        explicit Task(const uint8_t pri) : Task("", pri) { }
        explicit Task(char const* name) : Task(name, DEFAULT_TASK_PRIORITY) { }
        Task() : Task(DEFAULT_TASK_PRIORITY) { }
        virtual void BeforeLoop() { }
        virtual void Loop() { }
        virtual void AfterLoop() { }

        bool Send(uint32_t ev, int delay) { return true; }

        bool Send(RTOS::ShortMsg msg, int num) { return true; }

        void Execute() override
        {
            _keepRun = true;

            BeforeLoop();

            while (_keepRun)
            {
                Loop();
            }

            AfterLoop();

            // Clean up
            Delete();
        }

        void Stop()
        {
            _keepRun = false;
        }

    private:
        bool _keepRun;
        uint32_t _stkMemory[stkSize];
    };

    // RTOS::Queue Class
    // RTOS Queue
    class Queue
    {
    public:
        Queue(char const* const name) :
            _name(name),
            _queue()
        { }

        result_t Create(uint32_t const count) { return OK; }
        result_t Post(const void* message, uint32_t  const size) { return OK; }
        result_t Pend(void** message, uint32_t* size, uint32_t const timeout) { return OK; }

    private:
        char const* const _name;
        uint8_t _queue[Q_SIZE];
    };
}

// RTOS::ImqItem Structure
// Interface of an item to be transmitted by message queue
struct IMqItem
{
    const char* _name;
    size_t _size;
    uint32_t _val;

    IMqItem(const char* name, size_t size, uint32_t val) : _name(name), _size(size), _val(val) { }
    IMqItem(const char* name, size_t size) : _name(name), _size(size), _val(0) { }
    IMqItem(uint32_t val) : IMqItem("", 0, val) { }
    IMqItem() : IMqItem("", 0) { }
};

// MsgObj Structure
/// Definition of RTOS Task message posted to task queues
struct MsgObj : public IMqItem
{
    const IMqItem* id;
    uint32_t param1;
    uint32_t param2;
    const void* data;
    uint32_t dataSize;

    MsgObj() :
        id(0),
        param1(0),
        param2(0),
        data(0),
        dataSize(0)
    { }

    static MsgObj* Acquire();
    static RTOS::result_t Release(MsgObj* pMsg);
};

namespace RTOS
{
    struct Ev : public IMqItem
    {
        Ev(const char* name) : IMqItem(name, 0) { }
        Ev(uint32_t val) : IMqItem(val) { }
        Ev() : Ev("") { }
    };

    struct ShortMsg : public IMqItem
    {
        ShortMsg(const char* name) : IMqItem(name, sizeof(MsgObj)) { }
        ShortMsg(uint32_t val) : IMqItem(val) { }
        ShortMsg() : ShortMsg("") { }
    };

    struct LongMsg : public IMqItem
    {
        LongMsg(const char* name) : IMqItem(name, -1) { }
        LongMsg(uint32_t val) : IMqItem(val) { }
        LongMsg() : LongMsg("") { }
    };

    namespace Motion
    {
        class MotorMonitor;
    }

    // RTOS::MQ Class
    // Message Queue
    // note The design may not support multiple task pending on the same Q
    class MQ : private Queue
    {
    public:
        static const RTOS::Ev evResetPosition;
        static const RTOS::Ev evEnable;
        static const RTOS::Ev evDisable;
        static const RTOS::Ev evReportTimer;
        RTOS::Ev Sent_EV;

        MQ() :
            Queue(""),
            _isCreated(false),
            sendEvValue(),
            sendEventIndex()
        { }

        bool Send(const Ev& ev)
        {
            if (&ev == &(evEnable))
            {
                En_EV_Sent = true;
            }

            if (&ev == &(evResetPosition))
            {
                Rst_EV_Sent = true;
            }

            if (&ev == &(evDisable))
            {
                En_EV_Sent = false;
            }

            Sent_EV = ev;
            return true;
        }

        bool Send(const LongMsg& longMsg, const void* data, uint32_t dataSize) { return true; }
        bool Send(const ShortMsg& shortMsg, uint32_t param1, uint32_t param2) { return true; }
        
        bool Send(const ShortMsg& shortMsg, uint32_t param1)
        {
            MQ_Send_executed = true;
            return true;
        }

        bool Send(const MsgObj& msgObj) { return true; }
        static RTOS::Ev* getEvReportTimer() { return (RTOS::Ev*)&evReportTimer; }

        const IMqItem* Receive(uint32_t timeout, MsgObj* pMsg)
        {
            if (idIndex == 1)
            {
                return &evEnable;
            }
            else if (idIndex == 2)
            {
                return &evDisable;
            }
            else if (idIndex == 3)
            {
                got_mutex = true;
                mutex_release = true;

                return &evResetPosition;
            }
            else if (idIndex == 5)
            {
                const RTOS::Ev* reportTimer = getEvReportTimer();
                return reportTimer;
            }

            return nullptr;
        }

        friend Rtos;

    private:
        enum { MAX_MSG_NUM = 75, REQ_FLAG = 0x80000000 };
        bool _isCreated;
        uint8_t sendEvValue;
        uint8_t sendEventIndex;
    };

    // RTOS::Timer Class
    // This class provides a wrapper of RTOS timer.  Timers can be periodical
    // or one-shot. A timer can send a message, send an event (message without
    // payload), or call a function.  All timers run in a timer task
    // Time unit used in Timer is millisecond
    class Timer
    {
    public:
        enum Type
        {
            PERIODICAL,
            ONESHOT
        };

        Timer(char const* name, Type type) :
            _name(name),
            _type(type),
            _pQ(0),
            _pEv(0),
            _pTmr(),
            _pCallback(0),
            _pCallbackParams(0),
            _isSet(false)
        { }

        virtual ~Timer() { }

        typedef void(*callback_func_t)(void* pParams);

        result_t Setup(uint32_t time, MQ& targetMQ, const Ev& ev)
        {
            Tmr_Setup_done = true;
            return OK;
        }

        result_t Setup(uint32_t time, callback_func_t pCallback, void* pParams)
        {
            pCallback(pParams);
            return OK;
        }

        result_t Start()
        {
            tmr_running = true;

            if (setStartError == 1)
            {
                return ERROR;
            }

            return OK;
        }

        result_t Stop()
        {
            tmr_running = false;
            return ERROR;
        }

        static int num;

    protected:
        static void TimerCallback(void* timer, void* pObj) { }

    private:
        char const* _name;
        uint8_t _pTmr[TMR_SIZE];
        Type _type;
        MQ* _pQ;
        const Ev* _pEv;
        callback_func_t _pCallback;
        void* _pCallbackParams;
        bool _isSet;

        bool _Create(uint32_t time);
    };

    // RTOS::Semaphore Class
    // RTOS Semaphore
    class Semaphore
    {
    public:
        Semaphore(char const* const name) :
            _semaphore(),
            _name()
        { }

        result_t Create(uint32_t const count);
        result_t Post(void);
        result_t Pend(uint32_t& count, uint32_t const timeout);

        char const* const _name;
        uint8_t _semaphore[SEM_SIZE];
    };

    // RTOS::Mutex Class
    // RTOS Mutex
    class Mutex
    {
    public:
        Mutex(char const* const name) :
            _name(),
            _mutex()
        {
            mtex_created = true;
        }

        result_t Create(void)
        {
            mtex_created = true;
            return OK;
        }

        result_t Acquire(uint32_t const timeout)
        {
            if (MUTEX_TIMEOUT == timeout)
            {
                got_mutex = true;
                return ERROR;
            }

            return OK;
        }

        result_t Acquire();

        result_t Release(void)
        {
            mutex_release = true;
            return OK;
        }

    private:
        enum { DEFAULT_MUTEX_TIMEOUT = 1000 };

        char const* const _name;
        uint8_t _mutex[MUTEX_SIZE];
    };

    // RTOS::MemPool Class
    // RTOS Memory Pool
    class MemPool
    {
    public:
        MemPool() :
            _name(),
            _pool(),
            _diag(),
            _mem_start(),
            _mem_stop(),
            _item_size()
        { }

        MemPool(char const* const name) :
            _name(name),
            _pool(),
            _diag(),
            _mem_start(0),
            _mem_stop(0),
            _item_size(0)
        { }

        result_t Create(uint8_t* const allocation, uint32_t const count, uint32_t const size);
        result_t Acquire(void** const block) { return OK; };
        result_t Release(const void* block);

    private:
        char const* const _name;
        uint8_t _pool[MEM_SIZE];
        uint32_t _mem_start;
        uint32_t _mem_stop;
        uint32_t _item_size;

        struct _Diag
        {
            uint32_t acquireCnt;
            uint32_t releaseCnt;
        };

        _Diag _diag;
    };

    typedef void(*init_func_t)();
}

#endif // INCLUDE_RTOS_TASK_H

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_HAL_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_HAL_H_

// Hardware Abstraction Layer
// This class provides the abstracted interface to access on-chip
// and off-chip peripherals

#define PASS 1

namespace HAL
{
    // HAL::IOs Class
    // This is the class for the HAL to utilize the OS functions
    class IOs
    {
    public:
        virtual void Delay(uint32_t ms) = 0;
    };

    // Type of edge detections
    enum edge_t { EDGE_NONE, EDGE_RISING, EDGE_FALLING, EDGE_BOTH };

    // CPU Frequence
    enum { CPU_FREQUENCY_HZ = 204000000 };

    typedef void(*callback_func_t)(void* pObj);

    // HAL::Lock Class
    class Lock
    {
    public:
        Lock() :
            _lockVar(0)
        { }

        bool Get() { return true; }
        void Free() { _lockVar = 0; }

    private:
        volatile unsigned int _lockVar;
    };
};

#endif // INCLUDE_HAL_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_HAL_PIN_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_HAL_PIN_H_

namespace HAL
{
    // Forward declare
    class PinGroup_Interrupt;

    // HAL::IO Class
    // Interface for IO pins
    class IO
    {
    public:
        IO() { }

        virtual ~IO() { }

        // Set the output status of this IO
        // param val  : true, IO is asserted; false, IO is de-asserted
        virtual void Set(bool val) = 0;

        // Toggle the output status of this IO
        virtual void Toggle() = 0;

        // Get the input status of this IO
        // return  true, IO is asserted; false, IO is de-asserted
        virtual bool Get() = 0;
    };

    // HAL::Pin Class
    // Implementation of the IO pins (NXP chip dependent)
    class Pin : public IO
    {
    public:
        // Enum for pin function selection 
        enum FUNC
        {
            FUNC_0 = 0,
            FUNC_1 = 1,
            FUNC_2 = 2,
            FUNC_3 = 3,
            FUNC_4 = 4,
            FUNC_5 = 5,
            FUNC_6 = 6,
            FUNC_7 = 7
        };

        // Enum bitmap for pin options
        enum OPTION
        {
            NO_OPTION = 0,
            REVERSED = 1,	// the pin logic is reversted, 0-asserted, 1-de-asserted
            INPUT = 2,		// the pin is input
            OUTPUT = 4,		// the pin is output
            PULLUP = 8,		// enable internal pullback resistor
            PULLDOWN = 16,	// enable internal pulldown resistor
            DIS_FILT = 32	// disable the filter, otherwise the filter is enabled by default
        };

        bool isReversed;	// Pin is reversed
        uint8_t gport;		// GPIO port number of this pin
        uint8_t gpin;		// GPIO pin number of this pin

        // Constructor for GPIO pin
        // param port  : port number
        // param pin   : pin number
        // param gport : GPIO port number
        // param gpin  : GPIO pin number
        // param func  : pin function as defined in the data sheet
        // param options : pin options
        Pin(uint8_t port, uint8_t pin, uint8_t gport, uint8_t gpin, FUNC func, uint32_t options) :
            IO(),
            gport(gport),
            gpin(gpin),
            _gpinAddr(),
            _isNotUsed(),
            _isReversed(),
            _options(),
            _pPinInt()
        { }

        // Constructor for other peripheral pin (for example a I2C pin)
        // param port  : port number
        // param pin   : pin number
        // param func  : pin function as defined in the data sheet
        // param options : pin options
        Pin(uint8_t port, uint8_t pin, FUNC func, uint32_t options) :
            IO(),
            gport(gport),
            gpin(gpin),
            _gpinAddr(),
            _isNotUsed(),
            _isReversed(),
            _options(),
            _pPinInt()
        { }

        // Destructor
        virtual ~Pin() { }

        //	Runtime control

        // Set the output status of this IO
        // param val  : true, IO is asserted; false, IO is de-asserted
        void Set(bool val) override { }

        // Toggle the output status of this IO
        void Toggle() override { }

        // Get the input status of this IO
        // return  true, IO is asserted; false, IO is de-asserted
        bool Get() override { return true; }

        // Enable the pin interrupt
        void EnableInt();

        // Disable the pin interrupt
        void DisableInt();

        // HAL::Pin::Interrupt Class
        // Nested individual interrupt class
        class Interrupt
        {
        public:
            // Destructor
            virtual ~Interrupt();

            // Runtime control

            // Enable the interrupt
            void Enable() const;

            // Disable the interrupt
            void Disable() const;

            enum { NUM_OF_PIN_INT = 8 };					// Total 8 avaialbe individual interrupts
            static Interrupt* _pInterrupt[NUM_OF_PIN_INT];  // Record all created pin interrupts
            void _PinIntHandler();							// Interrupt handler

        private:
            Interrupt(Pin& pin) :
                _pin(pin),
                _ev(),
                _id(),
                _isReversed(),
                _pObj()
            { }

            static uint8_t _num_of_int;
            uint8_t _id;
            void* _pObj;
            uint32_t _ev;
            Pin& _pin;
            bool _isReversed;

            static Interrupt* Create(Pin& pin);
            static Interrupt* Create0(Pin& pin);
            static Interrupt* Create1(Pin& pin);
            static Interrupt* Create2(Pin& pin);
            static Interrupt* Create3(Pin& pin);
            static Interrupt* Create4(Pin& pin);
            static Interrupt* Create5(Pin& pin);
            static Interrupt* Create6(Pin& pin);
            static Interrupt* Create7(Pin& pin);
        };

    protected:
        uint32_t _options;
        volatile uint8_t* _gpinAddr;
        Interrupt* _pPinInt;
        bool _isReversed;
        bool _isNotUsed;

        friend PinGroup_Interrupt;
    };

    // HAL::PinGroup_Interrupt Class
    // Group interrupt, only detects de-assert to assert transition
    class PinGroup_Interrupt
    {
    public:
        // Factory method to create a pin group interrupt
        static PinGroup_Interrupt* CreatePinGroupInterrupt();

        // Destructor
        virtual ~PinGroup_Interrupt();

        // Runtime control

        // Enable the group interrupt
        void Enable() const;

        // Disable the group interrupt
        void Disable() const;

        // Interrupt handler
        void _IntHandler();

        static PinGroup_Interrupt* _pPinGroup_Interrupt[2];  // Record the created pin group interrupts

    private:
        enum INT_GROUP
        {
            GROUP_0,
            GROUP_1
        };

        Pin* _pPin[8];
        void* _callbackObj[8];
        void* _groupCallbackObj;
        static uint8_t _count;
        uint8_t _id;
        uint32_t _ev;
        bool _isReversed;
        uint8_t _numOfPins;

        static PinGroup_Interrupt* _Create();
        static PinGroup_Interrupt* _Create0();
        static PinGroup_Interrupt* _Create1();

        PinGroup_Interrupt();
    };
}

#endif // INCLUDE_HAL_PIN_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_HAL_TIMER_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_HAL_TIMER_H_

namespace HAL
{
    // HAL::Timer Class
    // Timer peripheral for Timer 0/1/2/3
    // note Match 0 is reserved for period
    class Timer
    {
    public:
        // Timer peripheral selections
        enum timer_t { TIMER_0, TIMER_1, TIMER_2, TIMER_3 };

        // Capture peripheral selection
        enum cap_in_t { CAP0, CAP1, CAP2, CAP3 };

        // Match peripheral selections
        enum match_t { MATCH0, MATCH1, MATCH2, MATCH3 };

        // Match function selections
        enum match_act_t { NOTHING, MATCH_TO_SET, MATCH_TO_CLR, MATCH_TO_TOGGLE };

        // Timer Constructor
        // param id : timer peripheral selection
        Timer(timer_t id);

        static Timer* GetTimer(timer_t id)
        {
            HAL::Timer* t = new Timer(id);
            return t;
        }

        void SetExtClock(cap_in_t clk_src, edge_t edge) const;
        void SetPrescaler(uint32_t prescaler) const;
        void SetPeriod(uint32_t period, callback_func_t callback, void* pObj);
        void SetCapture(cap_in_t cap_in, edge_t edge, callback_func_t callback, void* pObj);
        void SetMatch(match_t match, match_act_t act, uint32_t value) const;
        void ClearMatch(match_t match) const;
        uint32_t GetTimerVal() const { return 0; }
        uint32_t GetCapturedVal(cap_in_t cap_in) const;
        void Enable() const;
        void Disable() const;
        void Reset() const;
        void IntHandler();
        static int num;

    private:
        timer_t _id;
        callback_func_t _periodCallback;
        void* _periodCallbackObj;
        callback_func_t _captureCallback[4];
        void* _captureCallbackObj[4];

        void Init();
    };
}

#endif // INCLUDE_HAL_TIMER_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_HAL_PWM_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_HAL_PWM_H_

namespace HAL
{
    // HAL::PWM Class
    class PWM
    {
    public:
        enum pwm_ch_t { PWM_CH_0, PWM_CH_1, PWM_CH_2 };

        static HAL::PWM* Init();
        static HAL::PWM* GetPwm();

        void SetFrequency(pwm_ch_t ch, uint32_t freqHz);
        void SetDutyCycle(pwm_ch_t ch, float dutyCycle);
        void Enable(pwm_ch_t ch);
        void Disable(pwm_ch_t ch);

    private:
        PWM() :
            _period(),
            _dutyCycle()
        { }

        static HAL::PWM* _pPwm;
        uint32_t _period[3];
        uint32_t _dutyCycle[3];
    };
}

#endif // INCLUDE_HAL_PWM_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_HAL_PULSE_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_HAL_PULSE_H_

namespace HAL
{
    // HAL::Pulse Class
    class Pulse
    {
    public:
        virtual bool Start(uint32_t period, uint32_t pulseNum, callback_func_t callback, void* pObj) = 0;
        virtual void ChangePeriod(uint32_t period) = 0;
        virtual uint32_t GetPulses() = 0;
        virtual uint32_t Stop() = 0;
    };

    // HAL::Pulse_Timer Class
    // Pulse generator
    // This peripheral uses the Timer and generate simple pulse generator.  It
    // controls both the pulse period, and number of pulses. The pulses are 50%
    // It is designed to send pulses to the stepper drive
    class Pulse_Timer : public Pulse
    {
    public:
        Pulse_Timer(Timer::timer_t periodTimer, Timer::match_t pulseOutput,
            Timer::timer_t pulseCounter, Timer::cap_in_t counterInput) :
            _periodTimer(Timer::GetTimer(periodTimer)),
            _pulseCounter(Timer::GetTimer(pulseCounter)),
            _matchIdForPulseOutput(pulseOutput),
            _capIdForPulseNum(counterInput),
            _pulseNum(0),
            _endPulsesCallback(nullptr),
            _endPulsesCallbackObj(nullptr),
            _diag(),
            _isRunning(),
            _isTimingChanged(),
            _period()
        { }

        virtual bool Start(uint32_t period, uint32_t pulseNum, callback_func_t callback, void* pObj) override
        {
            return true;
        }

        virtual void ChangePeriod(uint32_t period) override { }
        virtual uint32_t GetPulses() override
        {
            return _pulseCounter->GetTimerVal();
        }

        virtual uint32_t Stop() override { return 0; }
        static void PulseCounterCallback(void* pObj) { }

    private:
        Timer* _periodTimer;
        Timer* _pulseCounter;
        Timer::match_t _matchIdForPulseOutput;
        Timer::cap_in_t _capIdForPulseNum;
        uint32_t _pulseNum;
        bool _isRunning;
        bool _isTimingChanged;
        uint32_t _period;

        struct diag
        {
            uint32_t startCounter;
            uint32_t pulseCounter;
            uint32_t completeCounter;
        };
        diag _diag;

        callback_func_t _endPulsesCallback;
        void* _endPulsesCallbackObj;
    };
}

#endif // INCLUDE_HAL_PULSE_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_HAL_GPIO_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_HAL_GPIO_H_

namespace HAL
{
    // Forward declare
    class GpioPortWriter;

    // HAL::GPIO CLass
    // General Purpose I/O
    // TODO the applications benefit from using port reading/writing
    class GPIO
    {
    public:
        typedef struct
        {
            uint8_t id;
            uint8_t port;
            uint8_t pin;
            uint8_t gpioPort;
            uint8_t gpioPin;
            uint8_t func;
            uint8_t direction;
            bool inverted;
        }
        gpio_def_t;

        enum _DIRECTION { IN, OUT, OTHER };
        enum INT_GROUP { INT_GROUP_0 = 0, INT_GROUP_1 = 1 };

        static GPIO* GetGpio() { return _pGpio; }
        static GPIO* Init(gpio_def_t const* table, uint8_t tableSize);
        void SetPinInt(uint8_t id, HAL::edge_t edge, callback_func_t callback, void* pObj)
        {
            if ((255 == id) && (HAL::EDGE_RISING == edge))
            {
                SetPinInt_called = true;
            }
        }

        void EnablePinInt(uint8_t id)
        {
            if (255 == id)
            {
                EnablePinInt_called = true;
            }
        }

        void DisablePinInt(uint8_t id);
        static void PinIntCallback(uint8_t index);
        void SetGroupInt(INT_GROUP group, uint8_t* idTable, uint8_t idTableSize, callback_func_t callback, void* pObj);
        static void EnableGroupInt(INT_GROUP group);
        static void DisableGroupInt(INT_GROUP group);
        void Set(uint8_t id, bool isOn) const;
        void Toggle(uint8_t id) const;
        bool Get(uint8_t id) const;

        callback_func_t _groupInt[2];
        void* _groupIntObj[2];

    private:
        enum { NUM_OF_PINT = 8 };

        static GPIO* _pGpio;
        gpio_def_t const* _gpioTable;
        uint8_t _gpioTableSize;
        uint8_t _pinIntList[NUM_OF_PINT];   // Record the corresponding GPIO index
        uint8_t _pinIntCount;               // 0-7
        callback_func_t _pinIntCallback[NUM_OF_PINT];
        void* _pinIntClallbackObj[NUM_OF_PINT];

        GPIO(gpio_def_t const* table, uint8_t tableSize);

        friend GpioPortWriter;
    };

    class GpioPortWriter
    {
    public:
        GpioPortWriter(const uint8_t idList[], uint8_t numOfIos);

        ~GpioPortWriter();

        void Assert() const;
        void Deassert() const;

    private:
        const uint8_t* _idList;
        const uint8_t _numOfIos;
        uint8_t _gport;
        uint32_t _gpinMask;
        bool _inverted;
    };
}

#endif // INCLUDE_HAL_GPIO_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_NXGEN_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_NXGEN_H_

namespace NXGen
{
    // Enumeration for the protocol channel identifiers
    enum Ch
    {
        CH_PC_TO_CMD,                       // PC      to command
        CH_CMD_TO_PC,                       // command to PC
        CH_MDU_TO_CMD,                      // MDU     to command
        CH_CMD_TO_MDU,                      // command to MDU
        CH_CMD_TO_PWR,                      // command to power
        CH_CMD_TO_OE,                       // command to OE
        CH_CMD_TO_OCT,                      // command to OCT
        CH_CMD_TO_IVUS,                     // command to IVUS
        CH_CMD_TO_SAFE,                     // command to safety
        CH_INVALID = 255
    };

    // Packet IDs
    enum PID
    {
        PID_TIME_STAMP = 10,
        PID_CHANGE_STATE = 11,
        PID_EVENT = 12,
        PID_CLEAR_FAULT = 13,
        PID_ENTER_READY = 14,
        PID_START_PULLBACK = 15,
        PID_START_SCAN = 16,
        PID_PULLBACK_RESET = 17,
        PID_SET_MODALITY = 20,
        PID_SET_PULLBACK = 21,
        PID_SET_PULLBACK_SPEED = 22,
        PID_SET_PULLBACK_SPEED_LIMIT = 23,
        PID_SET_SCAN_SPEED = 24,
        PID_SET_LASER_ON_OFF = 25,
        PID_SET_DELAY_LINE_POSITION = 26,
        PID_SET_OE_SWITCH = 27,
        PID_CMD_PC_STAT = 31,
        PID_CMD_MDU_STAT = 32,
        PID_CMD_PWR_STAT = 33,
        PID_CMD_OE_STAT = 34,
        PID_CMD_OCT_STAT = 35,
        PID_CMD_IVUS_STAT = 36,
        PID_CMD_SAFE_STAT = 37,
        PID_REPORT_SYSTEM_VERSIONS = 51,
        PID_GET_VERSION = 52,
        PID_READ_OE_PARAM = 53,
        PID_WRITE_OE_PARAM = 54,
        PID_SET_LOG_FILTER = 101,
        PID_GET_LOG_ENTRIES = 102,
        PID_CLEAR_LOG = 103,
        PID_READ_OE_MAP = 110,
        PID_WRITE_OE_MAP = 111,
        PID_ENTER_SERVICE = 200,
        PID_START_REPROGRAMMING = 201
    };

    enum NODE_ID
    {
        NODE_CONSOLE = 0,
        NODE_CMD = 1,
        NODE_MDU = 2,
        NODE_PWR = 3,
        NODE_OE = 4,
        NODE_OCT = 5,
        NODE_IVUS = 6,
        NODE_SAFE = 7,
        NODE_UNKNOWN = 10,
        NODE_DIAG = 15
    };

    enum SUB_ID
    {
        SUB_CMD = 1,
        SUB_CONSOLE,
        SUB_MDU,
        SUB_PWR,
        SUB_OE,
        SUB_OCT,
        SUB_IVUS,
        SUB_SAFE
    };

    enum RESPONSE_CODE
    {
        SUCCESS = 1,	                // Success is always One.
        BAD_CMD = 2,	                // The Command code is not valid
        BAD_MODE = 3,	                // Not in the proper System Mode 
        BAD_STATE = 4,	                // Not in the proper state 
        SW_ERR = 5,		                // Software error. eg. Default case
        SAFESTATE = 6,	                // In Safe State.
        RANGE_ERR = 7,	                // parameter failed its range check
        FAULT = 8,		                // Expect Event Message (TBD)
        EEPROM_ERR = 9                  // Read or write error on EEPROM
    };

    // High Level States for the common state machine
    enum SYSTEM_STATE
    {
        STATE_INIT = 0,                 // Init state, should be here when the system is in POST
        STATE_STANDBY = 100,
        STATE_POWERUP = 20,
        STATE_ILOCK_SYNC = 3,
        STATE_NORMAL = 4,
        STATE_SERVICE = 30,             // Service state
        STATE_POWERDOWN = 21,
        STATE_SAFE = 13,
        STATE_RECOVERY = 16,
        STATE_OE_CAL_BG = 50,
        STATE_OE_CAL_SC = 51,
        STATE_NO_CATHETER = 1,
        STATE_CONNECTED = 2,
        STATE_CONSOLE_BUSY = 32,
        STATE_CHANGE_TO_SCAN = 7,
        STATE_SCAN = 8,
        STATE_MANUAL_RECORDING = 31,
        STATE_CHANGE_TO_READY = 9,
        STATE_READY_TO_PULLBACK = 10,
        STATE_PULLBACK = 11,
        STATE_STOPPING = 15,
        STATE_POWERFAILURE = 22,
        STATE_POWERING = 23
    };

    enum UART_EV
    {
        UART_EV_HOME_BUTTON = 1,        // The reset button is pressed
        UART_EV_SCAN_BUTTON = 4,        // The scan button is pressed
        UART_EV_READY_BUTTON = 5,       // The Ready button is pressed
        UART_EV_PULLBACK_BUTTON = 6,    // The pullback button is pressed
        UART_EV_FLT_CLR = 7,            // Fault condition has been removed
        UART_EV_POWER_DOWN = 9,         // User request to power down
        UART_EV_OE_CALIB_BG = 10,       // Start optical eng. calibration
        UART_EV_OE_CALIB_SCAN = 11,     // Start optical eng. scan 1
        UART_EV_STOP = 20,
        UART_EV_MANUAL_REC = 21,
        UART_EV_REC_DONE = 22,
        UART_EV_CONSOLE_BUSY = 23,
        UART_EV_CONSOLE_FREE = 24,
        UART_EV_CATH_CAL_DONE = 25,
        UART_EV_CATH_CAL_CLEAR = 26,
        UART_EV_ORIGIN_SET = 30,
        UART_EV_DELAYLINE_DONE = 31,
        UART_EV_GO_SAFE = 99            // Go to safe state
    };

    enum CATH_TYPE
    {
        CATH_DISCONNECTED = 0,
        CATH_OCT = 1,
        CATH_DUAL = 2
    };

    // TBD: clarification how scan mode is notified to EM
    // Based on info from console this can be sent to OE and IVUS cards
    // to let them know when to disable the laser or ultrasound
    enum MODALITY
    {
        MODE_OFF = 0,
        MODE_OCT = 1,
        MODE_IVUS = 2,
        MODE_DUAL = 3
    };

    enum LASER_STAT
    {
        LASER_OFF,
        LASER_DUMP,
        LASER_CAL,
        LASER_MDU
    };

    enum ULTRASOUND_STAT
    {
        ULTRASOUND_OFF,
        ULTRASOUND_ENABLED
    };

    enum PWR_STAT
    {
        PWR_OFF,
        PWR_ON
    };

    struct PULL_SETTING
    {
        uint8_t scanSpeed;			        // Scan speed enum, see [ICD-610]
        uint8_t reserved;
        uint16_t pullSpeed;			        // Pullback speed enum, see [ICD-609]
        uint32_t maxPullDistance;	        // Max pullback distance in um
    };

    struct DELAYLINE
    {
        int32_t targetPos;
        int32_t actualPos;
    };

    // System status
    struct SYS_STAT
    {
        uint8_t systemState;		        // See [ICD-608] 
        uint8_t catheterType;		        // See [ICD-712] 
        uint8_t laserOnOff;			        // 0: Off, 1: On
        uint8_t modality;			        // See [ICD-611] 
        uint8_t UltrasoundOnOff;	        // 0: Off, 1: On 
        uint8_t calibrated;
        uint8_t HomingStatus;
        uint8_t EMLogPercentage;
        PULL_SETTING pullSettings;
        int32_t pullbackSpeed;		        // Actual pullback speed in um/S 
        uint32_t scanSpeed;			        // Actual scan speed in 0.1RPM 
        int32_t pullbackPosition;	        // Actual pullback position in um
        int32_t originPosition;
        int32_t delaylinePosition;
        uint32_t systemFaults;		        // Bitmap for system top level faults
    };

    // Console status
    struct PC_STAT
    {
        uint8_t state;
        uint8_t _reserved;
        uint16_t _reserved2;
        uint32_t consoleTime;
    };

    // Version Information
    struct VER_INFO
    {
        uint16_t major;
        uint16_t minor;
        uint32_t crc;
    };

    struct NODE_INFO
    {
        VER_INFO bootVer;
        VER_INFO appVer;
        uint8_t sn[12];
    };

    struct SYS_INFO
    {
        NODE_INFO cmdInfo;	                // Info of Command EM
        NODE_INFO mduInfo;	                // Info of MDU EM
        NODE_INFO pwrInfo;	                // Info of Power EM
        NODE_INFO oeInfo;	                // Info of Optical Engine EM
        NODE_INFO octInfo;	                // Info of OCT EM
        NODE_INFO ivusInfo;	                // Info of IVUS EM
        NODE_INFO safeInfo;	                // Info of Safety EM
    };

    // Request to start a bootloader bypass
    struct BOOTLOADER_REQUEST
    {
        uint32_t subsystem;
    };

    // Response to request
    struct RESPONSE
    {
        uint32_t resp;
    };

    // Request message
    struct REQUEST
    {
        uint32_t req;
    };

    enum MAP_ID
    {
        MAP_BACKGROUP = 1,
        MAP_DIS = 2,
        MAP_INDEX = 4,
        MAP_WEIGHT = 5
    };

    // Read MAP file
    struct READ_MAP
    {
        uint16_t MAP_ID;
        uint16_t INDEX;
        uint32_t BYTES;
    };

    struct MAP_DATA
    {
        uint16_t MAP_ID;
        uint16_t INDEX;
        uint32_t BYTES;
        uint8_t DATA[128];
    };

    struct LOG_REQ
    {
        uint8_t INDEX;	                    // Values[0, 1, 2, 3, 4, 5, 6, 7] representing chunk number
        uint8_t RESV1;	                    // reserved for alignment
        uint16_t RESV2;                     // reserved for alignment
    };

    struct SYSLOG_ENTRY_T
    {
        uint32_t LOGCODE;
        uint32_t TIMESTAMP;
    };

    struct LOG_RESP
    {
        uint8_t INDEX;                      // Values[0,1,2,3,4,5,6,7] representing chunk number
        uint8_t CURRENT_ENTRIES;            // This is count of log entries which contains with this payload.Usually be 32, but can be smaller
        uint8_t REMAIN_ENTERIES;            // This is count of log entries which is not retrieved by Console app.If EM has 256 entries, each message will have one of these values : [224,192, 160, 128, 64,32], but can be any number smaller than that
        uint8_t RESV;                       // reserved for alignment
        SYSLOG_ENTRY_T SYSLOG_ENTRIES[32];  // Fill up the remaining unused log entry bytes information with 0s.
                                            // NOTE: 8bytes represent one log entry.Upper 4bytes represent time stamp from startup.This startup time is updated when EM receive TimeStamp command.Lower 4bytes is value representing an enumeration from EM that would correspond to a more detailed log string for parsing
    };

    // Console Fault Bits
    enum FAULT_CODE
    {
        FAULT_SCAN,
        FAULT_LINEAR,
        FAULT_LASER,
        FAULT_OP_SWITCH,
        FAULT_CONTROL,
        FAULT_COMM,
        FAULT_POWER = 10,
        FAULT_STATE,
        FAULT_HW,
        FAULT_SW,
        FAULT_POWER_UP,
        FAULT_TEST
    };

    enum CAN_EV_ID
    {
        GO_SAFE = 100,
        FAULT_CLEAR = 101,
        BTN_HOME = 102,
        BTN_ORIGIN = 103,
        BTN_SCAN = 104,
        BTN_READY = 105,
        BTN_PULLBACK = 106,
        PROP_ACT = 107,
        PROP_DEACT = 108,
        PULLBACK_DONE = 109,
        POWER_DOWN = 110,
        OE_DELAYLINE_DONE = 111,
        LIN_MARK = 200,
        CAL_DONE = 201,
        HOME_DONE = 202,
        LIN_CTRL_ACT = 203,
        LIN_CTRL_DEACT = 204
    };

    enum CAN_EXG_ID
    {
        CMD_MDU_GET_LOG = 5,
        CMD_PWR_GET_LOG = 5,
        CMD_OCT_GET_LOG = 5,
        CMD_IVUS_GET_LOG = 5,
        CMD_SAFE_GET_LOG = 5,
        CMD_OE_GET_LOG = 5,
        CMD_OE_READ_MAP = 6,
        CMD_OE_WRITE_MAP = 7
    };

    enum CAN_REG_ID
    {
        SYSTEM_STATE_REG = 100,
        TIMESTAMP_REG = 101,
        PWR_STATUS = 102,
        LASER_STATUS = 103,
        MDU_SCAN_SPEED = 104,
        SAFE_SCAN_SPEED = 105,
        MDU_PULL_SPEED = 106,
        SAFE_PULL_SPEED = 107,
        MDU_PULL_POS = 108,
        SAFE_PULL_POS = 109,
        PULLBACK_SETTINGS = 110,
        DELAY_LINE_POS = 111,
        CATH_TYPE_REG = 112,
        MODALITY_REG = 113,
        ULTRASOUND_STATUS = 114,
        OE_PARAMS_REG = 115,
        MDU_FAULTS = 120,
        OCT_FAULTS = 121,
        IVUS_FAULTS = 122,
        SAFE_FAULTS = 123,
        OE_FAULTS = 124,
        PWR_FAULTS = 125,
        MDU_INFO = 140,
        OCT_INFO = 141,
        IVUS_INFO = 142,
        SAFE_INFO = 143,
        OE_INFO = 144,
        PWR_INFO = 145,
        OCT_DAQ_REQ_REG = 146,
        OCT_DAQ_DATA_REG = 147
    };

    // Parameter definitions stored in NV memory
    struct DEF_PARAMS
    {
        char serno[EEPROM_SERIAL_NO_SIZE];  // Place for serial number storage
        int32_t continuePwrUp;              // Continue powering up flag storage
        uint32_t LogLevel;                  // Log level
    };

    // CMD Specific NV Storage
    // Size = 32 bytes
    struct CMD_PARAMS
    {
        DEF_PARAMS  commonParams;
        uint32_t storageCRC;            // CRC check value for stored data
    };

    // OCT Specific NV Storage
    // Size = 32 bytes
    struct OCT_PARAMS
    {
        DEF_PARAMS commonParams;
        uint32_t storageCRC;            // CRC check value for stored data
    };

    // IVUS Specific NV Storage
    // Size = 32 bytes
    struct IVUS_PARAMS
    {
        DEF_PARAMS commonParams;
        uint32_t storageCRC;            // CRC check value for stored data
    };

    // Safety Specific NV Storage
    // Size = 40 bytes
    struct SAFETY_PARAMS
    {
        DEF_PARAMS commonParams;
        int32_t cabinetFanSpeed;        // Cabinet FAN Speed
        uint32_t storageCRC;            // CRC check value for stored data
    };

    // PWR Specific NV Storage
    // Size = 32 bytes
    struct PWR_PARAMS
    {
        DEF_PARAMS commonParams;
        uint32_t storageCRC;            // CRC check value for stored data
    };

    // OE Specific NV Storage
    // Size = 60 bytes
    struct OE_PARAMS
    {
        uint32_t COEFF_A;
        uint32_t MDU_HOME;
        uint32_t CAL_HOME;
        uint32_t CRC;
        uint32_t DIS_CRC;
        uint32_t IDX_CRC;
        uint32_t WT_CRC;
    };

    struct OE_NV_PARAMS
    {
        DEF_PARAMS commonParams;
        OE_PARAMS OEparams;
        uint32_t storageCRC;            // CRC check value for stored data
    };

    // MDU Specific NV Storage
    // Size = 48 bytes
    struct MDU_PARAMS
    {
        DEF_PARAMS  commonParams;
        int32_t stepperCorrection;      // Correction value used during pullback to correct for length variations
        uint32_t LinearTrimvalue;       // Offset positive trim with pullback length
        uint32_t BaseDriveCurrent;      // User adjustable drive current setting
        uint32_t storageCRC;            // CRC check value for stored data
    };

    enum CAN_STREAM_ID
    {
        CLI = 5
    };

    enum MDU_FAULTS
    {
        MDU_FAULT_SCAN = 1UL << 0,		// scan motion fault
        MDU_FAULT_PULL = 1UL << 1,		// linear motion fault
        MDU_FAULT_POWER = 1UL << 10,	// power supply fault
        MDU_FAULT_STATE = 1UL << 11,	// state machine fault
        MDU_FAULT_HW = 1UL << 12,		// hardware fault
        MDU_FAULT_SW = 1UL << 13,		// software fault
        MDU_FAULT_POWERUP = 1UL << 14	// Power Up fault
    };

    enum OCT_FAULTS
    {
        OCT_FAULT_POWER = 1UL << 10,	// power supply fault
        OCT_FAULT_STATE = 1UL << 11,	// state machine fault
        OCT_FAULT_HW = 1UL << 12,		// hardware fault
        OCT_FAULT_SW = 1UL << 13,		// software fault
        OCT_FAULT_POWERUP = 1UL << 14	// Power Up fault
    };

    enum IVUS_FAULTS
    {
        IVUS_FAULT_POWER = 1UL << 10,	// power supply fault
        IVUS_FAULT_STATE = 1UL << 11,	// state machine fault
        IVUS_FAULT_HW = 1UL << 12,		// hardware fault
        IVUS_FAULT_SW = 1UL << 13,		// software fault
        IVUS_FAULT_POWERUP = 1UL << 14	// Power Up fault
    };

    enum SAFE_FAULTS
    {
        SAFE_FAULT_SCAN = 1UL << 0,		// scan motion fault
        SAFE_FAULT_PULL = 1UL << 1,		// linear motion fault
        SAFE_FAULT_POWER = 1UL << 10,	// power supply fault
        SAFE_FAULT_STATE = 1UL << 11,	// state machine fault
        SAFE_FAULT_HW = 1UL << 12,		// hardware fault
        SAFE_FAULT_SW = 1UL << 13,		// software fault
        SAFE_FAULT_POWERUP = 1UL << 14	// Power Up fault
    };

    enum OE_FAULTS
    {
        OE_FAULT_LASER = 1UL << 2,		// laser control fault
        OE_FAULT_SWITCH = 1UL << 3,		// optical switch fault
        OE_FAULT_POWER = 1UL << 10,		// power supply fault
        OE_FAULT_STATE = 1UL << 11,		// state machine fault
        OE_FAULT_HW = 1UL << 12,		// hardware fault
        OE_FAULT_SW = 1UL << 13,		// software fault
        OE_FAULT_POWERUP = 1UL << 14	// Power Up fault
    };

    enum PWR_FAULTS
    {
        PWR_FAULT_POWER = 1UL << 10,	// power supply fault
        PWR_FAULT_STATE = 1UL << 11,	// state machine fault
        PWR_FAULT_HW = 1UL << 12,		// hardware fault
        PWR_FAULT_SW = 1UL << 13,		// software fault
        PWR_FAULT_POWERUP = 1UL << 14	// Power Up fault
    };

    enum SYS_FAULTS
    {
        SYS_FAULT_SCAN = 1UL << 0,		// scan motion fault
        SYS_FAULT_PULL = 1UL << 1,		// linear motion fault
        SYS_FAULT_LASER = 1UL << 2,		// laser control fault
        SYS_FAULT_SWITCH = 1UL << 3,	// optical switch fault
        SYS_FAULT_POWER = 1UL << 10,	// power supply fault
        SYS_FAULT_STATE = 1UL << 11,	// state machine fault
        SYS_FAULT_HW = 1UL << 12,		// hardware fault
        SYS_FAULT_SW = 1UL << 13,		// software fault
        SYS_FAULT_POWERUP = 1UL << 14	// Power Up fault
    };

    enum ILOCK_STATES
    {
        ILOCK_STATE_LOCKOUT,            // ILock Lockout state
        ILOCK_STATE_CLEAR               // ILock Lockout state clear
    };

    enum UART_BAUD_RATES
    {
        SERVICE_UART_BAUD = 19200,      // Service port Baudrate
        CONSOLE_UART_BAUD = 115200      // PC port Baudrate
    };

    // Common Tasks
    // Supports Only maximum of 32 Tasks
    enum EM_TASK_NO
    {
        COMMON_COMM,                    // Comm task used by all EMs
        COMMON_FAULT,                   // Fault task
        COMMON_CONTROL,                 // Control task
        COMMON_POWER,                   // Power monitor task
        COMMON_SERVICE_PORT,            // Also applies for COMMON_CANSERVICE_PORT
        COMMON_LOGGER,                  // Logger task
        COMMON_EEPROM,                  // Used by all ems that store parameters
        COMMON_TASK_MAX,                // IVUS, PWR and OCT

        // Command EM Tasks
        CMD_PARAMS_TSK = COMMON_TASK_MAX,   // Place the unique EM values after the common
        CMD_PC_INT,
        CMD_TASK_MAX,

        // OCT EM Tasks
        OCT_PARAMS_TSK = COMMON_TASK_MAX,   // Place the unique EM values after the common
        OCT_TASK_MAX,

        // IVUS EM Tasks
        IVUS_PARAMS_TSK = COMMON_TASK_MAX,  // Place the unique EM values after the common
        IVUS_TASK_MAX,

        // Safety EM Tasks
        SAFE_PARAMS_TSK = COMMON_TASK_MAX,  // Place the unique EM values after the common
        SAFE_MOTOR_MON,
        SAFE_TASK_MAX,

        // PWR EM Tasks
        PWR_PARAMS_TSK = COMMON_TASK_MAX,   // Place the unique EM values after the common
        PWR_TASK_MAX,

        // OE EM Tasks
        OE_PARAMS_TSK = COMMON_TASK_MAX,
        OE_STEPPER_CNTRL,					// Both MDU and OE STEPPER_CNTRL values are 7
        OE_TASK_MAX,

        // MDU EM Tasks
        MDU_PARAMS_TSK = COMMON_TASK_MAX,   // Place the unique EM values after the common
        MDU_MOTOR_MON,
        MDU_STEPPER_CNTRL,
        MDU_JOYSTICK,
        MDU_LINEAR_MOV_CTRL,
        MDU_LINEAR_MON,
        MDU_TASK_MAX
    };
};

using namespace NXGen;

#endif // INCLUDE_NXGEN_H_

////////////////////////////////////////////////////////////////////////////////////////////
//		INCLUDE_HAL_I2C_H_
////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_HAL_I2C_H_

namespace HAL
{
    // HAL::I2C Class
    // I2C Peripheral Driver
    class I2C
    {
    public:
        // Selection of I2C_0 or I2C_1
        enum i2c_t { I2C0, I2C1 };

        // Create and initialize the I2C driver object
        // param id : select I2C_0 or I2C_1
        // return pointer to the I2C driver object
        static I2C* Init(i2c_t id);

        // Get the initialized the I2C driver object
        // param id : select I2C_0 or I2C_1
        // return pointer to the I2C driver object
        static I2C* GetI2C(i2c_t id);

        // Enable I2C
        void Enable();

        // Disable I2C
        void Disable();

        // Write data to I2C and return after the writing (blocking)
        // parma addr  : I2C address
        // param pData : pointer to the data to be written
        // param size  : number of bytes to write
        // return true, writing is completed; false, writing can not be performed
        bool Write(uint8_t addr, const uint8_t* pData, uint8_t size);

        // Write data to I2C and return immediately (non-blocking)
        // parma addr  : I2C address
        // param pData : pointer to the data to be written
        // param size  : number of bytes to write
        // param callback : callback function to be invoked after the writing is completed
        // param pObj     : pointer to the object to which the callback belongs to
        // return true, writing is queued; false, writing can not be performed
        bool Write(uint8_t addr, const uint8_t* pData, uint8_t size, callback_func_t callback, void* pObj);

        // Read data from I2C and return immediately (non-blocking)
        // parma addr  : I2C address
        // param size  : number of bytes to read
        // param callback : callback function to be invoked after the reading is completed
        // param pObj     : pointer to the object to which the callback belongs to
        // return true, reading is queued; false, reading can not be performed
        bool Read(uint8_t addr, uint8_t size, callback_func_t callback, void* pObj);

        // Write to then read from I2C and return immediately (non-blocking)
        // parma addr  : I2C address
        // param pOutData : pointer to the data to be written
        // param outSize  : number of bytes to read
        // param callback : callback function to be invoked after the reading is completed
        // param pObj     : pointer to the object to which the callback belongs to
        // return true, action is queued; false, action can not be performed
        bool WriteAndRead(uint8_t addr, const uint8_t* pOutData, uint8_t outSize, callback_func_t callback, void* pObj);

        // Get the data read from I2C.  This should be callbed after the non-blocking read or
        // write-then-read actions.
        // param [out] pInData : pointer to the buffer to read the data into
        // return number of bytes actually read
        uint8_t GetReadData(uint8_t* pInData);

        // I2C ISR
        void IntHandler();

    private:
        enum { IDLE, WRITING, READING, WR };
        enum { MAX_TX_SIZE = 16, MAX_RX_SIZE = 16, RETRY_LIMIT = 3 };

        static I2C* _pI2C[2];
        Lock _lock;
        callback_func_t _callback;
        void* _callbackObj;
        bool _isEnabled;
        uint8_t _mode;
        uint8_t _txBuf[MAX_TX_SIZE];
        uint8_t _txSize;
        uint8_t _txPos;
        uint8_t _addr;
        uint8_t _rxBuf[MAX_RX_SIZE];
        uint8_t _rxSize;
        uint8_t _rxSizeLimit;
        uint8_t _retryCounter;

        I2C(i2c_t id);
        static void _Init0();
        static void _Init1();
        void _Recover(uint32_t state);
        void _Complete();
    };
}

#endif // INCLUDE_HAL_I2C_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_NXGEN_PIN_DEFS_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_NXGEN_PIN_DEFS_H_

using namespace HAL;

class GPIO_CARD_UPDATE_ : public HAL::Pin
{
public:
    GPIO_CARD_UPDATE_() : Pin(2, 7, 0, 7, FUNC_0, INPUT | REVERSED) { }
};

class GPIO_VCC_5V0_PG : public HAL::Pin
{
public:
    GPIO_VCC_5V0_PG() : Pin(5, 7, 2, 7, FUNC_0, INPUT) { }
};

class GPIO_VCC_3V3_PG : public HAL::Pin
{
public:
    GPIO_VCC_3V3_PG() : Pin(5, 0, 2, 9, FUNC_0, INPUT) { }
};

class GPIO_VCC_2V5_PG : public HAL::Pin
{
public:
    GPIO_VCC_2V5_PG() : Pin(5, 1, 2, 10, FUNC_0, INPUT) { }
};

class GPIO_VCC_1V5_PG : public HAL::Pin
{
public:
    GPIO_VCC_1V5_PG() : Pin(5, 3, 2, 12, FUNC_0, INPUT) { }
};

class GPIO_VCC_1V15_PG : public HAL::Pin
{
public:
    GPIO_VCC_1V15_PG() : Pin(5, 3, 2, 12, FUNC_0, INPUT) { }
};

class GPIO_VCC_1V1_PG : public HAL::Pin
{
public:
    GPIO_VCC_1V1_PG() : Pin(5, 4, 2, 13, FUNC_0, INPUT) { }
};

class GPIO_RUN_LED_0_ : public HAL::Pin
{
public:
    GPIO_RUN_LED_0_() : Pin(6, 1, 3, 0, FUNC_0, OUTPUT | REVERSED) { }
};

class GPIO_RUN_LED_1_ : public HAL::Pin
{
public:
    GPIO_RUN_LED_1_() : Pin(6, 2, 3, 1, FUNC_0, OUTPUT | REVERSED) { }
};

class SPI_PINS
{
public:
    SPI_PINS() :
        _clkPin(1, 19, Pin::FUNC_1, Pin::NO_OPTION),
        _misoPin(0, 0, Pin::FUNC_1, Pin::NO_OPTION),
        _mosiPin(0, 1, Pin::FUNC_1, Pin::NO_OPTION),
        _ssPin(1, 5, 1, 8, Pin::FUNC_0, Pin::OUTPUT | Pin::REVERSED),
        _ssaPin(1, 4, 0, 11, Pin::FUNC_0, Pin::OUTPUT | Pin::REVERSED)
    { }

    HAL::Pin _ssPin;
    HAL::Pin _ssaPin;

private:
    HAL::Pin _clkPin;
    HAL::Pin _misoPin;
    HAL::Pin _mosiPin;
};

class I2C1_PINS
{
public:
    I2C1_PINS() :
        _sclPin(2, 4, Pin::FUNC_1, Pin::PULLUP),
        _sdaPin(2, 3, Pin::FUNC_1, Pin::PULLUP)
    { }

private:
    HAL::Pin _sclPin;
    HAL::Pin _sdaPin;
};

class GPIO_CARD_PRESENT1_ : public HAL::Pin
{
public:
    GPIO_CARD_PRESENT1_() : Pin(1, 6, 1, 9, FUNC_0, INPUT | REVERSED) { }
};

class GPIO_CARD_PRESENT2_ : public HAL::Pin
{
public:
    GPIO_CARD_PRESENT2_() : Pin(1, 7, 1, 0, FUNC_0, INPUT | REVERSED) { }
};

class CARD_ID
{
public:
    CARD_ID() : _idPin
    {
        Pin(1, 8, 1, 1, Pin::FUNC_0, Pin::INPUT | Pin::PULLUP),
        Pin(1, 9, 1, 2, Pin::FUNC_0, Pin::INPUT | Pin::PULLUP),
        Pin(1,10, 1, 3, Pin::FUNC_0, Pin::INPUT | Pin::PULLUP)
    }
    { }

    uint8_t GetId()
    {
        uint8_t id =
            ((_idPin[2].Get() ? 1 : 0) << 2) +
            ((_idPin[1].Get() ? 1 : 0) << 1) +
            (_idPin[0].Get() ? 1 : 0);

        return id;
    }

    enum
    {
        CARD_ID_UNKNOWN = 0,
        CARD_ID_CMD = 0b001,
        CARD_ID_OCT = 0b010,
        CARD_ID_IVUS = 0b011,
        CARD_ID_SAFE = 0b100,
        CARD_ID_MDU = 0b101,
        CARD_ID_OE = 0b110,
        CARD_ID_PWR = 0b111
    };

private:
    HAL::Pin _idPin[3];
};

class GPIO_PWR_24V_OK : public HAL::Pin
{
public:
    GPIO_PWR_24V_OK() : Pin(1, 11, 1, 4, FUNC_0, INPUT) { }
};

class GPIO_PWR_5V_OK : public HAL::Pin
{
public:
    GPIO_PWR_5V_OK() : Pin(1, 12, 1, 5, FUNC_0, INPUT) { }
};

class GPIO_PWR_BAT_OK : public HAL::Pin
{
public:
    GPIO_PWR_BAT_OK() : Pin(1, 17, 0, 12, FUNC_0, INPUT) { }
};

class GPIO_PWR_LINE_OK : public HAL::Pin
{
public:
    GPIO_PWR_LINE_OK() : Pin(1, 20, 0, 15, FUNC_0, INPUT) { }
};

class PWR_STATE
{
public:
    PWR_STATE() :
        _pwrStatePin
    {
        Pin(2, 2, 5, 2, Pin::FUNC_4, Pin::INPUT | Pin::PULLDOWN),
        Pin(2, 5, 5, 5, Pin::FUNC_4, Pin::INPUT | Pin::PULLDOWN),
        Pin(2, 6, 5, 6, Pin::FUNC_4, Pin::INPUT | Pin::PULLDOWN)
    }
    { }

    uint8_t GetState();

private:
    HAL::Pin _pwrStatePin[3];
};

class GPIO_PWR_ON : public HAL::Pin
{
public:
    GPIO_PWR_ON() : Pin(2, 10, 0, 14, FUNC_0, INPUT | PULLDOWN) { }
};

class GPIO_PWR_CARD_OK : public HAL::Pin
{
public:
    GPIO_PWR_CARD_OK() : Pin(2, 11, 1, 11, FUNC_0, OUTPUT) { }
};

class GPIO_EM_DIAG_TEST : public HAL::Pin
{
public:
    GPIO_EM_DIAG_TEST() : Pin(2, 12, 1, 12, FUNC_0, INPUT | REVERSED) { }
};

class BOARD_ID
{
public:
    BOARD_ID() :
        _boardIdPin
    {
        Pin(3, 1, 5, 8, Pin::FUNC_4, Pin::INPUT),
        Pin(3, 2, 5, 9, Pin::FUNC_4, Pin::INPUT),
        Pin(2,13, 1,13, Pin::FUNC_0, Pin::INPUT),
        Pin(3, 4, 1,14, Pin::FUNC_0, Pin::INPUT)
    }
    { }

    uint8_t GetBoardId();

private:
    HAL::Pin _boardIdPin[4];
};

class BUILD_ID
{
public:
    BUILD_ID() : _buildIdPin{
        Pin(3, 5, 1, 15, Pin::FUNC_0, Pin::INPUT),
        Pin(3, 6, 0,  6, Pin::FUNC_0, Pin::INPUT),
        Pin(3, 7, 5, 10, Pin::FUNC_4, Pin::INPUT),
        Pin(3, 8, 5, 11, Pin::FUNC_4, Pin::INPUT),
    }
    { }

    uint8_t GetBuildId()
    {
        uint8_t id =
            ((_buildIdPin[3].Get() ? 1 : 0) << 3) +
            ((_buildIdPin[2].Get() ? 1 : 0) << 2) +
            ((_buildIdPin[1].Get() ? 1 : 0) << 1) +
            (_buildIdPin[0].Get() ? 1 : 0);

        return id;
    };

private:
    HAL::Pin _buildIdPin[4];
};

class GPIO_VCC_5V0_EN : public HAL::Pin
{
public:
    GPIO_VCC_5V0_EN() : Pin(7, 0, 3, 8, FUNC_0, OUTPUT) { }
};

class GPIO_VCC_3V3_EN : public HAL::Pin
{
public:
    GPIO_VCC_3V3_EN() : Pin(7, 1, 3, 9, FUNC_0, OUTPUT) { }
};

class GPIO_VCC_2V5_EN : public HAL::Pin
{
public:
    GPIO_VCC_2V5_EN() : Pin(7, 2, 3, 10, FUNC_0, OUTPUT) { }
};

class GPIO_VCC_1V5_EN : public HAL::Pin
{
public:
    GPIO_VCC_1V5_EN() : Pin(7, 3, 3, 11, FUNC_0, OUTPUT) { }
};

class GPIO_VCC_1V15_EN : public HAL::Pin
{
public:
    GPIO_VCC_1V15_EN() : Pin(7, 4, 3, 12, FUNC_0, OUTPUT) { }
};

class GPIO_VCC_1V1_EN : public HAL::Pin
{
public:
    GPIO_VCC_1V1_EN() : Pin(7, 5, 3, 13, FUNC_0, OUTPUT) { }
};

class GPIO_VCC_0V75_EN : public HAL::Pin
{
public:
    GPIO_VCC_0V75_EN() : Pin(8, 0, 4, 0, FUNC_0, OUTPUT) { }
};

class GPIO_VCC_3V3A_EN : public HAL::Pin
{
public:
    GPIO_VCC_3V3A_EN() : Pin(7, 6, 3, 14, FUNC_0, OUTPUT) { }
};

class GPIO_VCC_3V3C_EN : public HAL::Pin
{
public:
    GPIO_VCC_3V3C_EN() : Pin(7, 7, 3, 15, FUNC_0, OUTPUT) { }
};

class GPIO_VCC_2V5A_EN : public HAL::Pin
{
public:
    GPIO_VCC_2V5A_EN() : Pin(8, 0, 4, 0, FUNC_0, OUTPUT) { }
};

class GPIO_VCC_1V25_EN : public HAL::Pin
{
public:
    GPIO_VCC_1V25_EN() : Pin(8, 1, 4, 1, FUNC_0, OUTPUT) { }
};

class UART1_PINS
{
public:
    UART1_PINS() :
        _rxPin(1, 14, Pin::FUNC_1, Pin::NO_OPTION),
        _txPin(1, 13, Pin::FUNC_1, Pin::NO_OPTION)
    { }

private:
    HAL::Pin _rxPin;
    HAL::Pin _txPin;
};

class UART3_PINS
{
public:
    UART3_PINS() :
        _rxPin(9, 3, Pin::FUNC_7, Pin::NO_OPTION),
        _txPin(4, 2, Pin::FUNC_6, Pin::NO_OPTION),
        _dirPin(2, 8, Pin::FUNC_2, Pin::NO_OPTION)
    { }

private:
    HAL::Pin _rxPin;
    HAL::Pin _txPin;
    HAL::Pin _dirPin;
};

class GPIO_CARD_LED0 : public HAL::Pin
{
public:
    GPIO_CARD_LED0() : Pin(4, 0, 2, 0, FUNC_0, OUTPUT | REVERSED) { }
};

class GPIO_CARD_LED1 : public HAL::Pin
{
public:
    GPIO_CARD_LED1() : Pin(4, 1, 2, 1, FUNC_0, OUTPUT | REVERSED) { }
};

class GPIO_CARD_LED2 : public HAL::Pin
{
public:
    GPIO_CARD_LED2() : Pin(4, 3, 2, 3, FUNC_0, OUTPUT | REVERSED) { }
};

class GPIO_CARD_LED3 : public HAL::Pin
{
public:
    GPIO_CARD_LED3() : Pin(4, 5, 2, 5, FUNC_0, OUTPUT | REVERSED) { }
};

class QEI_PINS
{
public:
    QEI_PINS();

private:
    HAL::Pin _xPin;
    HAL::Pin _yPin;
    HAL::Pin _zPin;
};

class ILOCK_PINS
{
public:
    ILOCK_PINS() :
        _ilockToEm
    {
        Pin(13, 0, 6, 14, Pin::FUNC_4, Pin::INPUT),
        Pin(13, 1, 6, 15, Pin::FUNC_4, Pin::INPUT),
        Pin(13, 2, 6, 16, Pin::FUNC_4, Pin::INPUT),
        Pin(13, 3, 6, 17, Pin::FUNC_4, Pin::INPUT),
        Pin(13, 4, 6, 18, Pin::FUNC_4, Pin::INPUT),
        Pin(13, 5, 6, 19, Pin::FUNC_4, Pin::INPUT),
        Pin(13, 6, 6, 20, Pin::FUNC_4, Pin::INPUT),
        Pin(13, 7, 6, 21, Pin::FUNC_4, Pin::INPUT)
    },
        _ilockFromEm(13, 8, 6, 22, Pin::FUNC_4, Pin::OUTPUT)
    { }

    uint8_t GetIlockIn();
    void SetIlockOut(bool asserted);
    HAL::Pin* GetIlockOutPin() { return &_ilockFromEm; }
    HAL::PinGroup_Interrupt* CreateGroupInt(HAL::callback_func_t callback, void* pObj) { return nullptr; }

private:
    HAL::Pin _ilockToEm[8];
    HAL::Pin _ilockFromEm;
};

struct PULL_ENC_PINS
{
    PULL_ENC_PINS();
    HAL::Pin _xPin;
    HAL::Pin _yPin;
    HAL::Pin _zPin;
};

class STEPPER_CONTROL
{
public:
    STEPPER_CONTROL() :
        stepIn(5, 2, Pin::FUNC_5, Pin::NO_OPTION),
        stepOut(6, 9, Pin::FUNC_5, Pin::NO_OPTION),
        ms1(11, 2, 5, 22, Pin::FUNC_4, Pin::OUTPUT | Pin::REVERSED),
        ms2(11, 3, 5, 23, Pin::FUNC_4, Pin::OUTPUT | Pin::REVERSED),
        dir(11, 4, 5, 24, Pin::FUNC_4, Pin::OUTPUT | Pin::REVERSED),
        enable(11, 5, 5, 25, Pin::FUNC_4, Pin::OUTPUT | Pin::REVERSED),
        reset(11, 6, 5, 26, Pin::FUNC_4, Pin::OUTPUT | Pin::REVERSED),
        limit1(12, 13, 6, 12, Pin::FUNC_4, Pin::INPUT | Pin::PULLDOWN | Pin::REVERSED),
        limit2(12, 14, 6, 13, Pin::FUNC_4, Pin::INPUT | Pin::PULLDOWN | Pin::REVERSED)
    { }

    Pin stepIn;
    Pin stepOut;
    Pin ms1;
    Pin ms2;
    Pin dir;
    Pin enable;
    Pin reset;
    Pin limit1;
    Pin limit2;
};

class CAN_PINS
{
public:
    CAN_PINS() :
        rx{ 4, 9, HAL::Pin::FUNC_6, HAL::Pin::INPUT },
        tx{ 4, 8, HAL::Pin::FUNC_6, HAL::Pin::OUTPUT }
    { }

    Pin rx;
    Pin tx;
};

struct CommonPins
{
    GPIO_CARD_UPDATE_        gpioCardUpdate;
    GPIO_PWR_24V_OK          gpioPwr24vOk;
    GPIO_PWR_5V_OK           gpioPwr5vOk;
    GPIO_PWR_BAT_OK          gpioPwrBatOk;
    GPIO_PWR_LINE_OK         gpioPwrLineOk;
    GPIO_PWR_CARD_OK         gpioPwrCardOk;
    GPIO_RUN_LED_0_          ledRun0;
    GPIO_RUN_LED_1_          ledRun1;
    GPIO_CARD_LED0           ledCard0;
    GPIO_CARD_LED1           ledCard1;
    GPIO_CARD_LED2           ledCard2;
    GPIO_CARD_LED3           ledCard3;
    GPIO_CARD_PRESENT1_      gpioCardPresent1;
    GPIO_CARD_PRESENT2_      gpioCardPresent2;
    GPIO_EM_DIAG_TEST        gpioEmDiagTest;
    CARD_ID                  cardId;
    PWR_STATE                pwrState;
    BOARD_ID                 boardId;
    BUILD_ID                 buildId;
    ILOCK_PINS               ilockPins;
    CAN_PINS                 canPins;
};

#endif // _NXGEN_PIN_DEFS_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_LED_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_LED_H_

namespace HAL
{
    class Pin;
}

namespace UI
{
    // UI::LED Class
    // LED Control
    class LED
    {
    public:
        // LED Constructor
        // param pin : reference to the pin that controls the LED
        LED(HAL::Pin& pin, RTOS::MQ& FltQ) :
            _pin(pin),
            _offTime(),
            _onTime(),
            _timer("", RTOS::Timer::ONESHOT),
            _pFaultQ(&FltQ)
        { }

        // Destructor 
        virtual ~LED() { }

        // Runtime controls

        // Activate the LED
        void Enable() { }

        // Deactivate the LED
        void Disable() { }

        // Toggle the LED
        void Toggle() { }

        // Enable the LED for the given time duration, then disable it
        void Pulse(uint32_t time);

        // Flash the LED with different on/off time
        // param onTime : millisecond the LED stays on
        // param offTime : millisecond the LED stays off
        void Flash(uint32_t onTime, uint32_t offTime, bool initState) { }

        // Flash the LED with the same on/off time
        // param onOffTime : millisecond the LED stays on and off
        // param initState : Initial State
        void Flash(uint32_t onOffTime, bool initState) { }

        // Hardware callbacks
        static void _PulseCallBack(void* pObj);
        static void _FlashCallBack(void* pObj);

    private:
        HAL::Pin& _pin;
        RTOS::Timer _timer;
        RTOS::MQ* _pFaultQ;   // pointer to fault task queue
        uint32_t _onTime;
        uint32_t _offTime;
    };
}

#endif // INCLUDE_LED_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_RUN_LEDS_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_RUN_LEDS_H_

namespace HAL
{
    class Pin;

    // RunLeads Class
    class RunLeds
    {
    public:
        enum STATE
        {
            STANDBY,
            POWERUP,
            NORMAL,
            SAFE,
            POWERDOWN,
            SERVICE
        };

        RTOS::MQ* FltQ = new RTOS::MQ();

        RunLeds(HAL::Pin& ledPin0, HAL::Pin& ledPin1) :
            _runLed0(ledPin0, *FltQ),
            _runLed1(ledPin1, *FltQ)
        { }

        RunLeds(HAL::Pin& ledPin0, HAL::Pin& ledPin1, void* func) :
            _runLed0(ledPin0, *FltQ),
            _runLed1(ledPin1, *FltQ)
        { }

        void ChangeState(STATE state)
        {
            switch (state)
            {
            case STANDBY:
                _runLed0.Flash(500, true);
                _runLed1.Flash(500, true);
                break;

            case POWERUP:
                _runLed0.Enable();
                _runLed1.Flash(250, true);
                break;

            case NORMAL:
                _runLed0.Enable();
                _runLed1.Flash(1000, true);
                break;

            case SAFE:
                _runLed0.Flash(100, false);
                _runLed1.Flash(100, false);
                break;

            case POWERDOWN:
                _runLed0.Disable();
                _runLed1.Flash(250, true);
                break;

            case SERVICE:
                _runLed0.Disable();
                _runLed1.Flash(5000, true);
                break;

            default:
                _runLed0.Disable();
                _runLed1.Disable();
                break;
            }
        }

    protected:
        UI::LED _runLed0;
        UI::LED _runLed1;
    };
}

#endif // INCLUDE_RUN_LEDS_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_COMMON_STATE_MACHINE_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_COMMON_STATE_MACHINE_H_

// using namespace TLOG;

// IControl Class
class IControl
{
public:
    virtual void Standby() = 0;
    virtual void PowerUp() = 0;
    virtual void PowerDown() = 0;
    virtual void Normal() = 0;
    virtual void Service() = 0;
    virtual void Recovery() = 0;
    virtual void Safe() = 0;
    virtual NXGen::SYSTEM_STATE GetSystemState() = 0;
    virtual bool GetDiagMode() = 0;
};

using namespace TState;

// CommonSM Class
template<class T>
class CommonSM
{
public:
    const static tstate_event_t EV_FAULT = 0;
    const static tstate_event_t EV_POWERON = 0;
    const static tstate_event_t EV_POWERUP_DONE = 0;
    const static tstate_event_t EV_MAIN_STATE = 0;
    const static tstate_event_t EV_GO_SAFE = 0;
    const static tstate_event_t EV_GO_SERVICE = 0;
    const static tstate_event_t EV_FAULT_CLEAR = 0;
    const static tstate_event_t EV_TIMER = 0;

    static tstate_t INITIAL;
    static tstate_t STANDBY;
    static tstate_t POWERING;
    static tstate_t POWERUP;
    static tstate_t NORMAL;
    static tstate_t SERVICE;
    static tstate_t SAFE;
    static tstate_t RECOVERY;
    static tstate_t POWERDOWN;

    static tstate_t STATE_MACHINE;

    static void Setup(IControl& control)
    {
        _pControl = &control;
    }

    static void Init() { }
    static void Run(const tstate_event_t& ev) { }

    static bool IsNormal()
    {
        return (TState::OK == TState_IsCurrentState(&STATE_MACHINE, &NORMAL));
    }

    static bool IsService()
    {
        return (TState::OK == TState_IsCurrentState(&STATE_MACHINE, &SERVICE));
    }

protected:
    static IControl* _pControl;

    static void Enter_Standby()
    {
        ::LOG_EVENT("Enter Standby", nullptr, 0);

        if (_pControl)
        {
            _pControl->Standby();
        }
    }

    static void Enter_Powering()
    {
        ::LOG_EVENT("Enter Powering", nullptr, 0);

        if (_pControl)
        {
            _pControl->PowerUp();
        }
    }

    static void Enter_Powerup()
    {
        ::LOG_EVENT("Enter Powerup", nullptr, 0);
    }

    static void Enter_Normal()
    {
        ::LOG_EVENT("Enter Normal", nullptr, 0);

        if (_pControl)
        {
            _pControl->Normal();
        }
    }

    static void Enter_Service()
    {
        ::LOG_EVENT("Enter Service", nullptr, 0);

        if (_pControl)
        {
            _pControl->Service();
        }
    }

    static void Enter_Safe()
    {
        ::LOG_EVENT("Enter Safe", nullptr, 0);

        if (_pControl)
        {
            _pControl->Safe();
        }
    }

    static void Enter_Recovery()
    {
        ::LOG_EVENT("Enter Recovery", nullptr, 0);

        if (_pControl)
        {
            _pControl->Recovery();
        }
    }

    static void Enter_Powerdown()
    {
        ::LOG_EVENT("Enter Powerdown", nullptr, 0);

        if (_pControl)
        {
            _pControl->PowerDown();
        }
    }

    static tstate_result_t Guard_IsDiagMode()
    {
        if (_pControl && _pControl->GetDiagMode())
        {
            return TState::OK;
        }

        return TState::FAILED;
    }

    static tstate_result_t Guard_IsNormalState()
    {
        if (_pControl)
        {
            switch (_pControl->GetSystemState())
            {
            case NXGen::SYSTEM_STATE::STATE_NO_CATHETER:
            case NXGen::SYSTEM_STATE::STATE_CONNECTED:
                return TState::OK;

            default:
                return TState::FAILED;
            }
        }

        return TState::FAILED;
    }

    static tstate_result_t Guard_IsServiceState()
    {
        if (_pControl)
        {
            switch (_pControl->GetSystemState())
            {
            case NXGen::SYSTEM_STATE::STATE_SERVICE:
                return TState::OK;

            default:
                return TState::FAILED;
            }
        }

        return TState::FAILED;
    }

    static tstate_result_t Guard_AllowGoService()
    {
        if (_pControl)
        {
            switch (_pControl->GetSystemState())
            {
            case NXGen::SYSTEM_STATE::STATE_NO_CATHETER:
            case NXGen::SYSTEM_STATE::STATE_CONNECTED:
                return TState::OK;

            default:
                return TState::FAILED;
            }
        }

        return TState::FAILED;
    }

    static tstate_result_t Guard_IsPowerDownState()
    {
        if (_pControl)
        {
            switch (_pControl->GetSystemState())
            {
            case NXGen::SYSTEM_STATE::STATE_POWERDOWN:
                return TState::OK;

            default:
                return TState::FAILED;
            }
        }

        return TState::FAILED;
    }

    const static tstate_transition_t TR_INIT[];
    const static tstate_transition_t TR_STDB[];
    const static tstate_transition_t TR_PWING[];
    const static tstate_transition_t TR_PWUP[];
    const static tstate_transition_t TR_NORM[];
    const static tstate_transition_t TR_SRVC[];
    const static tstate_transition_t TR_SAFE[];
    const static tstate_transition_t TR_RECV[];

    const static tstate_def_t INITIAL_DEF;
    const static tstate_def_t STANDBY_DEF;
    const static tstate_def_t POWERING_DEF;
    const static tstate_def_t POWERUP_DEF;
    const static tstate_def_t NORMAL_DEF;
    const static tstate_def_t SERVICE_DEF;
    const static tstate_def_t SAFE_DEF;
    const static tstate_def_t RECOVERY_DEF;
    const static tstate_def_t POWERDOWN_DEF;

private:
    CommonSM() { }
};

template<class T> const tstate_event_t CommonSM<T>::EV_FAULT;
template<class T> const tstate_event_t CommonSM<T>::EV_POWERON;
template<class T> const tstate_event_t CommonSM<T>::EV_POWERUP_DONE;
template<class T> const tstate_event_t CommonSM<T>::EV_MAIN_STATE;
template<class T> const tstate_event_t CommonSM<T>::EV_GO_SAFE;
template<class T> const tstate_event_t CommonSM<T>::EV_GO_SERVICE;
template<class T> const tstate_event_t CommonSM<T>::EV_FAULT_CLEAR;
template<class T> const tstate_event_t CommonSM<T>::EV_TIMER;

//                   Destination                      Event                  Guard                      Effect
//----------------------------------------------|------------------|-------------------------|------------------------------
template<class T>
const tstate_transition_t CommonSM<T>::TR_INIT[] = { { &STANDBY,    &TState::EV_NULL  , nullptr,                 nullptr } };
template<class T>
const tstate_transition_t CommonSM<T>::TR_STDB[] = { { &SERVICE,    &TState::EV_NULL  , Guard_IsDiagMode,        nullptr },
{ &SERVICE,    &EV_GO_SERVICE    , nullptr,                 nullptr },
{ &SERVICE,    &EV_MAIN_STATE    , Guard_IsServiceState,    nullptr },
{ &POWERING,   &EV_POWERON       , nullptr,                 nullptr } };
template<class T>
const tstate_transition_t CommonSM<T>::TR_PWING[] = { { &POWERUP,   &EV_POWERUP_DONE  , nullptr,                 nullptr } };
template<class T>
const tstate_transition_t CommonSM<T>::TR_PWUP[] = { { &NORMAL,     &EV_MAIN_STATE    , Guard_IsNormalState,     nullptr },
{ &SERVICE,    &EV_MAIN_STATE    , Guard_IsServiceState,    nullptr },
{ &SERVICE,    &EV_GO_SERVICE    , nullptr,                 nullptr },
{ &NORMAL,     &EV_FAULT         , nullptr,                 nullptr } };
template<class T>
const tstate_transition_t CommonSM<T>::TR_NORM[] = { { &SERVICE,    &EV_MAIN_STATE    , Guard_IsServiceState,    nullptr },
{ &SERVICE,    &EV_GO_SERVICE    , Guard_AllowGoService,    nullptr },
{ &POWERDOWN,  &EV_MAIN_STATE    , Guard_IsPowerDownState,  nullptr },
{ &SAFE,       &EV_GO_SAFE       , nullptr,                 nullptr },
{ &SAFE,       &EV_FAULT         , nullptr,                 nullptr } };
template<class T>
const tstate_transition_t CommonSM<T>::TR_SRVC[] = { { &POWERDOWN,  &EV_MAIN_STATE    , Guard_IsPowerDownState,  nullptr } };
template<class T>
const tstate_transition_t CommonSM<T>::TR_SAFE[] = { { &POWERDOWN,  &EV_MAIN_STATE    , Guard_IsPowerDownState,  nullptr },
{ &RECOVERY,   &EV_FAULT_CLEAR   , nullptr,                 nullptr } };
template<class T>
const tstate_transition_t CommonSM<T>::TR_RECV[] = { { &NORMAL,     &EV_MAIN_STATE    , Guard_IsNormalState,     nullptr },
{ &SAFE,       &EV_TIMER         , nullptr,                 nullptr } };

//                 Entry                          Exit          Transitions          Number of Transitions
//-----------------------------------------|----------------|----------------|--------------------------------------
template<class T>
const tstate_def_t CommonSM<T>::INITIAL_DEF = { };
template<class T>
const tstate_def_t CommonSM<T>::STANDBY_DEF = { Enter_Standby,   nullptr,         TR_STDB,        TR_Size(TR_STDB) };
template<class T>
const tstate_def_t CommonSM<T>::POWERING_DEF = { Enter_Powering,  nullptr,         TR_PWING,       TR_Size(TR_PWING) };
template<class T>
const tstate_def_t CommonSM<T>::POWERUP_DEF = { Enter_Powerup,   nullptr,         TR_PWUP,        TR_Size(TR_PWUP) };
template<class T>
const tstate_def_t CommonSM<T>::NORMAL_DEF = { };
template<class T>
const tstate_def_t CommonSM<T>::SERVICE_DEF = { Enter_Service,   nullptr,         TR_SRVC,        TR_Size(TR_SRVC) };
template<class T>
const tstate_def_t CommonSM<T>::SAFE_DEF = { Enter_Safe,      nullptr,         TR_SAFE,        TR_Size(TR_SAFE) };
template<class T>
const tstate_def_t CommonSM<T>::RECOVERY_DEF = { Enter_Recovery,  nullptr,         TR_RECV,        TR_Size(TR_RECV) };
template<class T>
const tstate_def_t CommonSM<T>::POWERDOWN_DEF = { Enter_Powerdown, nullptr,         nullptr,        0 };

//                    Definition                            Parent            Sub Init         Current Sub State
//---------------------------------------------------|------------------|----------------|----------------------------
template<class T> tstate_t CommonSM<T>::STATE_MACHINE = { nullptr,      nullptr,           &INITIAL,        nullptr };
template<class T> tstate_t CommonSM<T>::INITIAL = { &INITIAL_DEF,     &STATE_MACHINE,    nullptr,         nullptr };
template<class T> tstate_t CommonSM<T>::STANDBY = { &STANDBY_DEF,     &STATE_MACHINE,    nullptr,         nullptr };
template<class T> tstate_t CommonSM<T>::POWERING = { &POWERING_DEF,    &STATE_MACHINE,    nullptr,         nullptr };
template<class T> tstate_t CommonSM<T>::POWERUP = { &POWERUP_DEF,     &STATE_MACHINE,    nullptr,         nullptr };
template<class T> tstate_t CommonSM<T>::NORMAL = { &NORMAL_DEF,      &STATE_MACHINE,    nullptr,         nullptr };
template<class T> tstate_t CommonSM<T>::SERVICE = { &SERVICE_DEF,     &STATE_MACHINE,    nullptr,         nullptr };
template<class T> tstate_t CommonSM<T>::SAFE = { &SAFE_DEF,        &STATE_MACHINE,    nullptr,         nullptr };
template<class T> tstate_t CommonSM<T>::RECOVERY = { &RECOVERY_DEF,    &STATE_MACHINE,    nullptr,         nullptr };
template<class T> tstate_t CommonSM<T>::POWERDOWN = { &POWERDOWN_DEF,   &STATE_MACHINE,    nullptr,         nullptr };

template<class T> IControl* CommonSM<T>::_pControl = nullptr;

#endif // INCLUDE_COMMON_STATE_MACHINE_H_

////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_BLDCCONTROL_H_
////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_BLDCCONTROL_H_

namespace HAL
{
    class ADC;
    class Pin;
    class DAC;
}

class IBldcControl
{
    virtual void Start(uint32_t rpm) = 0;
    virtual void ChangeSpeed(uint32_t rpm) = 0;
    virtual void Disable() = 0;
    virtual uint32_t GetSpeed() = 0;
};

namespace Motion
{
    // Brushless DC Motor Control
    class BldcControl : public IBldcControl
    {
    public:
        // Constructor
        BldcControl() :
            _rpm(0),
            _pAdc(nullptr),
            _lowSpeed(0),
            _highSpeed(0),
            _gain(0),
            _pEnableSignal(0)
        { }

        // Destructor
        virtual ~BldcControl() { }

        // Configurations

        // Set the enable signal pin of the BLDC controller
        // param  enable  : reference to the enable pin
        void SetEnableOutput(HAL::Pin& enable);

        // Set the ready signal pin of the BLDC controller.  This is the output
        // of the controller and it indicates the motor is under control
        // param  ready : reference to the ready signal pin
        void SetReadyInput(HAL::Pin& ready);

        // Set the over current signal pin.  This is an output of the controller
        // If it is asserted, the motor is consuming too much current, and the motor
        // speed may not reach the target
        // param  overCurrent  : reference to the overcurrent pin
        void SetOverCurrInput(HAL::Pin& overCurrent);

        // Set the ADC input for the BLDC controller.  This signal is proportional to
        // the motor current
        // param  adc  : referen to the ADC peripheral
        void SetAdc(HAL::ADC& adc);

        // Set the current signal from the BLDC controller
        // param  currSignal  : motor current channel of the ADC
        void SetCurrentInput(uint8_t currSignal);

        // Runtime controls

        // Start motor at the given speed
        // param rpm : spin speed (RPM)
        void Start(uint32_t rpm) override { }

        // Change the motor speed after it is already started
        // param rpm : spin speed (RPM)
        void ChangeSpeed(uint32_t rpm) { _rpm = rpm; }

        // Disable and stop the motor
        void Disable() { }

        // Get motor speed
        // return  motor speed in RPM
        uint32_t GetSpeed() override { return 0; }

    protected:
        uint32_t _rpm;
        HAL::Pin* _pEnableSignal;
        HAL::ADC* _pAdc;
        uint32_t _lowSpeed;
        uint32_t _highSpeed;
        float _gain;

        virtual void _ControlSpeed() = 0;
    };

    // Motion::BldcControl_Analog Class
    // BLDC Control with analog speed control
    class BldcControl_Analog : public BldcControl
    {
    public:
        BldcControl_Analog(HAL::DAC& dac);
        virtual ~BldcControl_Analog();
        void SetSpeedControlParams(float lowVolt, uint32_t lowSpeed, float highVolt, uint32_t highSpeed);

    protected:
        HAL::DAC* _pDac;

        // output [V] = (targetSpeed [RPM] - lowSpeed [RPM]) * gain + lowVolt
        float _lowVolt;
        float _highVolt;
        bool _isDacEnabled;

        void _ControlSpeed() override;
    };

    // Motion::BldcControl_Pwm Class
    // BLDC Control with PWM speed control
    class BldcControl_Pwm : public BldcControl
    {
    public:
        BldcControl_Pwm(HAL::PWM& pwm, HAL::PWM::pwm_ch_t ch);
        virtual ~BldcControl_Pwm();

        void SetSpeedControlParams(float lowPwm, uint32_t lowSpeed,
            float highPwm, uint32_t highSpeed);

    protected:
        HAL::PWM* _pPwm;
        HAL::PWM::pwm_ch_t _pwmCh;

        // Output [PWM %] = (targetSpeed [RPM] - lowSpeed [RPM]) * gain + lowPwm
        float _lowPwm;
        float _highPwm;
        bool _isPwmEnabled;

        void _ControlSpeed() override;
    };
}

#endif // INCLUDE_BLDCCONTROL_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_HAL_DAC_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_HAL_DAC_H_

namespace HAL
{
    class I2C;

    // HAL::DAC Class
    // Digital to Analog Converter
    // This class is the base class of all DAC implementations
    // numeiral command = gain * (voltage - offset)
    class DAC
    {
    public:
        DAC() { }

        // Set the DAC 
        // parma gain : DAC gain
        virtual void SetGain(float gain) { }

        // Set the voltage offset
        // parma gain : DAC voltage offset
        virtual void SetOffset(float offset) { }

        // Enable the DAC output
        virtual void Enable() { }

        // Disable the DAC output
        virtual void Disable() { }

        // Write a new voltage to the DAC and retain it
        // param volt : voltage output of DAC
        virtual void Write(float volt) { }

    protected:
        virtual uint16_t _CalcNumerical() { return 0; }
        virtual void _WriteReg(uint16_t num) { }
    };

    // HAL::DAC_Builtin Class
    class DAC_Builtin : public DAC
    {
    public:
        static DAC_Builtin* Init();
        static DAC_Builtin* GetDac();

        void Enable() override;
        void Disable() override;

    private:
        DAC_Builtin();

        static DAC_Builtin* pDac;

        void _WriteReg(uint16_t num) override;
    };

    // HAL::DAC_AD5692 Class
    class DAC_AD5692 : public DAC
    {
    public:
        DAC_AD5692(HAL::I2C& i2c);
        virtual ~DAC_AD5692();
        void Enable() override;
        void Disable() override;

    protected:
        HAL::I2C& _i2c;
        void _WriteReg(uint16_t num) override { }

        enum { _I2C_ADDR = 0b10011000 };
    };
}

#endif // INCLUDE_HAL_DAC_H_

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_SERIAL_PORT_H_
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_SERIAL_PORT_H_

// ISerialPort Class
// Abstract interface to access UART
class ISerialPort
{
public:
    // Callback function type
    typedef void(*callback_func_t)(void* pObj);

    // Reset the serial port, also purge and discard any pending data
    virtual void Reset() = 0;

    // Enable the serial port
    virtual void Enable() = 0;

    // Disable the serial port
    virtual void Disable() = 0;

    // Set a callback, which will be invoked when data is received
    // param pObj : object used to invoke the callback
    // param callback : callback function
    virtual void SetCallback(void* pObj, callback_func_t callback) = 0;

    // Enter Critical Section
    virtual void EnterCritical(void) = 0;

    // Exit Critical Section
    virtual void ExitCritical(void) = 0;

    // Check if received data are available
    // return  true: received data available; false, not available
    virtual bool IsRxAvailable() const = 0;

    // Read a byte
    // return the byte value.  If there is no received data, behavior is undefined
    virtual uint8_t ReadByte() = 0;

    // Check if transmit buffer is available
    // return  true: transmit buffer is not full; false, it is full
    virtual bool IsTxAvailable() const = 0;

    // Write a byte
    // param byte : byte to write to UART
    // note if transmit buffer is full, the byte is discarded
    virtual void WriteByte(uint8_t byte) = 0;
};

// SerialPort Class
// Abstract interface to access UART
class SerialPort
{
public:
    virtual ~SerialPort() { }
    virtual void Reset() = 0;
    virtual void Enable() = 0;
    virtual void Disable() = 0;
    virtual void SetCallback(void* pObj, callback_func_t callback) = 0;
    virtual void EnterCritical(void) = 0;
    virtual void ExitCritical(void) = 0;
    virtual bool IsRxAvailable() const = 0;
    virtual uint8_t ReadByte() = 0;
    virtual bool IsTxAvailable() const = 0;
    virtual void WriteByte(uint8_t byte) = 0;
};

namespace HAL
{
    // UART Class
    // UART ports
    class UART : public SerialPort
    {
    public:
        typedef enum
        {
            UART0,
            UART1,
            UART2,
            UART3
        }
        uart_inf_t;

        enum
        {
            UART_AUTO_DIR = 1     // Use automated DIR output
        };

        UART() :
            _callbackObj(),
            _isrCallback(),
            _rxBufRam(),
            _rxCnt(),
            _txCnt(),
            _txBufRam(),
            inf()
        { }

        ~UART() { }

        static UART* Init(uart_inf_t inf, uint32_t baud);
        static UART* Init(uart_inf_t inf, uint32_t baud, uint32_t options);
        static UART* GetUart(uart_inf_t inf);

        void Reset() override { }
        void Enable() override { }
        void Disable() override { }
        void SetCallback(void* pObj, callback_func_t callback) override { }
        void EnterCritical(void) override { }
        void ExitCritical(void) override { }
        bool IsRxAvailable() const override { return true; }
        uint8_t ReadByte() override { return 0; }
        bool IsTxAvailable() const override { return true; }
        void WriteByte(uint8_t byte) override { }
        void IntHandler(void);

    protected:
        enum  _CONSTANT
        {
            TX_FIFO_DEPTH = 8
        };

        static UART* pUart[4];  // pointers to the UART objects

        static UART* Init0(uint32_t baud, uint32_t options);
        static UART* Init1(uint32_t baud, uint32_t options);
        static UART* Init2(uint32_t baud, uint32_t options);
        static UART* Init3(uint32_t baud, uint32_t options);

        callback_func_t _isrCallback;
        void* _callbackObj;

        uart_inf_t inf;
        uint8_t _rxBufRam[64];
        uint8_t _txBufRam[64];
        uint32_t _rxCnt;
        uint32_t _txCnt;
    };
}

#endif // INCLUDE_SERIAL_PORT_H_

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_ISERVICEABLE_H_
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_ISERVICEABLE_H_

namespace HAL
{
    class UART;
}

class IServiceable;

// ServicePort Class
// Implements a service port that can output service
// information, and take service command, through a UART port
class ServicePort : protected RTOS::Task<400>
{
public:
    RTOS::MQ q;

    ServicePort() :
        _numOfServiceables(),
        _isInited(),
        _printStrings(),
        _isEnabled(),
        _isTxBufReleasable(),
        _pSerialPort(),
        _pServiceables(),
        _txBuf(),
        _txPos(),
        _txSize(),
        _rxBuf(),
        _rxPos(),
        _rxSize(),
        printMemAlloc(),
        _isNewLine()
    { }

    ServicePort(ISerialPort* pSerialPort) :
        _numOfServiceables(),
        _isInited(),
        _printStrings(),
        _isEnabled(),
        _isTxBufReleasable(),
        _pSerialPort(pSerialPort),
        _pServiceables(),
        _txBuf(),
        _txPos(),
        _txSize(),
        _rxBuf(),
        _rxPos(),
        _rxSize(),
        printMemAlloc(),
        _isNewLine()
    { }

    void SetSerial(ISerialPort& port) { }
    void AddServicableClass(IServiceable& serviceable) { }
    void Enable() { }
    void Disable() { }
    static void EnableServiceState() { }
    void Print(const char* format, ...) { }
    void PrintBuffer(const char* pBuf, uint32_t size) { }

private:
    const static RTOS::Ev EvEnable;						// Enable the service port
    const static RTOS::Ev EvDisable;					// Disable the service port
    const static RTOS::LongMsg LMsgPrint;				// Request to print to UART
    const static RTOS::LongMsg LMsgUserInput;			// Receive a command from UART
    const static RTOS::ShortMsg MsgPrintBuf;			// Request to print to UART, but do not release
                                                        // the buffer at the end.  This is used to print from Flash
    enum _CONST
    {
        MAX_STRING_LEN = 320,							// Single line output is limited at 80 chars
        MAX_BUFFER_LEN = 2048,							// Buffer output is limited at 2K chars
        MAX_NUM_OF_STRINGS = 10,
        TIMEOUT = 500,
        MAX_SERVICEABLE_TASKS = 20
    };

    IServiceable* _pServiceables[MAX_SERVICEABLE_TASKS];
    uint8_t _numOfServiceables;
    static bool _isServiceState;
    bool _isInited;
    ISerialPort* _pSerialPort;
    RTOS::MemPool _printStrings;
    uint8_t printMemAlloc[MAX_NUM_OF_STRINGS * MAX_STRING_LEN];
    bool _isEnabled;
    bool _isTxBufReleasable;
    uint8_t* _txBuf;
    uint32_t _txPos;
    uint32_t _txSize;
    uint8_t* _rxBuf;
    uint32_t _rxPos;
    uint32_t _rxSize;
    bool _isNewLine; // This is a flag to add '\r' before '\n'

    static void _UartCallback(void* pObj) { }
    void _HandleUart() { }
    void BeforeLoop() override { }
    void Loop() override { }
};

// IServiceCmd Class
// Interface for the serviceable tasks to access the service command passed to them,
// and to access the functions of the port the command came from
class IServiceCmd
{
public:
    // Check if the command match an expected one
    // param  expectedCmd : string of the expected command
    // param  expectedNumOfParams : number of parameters this command needs
    // return true, matched; false, not matched
    virtual bool Match(const char* expectedCmd, uint8_t expectedNumOfParams) = 0;

    // Get a specific integer parameter from the received commad
    // param  index : index of the parameter, 0 based.
    // return the value of this parameter represended as 32-bit unsigned number
    virtual uint32_t GetParm(uint8_t index) = 0;

    // Get a specific float number parameter from the received commad
    // param  index : index of the parameter, 0 based.
    // return the value of this parameter
    virtual float GetFloatParm(uint8_t index) = 0;

    // Get the port from which the command is received, so the reponse can be send back
    // to the proper one, when there are multiple service ports
    // return pointer to the service port
    virtual ServicePort* GetResponsePort() = 0;

    // Get the string parameter from the received commad
    // param  [out] pBuf  : output buffer to receive the string
    // param  [in] maxSize : the maximum size of the string to received (usually the size of the receive buffer)
    virtual void GetString(char* pBuf, uint32_t maxSize) = 0;
};

// ServiceCmd Class
class ServiceCmd : public IServiceCmd
{
public:
    ServiceCmd() :
        _buf(),
        _code(),
        _fParams(),
        _isDecimalPointDetected(),
        _outputBuf(),
        _pServicePort(),
        _paramNum(),
        _params(),
        _size(),
        _stringLen(),
        _stringParam()
    { }

    ServiceCmd(ServicePort& port) :
        _buf(),
        _code(),
        _fParams(),
        _isDecimalPointDetected(),
        _outputBuf(),
        _pServicePort(&port),
        _paramNum(),
        _params(),
        _size(),
        _stringLen(),
        _stringParam()
    { }

    bool Interprete(const uint8_t* buf, uint8_t size, ServicePort* pServicePort) { return true; }
    bool Match(const char* code, uint8_t numOfParams) override;
    uint32_t GetParm(uint8_t index) override;
    float GetFloatParm(uint8_t index) override { return 0; }
    ServicePort* GetResponsePort() override { return nullptr; }
    void GetString(char* pBuf, uint32_t maxSize) override { }

protected:
    enum
    {
        MAX_CODE_LEN = 12,
        MAX_PARAM_NUM = 4,
        MAX_STRING_PARAM_LEN = 20,
        MAX_STRING_LEN = 256
    };

    uint8_t _code[MAX_CODE_LEN];    // Null terminated command code
    int32_t _params[MAX_PARAM_NUM];
    float _fParams[MAX_PARAM_NUM];
    char _stringParam[MAX_STRING_PARAM_LEN];
    uint8_t _stringLen;
    bool _isDecimalPointDetected;
    uint8_t _paramNum;
    uint8_t _outputBuf[MAX_STRING_LEN];
    ServicePort* _pServicePort;

private:
    const uint8_t* _buf;
    uint8_t _size;

    void _Parse() { }
    uint8_t _ToCaps(uint8_t byte) { }
    bool _IsSep(uint8_t byte) { }
    bool _IsEnd(uint8_t byte) { }
};

// IServiceable Class
// Interface for all serviceable tasks
// For all the task or class that need to be controlled by the service port (via Serial or CAN),
// need to implement this interface
class IServiceable
{
public:
    // Process the general service command (available in all states)
    // param cmd  : reference to the received command
    // return true, the command is accepted by the task/class; false, the command is not accepted
    virtual bool ProcessGeneralServiceCmd(IServiceCmd& cmd) = 0;

    // Help information this task/class provide to display when queried by "?" command
    // return the help information string
    virtual const char* GetGeneralServiceHelp() = 0;

    // Process the special service command (available ONLY in service state)
    // param[in]    cmd  : reference to the user input command
    // return       if the command is processed by the serviceable class
    virtual bool ProcessSpecialServiceCmd(IServiceCmd& cmd) { return false; }

    // Help information for the special service commands of this task/class
    // provide to display when queried by "?" command
    // return       Help information
    virtual const char* GetSpecialServiceHelp() { return nullptr; }
};

#endif // #ifdef INCLUDE_ISERVICEABLE_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_POWER_MANAGER_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_POWER_MANAGER_H_

// PowerManager Class
// Power Manager class provide the functionality to control the standby,
// power up, and power down.  It also provides power status monitoring
class PowerManager : RTOS::Task<100>, public IServiceable
{
public:
    // Task incoming queue
    RTOS::MQ q;

    const static RTOS::Ev evPowerUp;		// Event to request performing power up sequence
    const static RTOS::Ev evPowerDown;		// Event to request performing power down sequence
    const static RTOS::Ev evStandby;		// Event to request entering standby
    const static RTOS::Ev evReset;			// Event to request resetting
    const static RTOS::Ev evEnterService;   // Event to enter service state, this allow manual power control
    const static RTOS::Ev evNodeInfo;       // Event to read Node Info

    // PowerManager Constructor
    PowerManager(CARD_ID& cardIdPins, HAL::Pin& pin24vOk, HAL::Pin& pinPresent0, HAL::Pin& pinPresent1) :
        _pCardIdPins(&cardIdPins),
        _p24vOk(&pin24vOk),
        _pPresent0Pin(&pinPresent0),
        _pPresent1Pin(&pinPresent1),
        _pBuildIdPins(),
        _cardId(0xFF),
        _pControlQ(nullptr),
        _isServiceState(false),
        _isPowerUp(false)
    { }

    PowerManager() :
        _pCardIdPins(),
        _p24vOk(),
        _pPresent0Pin(),
        _pPresent1Pin(),
        _pBuildIdPins(),
        _cardId(0xFF),
        _pControlQ(),
        _isServiceState(),
        _isPowerUp()
    { }

    // Destructor
    virtual ~PowerManager() { }

    // Set queue to control task
    // param[in] q : reference to the queue
    void SetControlQ(RTOS::MQ& q);
    void SetBuildIdPins(BUILD_ID& buildIdPins) { }
    void SetCardIdPins(CARD_ID& cardIdPins);
    void Set24vOkPin(HAL::Pin& pin);
    void SetPresentPins(HAL::Pin& pin0, HAL::Pin& pin1);

    // Entering low power standby state (12MHz)
    virtual void Standby() { }

    // Reset processor
    virtual void Reset() { }

    // Check the "card present" signal to determine if the card is connected to the system
    // retval   true  : card is installed in the system
    // retval   false : card is working stand-alone
    virtual bool CheckCardInstalled() const { return true; }

    // Check the "card ID" signal to match the expected value in the software
    // retval   true  : card matches the software
    // retval   false : card does not match the software
    virtual bool CheckCardId() { return true; }

    // Check the "24V OK" signal to determine if the main power supply is available
    // retval   true  : main power supply is available
    // retval   false : main power supply is not available
    virtual bool CheckReadyPowerUp() { return true; }

    // Enable power task
    virtual void Enable() { }

    // Actions before task loop
    void BeforeLoop() override { }

    // Task loop
    void Loop() override { }

    // Power manager service commands handling
    // param[in] cmd : reference to the user input command
    // return if the command is processed
    bool ProcessGeneralServiceCmd(IServiceCmd& cmd) override { return true; }

    const char* GetGeneralServiceHelp() override
    {
        static const char* helpInfo =
            "PWRSTAT     :  Get power status\n";

        return helpInfo;
    }

    bool ProcessSpecialServiceCmd(IServiceCmd& cmd) override { return true; }

    const char* GetSpecialServiceHelp() override
    {
        static const char* helpInfo =
            "POWER, 0/1  :  Disable/Enable Power\n"
            "K           :  Reset EM\n";

        return helpInfo;
    }

protected:

    uint8_t _cardId;          // Card ID, this is initialized by the derived class
    RTOS::MQ* _pControlQ;     // Pointer to the control task queue
    bool _isServiceState;     // Flag for service state
    bool _isPowerUp;          // Power up sequence already performed
    CARD_ID* _pCardIdPins;
    BUILD_ID* _pBuildIdPins;
    HAL::Pin* _p24vOk;
    HAL::Pin* _pPresent0Pin;
    HAL::Pin* _pPresent1Pin;

    uint8_t _ReadCardId() const;						// Read card ID
    uint8_t _ReadBuildId() const;						// Read Build ID
    virtual bool _Check24VOk() const { return true; }     // Check the 24V OK signal										  

    // Each EM defines its own power up sequence
    virtual bool _PowerUpSequence() = 0;

    // Each EM defines its own power down sequence
    virtual void _PowerDownSequence() = 0;

    // Each EM defines its own power monitoring function
    virtual bool _PowerMonitor() = 0;

    // This is for service command to provide the power status report, each EM defines its own
    virtual void _GetStatus(ServicePort& port) = 0;

    // MQ Hander
    void _HandleEvMsg(const IMqItem* id, MsgObj& msg) { }
};

#endif // INCLUDE_POWER_MANAGER_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_TIMEKEEPER_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_TIMEKEEPER_H_

// TimeKeeper Class
class TimeKeeper
{
public:
    // Constructor
    TimeKeeper() { }

    // Adjust the present time counter
    // @param[in] newtime : new time counter
    void Adjust(uint32_t newtime) { }

    // Get the current time counter
    // @return   Millisecond counter
    uint32_t GetTime() { return 0; }

protected:
    uint32_t _offset;    // Offset between the native time and the adjusted time
};

#endif // INCLUDE_TIMEKEEPER_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_FAULT_MONITOR_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_FAULT_MONITOR_H_

class TimeKeeper;

// FaultMonitor Class
// This task is a high priority task that:
// # Monitors the incoming interlock signals
// # Receives fault activation messages from other tasks
// # Controls the output interlock signal
// # Allows the activated faults being cleared and recovers the system
// # Generate LED visual indication
// The specific classes of each sub-system should be derived from this class
// and add specific enter/exit safe state actions, such as disable/enable laser
class FaultMonitor : public RTOS::Task<160>, public IServiceable
{
public:
    const static RTOS::Ev       evClearAllFaults;			// Event to clear all faults
    const static RTOS::Ev       evStartDumpFaultInfo;		// Event to start dumping the fault information
    const static RTOS::Ev       evClearLockouts;			// Event to assert ILocks
    const static RTOS::Ev       evCheckILocks;				// Event to check ILocks
    const static RTOS::Ev       evChkCommonParam;			// Event to check the common parameters
    const static RTOS::Ev       evIssueLockout;				// Event to deassert ILocks
    const static RTOS::Ev       evEnterService;				// Event to enter service
    const static RTOS::Ev		evDeassertILocks;			// Event to Deassert Locks
    const static RTOS::Ev       evAssertILocks;				// Event to Assert Locks

    // Common faults
    const static RTOS::ShortMsg msgPowerUpFault;			// Power Up Fault message
    const static RTOS::ShortMsg msgPowerSupplyFault;		// Power Supply Fault message
    const static RTOS::ShortMsg msgScanFault;				// Scan Fault message
    const static RTOS::ShortMsg msgLinearFault;				// Linear Fault message
    const static RTOS::ShortMsg msgControlFault;			// Control Fault message
    const static RTOS::ShortMsg msgCommFault;				// Communication Fault message
    const static RTOS::ShortMsg msgSoftwareFault;			// Software Fault message
    const static RTOS::ShortMsg msgHardwareFault;			// Hardware Fault message
    const static RTOS::ShortMsg msgTestFault;				// Test Fault message
    const static RTOS::ShortMsg msgSnoCrcResult;			// Serial no CRC result

    //Task message queue
    RTOS::MQ q;

    // common storage parameters stored in EEPROM
    struct _CommonParameters
    {
        int32_t continuePwrUp;    // Continue powering up.
        uint32_t crc;             // CRC check value for the common storage parameters
    };

    struct _CommonParameters _commonParam;

    //Method to return the value of the flag to continue power up sequence with fail
    //return:  The flag value stored in the EEPROM
    uint32_t GetCntPwrUpFlag() { return 1; }

    // Verifies the common parameters crc
    // return:  "true" if the computed and stored CRC are matching. "false", otherwise
    bool VerifyParamCrc() { return true; }

    // Update the common parameters crc
    void UpdateParamCrc() { }

    // Service Method to set the ability of the power on functionality
    // to ignore failures in the power up sequence
    // param value:       Boolean flag that allows failures when true (1)
    void SetCtnPwrUpWithFail(bool value) { }

    // Get the continue power up flag value
    // return:  flag value in boolean
    bool GetCtnPwrUpWithFail() { return true; }

    // FaultMonitor Constructor
    // Description:       Method to construct and initialize the FaultMonitor class object
    // param iLockIn:      Reference to the iLock pin inputs set by the other system components
    // param iLockOut:     Reference to the iLock pin output set by this module
    // param diagPin:      Reference to the Diagnostic pin used to force the EM into Service Mode
    // param led:          Reference to the LED control pin
    FaultMonitor(HAL::Pin& iLockIn, HAL::Pin& iLockOut, HAL::Pin& diagPin, HAL::Pin& led) { }

    // FaultMonitor Constructor
    // Description:       Method to construct and initialize the FaultMonitor class object
    // param iLockPins:    Reference to the iLock pins class object for the interlock input and output pins
    // param diagPin:      Reference to the Diagnostic pin used to force the EM into Service Mode
    // param led:          Reference to the LED control pin.
    FaultMonitor(ILOCK_PINS& iLockPins, HAL::Pin& diagPin, HAL::Pin& led) { }

    // Method to set the TimeKeeper to the FaultMonitor object
    // param tKeeper: Reference to the TimeKeeper class object used for the fault
    void SetTimeKeeper(TimeKeeper& tKeeper) { }

    // Method to set the communication queue reference
    // param q: Reference to the Queue class object for task communication
    void SetCommQ(RTOS::MQ& q) { }

    // Method to set the control queue
    // param q: Reference to the Queue class object for task communication
    void SetControlQ(RTOS::MQ& q) { }

    // Method to perform before the loop functionality. (Set ILock output)
    void BeforeLoop() { }

    // Method to perform the loop functionality. If fault is
    // received, set the ILock output to flase, send the fault event on the
    // control queue and set the fault message. If faults are cleared, update it
    // and assert the ILock output. If ILock is changed, send the respective ILock
    // output staus event on the control queue. In Normal node, upon receiving
    // evClearLockouts, set the ILock output to true, if no faults. In Service
    // mode, upon receiving evClearLockouts, set the ILock output to true
    // unconditionally. Upon receiving evIssueLockout, set the ILock output to
    // false. Upon evEnterService, enter service
    void Loop() { }

    // Method to process the General Service Command (FAULTSTAT)
    // and prints the interlock status for the processor, OCT, IVUS, Control,
    // MDU, OE, PWR and Laser
    // param cmd: Command message
    // return: true if matched command, false otherwise
    bool ProcessGeneralServiceCmd(IServiceCmd& cmd) { return true; }

    // GetGeneralServiceHelp Method
    // Method to get General Service Help and prints the
    // possible General Service Help commands
    // return: The char buffer pointer to a list of possible commands
    const char* GetGeneralServiceHelp() { return ""; }

    // ProcessSpecialServiceCmd Method
    // Method to process the Special Service Commands available
    // only in Service Mode that provide the FAULTTEST and ILOCK command message
    // processing. The FAULTTEST command sends a Test fault message while the ILOCK
    // command will issue an evClearLockouts event if the first parameter is a 1 or
    // issues an evIssueLockout event if the first parameter is 0
    // param cmd: Command message
    // return: true if command match found, false otherwise
    bool ProcessSpecialServiceCmd(IServiceCmd& cmd) { return true; }

    // Method available while in Service Mode to get Special
    // Service Help and prints the possible General Service Help commands
    // return: The char buffer pointer to a list of possible commands
    const char* GetSpecialServiceHelp() { return ""; }

    // Callback Method that is called whenever the interrupt
    // service runs when the iLock pins change state
    // param pObj: Pointer to the calling object
    static void ILockCallback(void* pObj) { }

    // Method to start the FaultMonitor object task
    virtual void Enable() { }

    // Method to set or clear the ILock Out pin
    // Param val:  "true" sets the ILock to lockout. "false" clears the ILock
    void SetILockOut(bool val) { }

    // Get the Ilock pins value.
    // Method to return the ilock byte value
    // return: Value of the ilockpins
    uint8_t* GetILockPinsValue() { return nullptr; }

    // Start the application integrity checks method
    // Method to  setup and start the timer to check the application integrity
    void StartAppIntegrityCheck() { }

    // Test the usage faults
    // Method to test the usage faults
    void TestUsageFault() { }

    // ILockCallback Method:
    // Description:       Callback Method that is called whenever the interrupt
    // service runs when the iLock pins change state
    // param pObj:     Pointer to the calling object
    static void Faultcallback(void* pObj) { }

    // Reset 3 Sec Periodic Comm Timer
    void Reset3SecCommMsgTmr() { }

protected:
    enum
    {
        PRIO = 5,
        NUM_OF_ILOCK_IN = 6,                 // Number of individual iLock signals
        MAX_NUM_OF_FAULTS = 32,
        FAULT_TX_PERIOD = 200,
        APP_INTEGRITY_CHECK_PERIOD = 3600000 // Application integrityc check interval (1-hour)
    };

    // This structure describes the fault information for a single row of the fault table stored in RAM
    struct fault_t
    {
        uint16_t counter;                   // Counter for repeating fault activations
        uint32_t timestamp;                 // 0 : fault is not set, non-0 : time-stamp of the fault
        uint32_t param1;                    // Fault parameter
        uint32_t param2;                    // Fault parameter
    };

    const static RTOS::Ev _evIlockChanged;	// Event that is issued whenever the ILocks change
    ILOCK_PINS* _pILockPins;
    HAL::Pin* _pILockOut;					// GPIO ID for the interlock output pin
    HAL::Pin* _pILockIn;					// GPIO ID for the interlock output pin
    HAL::Pin* _pDiagPin;
    HAL::Pin* _pLedPin;
    HAL::PinGroup_Interrupt* _pGroupInt;
    HAL::Pin::Interrupt* _pPinInt;
    TimeKeeper* _pTimeKeeper;
    RTOS::MQ* _pCommQ;
    RTOS::MQ* _pControlQ;
    fault_t _faultTable[MAX_NUM_OF_FAULTS];
    uint32_t _faultBitmap;					// A bitmap showing all active faults, used by communication
    uint8_t _commFaultIndex;
    bool _isServiceState;
    uint8_t _iLockPinsValue;

    void _SetFault(MsgObj& msg) { }
    void _DumpFaultsToComm() { }
    void _ClearFaults() { }
    bool _CheckILocks() const { return true; }
    virtual void _SendOneFaultToComm(fault_t& faultEntry) { }
    virtual void _HandleMq(const IMqItem* id, const MsgObj& msg) { }
    virtual bool _IsFault(const IMqItem* id) { return true; }

private:
    enum _CONST
    {
        TIMER3SEC_PERIOD = 3000
    };

    // Stored pointer to the timer (3 Sec) instance
    static RTOS::Timer* _pTimer3Sec;

    // Callback method that is called by the periodic timer3Sec
    // param pParam:   Pointer to the object that has the members used by the callback
    static void _Timer3SecCallback(void* pParam) { }
};

#endif // INCLUDE_FAULT_MONITOR_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_CONTROL_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CONTROL_H_

// ControlMQ Class
// Shared events and messages used by control task, works like an interface
// These events and messages are used to driven the common state machine
class ControlMQ
{
public:
    RTOS::MQ q;   // Message queue to access Control task

    const static RTOS::Ev       evPowerOn;           // Event for Go-Safe
    const static RTOS::Ev       evGoSafe;            // Event for Go-Safe
    const static RTOS::Ev       evFault;             // Event when fault(s) are detected
    const static RTOS::Ev       evMissingILock;      // Event when interlock signals are missing
    const static RTOS::Ev       evAllILockOk;        // Event when interlock signals are all restored
    const static RTOS::Ev       evPowerUpComplete;   // Event when the power up sequence is completed
    const static RTOS::Ev       evPowerUpFailed;     // Event when the power up sequence is completed
    const static RTOS::Ev       evPowerDownComplete; // Event when the power down sequence is completed
    const static RTOS::Ev       evSmTimer;           // State Machine Timer
    const static RTOS::ShortMsg msgPowerStatus;      // Message for new power status
    const static RTOS::ShortMsg msgSystemState;      // Message for new system state
    const static RTOS::Ev       evLineOkMissing;     // Message for OK missing
    const static RTOS::Ev		evStartPwrOnTimer;   // Event issued to trigger the start of the PwrOn Timer
    const static RTOS::Ev		evStopPwrOnTimer;    // Event issued to trigger the stop of the PwrOn Timer
    const static RTOS::Ev		evStartLineOkTimer;	 // Event to Start the Line OK timer
    const static RTOS::Ev		evStopLineOkTimer;	 // Event to Stop the Line OK timer

protected:
    const static RTOS::Ev       _evGoService;        // Message for Go Service
};

// Control Class
// Common Control Task
template <class T, size_t stkSize>
class Control : public RTOS::Task<stkSize>, public ControlMQ, public IControl
{
    class Communication;

public:
    // Constructor
    Control() :
        RTOS::Task<stkSize>("Control Task", CTRL_PRI),
        _pCommQ(0),
        _pFaultQ(0),
        _pPowerQ(0),
        _timer("CTRL TMR", RTOS::Timer::ONESHOT),
        _systemState(NXGen::STATE_INIT)
    { }

    // Set queue to communication task
    // param[in] commQ : reference to the queue
    virtual void SetCommQ(RTOS::MQ& commQ)
    {
        _pCommQ = &commQ;
    }

    // Set queue to fault task
    // param[in]   faultQ  : reference to the queue
    virtual void SetFaultQ(RTOS::MQ& faultQ)
    {
        _pFaultQ = &faultQ;
    }

    // Set queue to power task
    // param[in]   powerQ  : reference to the queue
    virtual void SetPowerQ(RTOS::MQ& powerQ)
    {
        _pPowerQ = &powerQ;
    }

    virtual void SetRunLeds(RunLeds* pRunLedsInside, RunLeds* pRunLedsFront)
    {
        _pRunLedsInside = pRunLedsInside;
        _pRunLedsFront = pRunLedsFront;
    }

    virtual void SetDiagPin(HAL::Pin& pin)
    {
        _pDiagPin = &pin;
    }

    // Enable the control task
    virtual void Enable()
    {
        RTOS::Task<stkSize>::Start();
    }

    // Disable the control task
    virtual void Disable()
    {
        RTOS::Task<stkSize>::Suspend();
    }

    // Get the system state (such as IDLE, CALIBRATING, etc.)
    // return System state
    NXGen::SYSTEM_STATE GetSystemState() override
    {
        return _systemState;
    }

    // Check the EM Diag signal status
    // retval true   : EM Diag Jumper is installed
    // retval false  : EM Diag Jumper is not installed
    bool GetDiagMode() override
    {
        if (_pDiagPin)
        {
            return _pDiagPin->Get();
        }

        return false;
    }

    // Actions when entering standby state
    virtual void Standby()
    {
        if (_pPowerQ)
        {
            _pPowerQ->Send(PowerManager::evStandby);
        }

        if (_pRunLedsInside)
        {
            _pRunLedsInside->ChangeState(RunLeds::STANDBY);
        }

        if (_pRunLedsFront)
        {
            _pRunLedsFront->ChangeState(RunLeds::STANDBY);
        }
    }

    // Actions when entering power up state
    virtual void PowerUp()
    {
        if (_pPowerQ)
        {
            _pPowerQ->Send(PowerManager::evPowerUp);
        }

        if (_pRunLedsInside)
        {
            _pRunLedsInside->ChangeState(RunLeds::POWERUP);
        }

        if (_pRunLedsFront)
        {
            _pRunLedsFront->ChangeState(RunLeds::POWERUP);
        }
    }

    // Actions when entering power down state 
    virtual void PowerDown()
    {
        if (_pPowerQ)
        {
            _pPowerQ->Send(PowerManager::evPowerDown);
        }

        if (_pRunLedsInside)
        {
            _pRunLedsInside->ChangeState(RunLeds::POWERDOWN);
        }

        if (_pRunLedsFront)
        {
            _pRunLedsFront->ChangeState(RunLeds::POWERDOWN);
        }
    }

    // Actions when entering normal state
    virtual void Normal()
    {
        if (_pFaultQ)
        {
            _pFaultQ->Send(FaultMonitor::evAssertILocks);
        }

        if (_pRunLedsInside)
        {
            _pRunLedsInside->ChangeState(RunLeds::NORMAL);
        }

        if (_pRunLedsFront)
        {
            _pRunLedsFront->ChangeState(RunLeds::NORMAL);
        }
    }

    // Actions when entering service state
    virtual void Service()
    {
        if (_pPowerQ)
        {
            _pPowerQ->Send(PowerManager::evEnterService);
            _pFaultQ->Send(FaultMonitor::evEnterService);
        }

        if (_pRunLedsInside)
        {
            _pRunLedsInside->ChangeState(RunLeds::SERVICE);
        }

        if (_pRunLedsFront)
        {
            _pRunLedsFront->ChangeState(RunLeds::SERVICE);
        }
    }

    // Actions when entering safe state
    virtual void Safe()
    {
        if (_pFaultQ)
        {
            _pFaultQ->Send(FaultMonitor::evDeassertILocks);
        }

        if (_pRunLedsInside)
        {
            _pRunLedsInside->ChangeState(RunLeds::SAFE);
        }

        if (_pRunLedsFront)
        {
            _pRunLedsFront->ChangeState(RunLeds::SAFE);
        }
    }

    // Actions when entering recovery state
    virtual void Recovery()
    {
        if (_pFaultQ)
        {
            _pFaultQ->Send(FaultMonitor::evClearAllFaults);
        }
    }

    // Actions before the task loop, initialize the command state machine
    void BeforeLoop() override
    {
        using SM = CommonSM<T>;
        SM::Setup(*this);
        SM::Init();
    }

    // Task loop, handles the event and messages for the queue
    void Loop() override
    {
        using SM = CommonSM<T>;

        MsgObj msg;
        const auto id = q.Receive(RTOS_INDEFINITE_TIMEOUT, &msg);

        if (id == &evLineOkMissing)
        {
            _pPowerQ->Send(PowerManager::evPowerDown);
        }
        else if (id == &evPowerOn)
        {
            SM::Run(SM::EV_POWERON);
        }
        else if (id == &msgSystemState)
        {
            auto newState = (NXGen::SYSTEM_STATE)msg.param1;
            _systemState = newState;
            SM::Run(SM::EV_MAIN_STATE);

            if (newState != _prevState)
            {
                _prevState = _systemState;
                _HandleSystemStateChange();
            }
        }
        else if (id == &evSmTimer)
        {
            SM::Run(SM::EV_TIMER);
        }
        else if (id == &evGoSafe)
        {
            SM::Run(SM::EV_GO_SAFE);
        }
        else if (id == &evMissingILock)
        {
            SM::Run(SM::EV_GO_SAFE);
        }
        else if (id == &evFault)
        {
            SM::Run(SM::EV_FAULT);
        }
        else if (id == &evPowerUpComplete)
        {
            SM::Run(SM::EV_POWERUP_DONE);
            _pCommQ->Send(PowerManager::evNodeInfo);
        }
        else if (id == &evPowerDownComplete)
        {
            if (_pPowerQ)
            {
                _pPowerQ->Send(PowerManager::evReset);
            }
        }
        else if (id == &_evGoService)
        {
            SM::Run(SM::EV_GO_SERVICE);
        }
        else
        {
            _HandleMq(id, msg);
        }
    }

    bool IsServiceState()
    {
        using SM = CommonSM<T>;
        return SM::IsService();
    }

protected:
    RTOS::MQ* _pCommQ;							// pointer to communication queue
    RTOS::MQ* _pFaultQ;							// pointer to fault queue
    RTOS::MQ* _pPowerQ;							// pointer to power queue
    RunLeds* _pRunLedsInside;
    RunLeds* _pRunLedsFront;
    HAL::Pin* _pDiagPin;
    RTOS::Timer _timer;							// control task timer, used by the control state machine
    NXGen::SYSTEM_STATE _systemState;			// system state
    NXGen::SYSTEM_STATE _prevState;				// previous system state

    const static NXGen::NODE_INFO _nodeInfo;	// node information

    // additional handling for the system state change, can be override by EM control task
    virtual void _HandleSystemStateChange() { }

    // additional handling EM specific event and messages, can be override by EM control task
    virtual void _HandleMq(const IMqItem* id, const MsgObj& msg) { }

    // control task priority
    enum { CTRL_PRI = 12 };
};

#endif // INCLUDE_CONTROL_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_STEPPER_CONTROL_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_STEPPER_CONTROL_H_

// Forward Declare
namespace HAL
{
    class DAC;
    class Pin;
    class Pulse;
}

namespace Motion
{
#define DEFAULT_MSTEP               (1UL) // 16-fullstep, 8-halfstep, ...
#define DEFAULT_HOLD_CURRENT_MA     (100)
#define RAMP_TIMER_PERIOD           (10)
#define STEP_PER_REV	            (3200UL)

    // Motion::IStepperControl Class
    class IStepperControl
    {
    public:
        // Set the status events generated by the stepper controller
        virtual void SetEvents(RTOS::MQ* destQ, const RTOS::Ev* pEvDone, const RTOS::Ev* pPosLimEv, const RTOS::Ev* pNegLimEv) = 0;
        virtual void Enable() = 0;
        virtual void Disable() = 0;
        virtual void Hold() = 0;
        virtual bool Go(uint32_t speed, int32_t displacement) = 0;
        virtual void ChangeCurrent(uint32_t mA) = 0;
        virtual bool ChangeSpeed(uint32_t speed) = 0;
        virtual bool SetPosition(int32_t position) = 0;
        virtual int32_t  GetPosition() = 0;
        virtual int32_t GetSpeed() = 0;
        virtual int32_t  GetDirection() = 0;
    };

    // Motion::StepperControl Class
    // Stepper motor control class
    // This class uses PWM and GPIO peripherals to generate a serials of pulses
    // at the specified speed, and a direction signal.  It requires an external
    // stepper motor drive to convert the pulses to Phase A/B currents
    // The stepper object must be first configured before use
    class StepperControl : public IStepperControl
    {
    public:
        // Look up table to find the proper current for different speeds.
        // Using this table, the stepper's torque can be controlled. The
        // values used in this table should be generated from tests
        // See ref SetControlTable() for an example
        struct control_table_t
        {
            uint32_t speedUpperLimit;    // upper limit of this row
            uint32_t mA;                 // current setting below this speed
        };

        // Constructor
        // param pulse : reference to the pulse generator (derived from PWM)
        // param screw_lead : gear ratio, um/rev
        StepperControl(HAL::Pulse& pulse, int32_t screw_lead) :
            _currentPosition(),
            _direction(),
            _backwardLimit(40000),
            _destQ(nullptr),
            _forwardLimit(40000),
            _holdingCurrentMa(DEFAULT_HOLD_CURRENT_MA),
            _mA(),
            _mStep(DEFAULT_MSTEP),
            _overrideCurrentMa(),
            _pDac(),
            _pTable(),
            _p_in_negSwitch(),
            _p_in_posSwitch(),
            _p_out_dir(),
            _p_out_enable(),
            _p_out_ms1(),
            _p_out_ms2(),
            _pEvDone(0),
            _pNegLimEv(0),
            _pPosLimEv(0),
            _pulseNum(),
            _pulsePeriod(),
            _pulse(&pulse),
            _ramp1Per10mS(),
            _ramp1TimeIn10mS(),
            _ramp2Per10mS(),
            _ramp2TimeIn10mS(),
            _rampTimer("Motor Monitor Timerr", RTOS::Timer::PERIODICAL),
            _screw_lead(screw_lead),
            _speedActual(),
            _speedFinal(),
            _startSpeed1(),
            _startSpeed2(),
            _state(DISABLED),
            _tableSize(),
            _travelRange(),
            _volt(),
            _voltPerMilliAmp()
        { }

        // Destructor
        virtual ~StepperControl() { }

        // Stepper control configuration methods:

        // Set the output event for stepper control
        // param destQ      : queue to receive stepper events
        // param pEvDone    : pointer to the event for movement complete
        // param pPosLimEv  : pointer to the event for positive travel limit
        // param pNegLimEv  : pointer to the event for negative travel limit
        void SetEvents(RTOS::MQ* destQ, const RTOS::Ev* pEvDone, const RTOS::Ev* pPosLimEv, const RTOS::Ev* pNegLimEv)
        {
            _destQ = destQ;
            _pEvDone = pEvDone;
            _pPosLimEv = pPosLimEv;
            _pNegLimEv = pNegLimEv;
        }

        // Set the hardware pins for the limit switches
        // param negSwitch  : reference to negative switch input pin
        // param posSwitch  : reference to positive switch input pin
        void SetLimitInputs(HAL::Pin& negSwitch, HAL::Pin& posSwitch)
        {
            _p_in_negSwitch = &negSwitch;
            _p_in_posSwitch = &posSwitch;
        }

        // Set the hardware pin for direction output
        // param dir        : reference to the direction output pin
        void SetDirOutput(HAL::Pin& dir)
        {
            _p_out_dir = &dir;
        }

        // Set the hardware pin for enable output
        // param enable     : reference to the enable output pin
        void SetEnableOutput(HAL::Pin& enable)
        {
            _p_out_enable = &enable;
        }

        // Set the hardware DAC for control the stepper current
        // param dac     : reference to the stepper current control DAC
        void SetCurrentControlDac(HAL::DAC& dac)
        {
            _pDac = &dac;
            _pDac->Enable();
            _pDac->Write(0.2f);  // TODO replace the hard coded number
        }

        // Set the gain for the current control DAC
        // param voltPerMillAmp  : voltage per milliamp
        void SetCurrentControlGain(float voltPerMillAmp)
        {
            _voltPerMilliAmp = voltPerMillAmp;
        }

        // Set the stepper current control table
        // param  pTable    : pointer to the table
        // param  tableSize : number of rows of the table
        // note   During ramping, or user changes speed while the stepper is runner,
        // the current will change for the acutal speed
        // The current control table changes the stepper driving current at difference speeds.  If the table is:
        // {
        // { 10E6,  200},
        // { 20E6,  400},
        // { 40E6,  600}
        // }
        // The the current for speed from 0 to 10 mm/S will be 200mA, from 10 mm/S to 20 mm/S is 400 mS,
        // and from 20 mm/S to 40 mm/S is 600 mA.  For speed above 40 mm/S, the internal default current will be used
        void SetControlTable(const control_table_t* pTable, uint8_t tableSize)
        {
            _pTable = pTable;
            _tableSize = tableSize;
        }

        // Defines the full travel range
        // param  fullSteps : full travel range in steps
        void SetTravelRange(uint32_t fullSteps) { }

        // Defines the speed limit at both directions
        // param   forwardLimit  : speed limit (um/S) in negative direction
        // param   backwardLimit : speed limit (um/S) in positive direction
        void SetSpeedLimits(uint32_t forwardLimit, uint32_t backwardLimit) { }

        // Set the ramping parameters, time are in mS, accelerations are in um/S/S
        // param  startSpeed : start speed for the accelration
        // param  time1      : time duration to accelerate from startSpeed to startSpeed2
        // param  startSpeed2 : speed for 1st phase of accleration
        // param  time2      : time duration to accelerate from startSpeed2 to the final speed
        // The ramp is only applicable to the acceleration.  The ramp digram is
        void SetRamp(uint32_t startSpeed, uint32_t time1, uint32_t startSpeed2, uint32_t time2) { }

        // Enable the stepper current
        virtual void Enable() override { }

        // Disable the stepper current
        virtual void Disable() override { }

        // Enable the current but not sending any pulse
        virtual void Hold() override { }

        // Start the stepper motor movement.  The Stepper unit must be configured already
        // before using this method
        // param speed        : movement speed (in mm/Sec)
        // param displacement : displacement to current position(in um)
        virtual bool Go(uint32_t speed, int32_t displacement) override;

        // Overrides the stepper current from the table
        // param[in]  mA   : milliamp, if 0, use the table values
        virtual void ChangeCurrent(uint32_t mA) override { }

        // Overrides the stepper holding current form the table
        // param  mA   : milliamp, if 0, use the table values
        virtual void ChangeHoldCurrent(uint32_t mA) { }

        // Change the stepper speed when it is already running.  If acceleration,
        // the ramp will be used
        // param speed  : new speed
        virtual bool ChangeSpeed(uint32_t speed) override { return true; }

        // Override the internal position counter
        // param position : new position
        virtual bool SetPosition(int32_t position) override { _currentPosition = position; return true; }

        // Get the internal position counter value
        // return  position in um
        virtual int32_t  GetPosition() override { return _currentPosition; }

        // Get the current speed value
        // return : speed in um/S
        virtual int32_t GetSpeed() override { return _speedActual; }

        // Get the current moving direction
        // return 0 : not moving, 1 : moving backwards, -1 : moving forward
        virtual int32_t  GetDirection() override
        {
            if (_speedActual > 0)
            {
                return _direction;
            }
            else
            {
                return 0;
            }
        }

        // Peripheral callbacks
        // param  pObj : need to pass "this" so the callback can access the member methods and variables

        // Callback function used by hardware periperal,
        // called when the specific amount of pulses are generated
        static void PulseCallback(void* pObj);

        // Callback function used by hardware periperal
        // called when the positive limit switch is triggered
        static void PosLimitCallback(void* pObj);

        // Callback function used by hardware periperal
        // called when the negative limit switch is triggered
        static void NegLimitCallback(void* pObj);

    private:
        enum _STATE
        {
            DISABLED,  // Stepper is disabled and has not holding force
            HOLD,      // Stepper is enabled but no movement
            RAMPING,   // Stepper is accelerating
            MOVING     // Stepper is moving at constant speed
        };

        _STATE _state;					// Stepper state
        HAL::Pulse* _pulse;				// Pointer to the pulse generator, mandatory
        int32_t _screw_lead;			// lead of screw measured in um_per_revolution
        HAL::DAC* _pDac;				// Pointer to the current control DAC, optional
        RTOS::MQ* _destQ;				// Pointer to the MQ which receives event when pullback is done
        const RTOS::Ev* _pEvDone;       // The event ID for pullback done
        const RTOS::Ev* _pPosLimEv;     // The event ID for reaching positive limit
        const RTOS::Ev* _pNegLimEv;     // The event ID for reaching negative limit
        uint32_t _travelRange;			// Full travel range (in 1/16 micro-steps)
        uint32_t _forwardLimit;			// Forward (toward distal end) moving speed limit
        uint32_t _backwardLimit;		// Backward (toward proximal end) moving speed limit
        float _voltPerMilliAmp;
        HAL::Pin* _p_in_negSwitch;		// Index of negative limit switch in GPIO
        HAL::Pin* _p_in_posSwitch;		// Index of position limit switch in GPIO
        HAL::Pin* _p_out_ms1;			// Index of MS1 signal in GPIO
        HAL::Pin* _p_out_ms2;			// Index of MS2 signal in GPIO
        HAL::Pin* _p_out_enable;		// Index of Enable signal in GPIO
        HAL::Pin* _p_out_dir;			// Index of Direction signal in GPIO
        const control_table_t* _pTable;	// Pointer to a speed control table, optional
        uint8_t _tableSize;				// Size of the control table
        uint32_t _overrideCurrentMa;	// Current value to override the default or table
        uint32_t _holdingCurrentMa;
        int32_t _currentPosition;		// Current position of the stepper
        int32_t _direction;				// Direction: -1, 0, 1
        RTOS::Timer _rampTimer;			// Timer used to control the ramp
        uint32_t _mA;					// Drive current
        float _volt;					// Drive voltage
        uint32_t _mStep;				// Micro-step setting (1, 2, 4, 8, 16)
        uint32_t _pulsePeriod;			// Pulse period calculated from micro-step and speed
        uint32_t _pulseNum;				// Number of pulses to reach the target position
        uint32_t _startSpeed1;			// Start speed 
        uint32_t _startSpeed2;			// Start speed 
        uint32_t _ramp1TimeIn10mS;
        uint32_t _ramp2TimeIn10mS;
        uint32_t _ramp1Per10mS;			// The 1st stage acceleration every 10mS, this is fixed
        uint32_t _ramp2Per10mS;			// The 2nd stage acceleration every 10mS
        uint32_t _speedActual;
        uint32_t _speedFinal;

        virtual void _Stop() { }
        float _ConvertCurrToVolt(uint32_t mA) const;
        void _GetControlParameters(uint32_t speed, uint32_t distance);
        int32_t _ConvertMicronToSteps(int32_t um) const;
        int32_t _ConvertStepsToMicron(int32_t steps) const
        {
            return (int32_t)(steps * _screw_lead / (int32_t)STEP_PER_REV);
        }
        uint32_t _CalcPeriodFromSpeed(uint32_t speed) const;
        static void _RampTimerCallback(void* pObj);
        void _RampControl();
        void _CalcRamp2();
    };
}

#endif 	// INCLUDE_STEPPER_CONTROL_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_CAN_ENGINE_TYPES_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CAN_ENGINE_TYPES_H_

#ifdef __cplusplus
extern "C"
{
    namespace CanEngine
    {
#endif

        // Result of the CAN Interface APIs
        typedef enum
        {
            FAILED = 0,                 // Operation failed
            OK,                         // Operation completed
            PENDING                     // Operation can not be completed at this time
        }
        can_result_t;

        // CAN packet data structure
        // note This structure is not aligned for the actual register usage
        typedef struct
        {
            uint32_t canId;             // 29-bit extended CAN ID
            uint8_t data[8];            // Up to 8 byte of the CAN packet payload
            uint8_t size;               // Size of the CAN packet payload (0 - 8)
        }
        can_packet_t;

        // Handle to the CAN Interface instance
        typedef void* can_interface_h;

        // Handle to the CAN mailbox
        typedef void* can_mailbox_h;

        // Handle to the CAN Communication Object
        typedef void* comm_obj_h;

        // Handle to a CAN event callback
        typedef void(*can_callback_t)(void*);

        // Communication Object Configuration
        typedef struct
        {
            can_interface_h canInf;		// The CAN interface the comm obj will be attached to
            can_mailbox_h canMailbox;	// The Mailbox the comm obj will be attached to
            uint16_t id;				// ID of the comm obj
            uint8_t peerNode;			// for point-to-point comm obj, the node ID of the other node
            void* pAppBuf;				// This buffer holds the latest and completed received buffer
            void* pRxBuf;				// This is the working buffer to receive data
            uint32_t rxLength;			// Size of ref _pAppBuf and ref _pRxBuf
            void* pTxBuf;				// This buffer holds the data to be transmitted 
            uint32_t txLength;			// Size of the TX buffer
            can_callback_t rxCallback;  // RX callback function
            void* rxCallbackParam;      // Parameter to be passed when invoke the RX callback
            can_callback_t txCallback;  // TX callback function
            void* txCallbackParam;      // Parameter to be passed when invoke the TX callback
        }
        can_comm_obj_config_t;

#ifdef __cplusplus
    }
}
#endif

#endif // INCLUDE_CAN_ENGINE_TYPES_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_CAN_HARDWARE_API_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CAN_HARDWARE_API_H_

typedef void ____; // non-functional code to sync with EA model

// #include "CanEngineTypes.h"

#ifdef __cplusplus
extern "C"
{
    namespace  CanEngine
    {
#endif
        // Pointer to the CAN hardware handle
        // note This handle is used in case the same API is used for more than one
        // CAN hardware channels.  In C++ we can easily created multiple instances
        // so it is not necessary to use an additional handle.  In C, however, it can
        // be more convenient to do it this way.  So the usage of this handle will
        // be defined by the provided API
        typedef void* can_hardware_h;

        // Enable CAN hardware, so it starts to receive and transmit CAN packets
        // param[in] canHardware : Handle to CAN hardware
        // return    Result of enabling CAN hardware
        typedef can_result_t(*pEnableCanHardware)(can_hardware_h canHardware);

        // Disable CAN hardware
        // param[in] canHardware : Handle to CAN hardware
        // return    Result of disabling CAN hardware
        typedef can_result_t(*pDisableCanHardware)(can_hardware_h canHardware);

        // Set a receiving mailbox with specific mask and filter
        // param[in] canHardware     : Handle to CAN hardware
        // param[in] mask            : if a bit is 1, it is compared with filter, if 0, don't care
        // param[in] filter          : filter CAN ID to receive or reject packets
        // param[out] pMailboxNumber : Output buffer to hold the mailbox number 
        // return    Result of setting the mailbox
        typedef can_result_t(*pSetCanMailBox)(can_hardware_h canHardware, uint32_t mask, uint32_t filter, uint8_t* pMailboxNumber);

        // Receive a CAN packet from the specific mailbox
        // param[in] canHardware     : Handle to CAN hardware
        // param[in] mailboxNumber   : Interested mailbox
        // param[out] pMsg           : Output buffer to hold the received packet
        // return    OK: packet is available, FAILED: no packet in the mailbox
        typedef can_result_t(*pGetCanPacket)(can_hardware_h canHardware, uint8_t mailboxNumber, can_packet_t* pMsg);

        // Transmit a CAN packet 
        // param[in] canHardware     : Handle to CAN hardware
        // param[in] pMsg            : Buffer to the packet to be transmitted
        // return    OK: packet is in the transmission queue, FAILED: CAN hardware cannot transmit the packet 
        typedef can_result_t(*pSendCanPacket)(can_hardware_h canHardware, const can_packet_t* pMsg);

        typedef struct
        {
            pEnableCanHardware EnableCanHardware;
            pDisableCanHardware DisableCanHardware;
            pSetCanMailBox SetCanMailbox;
            pGetCanPacket GetCanPacket;
            pSendCanPacket SendCanPacket;
        }
        can_hardware_api_t;

#ifdef __cplusplus
    }
}
#endif

#endif // INCLUDE_CAN_HARDWARE_API_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_CAN_ENGINE_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CAN_ENGINE_H_

typedef void ____; // non-functional code to sync with EA model

#ifdef BUILD_AS_DLL  
#define DLL_EXPORT __declspec(dllexport)
#else 
#define DLL_EXPORT
#endif

// #include "CanEngineTypes.h"
// #include "CanHardwareAPI.h"

#ifdef __cplusplus
extern "C"
{
    namespace CanEngine
    {
#endif

#define CAN_ENGINE_OFFSET_PRIO	(27)
#define CAN_ENGINE_OFFSET_TYPE	(25)
#define CAN_ENGINE_OFFSET_ID	(15)
#define CAN_ENGINE_OFFSET_SPEC	(13)
#define CAN_ENGINE_OFFSET_SRC	(8)
#define CAN_ENGINE_OFFSET_SEQ	(0)

#define CAN_ENGINE_MASK_PRIO	(3UL << CAN_ENGINE_OFFSET_PRIO)
#define CAN_ENGINE_MASK_TYPE	(3UL << CAN_ENGINE_OFFSET_TYPE)
#define CAN_ENGINE_MASK_ID		(0x3FFUL << CAN_ENGINE_OFFSET_ID)
#define CAN_ENGINE_MASK_SPEC	(3UL << CAN_ENGINE_OFFSET_SPEC)
#define CAN_ENGINE_MASK_SRC		(0x1FUL << CAN_ENGINE_OFFSET_SRC)
#define CAN_ENGINE_MASK_SEQ		(0xFFUL)

#define CAN_TYPE_EVENT			(0)
#define CAN_TYPE_EXCHANGE		(1)
#define CAN_TYPE_REG			(2)
#define CAN_TYPE_STREAM			(3)

        // Callback function signature for CAN Interface error
        typedef void(*can_fault_callback_t)(void*);

        // Open and initialize a CAN Interface
        // note The CAN Interface structure must be allocated by the application
        // param[in] can           : handle to the CAN Interface
        // param[in] pHwApi        : pointer to the hardware API
        // param[in] canHardware   : handle to the CAN hardware
        // param[in] selfId        : ID (5-bit) to this CAN node
        // retval    OK            : initialization completed
        // retval    FAILED        : initialization failed
        DLL_EXPORT can_result_t InitializeCanInterface(can_interface_h can, const can_hardware_api_t* pHwApi, can_hardware_h canHardware, uint16_t selfId);

        // Open and initialize a CAN mailbox
        // note The CAN mailbox structure must be allocated by the application
        // param[inout] hCanInf    : handle to the CAN Interface
        // param[inout] hMailbox   : handle to the CAN Mailbox
        // param[in] mask          : handle to the CAN Mailbox
        // param[in] filter        : handle to the CAN Mailbox
        // retval    OK            : initialization completed
        // retval    FAILED        : initialization failed
        DLL_EXPORT can_result_t InitializeCanMailbox(can_interface_h hCanInf, can_mailbox_h hMailbox, uint32_t mask, uint32_t filter);

        // Set an optional callback when the CAN Interface detects fault conditions
        // param[inout] can        : handle to the CAN Interface
        // param[in] faultCallback : callback function to be invoked when there is a fault
        // retval    OK            : callback set completed
        // retval    FAILED        : callback set failed
        DLL_EXPORT void SetCanFaultCallback(can_interface_h can, can_fault_callback_t faultCallback);

        // Process CAN Interface and handle all receiving and transmitting activities
        // note This function needs to be called periodically in a task or super-loop
        // param[inout] can        : handle to the CAN Interface
        // param[in] timestamp     : timestamp with millisecond resolution
        // retval    OK            : completed
        DLL_EXPORT can_result_t ProcessCanInterface(can_interface_h can, uint32_t timestamp);

        // Close the CAN Interface
        // param[inout] can        : handle to the CAN Interface
        DLL_EXPORT void CloseCanInterface(can_interface_h can);

        // Reset and clear all counters and states of the CAN Interface
        // param[inout] can        : handle to the CAN Interface
        DLL_EXPORT void ClearCanInterface(can_interface_h can);

        // Configure a communication object to transmit periodically
        // param[inout] hCommObj   : handle to the communication object
        // parma[in]  period       : period in milliseconds
        // param[in]  callback     : optional callback function
        // param[in]  param        : optional callback parameter
        // retval     OK           : periodical transmission is setup
        // retval     FAILED       : configuration failed
        // note if application defines the callback here, only this callback is invoked periodically, then application
        // needs to use the other access functions to start the transmission.  If callback is left null, the TX
        // buffer will be transmitted automatically.
        DLL_EXPORT can_result_t SetPeriodical(comm_obj_h hCommObj, uint32_t period, can_callback_t callback, void* param);

        // Configure the non-default transfer timeout for the communication object
        // param[inout] hCommObj   : handle to the communication object
        // parma[in]  timeout      : timeout value in millisecond
        // retval     OK           : transfer timeout is setup
        // retval     FAILED       : configuration failed
        DLL_EXPORT can_result_t SetXferTimeout(comm_obj_h hCommObj, uint32_t timeout);

        // Configure the fault handling callback for a communication object
        // param[inout] hCommObj   : handle to the communication object
        // param[in]  callback     : fault handling callback function
        // param[in]  param        : optional callback parameter
        // retval     OK           : fault handling callback is setup
        // retval     FAILED       : configuration failed
        DLL_EXPORT can_result_t SetFaultCallback(comm_obj_h hCommObj, can_callback_t callback, void* param);

        // Configure the communication object to expect incoming transfer periodically
        // param[inout] hCommObj   : handle to the communication object
        // param[in]  timeout      : timeout value in millisecond
        // retval     OK           : fault handling callback is setup
        // retval     FAILED       : configuration failed
        // note if the communication object does not receive a valid transfer within the
        //timeout, fault handling callback will be invoked.
        DLL_EXPORT can_result_t SetExpectedPeriodical(comm_obj_h hCommObj, uint32_t timeout);

#ifdef __cplusplus
    }
}
#endif

// #include "Event.h"
// #include "Exchange.h"
// #include "Register.h"
// #include "Stream.h"

#endif // INCLUDE_CAN_ENGINE_H_

/////////////////////////////////////////////////////////////////////
//	   INCLUDE_CAN_DAQ_VARS_H_
/////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CAN_DAQ_VARS_H_

#define DICTIONARY_NAME_MAX     32
#define CAN_DAQ_CHIP_ID         0x40043200      // Default register for ADDRESS_VALUE request

// Note: !!!!
// These values must be sequential!
// Metadata must match this enumeration.  (See CanDaqMetaData[] in CanDaq.cpp)
// 
// *** WARNING!!! *** -----------------------------------------------------------
// All entries must match entries in CanDaqMetaData[] in CanDaq_Vars.cpp
// ------------------------------------------------------------------------------

enum CAN_DAQ_VAR
{
    STOP,				// 0
    DICTIONARY,			// 1
    TEST_VAL,			// 2
    MDU_EXT_QEI_CNTS,	// 3 Reports new counts of External QEI (linear monitor)
    CMD_CNTRL_STATE,	// 4
    SERIAL_TX_ID,		// 5 Uses Traveler to track MDU Button Pull through CMD to Serial
    CMD_PULL_BTN,		// 6
    ADDRESS_VALUE,		// 7 Returns value at address defined by lowerest 32bits of payload
    MDU_CATH_DETECT,	// 8 returns Cath Detect -> UUB:TypeTimerOff, ULB:TypeTimerOn, LUB:Discon, LLB:Conn
    MDU_INPUTS,			// 9 returns Current Buttons 100ms as: b0:
    NV_MEM_CHECK,		// 10 returns result of latest periodic NV memory test
    ENUM_END_VAR		// Defines the last value
};

enum DAQ_MDU_INPUTS
{
    DAQ_MDU_PULLBACK_BTN = 0,
    DAQ_MDU_READY_BTN = 1,
    DAQ_MDU_SCAN_BTN = 2,
    DAQ_MDU_ORIGIN_BTN = 3,
    DAQ_MDU_HOME_BTN = 4,
    DAQ_MDU_CATH_TYPE = 5,
    DAQ_MDU_PROP_BTN = 8,
    DAQ_MDU_CATH_DETECT = 9
};

const uint32_t DAQ_MDU_INPUTS_MASK =
((uint32_t)1 << DAQ_MDU_PROP_BTN) +
((uint32_t)1 << DAQ_MDU_CATH_DETECT);

// union is used to gain access to individual bytes, words, doubles for
// longs, doubles and words
// Used for decoding CAN Bus messages
typedef union
{
    uint64_t value;
    uint32_t D[2];
    uint16_t W[4];
    uint8_t B[8];
}
uint64_splitA;

typedef union
{
    uint32_t value;
    uint16_t W[2];
    uint8_t B[4];
}
uint32_splitA;

typedef union
{
    uint16_t value;
    uint8_t B[2];
}
uint16_splitA;

typedef struct
{
    uint16_t varId;
    uint16_t mult;
    uint16_t offset;
    uint8_t radix;
    uint8_t signSizeUnits;
}
_can_daq_var_t;

typedef struct
{
    char name[DICTIONARY_NAME_MAX];
    _can_daq_var_t   metaData;
}
_can_daq_variable;

typedef union
{
    _can_daq_var_t	metaData;
    uint64_t L;
    uint32_t D[2];
    uint16_t W[4];
    uint8_t B[8];
}
canDaqVarT_split;

extern const _can_daq_variable  CanDaqVariable[];
extern const uint16_t CanDaqVarCount;
extern const uint8_t CanPayloadSize;
extern const uint16_t CanDaqBufSize;
extern const uint16_t GetDictionarySize;

#endif // INCLUDE_CAN_DAQ_VARS_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_CAN_DAQ_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CAN_DAQ_H_

//- Description: ***************************************************************
//	CAN DAQ is a simplified real-time data aquisition system exposing internal EM
//	data to the CAN Bus.  Messages are defined in CanDaq_Vars.h and CanDAq_Vars.cpp
//
//	Three mechanisms are available:
//	1) Direct value return - Send a uint32_t value if activated
//	2) Accumulate information in a "Traveler" and send as uint32_t value
//	3) Indirect value return - Send the value at address defined by the Request
//	   Message Payload
//
//	Usage:
//	Header file must be included to log variables updated in the files
//		#include "CanDaq/CanDaq.h"
//	The main file must instantiate a global instance of the CanDaq
//		CanDaq canDaq;
//	The system initialization requires configuring of canDaq to have access to
//	the CAN bus.
//		pComm->SetCanDaq(canDaq);
//		canDaq.Setup(&pComm->q, &CmdComm::msgDaqData);
//
//	For direct data transmission use
//		canDaq.xmitIfActive(CAN_DAQ_VAR::TEST_VAL, (uint32_t)(canDaq.getNumSamples()));
//	  (where CAN_DAQ_VAR::TEST_VAL is found in CanDaq_Var.h and the second parameter is
//	   the desired value to send)
//
//	For indirect data transmission use
//		xmitIfRegActive()
//	  (it assumed that the request payload contained a value address)
//
//	For "Traveler" data transmission use
//		ifActive()
//	  ( when accumulating data in the traveler)
//	  ( then, )
//		canDaq.xmitIfActive(CAN_DAQ_VAR::TEST_VAL, canDaq.GetTraveler());
//	  (where CAN_DAQ_VAR::ADDRESS_VALUE specifies relative mode and GetTraveler()
//	   as desired value to send)
// *******************************************************************************

// CanDaq class
// This class manages communications with CAN based data acquisition
class CanDaq
{
public:
    // Constructor 
    CanDaq() :
        _canDaqTraveler(),
        _canDaqTravelerPrev(),
        _newDaqRequest(),
        _numSamples(),
        _pCanDaqAddr(),
        _pCanDaqDataMsg(),
        _rtLogVarName(),
        pCanQ()
    { }

    // Destructor
    ~CanDaq() { }

    // Pointer to the CAN communications queing mechanism
    RTOS::MQ* pCanQ;

    // Traveler Manipulation Methods

    void SetTravelerPrev(uint32_t newVal)
    {
        _canDaqTravelerPrev = newVal;
    }

    uint32_t GetTravelerPrev(void)
    {
        return _canDaqTravelerPrev;
    }

    void SetTraveler(uint32_t newVal)
    {
        _canDaqTraveler.value = newVal;
    }

    uint32_t GetTraveler(void)
    {
        return _canDaqTraveler.value;
    }

    void SetTravelerBit(uint8_t bitNumber)
    {
        if (bitNumber < 32) _canDaqTraveler.value |= ((uint32_t)1 << bitNumber);
    }

    void ClrTravelerBit(uint8_t bitNumber)
    {
        if (bitNumber < 32) _canDaqTraveler.value &= (~((uint32_t)1 << bitNumber));
    }

    void TglTravelerBit(uint8_t bitNumber)
    {
        if (bitNumber < 32)
        {
            if (_canDaqTraveler.value & ((uint32_t)1 << bitNumber))
            {
                _canDaqTraveler.value |= ((uint32_t)1 << bitNumber);
            }
            else
            {
                _canDaqTraveler.value &= (~((uint32_t)1 << bitNumber));
            }
        }
    }

    // Connecting CAN DAQ to the CAN communications queing mechanism
    void Setup(RTOS::MQ* canQ, const RTOS::ShortMsg* pCanDaqDataMsg)
    {
        _pCanDaqDataMsg = pCanDaqDataMsg;
        pCanQ = canQ;
    }

    // Interpret the request from the CAN DAQ device by setting up the real-
    // time logging configurations
    uint16_t getNumSamples()
    {
        return _numSamples;
    }

    // Interpret the request from the CAN DAQ device by setting up the real-
    // time logging configurations
    CAN_DAQ_VAR GetRtLogVarName()
    {
        return _rtLogVarName;
    }

    // Interpret the request from the CAN DAQ device by setting up the real-
    // time logging configurations
    uint64_t Decode(uint64_t daqRequest) { }

    // This routine is called when the variable has been requested by the
    // CAN-DAQ Request message
    // This method is inlined for efficient code.  In most cases, the result
    // is false, so generally, a branck to subroutine is not taken
    inline bool ifActive(CAN_DAQ_VAR varName)
    {
        if (varName == _rtLogVarName)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // This routine is called when the variable has been requested by the
    // CAN-DAQ Request message
    // This method is inlined for efficient code.  In most cases, the result
    // is false, so generally, a branck to subroutine is not taken
    inline bool xmitIfActive(CAN_DAQ_VAR varName, uint32_t data)
    {
        if (varName == _rtLogVarName)
        {
            return _manageCanDaq(varName, data);
        }
        else
        {
            return false;
        }
    }

    // This routine is called when the variable has been requested by the
    // CAN-DAQ Request message
    // This method is inlined for efficient code.  In most cases, the result
    // is false, so generally, a branck to subroutine is not taken
    inline bool xmitIfRegActive()
    {
        if ((CAN_DAQ_VAR::ADDRESS_VALUE == _rtLogVarName) && _pCanDaqAddr)
        {
            return _manageCanDaq(CAN_DAQ_VAR::ADDRESS_VALUE, *_pCanDaqAddr);
        }
        else
        {
            return false;
        }
    }

private:
    uint32_t* _pCanDaqAddr;
    uint32_splitA _canDaqTraveler;      // Used to accumulate info between transmissions
    uint32_t _canDaqTravelerPrev;		// Used to accumulate info between transmissions
    const RTOS::ShortMsg* _pCanDaqDataMsg;
    CAN_DAQ_VAR _rtLogVarName;
    uint16_t _numSamples;
    bool _newDaqRequest;				// poor man's mutex

    bool _manageCanDaq(CAN_DAQ_VAR var, uint32_t data) { return true; }
};

namespace OE
{
    extern CanDaq canDaq;
}

#endif // INCLUDE_CAN_DAQ_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_STREAM_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_STREAM_H_

#ifdef __this_is_non_function_code_for_ea_sync__
typedef void ____;
#endif

// #include "CanEngineTypes.h"
// #include "Fifo.h"

#ifdef __cplusplus
extern "C"
{
    namespace CanEngine
    {
#endif
        // Handle to CAN Stream Controller
        typedef void* can_stream_h;

        // CAN Stream receive callback
        typedef void(*can_stream_rx_callback_t)(void* pParam);

        // Open and initialize a CAN Stream
        // param[in] canStream : handle to the CAN Stream Controller
        // param[in] can       : handle to the CAN Interface
        // param[in] peerId    : ID of the other end of the stream link
        // param[in] streamId  : stream ID
        // retval    OK        : CAN Stream is opened
        // retval    FAILE     : CAN Stream is not valid
        // note The CAN Stream Controller and buffer must be allocated by client
        DLL_EXPORT can_result_t OpenCanStream(can_stream_h canStream, can_interface_h can, uint16_t peerId, uint16_t streamId);

        // Set an optional receive callback of the CAN Stream.  When new data is received
        // from the CAN Stream, this client provided callback will be invoked
        // param[in] canStream : handle to the CAN Stream Controller
        // param[in] callback  : callback function pointer
        // param[in] pParam    : parameter of callback
        DLL_EXPORT void SetCanStreamRxCallback(can_stream_h canStream, can_stream_rx_callback_t callback, void* pParam);

        // Receive data from CAN Stream
        // param[in]  canStream : handle to the CAN Stream Controller
        // param[out] pBuf      : pointer to the receiving buffer
        // param[in]  maxLength : maximum number of bytes to receive
        // return     Actual received bytes
        DLL_EXPORT uint16_t CanStreamRx(can_stream_h canStream, uint8_t* pBuf, uint16_t maxLength);

        // Transmit data to CAN Stream
        // param[in]  canStream : handle to the CAN Stream Controller
        // param[out] pBuf      : pointer to the data buffer
        // param[in] length     : number of bytes to transmit
        // return    Actual transmitted bytes
        DLL_EXPORT uint16_t CanStreamTx(can_stream_h canStream, uint8_t* pBuf, uint16_t length);

        // Close the CAN stream
        // param[in]  canStream : handle to the CAN Stream Controller
        // note CAN stream can be re-opened after closing
        DLL_EXPORT void CloseCanStream(can_stream_h canStream);
        DLL_EXPORT void ClearCanStream(can_stream_h canStream);
        DLL_EXPORT uint16_t GetCanStreamRxFilled(can_stream_h canStream);
        DLL_EXPORT uint16_t GetCanStreamTxEmpty(can_stream_h canStream);

#ifdef __cplusplus
    }
}
#endif

#endif // INCLUDE_STREAM_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_CAN_ENGINE_INTERNAL_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CAN_ENGINE_INTERNAL_H_

#ifdef __cplusplus

// using namespace Lib;

extern "C"
{
    namespace CanEngine
    {
#endif

#define CAN_HEADER_GET_ID(header) ((header & CAN_ENGINE_MASK_ID) >> CAN_ENGINE_OFFSET_ID)
#define CAN_HEADER_SET_ID(header, id)  header = ((header & (~CAN_ENGINE_MASK_ID)) | ((uint32_t)id << CAN_ENGINE_OFFSET_ID))
#define CAN_HEADER_GET_SPEC(header) ((header & CAN_ENGINE_MASK_SPEC) >> CAN_ENGINE_OFFSET_SPEC)
#define CAN_HEADER_SET_SPEC(header, spec)  header = ((header & (~CAN_ENGINE_MASK_SPEC)) | ((uint32_t)spec << CAN_ENGINE_OFFSET_SPEC))
#define CAN_HEADER_GET_SRC(header) ((header & CAN_ENGINE_MASK_SRC) >> CAN_ENGINE_OFFSET_SRC)
#define CAN_HEADER_SET_SRC(header, src)  header = ((header & (~CAN_ENGINE_MASK_SRC)) | ((uint32_t)src << CAN_ENGINE_OFFSET_SRC))
#define CAN_HEADER_GET_TYPE(header) ((header & CAN_ENGINE_MASK_TYPE) >> CAN_ENGINE_OFFSET_TYPE)
#define CAN_HEADER_SET_TYPE(header, type)  header = ((header & (~CAN_ENGINE_MASK_TYPE)) | ((uint32_t)type << CAN_ENGINE_OFFSET_TYPE))
#define CAN_HEADER_SET_PRIO(header, prio) header = ((header & (~CAN_ENGINE_MASK_PRIO)) | ((uint32_t)prio << CAN_ENGINE_OFFSET_PRIO))
#define CAN_HEADER_GET_SEQ(header) ((header & CAN_ENGINE_MASK_SEQ) >> CAN_ENGINE_OFFSET_SEQ)
#define CAN_HEADER_SET_SEQ(header, seq)  header = ((header & (~CAN_ENGINE_MASK_SEQ)) | ((uint32_t)seq << CAN_ENGINE_OFFSET_SEQ))

#define INVALID_ID			(0)
#define CAN_PRIORITY_MAX	(0)
#define CAN_PRIORITY_HIGH	(1)
#define CAN_PRIORITY_NORMAL (2)
#define CAN_PRIORITY_LOW	(3)
#define REG_REQ_READ		(1)
#define REG_REQ_CHANGE		(2)
#define REG_UPDATE			(3)
#define EXG_REQ				(0)
#define EXG_RESP			(1)
#define MAX_LINK_LIST_LEN	(128)
#define RX_LIST_NUMBER		(4)
#define TX_LIST_LIMIT		(32)

        typedef uint32_t can_header_t;

        typedef struct
        {
            uint32_t linkedListError;
        }
        _can_diag_t;

        typedef struct
        {
            uint32_t currentByteCount;
            uint32_t nextSeqCount;
        }
        _can_muilti_packet_xfer_t;

        enum _can_ev_generator_state
        {
            STATE_EV_GEN_IDLE = 0,
            STATE_EV_GEN_TRANSMITTING
        };

        enum _can_ev_listener_state
        {
            STATE_EV_LSNR_IDLE = 0,
            STATE_EV_LSNR_RECEIVING
        };

        enum
        {
            STATE_EXG_REQ_IDLE = 0,
            STATE_EXG_REQ_PENDING_TX,
            STATE_EXG_REQ_TRANSMITTING,
            STATE_EXG_REQ_WAITING_RX,
            STATE_EXG_REQ_RECEIVING
        };

        enum
        {
            STATE_EXG_RESP_IDLE = 0,
            STATE_EXG_RESP_RECEIVING,
            STATE_EXG_RESP_WAITING_APP,
            STATE_EXG_RESP_PENDING_TX,
            STATE_EXG_RESP_TRANSMITTING
        };

        enum _can_reg_owner_state
        {
            STATE_REG_OWNER_IDLE = 0,
            STATE_REG_OWNER_RECEIVING,
            STATE_REG_OWNER_WAITING_APP_UPDATE,
            STATE_REG_OWNER_WAITING_APP_CHANGE,
            STATE_REG_OWNER_PENDING_TX,
            STATE_REG_OWNER_TRANSMITTING
        };

        enum _can_reg_client_state
        {
            STATE_REG_CLNT_STALE = 0,
            STATE_REG_CLNT_PENDING_REQ_UPDATE,
            STATE_REG_CLNT_PENDING_REQ_CHANGE,
            STATE_REG_CLNT_TRANSMITTING,
            STATE_REG_CLNT_REQED,
            STATE_REG_CLNT_RECEIVING,
            STATE_REG_CLNT_FRESH
        };

        typedef enum
        {
            CMO_TYPE_EV_GEN,
            CMO_TYPE_EV_LSNR,
            CMO_TYPE_EX_REQ,
            CMO_TYPE_EX_RESP,
            CMO_TYPE_REG_OWN,
            CMO_TYPE_REG_CLNT,
            CMO_TYPE_STREAM
        }
        _comm_obj_type_t;

        typedef enum
        {
            CMO_FAULT_NONE,
            CMO_FAULT_PACKET_TIMEOUT,
            CMO_FAULT_XFER_TIMEOUT,
            CMO_FAULT_TX,
            CMO_FAULT_MISSING_XN
        }
        _comm_obj_fault_t;

#define STREAM_RX_BUF_SIZE (256)
#define STREAM_TX_BUF_SIZE (256)
#define MAX_TX_PACKETS_FOR_STREAM (128)

        typedef struct
        {
            uint32_t overrunCounter;
        }
        _can_strm_diag_t;

        typedef struct
        {
            can_interface_h hCanInf;
            uint16_t _streamId;
            uint16_t _peerId;
            can_stream_rx_callback_t _rxCallback;
            void* _pParam;
            uint8_t _seq;
            uint8_t _rxFifo[STREAM_RX_BUF_SIZE];
            uint8_t _txFifo[STREAM_TX_BUF_SIZE];
            can_stream_h _nextStream;
            _can_strm_diag_t _diag;
        }
        _can_strm_t;

        typedef struct
        {
            uint32_t packetCounter;             // Packet received
            uint32_t packetError;               // Packet error
            uint32_t xferCounter;               // Transfer completed
            uint32_t xferError;                 // Transfer broken
            uint32_t xnCounter;                 // Transaction completed
            uint32_t xnError;                   // Transaction broken
            _comm_obj_fault_t _lastFault;
        }
        _comm_obj_diag_t;

        // Communication Object is the abstraction of the actual
        // transaction controller, such as an Event Generator, or a Registered Client
        typedef struct
        {
            _comm_obj_type_t _type;
            uint16_t _id;
            uint8_t _peerNode;
            uint8_t _priority;
            uint8_t _state;
            _can_muilti_packet_xfer_t _multi;
            void* _pAppBuf;						// This buffer holds the latest and completed received buffer
            void* _pRxBuf;						// This is the working buffer to receive data
            uint32_t _rxLength;					// Size of ref _pAppBuf and ref _pRxBuf
            void* _pTxBuf;
            uint32_t _txLength;
            can_callback_t _rxCallback;
            void* _rxCallbackParam;
            can_callback_t _txCallback;
            void* _txCallbackParam;
            can_callback_t _periodicalCallback;
            void* _periodicalCallbackParam;
            can_callback_t _faultCallback;
            void* _faultCallbackParam;
            void* _rxNext;
            void* _txNext;
            void* _timerNext;
            void* _periodicalNext;
            uint32_t _periodicalTimerVal;
            uint32_t _xferTimeoutVal;
            uint32_t _freshTimeoutVal;
            uint32_t _lastPeriodicalTime;
            uint32_t _lastXferTime;
            uint32_t _lastPacketTime;
            void* pCanInf;
            void* hMailbox;
            _comm_obj_diag_t _diag;
        }
        _comm_obj_t;

        typedef struct
        {
            uint32_t _mask;
            uint32_t _filter;
            uint8_t _number;  // Mailbox number returned from the CAN hardware
            _comm_obj_t* _pRxListHeaders[RX_LIST_NUMBER];
            void* _next;
            can_interface_h _hCanInf;
        }
        _mailbox_t;

        typedef struct
        {
            uint32_t _timeStamp;
            const can_hardware_api_t* _pHwApi;
            can_hardware_h _canHardware;
            uint16_t _selfId;
            can_fault_callback_t _faultCallback;
            _mailbox_t* _pMailboxListHeader;
            _comm_obj_t* _pTxListHeader;
            _comm_obj_t* _pTimerListHeader;
            _comm_obj_t* _pPeriodicalListHeader;
            uint8_t _streamMailboxNum;
            _can_strm_t* _pStreamListHeader;
            uint32_t _packetTimeoutVal;
            _can_diag_t _diag;
        }
        _can_inf_t;

        void _MultiPacketReset(_can_muilti_packet_xfer_t* pMulti);
        can_result_t _RxMultiPacketTransfer(void* pBuf, uint32_t length, _can_muilti_packet_xfer_t* pMulti, const can_packet_t* pMsg);
        void _PrepareTxHeader(_comm_obj_t* pCommObj, can_packet_t* pPacket);
        can_result_t _TxMultiPacketTransfer(_can_inf_t* pCanInf, _comm_obj_t* pCommObj, can_packet_t* pMsg);
        can_result_t AddCommObjToMailbox(_comm_obj_t* pCommObj);
        can_result_t RequestTx(_comm_obj_t* pCommObj, const void* pData, uint16_t length);
        void ResetDiag(_comm_obj_diag_t* pDiag);
        can_result_t InitCommObj(_comm_obj_type_t type, _comm_obj_t* pCommObj, const can_comm_obj_config_t* pConfig);
        void _ProcessRxStream(_can_inf_t* pCanInf);
        void _ProcessTxStream(_can_inf_t* pCanInf);
        void Process(_can_inf_t* pCanInf);
        can_result_t AddToPeriodicalHandler(_comm_obj_t* pCommObj);

#ifdef __cplusplus
    }
}
#endif

#endif // INCLUDE_CAN_ENGINE_INTERNAL_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_ICANINTERFACE_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_ICANINTERFACE_H_

namespace CanEngine
{
    // Forward declaration
    class IMailbox;
    class ICommObj;

    // CanEngine:ICanHardware Class
    // Interface to CAN hardware, must be implemented by client
    class ICanHardware
    {
    public:
        // return handle to the CAN hardware
        // note Usually this is the pointer to the concrete class itself
        virtual can_hardware_h GetHandle() = 0;

        // Get the API structure for CAN Engine Core
        // return pointer to the API structure
        virtual const can_hardware_api_t* GetApi() = 0;
    };

    // CanEngine:ICanInterfac Class
    // Interface to CAN Interface
    class ICanInterface
    {
    public:
        // Initialize CAN interface with the CAN hardware
        // param[in]  canHw   : reference to the CAN hardware Interface
        // param[in]  selfId  : Node ID of this CAN node
        // retval     OK      : initialization completed
        // retval     FAILED  : initialization failed
        virtual can_result_t Init(ICanHardware& canHw, uint16_t selfId) = 0;

        // Close the CAN interface
        virtual void Close() = 0;

        // Process the CAN interface
        // param[in] timestamp : timestamp with millisecond resolution
        // retval    OK        : process completed
        virtual can_result_t Process(uint32_t timestamp) = 0;

        // Reset the CAN interface
        virtual void Clear() = 0;

        // Get the handle of the interface to be used in CAN Core
        // return handle to the CAN Interface
        virtual can_interface_h GetHandle() = 0;
    };

    // CanEngine::IMailbox Class
    // Interface to CAN Mailbox
    class IMailbox
    {
    public:
        // Initialize the CAN mailbox
        // param[inout] canInf    : reference to the CAN interface this mailbox will be attached to
        // param[in]    mask      : CAN filter mask (0-don't care, 1-care)
        // param[in]    filter    : CAN filter
        // retval       OK        : initialization complete
        // retval       FAILED    : initialization failed
        virtual can_result_t Init(ICanInterface& canInf, uint32_t mask, uint32_t filter) = 0;

        // Get the handle to the mailbox for CAN Core
        // return handle to the mailbox
        virtual can_mailbox_h GetHandle() = 0;
    };

    // CanEngine::ICommObj Class
    // Interface to Communication Object
    class ICommObj
    {
    public:
        // Set the optional periodical transmission
        // param[in]  period    : period time in millisecond
        // param[in]  callback  : callback function to be invoked when this timer fires
        // param[in]  param     : parameter to be used for the callback invocation
        // retval     OK        : set completed
        // retval     FAILED    : set failed
        virtual can_result_t SetPeriodicalTx(uint32_t period, can_callback_t callback, void* param) = 0;

        // Set the optional transaction timeout value
        // param[in]  timeout   : period time in millisecond
        // retval     OK        : set completed
        // retval     FAILED    : set failed
        // note If this value is not set, the default value will be used
        virtual can_result_t SetTransactionTimeout(uint32_t timeout) = 0;

        // Set the optional fault handling callback
        // param[in]  callback  : callback function to be invoked when the fault is detected
        // param[in]  param     : parameter to be used for the callback invocation
        // retval     OK        : set completed
        // retval     FAILED    : set failed
        // note If this callback is not set, the communication object will simply be reset
        //without any further actions
        virtual can_result_t SetFaultHandler(can_callback_t callback, void* param) = 0;

        // Set the RX callback.  This will be called when the entire transfer is received
        // param[in]  callback  : callback function to be invoked when the fault is detected
        // param[in]  param     : parameter to be used for the callback invocation
        // retval     OK        : set completed
        // retval     FAILED    : set failed
        // note For following communication object, the RX callback must be set
        // * Event Listener
        // * Exchange Requester and Responder
        // * Register Owner and Client
        virtual can_result_t SetRxCallback(can_callback_t callback, void* param) = 0;

        // Set the optional TX callback
        // param[in]  callback  : callback function to be invoked when the fault is detected
        // param[in]  param     : parameter to be used for the callback invocation
        // retval     OK        : set completed
        // retval     FAILED    : set failed
        // note The callback will be invoked after the entire transfer is transmitted
        virtual can_result_t SetTxCallback(can_callback_t callback, void* param) = 0;

        // Get the handle to the communication object for CAN Engine Core
        // return handle to the communication object
        virtual comm_obj_h GetHandle() = 0;

        // Get the RX Buffer for application to read the data
        // return pointer to the RX Buffer
        virtual void* GetRxBuffer() = 0;

        // Get the TX Buffer for application to access the data
        // return pointer to the TX Buffer
        // note There is no reason for the application to directly use this interface
        // Each specific comm obj has the transmission method
        virtual void* GetTxBuffer() = 0;
    };

    // CanEngine:: IEventListener Class
    // Interface to CAN Event Listener
    class IEventListener
    {
    public:
        // Initialize the Event Listener
        // param[in]  config : configuration structure
        // retval     OK     : initialization completed
        // retval     FAILED : initialization failed
        virtual can_result_t Init(const can_comm_obj_config_t& config) = 0;
    };

    // CanEngine::IEventGenerator Class
    // Interface to CAN Event Generator
    class IEventGenerator
    {
    public:
        // Transmit an event
        // param[in]  pData  : pointer to the new event data
        // param[in]  length : length of the event data, must be same as event size
        // note The content of the pData will be copied to TX buffer before return
        virtual can_result_t Send(void* pData, uint16_t length) = 0;
    };

    // CanEngine::IRegClient Class
    // Interface to Register Client
    class IRegClient
    {
    public:
        // Initialize the Register Client
        // param[in]  config : configuration structure
        // retval     OK     : initialization completed
        // retval     FAILED : initialization failed
        virtual can_result_t Init(const can_comm_obj_config_t& config) = 0;

        // Set the optional fresh timer value
        // param[in]  mSec   : fresh timer value in millisecond
        // retval     OK     : set completed
        // retval     FAILED : set failed
        // note If this value is not set, the register value stays "fresh" after first receiving
        virtual can_result_t SetRefreshTiming(uint32_t mSec) = 0;

        // Read the register value
        // param[out] pData  : buffer to which the register value will be copied to
        // param[in]  length : length of the buffer, must be the same as the register size
        // retval     OK     : read completed, value in buffer
        // retval     PENDING: a request has been sent to Register Owner, buffer is not used
        // retval     FAILED : read failed
        virtual can_result_t Read(void* pData, uint16_t length) = 0;

        // Send a change request to the Register Owner
        // param[in]  pData  : buffer which has the proposed change value
        // param[in]  length : length of the buffer, must be the same as the register size
        // retval     OK     : request is sent
        // retval     FAILED : request failed
        virtual can_result_t RequestToChange(void* pData, uint16_t length) = 0;

        // Send an update request to the Register Owner
        // retval     OK     : request is sent
        // retval     FAILED : request failed
        virtual can_result_t RequestToUpdate() = 0;
    };

    // CanEngine::IRegOwner Class
    // Interface to Register Owner
    class IRegOwner
    {
    public:
        // Initialize the Register Owner
        // param[in]  config : configuration structure
        // retval     OK     : initialization completed
        // retval     FAILED : initialization failed
        virtual can_result_t Init(const can_comm_obj_config_t& config) = 0;

        // Transmit the updated register value
        // param[in]  pData  : buffer holding the latest register value
        // param[in]  length : length of the buffer, must be the same as the register size
        // retval     OK     : transmission queued
        // retval     FAILED : transmission failed
        virtual can_result_t Update(const void* pData, uint16_t length) = 0;
    };

    // CanEngine::ICanStream Class
    // Interface to CAN Streamer
    class ICanStream
    {
    public:
        // Initialize the Streamer
        // param[inout] canInf   : reference to the CAN Interface the streamer will be attached to
        // param[in]    peerId   : node ID to the other node
        // param[in]    streamId : stream ID
        // retval     OK     : initialization completed
        // retval     FAILED : initialization failed
        virtual can_result_t Init(ICanInterface& canInf, uint16_t peerId, uint16_t streamId) = 0;

        // Set the optional RX callback.
        // param[in]  callback  : callback function to be invoked when new data is received at the incoming queue
        // param[in]  pParam    : parameter to be used for the callback invocation
        // retval     OK        : setup completed
        // retval     FAILED    : setup failed
        virtual void SetRxCallback(can_stream_rx_callback_t callback, void* pParam) = 0;

        // Get data from the incoming queue
        // param[out]  pBuf      : buffer to which the received data will be copied to
        // param[in]   maxLength : maximum number of bytes to receive
        // return number of bytes actually received
        virtual uint16_t Rx(uint8_t* pBuf, uint16_t maxLength) = 0;

        // Send data to the outgoing queue
        // param[in]  pBuf      : buffer which contains data to be sent
        // param[in]  length    : number of bytes to send
        // return number of bytes actually sent
        virtual uint16_t Tx(uint8_t* pBuf, uint16_t length) = 0;

        // Close the streamer
        virtual void Close() = 0;
        virtual void Reset() = 0;
        virtual uint16_t GetAvailableRxBytes() = 0;
        virtual uint16_t GetAvailableTxBytes() = 0;
    };
}
#endif // INCLUDE_ICANINTERFACE_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_CPP_CANENGINE_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CPP_CANENGINE_H_

namespace CanEngine
{
    // CanEngine::CanInterface Class
    class CanInterface : public ICanInterface
    {
    public:
        CanInterface() :
            _canInf(),
            _handle(),
            _pCanHw()
        { }

        virtual ~CanInterface() { }

        virtual can_result_t Init(ICanHardware& canHw, uint16_t selfId) override { return OK; }
        virtual void Close() override { }
        virtual can_result_t Process(uint32_t timestamp) override { return OK; }
        virtual void Clear() override { }
        virtual can_interface_h GetHandle() override { return _handle; }

    private:
        can_interface_h _handle;
        _can_inf_t _canInf;
        ICanHardware* _pCanHw;
    };
}

#endif // INCLUDE_CPP_CANENGINE_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_COMM_OBJ_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_COMM_OBJ_H_

namespace CanEngine
{
    // CanEngine::CommObjBuf Class
    // Template of Communication Object buffer
    // param rxSize : Size of Application and RX buffer
    // param txSize : Size of TX buffer
    template<size_t rxSize, size_t txSize>
    struct CommObjBuf
    {
        uint8_t appBuffer[rxSize];
        uint8_t rxBuffer[rxSize];
        uint8_t txBuffer[txSize];

        // Get the pointer to Application buffer
        // return pointer to Applicaiton buffer
        void* GetRxBuffer()
        {
            return (void*)appBuffer;
        }

        // Get the pointer to transmission buffer
        // return pointer to transmission buffer
        void* GetTxBuffer()
        {
            return (void*)txBuffer;
        }
    };

    // CanEngine::CommObjBuf Structure
    // Specialized template for 0-RX and 0-TX buffer
    template <>
    struct CommObjBuf<0, 0>
    {
        uint8_t* appBuffer = nullptr;
        uint8_t* rxBuffer = nullptr;
        uint8_t* txBuffer = nullptr;

        void* GetRxBuffer()
        {
            return nullptr;
        }

        void* GetTxBuffer()
        {
            return nullptr;
        }
    };

    // CanEngine::CommObjBuf Structure
    // Specialized template for 0-TX buffer
    template <size_t rxSize>
    struct CommObjBuf<rxSize, 0>
    {
        uint8_t appBuffer[rxSize];
        uint8_t rxBuffer[rxSize];
        uint8_t* txBuffer = nullptr;

        void* GetRxBuffer()
        {
            return (void*)appBuffer;
        }

        void* GetTxBuffer()
        {
            return nullptr;
        }
    };

    // CanEngine::CommObjBuf Structure
    // Specialized template for 0-RX buffer
    template <size_t txSize>
    struct CommObjBuf<0, txSize>
    {
        uint8_t* appBuffer = nullptr;
        uint8_t* rxBuffer = nullptr;
        uint8_t txBuffer[txSize];

        void* GetRxBuffer()
        {
            return nullptr;
        }

        void* GetTxBuffer()
        {
            return (void*)txBuffer;
        }
    };

    // CanEngine::CommObj Class
    // Template for Communication Object with the buffers
    template<size_t rxSize, size_t txSize>
    class CommObj : public ICommObj
    {
    public:
        CommObj() :
            _commObj(),
            _buf()
        { }

        virtual ~CommObj() { }

        virtual can_result_t SetPeriodicalTx(uint32_t period, can_callback_t callback, void* param) override { return OK; }
        virtual can_result_t SetTransactionTimeout(uint32_t timeout) override { return OK; }
        virtual can_result_t SetFaultHandler(can_callback_t callback, void* param) override { return OK; }
        virtual can_result_t SetRxCallback(can_callback_t callback, void* param) override
        {
            _commObj._rxCallback = callback;
            _commObj._rxCallbackParam = param;
            return OK;
        }

        virtual can_result_t SetTxCallback(can_callback_t callback, void* param) override
        {
            _commObj._txCallback = callback;
            _commObj._txCallbackParam = param;
            return OK;
        }

        virtual comm_obj_h GetHandle() override
        {
            return (comm_obj_h)&_commObj;
        }

        virtual void* GetRxBuffer() override
        {
            return _buf.GetRxBuffer();
        }

        virtual void* GetTxBuffer() override
        {
            return _buf.GetTxBuffer();
        }

    protected:
        _comm_obj_t _commObj;             // Comm Obj structure from CAN core
        CommObjBuf<rxSize, txSize> _buf;  // Buffers
    };
}

#endif // INCLUDE_COMM_OBJ_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_EVENT_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_EVENT_H_

#ifdef __this_is_non_function_code_for_ea_sync__
typedef void ____;
#endif

#ifdef __cplusplus
extern "C"
{
    namespace CanEngine
    {
#endif
        // Event messages

        // Handle to a CAN event listener or writer
        typedef void* can_event_h;

        // Open and initialize a CAN event listener
        // The CAN event listener is the control object to listen to an specific event
        // When the event broadcast is received, the listener will be able to store its payload,
        // and invoke the callback.  The callback will be called right after the CAN packets are
        // processed.  So multiple instances of the same event will cause multiple callbacks
        // param[inout] hEv     : handle to the Event Listener
        // parma[in]    pConfig : pointer to the config structure
        // retval       OK      : initialization completed
        // retval       FAILED  : initialization failed due to invalid handle or config structure
        DLL_EXPORT can_result_t InitializeEvListner(can_event_h hEv, const can_comm_obj_config_t* pConfig);

        // Open and initialize a CAN event generator
        // param[inout] hEv     : handle to the Event Generator
        // parma[in]    pConfig : pointer to the config structure
        // retval       OK      : initialization completed
        // retval       FAILED  : initialization failed due to invalid handle or config structure
        DLL_EXPORT can_result_t InitializeEvGen(can_event_h hEv, const can_comm_obj_config_t* pConfig);

        // Generate a CAN event
        // This function will queue a CAN event for transmission.  The buffer
        // and event writer must be kept valid until the CAN event is transmitted
        // The client may provide callback
        // param[inout] hEv     : handle to the Event Generator
        // param[in]    pData   : pointer to the event data
        // param[in]    length  : length of the data
        // retval       OK      : event is in the TX queue
        // retval       FAILED  : failed due to invalid handle or data
        DLL_EXPORT can_result_t SendEvent(can_event_h hEv, const void* pData, uint16_t length);

#ifdef __cplusplus
    }
}
#endif

#endif // INCLUDE_EVENT_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_CPP_EVENTGENERATOR_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CPP_EVENTGENERATOR_H_

namespace CanEngine
{
    // CanEngine::EventGenerator Class
    template <size_t evSize>
    class EventGenerator : public CommObj<0, evSize>, public IEventGenerator
    {
    public:
        EventGenerator() { }

        virtual ~EventGenerator() { }

        virtual can_result_t Init(ICanInterface& canInf, IMailbox& mailbox, uint16_t id, can_callback_t callback, void* pParam)
        {
            return OK;
        }

        virtual can_result_t Init(ICanInterface& canInf, IMailbox& mailbox, uint16_t id)
        {
            return Init(canInf, mailbox, id, nullptr, nullptr);
        }

        can_result_t Send(void* pData, uint16_t length) override { return OK; }

        can_result_t Send()
        {
            return Send(nullptr, 0);
        }
    };
}

#endif // INCLUDE_CPP_EVENTGENERATOR_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_REGISTER_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_REGISTER_H_

#ifdef __this_is_non_function_code_for_ea_sync__
typedef void ____;
#endif

// #include "CanEngineTypes.h"

#ifdef __cplusplus
extern "C"
{
    namespace CanEngine
    {
#endif
        // Handle to CAN Register Writer
        typedef void* can_register_owner_h;

        // Handle to CAN Register Reader
        typedef void* can_register_client_h;

        // Open and initialize a CAN Register Owner
        // param[inout] hRegOwner : handle to the register owner to be configured
        // param[in]    pConfig   : pointer to the config structure
        // retval       OK        : initialization completed
        // retval       FAILED    : failed
        DLL_EXPORT can_result_t InitializeRegOwner(can_register_owner_h hRegOwner, const can_comm_obj_config_t* pConfig);

        // Provide new values for the CAN Register
        // param[inout] hRegOwner : handle to the register owner
        // param[in]    pNewValue : new value of the register
        // param[in]    length    : length of the new value, must match the register size
        // retval       OK        : the transmission is queued
        // retval       FAILED    : the request failed
        // note If the previous transmission is not completed, it will fail to send another update.
        DLL_EXPORT can_result_t SendRegUpdate(can_register_owner_h hRegOwner, const void* pNewValue, uint16_t length);

        // Open and initialize a CAN Register Client
        // param[inout] hRegClient: handle to the register client to be configured
        // param[in]    pConfig   : pointer to the config structure
        // retval       OK        : initialization completed
        // retval       FAILED    : failed
        DLL_EXPORT can_result_t InitializeRegClient(can_register_client_h hRegClient, const can_comm_obj_config_t* pConfig);

        // Manually read the CAN Register value
        // If the refresh timeout is set, when the value is still "fresh", the value
        // will be available right away.  Otherwise, a request will be queued, and
        // the client will get the value through callback (if listener callback is set),
        // or needs to read it again later
        // param[inout] hRegClient  : handle to the register client communication object
        // param[out] pData         : buffer to receive the register value
        // param[in]  length        : length of the expected data, must to be the same as the register's size
        // retval     OK            : the data is read into the provided buffer
        // retval     PENDING       : the request is sent to Register Owner, waiting for its update
        // retval     FAILED        : the reading failed
        // note When the register reading is PENDING, the pData buffer is not used at all.  Application must
        // read the register value again after receiving the callback
        DLL_EXPORT can_result_t ReadRegValue(can_register_client_h hRegClient, void* pData, uint16_t length);

        // Client requests to change the register value
        // param[inout] hRegClient  : handle to the register client communication object
        // param[out] pNewValue     : buffer to the proposed new register value
        // param[in]  length        : length of the expected data, must to be the same as the register's size
        // retval     OK            : the request is sent to Register Owner
        // retval     FAILED        : the request failed
        // note Application must read the register value after receiving the callback
        DLL_EXPORT can_result_t RequestRegChange(can_register_client_h hRegClient, const void* pNewValue, uint16_t length);

        // Client requests the owner to send the latest register value
        // param[inout] hRegClient  : handle to the register client communication object
        // retval     OK            : the request is sent to Register Owner
        // retval     FAILED        : the request failed
        DLL_EXPORT can_result_t RequestRegUpdate(can_register_client_h hRegClient);

        // Owner reads the register values proposed by the Client
        // param[inout] hRegOwner   : handle to the register owner communication object
        // param[out]   pData       : buffer to receive the proposed register value
        // param[in]    length      : length of the expected data, must to be the same as the register's size
        // retval       OK          : proposed values is read into the buffer
        // retval       FAILED      : the reading failed
        DLL_EXPORT can_result_t GetRegRequestToChangeValue(can_register_owner_h hRegOwner, void* pData, uint16_t length);

        // Set an optional refresh timeout on a CAN Register Reader.  After it times out,
        // an automatic request will be sent if the client tries to read this register
        // param[inout] hCommObj    : handle to the register client
        // param[in]    freshTime   : fresh value timeout in millisecond
        // retval       OK          : the timer is setup
        // retval       FAILED      : failed
        DLL_EXPORT can_result_t SetFreshTimeout(can_register_client_h hCommObj, uint32_t freshTime);

#ifdef __cplusplus
    }
}
#endif

#endif // INCLUDE_REGISTER_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_CPP_REGOWNER_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CPP_REGOWNER_H_

namespace CanEngine
{
    // CanEngine::RegOwner Class
    template<size_t regSize>
    class RegOwner : public CommObj<regSize, regSize>, public IRegOwner
    {
    public:
        RegOwner() { }

        virtual ~RegOwner() { }

        virtual can_result_t Init(ICanInterface& canInf, IMailbox& mailbox, uint16_t id) { return OK; }
        virtual can_result_t Update(const void* pData, uint16_t length) { return OK; }

    private:
        can_result_t Init(const can_comm_obj_config_t& config) final { return FAILED; }
    };
}

#endif // INCLUDE_CPP_REGOWNER_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_CPP_REGCLIENT_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CPP_REGCLIENT_H_

namespace CanEngine
{
    // CanEngine::RegClient Class
    template<size_t regSize>
    class RegClient : public CommObj<regSize, regSize>, public IRegClient
    {
    public:
        RegClient() { }

        virtual ~RegClient() { }

        virtual can_result_t Init(ICanInterface& canInf, IMailbox& mailbox, uint16_t id) { return OK; }
        can_result_t SetRefreshTiming(uint32_t mSec) override { return OK; }
        virtual can_result_t Read(void* pData, uint16_t length) override { return OK; }
        virtual can_result_t RequestToChange(void* pData, uint16_t length) override { return OK; }
        virtual can_result_t RequestToUpdate() override { return OK; }

    private:
        can_result_t Init(const can_comm_obj_config_t& config) final { return FAILED; }
    };
}

#endif // INCLUDE_CPP_REGCLIENT_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_EXCHANGE_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_EXCHANGE_H_

#ifdef __this_is_non_function_code_for_ea_sync__
typedef void ____;
#endif

// #include "CanEngineTypes.h"

#ifdef __cplusplus
extern "C"
{
    namespace CanEngine
    {
#endif
        // Handle to CAN Exchange Requester
        typedef void* can_exchange_requester_h;

        // Handle to CAN Exchange Responder
        typedef void* can_exchange_responder_h;

        // Initialize the Exchange Requester communication object from a configuration structure
        // param[inout] hExReq  : handle to Exchange Requester object
        // parma[in]    pConfig : pointer to comm. obj configuration structure
        // retval       OK      : initialization completed
        // retval       FAILED  : initialization failed due to invalid handle or config structure
        DLL_EXPORT can_result_t InitializeExgReq(can_exchange_requester_h hExReq, const can_comm_obj_config_t* pConfig);

        // Send a request to the Exchange Requester
        // param[inout] hExReq  : handle to Exchange Requester object
        // parma[in]    pData   : pointer to new request data
        // parma[in]    length  : size of the request data, must be the same as Exchange Requester's setting
        // retval       OK      : request is in the TX queue
        // retval       FAILED  : failed due to invalid handle or data
        DLL_EXPORT can_result_t SendExgRequest(can_exchange_requester_h hExReq, const void* pData, uint16_t length);

        // Read a response from the Exchange Requester
        // param[inout] hExReq  : handle to Exchange Requester object
        // parma[in]    pData   : pointer to new request data
        // parma[in]    length  : size of the request data, must be the same as Exchange Requester's setting
        // retval       OK      : request is in the TX queue
        // retval       FAILED  : failed due to invalid handle or data
        DLL_EXPORT can_result_t ReadExgResponse(can_exchange_requester_h hExReq, void* pData, uint16_t length);

        // Initialize the Exchange Responder communication object from a configuration structure
        // param[inout] hExResp : handle to Exchange Responder object
        // parma[in]    pConfig : pointer to comm. obj configuration structure
        // retval       OK      : initialization completed
        // retval       FAILED  : initialization failed due to invalid handle or config structure
        DLL_EXPORT can_result_t InitializeExgResp(can_exchange_responder_h hExResp, const can_comm_obj_config_t* pConfig);

        // Read a request from the Exchange Requester
        // param[inout] hExReq  : handle to Exchange Requester object
        // parma[in]    pData   : pointer to new request data
        // parma[in]    length  : size of the request data, must be the same as Exchange Requester's setting
        // retval       OK      : request is in the TX queue
        // retval       FAILED  : failed due to invalid handle or data
        can_result_t ReadExgRequest(can_exchange_responder_h hExResp, void* pData, uint16_t length);

        // Send a response from the Exchange Responder
        // note This can only be called after receiving the request
        // param[inout] hExResp : handle to Exchange Responder object
        // parma[in]    pData   : pointer to new request data
        // parma[in]    length  : size of the request data, must be the same as Exchange Requester's setting
        // retval       OK      : response is in the TX queue
        // retval       FAILED  : failed due to invalid handle or data
        DLL_EXPORT can_result_t SendExgResponse(can_exchange_responder_h hExResp, const void* pData, uint16_t length);

#ifdef __cplusplus
    }
}
#endif

#endif // INCLUDE_EXCHANGE_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_CPP_EXGRESPONDER_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CPP_EXGRESPONDER_H_

namespace CanEngine
{
    // CanEngine::ExgResponder Class
    template<size_t reqSize, size_t respSize>
    class ExgResponder : public CommObj<reqSize, respSize>
    {
    public:
        ExgResponder() { }

        virtual ~ExgResponder() { }

        virtual can_result_t Init(ICanInterface& canInf, IMailbox& mailbox, uint16_t peerId, uint16_t id) { return OK; }
        virtual can_result_t ReadRequest(void* pData, uint16_t length) { return OK; }
        virtual can_result_t Respond(const void* pData, uint16_t length) { return OK; }
    };
}

#endif // INCLUDE_CPP_EXGRESPONDER_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_MAILBOX_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_MAILBOX_H_

namespace CanEngine
{
    // CanEngine::Mailbox Class
    class Mailbox : public IMailbox
    {
    public:
        Mailbox() :
            _mailbox()
        { }

        virtual ~Mailbox() { }

        virtual can_result_t Init(ICanInterface& canInf, uint32_t mask, uint32_t filter) override
        {
            return OK;
        }

        virtual can_interface_h GetHandle() override
        {
            return (can_mailbox_h)&_mailbox;
        }

    protected:
        _mailbox_t _mailbox;
    };
}

#endif // INCLUDE_MAILBOX_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_VERSIONINFO_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_VERSIONINFO_H_

// VersionInfo Class
class VersionInfo
{
public:
    // Constructor
    // Method to initialize and configure the VersionInfo class
    VersionInfo() :
        major(1),
        minor(0),
        crc32(0xA0A0A0A0),
        appName("OeParameters")
    { }

    // Destructor
    ~VersionInfo() { }

    // Get the Major Version Number
    // Return:         16 bit major version number
    uint16_t GetMajorVer() { return major; }

    // Get the Minor Version Number
    // Return:         16 bit minor version number
    uint16_t GetMinorVer() { return minor; }

    // Get the CRC Number
    // Return: 32 bit CRC number
    uint32_t GetCRC32() { return crc32; }

    // Get the app name pointer
    // Return: String containing app name
    const char* GetAppName() { return appName; }

    const uint16_t major;   // Major version number
    const uint16_t minor;   // Minor version number
    const uint32_t crc32;   // CRC of the software image
    const char* appName;    // Name string of the application
};

#endif // INCLUDE_VERSIONINFO_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_COMMUNICATION_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_COMMUNICATION_H_

// Forward declare
namespace UI
{
    class LED;
}

class ProtocolInterface;
class VersionInfo;

// Communication Class
// Common Communication Task
class Communication : public RTOS::Task<160>, public IServiceable
{
public:
    // Message queue to access Communication task
    RTOS::MQ q;

    // Communication Constructor
    Communication() :
        _diag(),
        _nodeInfo(),
        _pAppVerInfo(),
        _pCtrlQ(),
        _pFaultQ(),
        _pLed()
    { }

    // Set the incoming queue of the control task
    // param[in]   controlQ : reference to the queue
    void SetControlQ(RTOS::MQ& controlQ);

    // Set the incoming queue of the fault task
    // param[in]   faultQ : reference to the queue
    void SetFaultQ(RTOS::MQ& faultQ);

    // Set the version info for Node Info
    // param[in]   verInfo : reference to the version info object
    void SetVersion(VersionInfo& verInfo);

    const static RTOS::Ev evSendGoSafe;		// Event for sending go-safe CAN event
    const static RTOS::Ev evNodeInfo;		// Event for sending Node information, version etc.
    const static RTOS::ShortMsg msgFault;	// Message for sending fault bitmap
    const static RTOS::Ev evCheckSnoCrc;

    // Set an LED
    void SetLed(UI::LED& led) { }

    // Enable the communication task
    virtual void Enable() { }

    // Disable the communication task
    virtual void Disable() { }

    // Get the Node ID for CAN communications, pure virtual
    // return CAN Node ID
    virtual NXGen::NODE_ID GetNodeId() = 0;

    // Actions before task loop
    void BeforeLoop() override { }

    // Actions of the task loop
    void Loop() override { }

    // Get the command line interface port as a serial port
    // return   Reference to the interface of the serial port
    ISerialPort& GetCliPort();

    static void _DaqRequestCallback(void* pObj) { }

    // Process the service commands, so this task is "serviceable"
    // return If the incoming command being accepted
    bool ProcessGeneralServiceCmd(IServiceCmd& cmd) override { return true; }

    const char* GetGeneralServiceHelp() override
    {
        static const char* helpInfo =
            "COMMSTAT    :  Get communication status\n";

        return helpInfo;
    }

    // Get the system state, such as IDLE, CALIBRATING, etc
    // return System state
    NXGen::SYSTEM_STATE GetSystemState();

protected:
    RTOS::MQ* _pCtrlQ;												// pointer to control task queue
    RTOS::MQ* _pFaultQ;												// pointer to fault task queue
    UI::LED* _pLed;													// Pointer to the communication action LED
    VersionInfo* _pAppVerInfo;										// Pointer to the app version info
    NXGen::NODE_INFO _nodeInfo;
    CanEngine::CanInterface _canInf;								// CAN Engine
    CanEngine::Mailbox _canMailboxEv;								// CAN Mailbox for CAN events
    CanEngine::Mailbox _canMailboxExg;								// CAN Mailbox for CAN events
    CanEngine::Mailbox _canMailboxReg;								// CAN Mailbox for CAN registers
    CanEngine::Mailbox _canMailboxStream;							// CAN Mailbox for CAN streams	

    CanEngine::RegClient<sizeof(uint8_t)> _canRegSystemState;	    // CAN register client to receive system state
    CanEngine::RegClient<sizeof(uint32_t)> _canRegTimestamp;	    // CAN register client to receive timestamp
    CanEngine::RegClient<sizeof(uint8_t)> _canRegPowerStatus;	    // CAN register client to receive power status
    CanEngine::RegClient<sizeof(uint64_t)> _canRegDaqRequest;	    // TS- CAN register owner to receive DAQ Request Message

    CanEngine::RegOwner<sizeof(uint64_t)> _canRegDaqData;			// TS- CAN register client to send DAQ Data Message
    CanEngine::RegOwner<sizeof(uint32_t)> _canRegFaults;			// Need to be initialized specifically by derived class
    CanEngine::RegOwner<sizeof(NXGen::NODE_INFO)> _canRegNodeInfo;	// Need to be initialized specifically by derived class

    static void _GoSafeCallback(void* pObj);						// Callback when receiving go-safe
    static void _ClearFaultCallback(void* pObj);					// Callback when receiving fault clear
    static void _SystemStateCallback(void* pObj);					// Callback when receiving system state
    static void _TimestampCallback(void* pObj);						// Callback when receiving timestamp
    static void _PowerStatusCallback(void* pObj);					// Callback when receiving power status
    static void _StreamCliCallback(void* pObj);						// Callback when CLI stream has incoming bytes

    // Initialize more node specific CAN comm. objects
    virtual CanEngine::can_result_t _NodeCommObjInit() = 0;

    // Handle the node specific events and messages coming from the queue
    virtual void _HandleExtraMsg(const IMqItem* id, const MsgObj& msg) { }

    // Diagnostics counters
    struct _Diag
    {
        uint32_t txCnt;
        uint32_t rxCnt;
    }
    _diag;

    // Communication task period
    enum { COMM_TASK_PERIOD = 20 };

    // Activate the comm action indicator
    void _FlashLed() const;
};

#endif // INCLUDE_COMMUNICATION_H_

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_OCT_PINS_H_
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_OCT_PINS_H_

namespace OCT
{
    struct OctPins
    {
        GPIO_CARD_UPDATE_        gpioCardUpdate;
        GPIO_PWR_24V_OK          gpioPwr24vOk;
        GPIO_PWR_5V_OK           gpioPwr5vOk;
        GPIO_PWR_BAT_OK          gpioPwrBatOk;
        GPIO_PWR_LINE_OK         gpioPwrLineOk;
        GPIO_PWR_ON              gpioPwrOn;
        GPIO_PWR_CARD_OK         gpioPwrCardOk;
        GPIO_RUN_LED_0_          ledRun0;
        GPIO_RUN_LED_1_          ledRun1;
        GPIO_CARD_LED0           ledCard0;
        GPIO_CARD_LED1           ledCard1;
        GPIO_CARD_LED2           ledCard2;
        GPIO_CARD_LED3           ledCard3;
        GPIO_CARD_PRESENT1_      gpioCardPresent1;
        GPIO_CARD_PRESENT2_      gpioCardPresent2;
        GPIO_EM_DIAG_TEST        gpioEmDiagTest;
        CARD_ID                  cardId;
        PWR_STATE                pwrState;
        BOARD_ID                 boardId;
        BUILD_ID                 buildId;
        ILOCK_PINS               ilockPins;
        CAN_PINS                 canPins;
        GPIO_VCC_5V0_PG          gpioVcc5V0Pg;
        GPIO_VCC_3V3_PG          gpioVcc3V3Pg;
        GPIO_VCC_2V5_PG          gpioVcc2V5Pg;
        GPIO_VCC_1V5_PG          gpioVcc1V5Pg;
        GPIO_VCC_1V15_PG         gpioVcc1V15Pg;
        GPIO_VCC_1V1_PG          gpioVcc1V1Pg;
        GPIO_VCC_5V0_EN          gpioVcc5v0En;
        GPIO_VCC_3V3_EN          gpioVcc3v3En;
        GPIO_VCC_2V5_EN          gpioVcc2v5En;
        GPIO_VCC_1V5_EN          gpioVcc1v5En;
        GPIO_VCC_1V15_EN         gpioVcc1v15En;
        GPIO_VCC_1V1_EN          gpioVcc1v1En;
        GPIO_VCC_0V75_EN         gpioVcc0v75En;
        GPIO_VCC_3V3A_EN         gpioVcc3v3aEn;
        GPIO_VCC_3V3C_EN         gpioVcc3v3cEn;
        GPIO_VCC_2V5A_EN         gpioVcc2v5aEn;
        GPIO_VCC_1V25_EN         gpioVcc1v25En;
        SPI_PINS                 spiPins;
        I2C1_PINS                i2c1Pins;
        UART1_PINS               uart1Pins;
        UART3_PINS               uart3Pins;
    };

    // OCT::OctFault Class
    class OctFault : public FaultMonitor
    {
    public:
        static OctFault* Init(OctPins& pins)
        {
            static OctFault of(pins);
            _pFault = &of;
            return _pFault;
        }

        static OctFault* GetOctFault() { return _pFault; }
        static void qFaultTask() { }
        void Reset3SecCommMsgTmr() { }

    private:
        OctFault(OctPins& pins) :
            FaultMonitor(pins.ilockPins, pins.gpioEmDiagTest, pins.ledCard3) { }

        static OctFault* _pFault;
    };
}

#endif // INCLUDE_OCT_PINS_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_OE_COMM_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_OE_COMM_H_

namespace OE
{
    // OE::OeComm Class
    // OE Communication Task (singleton)
    class OeComm : public Communication
    {
    public:
        const static RTOS::Ev evDelaylineDone;
        const static RTOS::ShortMsg msgDelayline;
        const static RTOS::ShortMsg msgLaserStat;
        const static RTOS::ShortMsg msgWriteMapFileResp;
        const static RTOS::LongMsg lmsgOeParams;
        const static RTOS::LongMsg lmsgReadMapFileResp;

        static OeComm* Init()
        {
            static OeComm* _pComm = new OeComm();
            return _pComm;
        }

        static OeComm* GetOeComm() { return _pComm; }
        NXGen::NODE_ID GetNodeId() override { return NXGen::NODE_CMD; }

    protected:
        OeComm() :
            _dlActualPos(),
            _dlTargetPos()
        { }

        static OeComm* _pComm;
        int32_t _dlTargetPos;
        int32_t _dlActualPos;

        CanEngine::EventGenerator<0> _canEvDelaylineDone;
        CanEngine::RegOwner<sizeof(uint8_t)> _canRegLaserStatus;
        CanEngine::RegOwner<sizeof(NXGen::DELAYLINE)> _canRegDelaylinePosition;
        CanEngine::RegOwner<sizeof(NXGen::OE_PARAMS)> _canRegOeParameters;
        CanEngine::RegClient<sizeof(uint8_t)> _canRegModality;
        CanEngine::RegClient<sizeof(uint32_t)> _canRegMduScanSpeed;
        CanEngine::ExgResponder<sizeof(NXGen::READ_MAP), sizeof(NXGen::MAP_DATA)> _canExgReadMapResp;
        CanEngine::ExgResponder<sizeof(NXGen::MAP_DATA), sizeof(uint8_t)> _canExgWriteMapResp;

        void _HandleExtraMsg(const IMqItem* id, const MsgObj& msg) override { }

        CanEngine::can_result_t _NodeCommObjInit() override { return CanEngine::OK; }

        static void _ModalityCallback(void* pParam);
        static void _ScanSpeedCallback(void* pParam);
        static void _DelaylineRxCallback(void* pParam);
        static void _DelaylineTxCallback(void* pParam);
        static void _LaserStatTxCallback(void* pParam);
        static void _OeParameterRxCallback(void* pParam);
        static void _ReadMapRequest(void* pParam);
        static void _WriteMapRequest(void* pParam);
    };
}

#endif // INCLUDE_OE_COMM_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_PIN_DETECTOR_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_PIN_DETECTOR_H_

// PinDetector Class
class PinDetector
{
public:
    // PinDetector Constructor
    // param  pin : reference to the pin peripheral
    // param  pQ : optional pointer to the queue to receive pin act / deact events
    // param  pActEv : pin activation event
    // param  pHoldMsg : pin holding message
    // param  pDeactEv : pin deactivation event
    PinDetector(
        HAL::Pin& pin,
        RTOS::MQ* pQ = nullptr,
        const RTOS::Ev* pActEv = nullptr,
        const RTOS::ShortMsg* pHoldMsg = nullptr,
        const RTOS::Ev* pDeactEv = nullptr) :
        _debounceCounter(),
        _holdTime(),
        _pin(pin),
        _pQ(pQ),
        _pActEv(pActEv),
        _pHoldMsg(pHoldMsg),
        _pDeactEv(pDeactEv),
        _stat64i32(),
        _timer("PIN TMR", RTOS::Timer::PERIODICAL)
    { }

    // Set receiving queueand events
    // param  pQ : pointer to the queue to receive pin act / deact events
    // param  pActEv : pin activation event
    // param  pHoldMsg : pin holding message
    // param  pDeactEv : pin deactivation event
    void SetEvents(
        RTOS::MQ* pQ = nullptr,
        const RTOS::Ev* pActEv = nullptr,
        const RTOS::ShortMsg* pHoldMsg = nullptr,
        const RTOS::Ev* pDeactEv = nullptr,
        const RTOS::Ev* pStartLineOk = nullptr,
        const RTOS::Ev* pStopLineOk = nullptr)
    { }

    // Create a single pin interrupt for this pin detection
    // note Only use either group or signle interrupt
    void SetSingleInt() { }

    // Use an already created group interrupt for this pin detection
    // note Only use either group or signle interrupt
    // parma groupInterrupt : reference to a group interrrupt object
    void SetGroupInt(PinGroup_Interrupt& groupInterrupt) { }

    // Enable this pin detection
    void Enable() { }

    // Disable this pin detection
    void Disable() { }

    // Set pin detection timing parameters
    // param  samplePeriod : number of milliseconds between pin sampling
    // param  debounceLimit : number of consistent reading before detecting the event
    void SetTiming(uint32_t samplePeriod, uint32_t debounceLimit) { }

    // Start Pin Timer
    void StartTimer() { }

    // Stop Pin Timer
    void StopTimer() { }

    uint32_t tTime = 0;
    uint32_t num = 0;

    // Read the holding time in a polling manner
    // return  hold time in millisecond
    uint32_t GetHoldTime()
    {
        if (tTime == 0)
        {
            tTime++;
            return 0;
        }
        else if (tTime == 1)
        {
            return 4000;
        }
        else
        {
            return 0;
        }
    }

    bool Get()
    {
        if (num == 0)
        {
            num++;
            return true;
        }
        else if (num == 1)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

protected:
    HAL::Pin& _pin;
    RTOS::MQ* _pQ;
    const RTOS::Ev* _pActEv;
    const RTOS::ShortMsg* _pHoldMsg;
    const RTOS::Ev* _pDeactEv;
    bool _stat;
    uint32_t _debounceCounter;
    uint32_t _holdTime;
    RTOS::Timer	_timer;

    static void _TimerCallback(void* pObj) { }
};

#endif // _PIN_DETECTOR_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_IVUS_PINS_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_IVUS_PINS_H_

namespace IVUS
{
    class GPIO_VCC_12V0_PG : public HAL::Pin
    {
    public:
        GPIO_VCC_12V0_PG() : Pin(5, 5, 2, 14, FUNC_0, INPUT) { }
    };

    class GPIO_VCC_12V0_EN : public HAL::Pin
    {
    public:
        GPIO_VCC_12V0_EN() : Pin(8, 2, 4, 2, FUNC_0, OUTPUT) { }
    };

    struct IvusPins
    {
        GPIO_CARD_UPDATE_        gpioCardUpdate;
        GPIO_PWR_24V_OK          gpioPwr24vOk;
        GPIO_PWR_5V_OK           gpioPwr5vOk;
        GPIO_PWR_BAT_OK          gpioPwrBatOk;
        GPIO_PWR_LINE_OK         gpioPwrLineOk;
        GPIO_PWR_ON              gpioPwrOn;
        GPIO_PWR_CARD_OK         gpioPwrCardOk;
        GPIO_RUN_LED_0_          ledRun0;
        GPIO_RUN_LED_1_          ledRun1;
        GPIO_CARD_LED0           ledCard0;
        GPIO_CARD_LED1           ledCard1;
        GPIO_CARD_LED2           ledCard2;
        GPIO_CARD_LED3           ledCard3;
        GPIO_CARD_PRESENT1_      gpioCardPresent1;
        GPIO_CARD_PRESENT2_      gpioCardPresent2;
        GPIO_EM_DIAG_TEST        gpioEmDiagTest;
        CARD_ID                  cardId;
        PWR_STATE                pwrState;
        BOARD_ID                 boardId;
        BUILD_ID                 buildId;
        ILOCK_PINS               ilockPins;
        CAN_PINS                 canPins;
        GPIO_VCC_5V0_PG          gpioVcc5V0Pg;
        GPIO_VCC_3V3_PG          gpioVcc3V3Pg;
        GPIO_VCC_2V5_PG          gpioVcc2V5Pg;
        GPIO_VCC_1V5_PG          gpioVcc1V5Pg;
        GPIO_VCC_1V15_PG         gpioVcc1V15Pg;
        GPIO_VCC_1V1_PG          gpioVcc1V1Pg;
        GPIO_VCC_12V0_PG         gpioVcc12v0Pg;
        GPIO_VCC_5V0_EN          gpioVcc5v0En;
        GPIO_VCC_3V3_EN          gpioVcc3v3En;
        GPIO_VCC_2V5_EN          gpioVcc2v5En;
        GPIO_VCC_1V5_EN          gpioVcc1v5En;
        GPIO_VCC_1V15_EN         gpioVcc1v15En;
        GPIO_VCC_1V1_EN          gpioVcc1v1En;
        GPIO_VCC_0V75_EN         gpioVcc0v75En;
        GPIO_VCC_12V0_EN         gpioVcc12v0En;
        GPIO_VCC_3V3A_EN         gpioVcc3v3aEn;
        GPIO_VCC_3V3C_EN         gpioVcc3v3cEn;
        GPIO_VCC_2V5A_EN         gpioVcc2v5aEn;
        GPIO_VCC_1V25_EN         gpioVcc1v25En;
        SPI_PINS                 spiPins;
        I2C1_PINS                i2c1Pins;
        UART1_PINS               uart1Pins;
        UART3_PINS               uart3Pins;
    };
}

#endif // INCLUDE_IVUS_PINS_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_IVUS_COMM_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_IVUS_COMM_H_

namespace IVUS
{
    // IVUS::IvusComm Class
    // IVUS Communication Task (singleton)
    class IvusComm : public Communication
    {
    public:
        const static RTOS::ShortMsg msgUltrasoundStat;

        static IvusComm* Init() { return _pComm; }
        static IvusComm* GetIvusComm() { return _pComm; }
        static void SetComm(IVUS::IvusComm* ivusComm);
        NXGen::NODE_ID GetNodeId() override;

    private:
        IvusComm();

        static IvusComm* _pComm;
        CanEngine::RegOwner<sizeof(uint8_t)> _canRegUltrasoundStatus;
        CanEngine::RegClient<sizeof(uint8_t)> _canRegModality;
        CanEngine::RegClient<sizeof(uint32_t)> _canRegMduScanSpeed;

        CanEngine::can_result_t _NodeCommObjInit() override;
        static void _ModalityCallback(void* pParam);
        static void _ScanSpeedCallback(void* pParam);
        void _HandleExtraMsg(const IMqItem* id, const MsgObj& msg) override;
    };
}

#endif // INCLUDE_IVUS_COMM_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_OE_FAULT_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_OE_FAULT_H_

namespace OE
{
    // Forward declare
    class OctControl;
    class OctComm;
    struct OePins;

    // OE::OeFault Class
    class OeFault : public FaultMonitor
    {
    public:
        static OE::OeFault* Init(OE::OePins& pins);
        static OE::OeFault* GetOctFault() { return _pFault; }
        static OE::OeFault* GetOeFault() { return _pFault; }
        static void SetComm(OeComm* comm) { _pComm = comm; }

    private:
        OeFault(OePins& pins);

        static OeComm* _pComm;
        static OeFault* _pFault;
        void _HandleMq(const IMqItem* id, const MsgObj& msg);
    };
}

#endif // INCLUDE_OE_FAULT_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// INCLUDE_OE_RTOS_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_OE_RTOS_H_

namespace RTOS
{
    // OE::Rtos Class
    class Rtos
    {
    public:
        static RTOS::result_t Start(RTOS::init_func_t initFunc);
        static void EnableSystemTicks(void);
        static uint32_t GetTime(void);
        static RTOS::result_t DelayFor(uint32_t const time) { return RTOS::OK; }
        static RTOS::result_t DelayUntil(uint32_t const time);
        static void* AcquireMsgData() { return new NXGen::MAP_DATA(); }
        static RTOS::result_t ReleaseMsgData(const void* data) { return RTOS::OK; }
        static void Lock();
        static void Unlock();
        static void StartStatistics();
        static void RunFullSpeed();

    private:
        Rtos() { }

        enum
        {
            MSG_DATA_SIZE = 320,
            MSG_DATA_NUM = 25,
            MSG_NUM = 50,
            INIT_TASK_PRI = 10,
            INIT_TASK_STK_SIZE = 120
        };

        static RTOS::MemPool _msgPool;
        static RTOS::MemPool _msgDataPool;

        friend RTOS::MQ;
        friend MsgObj;
    };
}

#endif // INCLUDE_OE_RTOS_H_

////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_MDU_PINS_H_
////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_MDU_PINS_H_

namespace MDU
{
#define MARKER_POS_IN_UM        12000
#define STEPPER_UM_PER_REV      3016
#define BUILTIN_DAC_BIT_WIDTH   1023
#define EXTERNAL_DAC_BIT_WIDTH  16383

    // 24 Volt Power Enable class:
    // This class is responsible for controlling the 24 volt power supply
    class GPIO_VCC_24V0_MOT_EN : public HAL::Pin
    {
    public:
        // Initialize the pin associated with the class
        GPIO_VCC_24V0_MOT_EN() : Pin(7, 0, 3, 8, FUNC_0, OUTPUT) { }
    };

    // 24 Volt Power Good class
    // This class is responsible for checking the 24 volt power supply
    class GPIO_VCC_24V0_MOT_PG : public HAL::Pin
    {
    public:
        // Initialize the pin associated with the class
        GPIO_VCC_24V0_MOT_PG() : Pin(5, 1, 2, 10, FUNC_0, INPUT | REVERSED) { }
    };

    // External DAC GPIO initialization class
    // This class is responsible for Setting up the GPIO pin used for the external DAC
    class GPIO_DAC_LDAC_ : public HAL::Pin
    {
    public:
        //Initialize the pin associated with the class
        GPIO_DAC_LDAC_() : Pin(1, 20, 0, 15, FUNC_0, OUTPUT | REVERSED) { }
    };

    // ILock GPIO output pin initialization class
    // This class is responsible for Setting up the GPIO pin used for the external DAC
    class GPIO_ILOCK_TO_MDU : public HAL::Pin
    {
    public:
        // Initialize the pin associated with the class
        GPIO_ILOCK_TO_MDU() : Pin(13, 0, 6, 14, FUNC_4, INPUT) { }
    };

    // ILock GPIO pin initialization class
    // This class is responsible for Setting up the GPIO pin used for the ILock Pin Group.
    class GPIO_ILOCK_FROM_MDU : public HAL::Pin
    {
    public:
        // Initialize the pin associated with the class
        GPIO_ILOCK_FROM_MDU() : Pin(13, 1, 6, 15, FUNC_4, OUTPUT) { }
    };

    // GPIO_NEG_LIMIT GPIO pin initialization class (not used)
    // This class is responsible for setting up the GPIO pin used for the negative switch
    // used by the MDU position (Not used)
    class GPIO_NEG_LIMIT : public HAL::Pin
    {
    public:
        // Initialize the pin associated with the class
        GPIO_NEG_LIMIT() : Pin(11, 0, 5, 20, Pin::FUNC_4, Pin::INPUT) { }
    };

    // GPIO_POS_LIMIT GPIO pin initialization class (not used)
    // This class is responsible for setting up the GPIO pin used for the positive switch
    // used by the MDU position (Not used)
    class GPIO_POS_LIMIT : public HAL::Pin
    {
    public:
        // Initialize the pin associated with the class
        GPIO_POS_LIMIT() : Pin(12, 5, 6, 4, Pin::FUNC_4, Pin::INPUT) { }
    };

    // GPIO_CATH_DET GPIO pin initialization class
    // This class is responsible for setting up the GPIO pin used for detecting
    // that a catheter is installed on the MDU
    class GPIO_CATH_DET : public HAL::Pin
    {
    public:
        // Initialize the pin associated with the class
        GPIO_CATH_DET() : Pin(13, 15, 6, 29, FUNC_4, Pin::INPUT | Pin::REVERSED) { }
    };

    // GPIO_CATH_DET_TYPE GPIO pin initialization class
    // This class is responsible for setting up the GPIO pin used for detecting
    // the type of catheter that is installed on the MDU.
    class GPIO_CATH_DET_TYPE : public HAL::Pin
    {
    public:
        // Initialize the pin associated with the class
        GPIO_CATH_DET_TYPE() : Pin(13, 16, 6, 30, FUNC_4, Pin::INPUT) { }
    };

    // GPIO_CATH_TYPE_POWER GPIO pin initialization class
    // This class is responsible for setting up the GPIO pin used to turn
    // on power to the catheter type detection hardware on the MDU
    class GPIO_CATH_TYPE_POWER : public HAL::Pin
    {
    public:
        // Initialize the pin associated with the class
        GPIO_CATH_TYPE_POWER() : Pin(1, 18, 0, 13, FUNC_0, Pin::OUTPUT) { }
    };

    // GPIO_QEI_CARRY GPIO pin initialization class (not used)
    // This class is responsible for setting up the GPIO pin used for the
    // Carry pin on the QEI output (Not used)
    class GPIO_QEI_CARRY : public HAL::Pin
    {
    public:
        // Initialize the pin associated with the class
        GPIO_QEI_CARRY() : Pin(12, 1, 6, 0, Pin::FUNC_4, Pin::INPUT) { }
    };

    // GPIO_QEI_BORROW GPIO pin initialization class (not used)
    // This class is responsible for setting up the GPIO pin used for the
    // Borrow pin on the QEI output (Not used)
    class GPIO_QEI_BORROW : public HAL::Pin
    {
    public:
        // Initialize the pin associated with the class
        GPIO_QEI_BORROW() : Pin(12, 2, 6, 1, HAL::Pin::FUNC_4, HAL::Pin::INPUT) { }
    };

    // MDU_LEDS initialization Structure
    // This struct sets up and initializes the GPIO pins used to
    // control the LEDs on the MDU
    struct MDU_LEDS
    {
        // Enumeration for LEDs on MDU
        enum
        {
            LED_ORIGIN,
            LED_SCAN,
            LED_READY,
            LED_PULLBACK,
            LED_HOME,
            LED_UNUSED1,
            LED_LASER,
            LED_POWER,
            LED_CATHETER,
            LED_PROP_LEFT,
            LED_PROP_RIGHT,
            LED_UNUSED2,
            NUM_OF_LEDS
        };

        // Initialize the pins associated with the class
        MDU_LEDS() :
            led
        {
            HAL::Pin(14, 0, 7, 0, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT | HAL::Pin::REVERSED),
            HAL::Pin(14, 1, 7, 1, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT | HAL::Pin::REVERSED),
            HAL::Pin(14, 2, 7, 2, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT | HAL::Pin::REVERSED),
            HAL::Pin(14, 3, 7, 3, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT | HAL::Pin::REVERSED),
            HAL::Pin(14, 4, 7, 4, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT | HAL::Pin::REVERSED),
            HAL::Pin(14, 5, 7, 5, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT | HAL::Pin::REVERSED),
            HAL::Pin(14, 6, 7, 6, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT | HAL::Pin::REVERSED),
            HAL::Pin(14, 7, 7, 7, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT | HAL::Pin::REVERSED),
            HAL::Pin(14, 8, 7, 8, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT | HAL::Pin::REVERSED),
            HAL::Pin(14, 9, 7, 9, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT | HAL::Pin::REVERSED),
            HAL::Pin(14,10, 7,10, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT | HAL::Pin::REVERSED),
            HAL::Pin(14,11, 7,11, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT | HAL::Pin::REVERSED)
        }
        { }

        // Array of pin objects associated with each LED
        HAL::Pin led[NUM_OF_LEDS];
    };

    // MDU_BUTTONS initialization Structure
    // This struct sets up and initializes the GPIO pins used to
    // detect the states of the buttons on the MDU
    struct MDU_BUTTONS
    {
        // Enumeration for buttons on MDU
        enum
        {
            BUTTON_ORIGIN = 0,
            BUTTON_SCAN = 1,
            BUTTON_READY = 2,
            BUTTON_PULLBACK = 3,
            BUTTON_HOME = 4,
            NUM_OF_BUTTONS
        };

        // Initialize the pins associated with the class
        MDU_BUTTONS() :
            button
        {
            HAL::Pin(12, 9, 6, 8, HAL::Pin::FUNC_4, HAL::Pin::INPUT | HAL::Pin::PULLDOWN),
            HAL::Pin(12,10, 6, 9, HAL::Pin::FUNC_4, HAL::Pin::INPUT | HAL::Pin::PULLDOWN),
            HAL::Pin(12,11, 6,10, HAL::Pin::FUNC_4, HAL::Pin::INPUT | HAL::Pin::PULLDOWN),
            HAL::Pin(12,12, 6,11, HAL::Pin::FUNC_4, HAL::Pin::INPUT | HAL::Pin::PULLDOWN),
            HAL::Pin(12,13, 6,12, HAL::Pin::FUNC_4, HAL::Pin::INPUT | HAL::Pin::PULLDOWN)
        }
        { }

        // Array of pin objects associated with each button
        HAL::Pin button[NUM_OF_BUTTONS];
    };

    // SCAN_MOTOR_PINS initialization struct:
    // This struct sets up and initializes the GPIO pins used to
    // control the scan motor of the MDU
    struct SCAN_MOTOR_PINS
    {
        // Initialize the pins associated with the class
        SCAN_MOTOR_PINS() :
            enable(14, 12, 7, 12, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT),
            pins
        {
            HAL::Pin(14, 13, 7, 13, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT),
            HAL::Pin(14, 14, 7, 14, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT),
            HAL::Pin(14, 15, 7, 15, HAL::Pin::FUNC_4, HAL::Pin::OUTPUT)
        }
        { }

        HAL::Pin enable;     // Enable Pin instance
        HAL::Pin pins[3];    // Array of pins associated with the scan motor control
    };

    // MduPins Struture
    // This struct instantiates the pin objects for IO Pins used by the MDU
    struct MduPins
    {
        // Instantiation of card update pin object
        // This object is responsible for reading the card update pin of the EM
        GPIO_CARD_UPDATE_ gpioCardUpdate;

        // Instantiation of power card ok pin object
        // This object is responsible for reading the power
        // card ok pin of the EM
        GPIO_PWR_CARD_OK gpioPwrCardOk;

        // Instantiation of the run LED 0 pin object
        // This object is responsible for controlling the run
        // LED 0 pin of the EM
        GPIO_RUN_LED_0_ ledRun0;

        // Instantiation of the run LED 1 pin object
        // This object is responsible for controlling the run
        // LED 1 pin of the EM
        GPIO_RUN_LED_1_ ledRun1;

        // Instantiation of card LED 0 pin object
        // This object is responsible for controlling the card
        // LED 0 pin of the EM
        GPIO_CARD_LED0 ledCard0;

        // Instantiation of card LED 1 pin object
        // This object is responsible for controlling the card
        // LED 1 pin of the EM
        GPIO_CARD_LED1 ledCard1;

        // Instantiation of card LED 2 pin object
        // This object is responsible for controlling the card
        // LED 2 pin of the EM
        GPIO_CARD_LED2 ledCard2;

        // Instantiation of card LED 3 pin object
        // This object is responsible for controlling the card
        // LED 3 pin of the EM
        GPIO_CARD_LED3 ledCard3;

        // Instantiation of card present 1 pin object
        // This object is responsible for reading the card
        // present 1 pin of the EM
        GPIO_CARD_PRESENT1_ gpioCardPresent1;

        // Instantiation of card present 2 pin object
        // This object is responsible for reading the card
        // present 2 pin of the EM
        GPIO_CARD_PRESENT2_ gpioCardPresent2;

        // Instantiation of the EM diagnostic test pin object
        // This object is responsible for reading the EM
        // diagnostic pin of the EM
        GPIO_EM_DIAG_TEST gpioEmDiagTest;

        // Instantiation of card id pins object
        // This object is responsible for reading the card
        // id pins of the EM
        CARD_ID cardId;

        // Instantiation of board id pins object
        // This object is responsible for reading the board id pins
        BOARD_ID boardId;

        // Instantiation of build id pins object
        // This object is responsible for reading the build id pins
        BUILD_ID buildId;

        // Instantiation of ilock output pin object
        // This object is responsible for reading the ilock output pin
        GPIO_ILOCK_FROM_MDU iLockOut;

        // Instantiation of ilock input pin object
        // This object is responsible for reading the ilock input pin
        GPIO_ILOCK_TO_MDU iLockIn;

        // Instantiation of can pins object
        // This class is responsible for reading and writing the can bus pins
        CAN_PINS canPins;

        // Instantiation of the 24.0 volt power good pin object
        // This object is responsible for reading the 24.0 volt power good pin
        GPIO_VCC_24V0_MOT_PG gpioVcc24v0Pg;

        // Instantiation of the 5.0 volt power good pin object
        // This object is responsible for reading the 5.0 volt power good pin
        GPIO_VCC_5V0_PG gpioVcc5V0Pg;

        // Instantiation of the 3.3 volt power good pin object
        // This object is responsible for reading the 3.3 volt power good pin
        GPIO_VCC_3V3_PG gpioVcc3V3Pg;

        // Instantiation of the 24.0 Volt Power Enable pin object.
        // This object is responsible for controlling the 24.0 volt power enable pin
        GPIO_VCC_24V0_MOT_EN gpioVcc24v0En;

        // Instantiation of the 5.0 Volt Power Enable pin object
        // This object is responsible for controlling the 5.0 volt power supply
        GPIO_VCC_5V0_EN gpioVcc5v0En;

        // Instantiation of 3.3 Volt Power Enable pin object
        // This object is responsible for controlling the 3.3 volt power enable pin
        GPIO_VCC_3V3_EN gpioVcc3v3En;

        // Instantiation of SPI bus pins object
        // This object is responsible for reading and writing the SPI bus pins
        SPI_PINS spiPins;

        // Instantiation of port 1 I2C bus pins object
        // This object is responsible for reading and writing the port 1 I2C bus pins
        I2C1_PINS i2c1Pins;

        // Instantiation of port 1 serial UART pins object
        // This object is responsible for reading and writing the port 1 serial UART pins
        UART1_PINS uart1Pins;

        // Instantiation of port 3 serial UART pins object
        // This object is responsible for reading and writing the port 3 serial UART pins
        UART3_PINS uart3Pins;

        // Instantiation of stepper control pins object
        // This object is responsible for reading and writing the stepper control pins
        STEPPER_CONTROL stepperControl;

        // Instantiation of scan motor control pins object
        // This object is responsible for reading and writing the scan motor control pins
        SCAN_MOTOR_PINS scanControl;

        // Instantiation of linear stepper negative limit pin object
        // This object is responsible for reading the linear stepper negative limit pin
        GPIO_NEG_LIMIT stepperNegLim;

        // Instantiation of linear stepper posative limit pin object
        // This object is responsible for reading the linear stepper posative limit pin
        GPIO_POS_LIMIT stepperPosLim;

        // Instantiation of catheter detect pin object
        // This object is responsible for reading the catheter detect pin
        GPIO_CATH_DET cathDet;

        // Instantiation of catheter type detect pin object
        // This object is responsible for reading the catheter type detect pin
        GPIO_CATH_DET_TYPE cathDetType;

        // Instantiation of catheter type detect power control pin object
        // This object is responsible for controlling the
        // catheter type detect power control control pin
        GPIO_CATH_TYPE_POWER cathTypePower;

        // Instantiation of MDU leds control pins object
        // This object is responsible for controlling the MDU leds control pins
        MDU_LEDS mduLeds;

        // Instantiation MDU button pins object
        // This object is responsible for reading the MDU button pins
        MDU_BUTTONS mduButtons;

        // Instantiation of external DAC control pins object
        // This object is responsible for reading and writing the
        // external DAC control pins
        GPIO_DAC_LDAC_ extDacEn;

        // Instantiation of linear quadrature decoder pins object
        // This object is responsible for reading and writing the
        // linear quadrature decoder pins
        QEI_PINS qeiPins;

        // Instantiation of linear quadrature decoder counter carry pins object
        // This object is responsible for reading the
        // linear quadrature decoder counter carry pins
        GPIO_QEI_CARRY qeiCarry;

        // Instantiation of linear quadrature decoder counter borrow pins object
        // This object is responsible for reading the
        // linear quadrature decoder counter borrow pins
        GPIO_QEI_BORROW qeiBorrow;

        // Instantiation of linear encoder pins object
        // This object is responsible for reading the linear encoder pins
        PULL_ENC_PINS pullEncPins;
    };
}

#endif // INCLUDE_MDU_PINS_H_

//////////////////////////////////////////////////////////////
//     INCLUDE_MDU_FAULT_H_
//////////////////////////////////////////////////////////////

#ifdef INCLUDE_MDU_FAULT_H_

namespace MDU
{
    // Forward declarations
    struct MduPins;
    class MduControl;
    class MduComm;

    // MduFault Class
    // MDU Fault class
    // This is the fault handler class for the MDU
    class MduFault : public FaultMonitor
    {
    public:
        // Initialize the MDU Fault handling and lockout GPIO pins
        // param pins: Reference to the associated pin class object
        // return: Pointer to MDU Fault handling singleton object
        static MduFault* Init(MduPins& pins)
        {
            static MduFault mf(pins);
            _pMduFault = &mf;
            return _pMduFault;
        }

        // Getter method to get the MDU Fault singleton object
        // return: Pointer to MDU Fault handling singleton object
        static MduFault* GetMduFault() { return _pMduFault; }

        void SetILockOut(int32_t val) { }

        // Set a pointer to the Comm class singleton object instance
        // Param Comm: Pointer to Comm singleton class object
        static void SetComm(MduComm* comm) { }

        // FOR FUTURE PURPOSE
        // Processes the user input from service port
        // param cmd: Reference to the user input command
        // return: "true" if the command is processed, "false" otherwise
        bool ProcessGeneralServiceCmd(IServiceCmd& cmd) { }

        const char* GetGeneralServiceHelp() { return ""; }

        MduFault::Task qFaultTask;

    private:
        // MduFault Constructor
        // Initializes the GPIO pins associated with the MDU fault handling
        // param pins: Reference to the associated pin class object
        MduFault(MduPins& pins) :_pPins(pins), FaultMonitor(pins.iLockIn, pins.iLockOut, pins.gpioEmDiagTest, pins.ledCard3) { }

        static MduFault* _pMduFault;
        static MduComm* _pComm;
        static const uint8_t iLockIn;
        static const uint8_t iLockOut;
        MduPins& _pPins;

        void _HandleMq(const IMqItem* id, const MsgObj& msg) { }
    };
}

#endif // INCLUDE_MDU_FAULT_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// INCLUDE_TLOG_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_TLOG_H_

// T-Log is a high performance, flexible, scalable log system

// using namespace Lib;

#ifdef __cplusplus
namespace TLOG
{
#endif

#define MAX_NUM_OF_TARGETS	(3)
#define MAX_NUM_OF_BUFS		(128)
#define FIRST_FAULT_ID		(10)
#define LAST_FAULT_ID		(199)
#define FIRST_EVENT_ID		(200)
#define LAST_EVENT_ID		(1999)
#define FIRST_TRACE_ID		(2000)
#define LAST_TRACE_ID		(65535)

    // Handle to the TLog object
    typedef void* tlog_h;

    // Handle to a log target (such as UART, CAN, FRAM, Flash ...)
    typedef void* tlog_target_h;

    typedef enum
    {
        TLOG_LEVEL_NONE = 0,    // log nothing
        TLOG_LEVEL_SYS_EV = 1,  // log the EV level
        TLOG_LEVEL_FAULT = 2,   // log the fault detection and clear
        TLOG_LEVEL_EVENT = 4,   // log the event (state change, user input, etc.)
        TLOG_LEVEL_TRACE = 8    // log the debug trace information
    }
    tlog_level_t;

    typedef const char* tlog_literal_t;

    typedef enum
    {
        FAILED,
        OK
    }
    tlog_result_t;

    typedef struct
    {
        tlog_literal_t code;
        uint32_t timestamp;
        uint32_t param1;
        uint32_t param2;
    }
    tlog_item_t;

    typedef struct
    {
        tlog_literal_t code;
        uint32_t timestamp;
        uint32_t size;
        uint32_t crc32;
    }
    tlog_data_dump_header_t;

    typedef struct
    {
        tlog_literal_t code;
        uint32_t data[3];
    }
    tlog_data_dump_t;

    typedef uint32_t(*pTimeApi)();
    typedef void(*pEnterCritical)();
    typedef void(*pExitCritical)();

    typedef struct
    {
        tlog_target_h _target[MAX_NUM_OF_TARGETS];
        tlog_item_t _incomingFifoBuf[MAX_NUM_OF_BUFS];
        tlog_level_t  _levelWaterMark;
        pTimeApi _pTimeApi;
        pEnterCritical _pEnterCritical;
        pExitCritical _pExitCritial;
    }
    tlog_t;

    typedef tlog_result_t(*pEraseApi)(tlog_target_h hTarget);
    typedef tlog_result_t(*pResetReadApi)(tlog_target_h hTarget);
    typedef tlog_result_t(*pReadApi)(tlog_target_h hTarget, uint8_t* pBuf, uint32_t length);
    typedef tlog_result_t(*pWriteApi)(tlog_target_h hTarget, const uint8_t* pBuf, uint32_t length);

    typedef enum
    {
        TLOG_TARGET_TYPE_STREAM, // Random write only
        TLOG_TARGET_TYPE_RAM,    // Random read and write, no need to erase
        TLOG_TARGET_TYPE_EEPROM, // Read and write in page, page erase
        TLOG_TARGET_TYPE_FLASH   // Read in page, page write, page erase
    }
    tlog_target_type_t;

    typedef struct
    {
        tlog_level_t _level;
        pEraseApi Erase;
        pResetReadApi ReadReset;
        pReadApi Read;
        pWriteApi Write;
        void* _pObj; // pointer to object, for OO programming
    }
    tlog_target_t;

    tlog_result_t TLogInit(tlog_h hTLog, pTimeApi api, pEnterCritical pEnterCri, pExitCritical pExitCri);
    tlog_result_t TLogProcess(tlog_h hTLog);
    tlog_result_t TLogSetTarget(tlog_h hTLog, tlog_target_h hTarget, tlog_level_t level);
    tlog_result_t TLogRemoveTarget(tlog_h hTLog, tlog_target_h hTarget);
    tlog_result_t TLogEntry(tlog_h hTLog, tlog_level_t level, tlog_literal_t logCode, void* logData, uint8_t logDataSize);
    tlog_result_t TLogDataDump(tlog_h hTLog, tlog_level_t level, tlog_literal_t logCode, void* pData, uint32_t dataSize);

#ifdef __cplusplus
}
#endif

#endif // INCLUDE_TLOG_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_CPP_TLOG_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CPP_TLOG_H_

namespace TLOG
{
    // TLOG::ITlogTarget Class
    class ITLogTarget
    {
    public:
        virtual tlog_target_h GetHandle() = 0;
    };

    // TLOG::TlogRamTarget Class
    template<size_t numOfEntries>
    class TLogRamTarget : public ITLogTarget
    {
    public:
        TLogRamTarget()
        {
            _target._pObj = (void*)this;
            TLogRamInit(&_ramTarget, &_target, _ramStorage, numOfEntries);
        }

        tlog_target_h GetHandle() override
        {
            return (tlog_target_h)&_target;
        }

        bool GetItem(tlog_item_t* pOutput)
        {
            return (1 == TLogRamGetOne(&_ramTarget, pOutput));
        }

    protected:
        tlog_target_t _target;
        tlog_item_t _ramStorage[numOfEntries];
    };

    // TLOG::TLog Class
    // High efficient and performance log system
    class TLog
    {
    public:
        // TLog Constructor
        // param[in] timeApi : Timestamp provider
        TLog(pTimeApi timeApi)
        {
            TLogInit(&_tlogCore, timeApi, nullptr, nullptr);
        }

        TLog(pTimeApi timeApi, pEnterCritical pEnterCri, pExitCritical pExitCri)
        {
            TLogInit(&_tlogCore, timeApi, pEnterCri, pExitCri);
        }

        void Log(tlog_level_t level, tlog_literal_t code, void* logData, uint8_t logDataSize)
        {
            TLogEntry(&_tlogCore, level, code, logData, logDataSize);
        }

        void Log(tlog_level_t level, tlog_literal_t code)
        {
            Log(level, code, nullptr, 0);
        }

        void LogData(tlog_level_t level, tlog_literal_t code, void* pData, uint8_t dataSize)
        {
            TLogDataDump(&_tlogCore, level, code, pData, dataSize);
        }

        void SetTarget(ITLogTarget& target, tlog_level_t level)
        {
            auto hTarget = target.GetHandle();

            if (level == TLOG_LEVEL_NONE)
            {
                TLogRemoveTarget(&_tlogCore, hTarget);
            }
            else
            {
                TLogSetTarget(&_tlogCore, hTarget, level);
            }
        }

        void Process()
        {
            TLogProcess(&_tlogCore);
        }

    protected:
        tlog_t _tlogCore;
    };
}

#endif // INCLUDE_CPP_TLOG_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_PARAMSTORE_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_PARAMSTORE_H_

// ParamStore Class
class ParamStore
{
public:
    static ParamStore* _pParamStore;
    NXGen::OE_NV_PARAMS* _accessBuff;
    bool _taskEnable;

    static ParamStore* Init()
    {
        if (_pParamStore == nullptr)
        {
            _pParamStore = new ParamStore();
        }

        return _pParamStore;
    }

    void TaskEnable() { _taskEnable = true; }

    NXGen::OE_NV_PARAMS* GetParamAccessBuff()
    {
        if (_accessBuff == nullptr)
        {
            _accessBuff = new NXGen::OE_NV_PARAMS();
        }

        return _accessBuff;
    }

    void AccessBuffWrLockSet() { }
    void AccessBuffWrLockClr() { }

private:
    ParamStore() :
        _accessBuff(nullptr),
        _taskEnable(false)
    { }
};

#endif // INCLUDE_PARAMSTORE_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_UTILITIES_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_UTILITIES_H_

// RingBUffer Class
class RingBuffer
{
public:
    RingBuffer(void* ram, uint32_t itemSize, uint32_t itemNum);

    virtual ~RingBuffer();

    void Push(const void* pItem);
    bool Peek(void* pItem);
    bool Pop(void* pItem);
    void Reset(void);
    bool IsEmpty(void) const;
    bool IsFull(void) const;
    uint32_t GetEmptySlots() const;
    uint32_t GetFilledSlots() const;

private:
    uint32_t _itemSize;
    uint8_t* _ram;
    uint32_t _itemNum;
    uint32_t _head;
    uint32_t _tail;
    bool _isOverwritten;

    void _Add(const void* pItem, uint32_t index);
    void _Get(void* pItem, uint32_t index);
    uint32_t _WrapInc(uint32_t i) const;
};

// CRC Class
class CRC
{
public:
    CRC() :
        _crcBeforeXor()
    { }

    ~CRC() { }

    uint32_t Crc32(const uint8_t* pBuf, uint16_t size);
    static bool Verify(const uint8_t* pBuf, uint16_t size);
    static void AddCrc(uint8_t* pBuf, uint16_t size);
    void Reset(void);
    uint32_t GetCrc(void) const;
    static uint32_t CalcWithoutXOR(const uint8_t* pBuf, uint32_t size, uint32_t input) { return 0xAAAA0000; };
    static uint32_t FinalXor(uint32_t crc) { return 0xA0A0A0A0; };

    enum
    {
        SEED = 0xFF,
        FINAL_XOR = 0xAA
    };

private:
    uint32_t _crcBeforeXor;
};

// CRC8 Class
class CRC8
{
public:
    enum
    {
        SEED = 0xFF,
        FINAL_XOR = 0xAA
    };

    static uint8_t Calc(const void* pBuf, uint32_t size, uint8_t seed) { return 100; }
    static bool Verify(const void* pBuf, uint32_t size, uint8_t seed) { return true; }
};

// CRC32 Class
class CRC32
{
public:
    enum
    {
        SEED = 0xFEEDC0DE,
        FINAL_XOR = 0xFFFFFFFF
    };

    static uint32_t Calc(const void* pBuf, uint32_t size, uint32_t input) { return 100; }
    static uint32_t CalcXor(const void* pBuf, uint32_t size, uint32_t input) { return 200; }
    static uint32_t Xor(uint32_t input) { return 300; }
};

// Helper Class
class Helper
{
public:
    static inline uint32_t StoreFloatAsU32(float fval)
    {
        uint32_t u32val = *(uint32_t*)(&fval);
        return u32val;
    }

    static inline float RetrieveFloatFromU32(uint32_t uval)
    {
        float fval = *(float*)(&uval);
        return fval;
    }
};

// MovingAverage Class
template<class T, int _size>
class MovingAverager
{
public:
    MovingAverager() :
        sum(0),
        _index(0),
        size(_size)
    {
        for (auto& b : _buf)
        {
            b = 0;
        }
    }

    void operator <<(T val)
    {
        sum -= _buf[_index];
        _buf[_index] = val;
        sum += val;
        _index++;
        if (_index >= _size)
        {
            _index = 0;
        }
    }

    T Get() { return sum / _size; }

    T sum;
    uint32_t size;

private:
    T _buf[_size];
    uint32_t _index;
};

// Get Text after last "\\"
static char* LastString(char* fullString)
{
    static std::string compareString(fullString);
    compareString = fullString;
    compareString = compareString.substr(compareString.find_last_of("\\") + 1, compareString.size());
    return (char*)compareString.c_str();
}

#endif // INCLUDE_UTILITIES_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_DEBUG_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_DEBUG_H_

// Debug Class
class Debug
{
public:
    static inline void Assert(bool cond) { }

    // Debug::Trace Class
    class Trace
    {
    public:
        Trace(const char* moduleName) : _moduleName(moduleName) { }

        void Print(const char* info) const
        {
            printf("%s: %s\n", _moduleName, info);
        }

    private:
        const char* _moduleName;
    };
};

#endif // INCLUDE_DEBUG_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_LOGGER_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_LOGGER_H_

using namespace TLOG;

// Forward declare
class TimeKeeper;
class ServicePort;

namespace HAL
{
    // HAL:: NVMEM Class
    class NVMEM
    {
    public:
        virtual bool Enable() = 0;
        virtual bool Disable() = 0;
        virtual bool Erase(uint32_t addr, uint32_t size) = 0;
        virtual bool Write(uint32_t addr, const uint8_t* pData, uint32_t size) = 0;
        virtual bool Read(uint32_t addr, uint8_t* pBuf, uint32_t size) = 0;
    };
}

// Logger configuration Structure
struct logger_setting_t
{
    uint8_t framLogLevel; // Log level for FRAM
    uint8_t ramLogLevel;  // Log level for RAM
    uint8_t canLogLevel;  // Log level for CAN stream
    uint8_t crc8;         // CRC for the structure

    // Check CRC of this structure
    // return true, structure is valid; false, structure is not valid
    bool CheckCrc()
    {
        if (setCrc)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // Calculate and update the structure CRC
    void UpdateCrc()
    {
        crc8 = CRC8::Calc(&framLogLevel, 3, CRC8::SEED);
    }
};

// Logger Class
// Logging Task
class Logger : RTOS::Task<160>, public IServiceable
{
public:
    // Task incoming queue
    RTOS::MQ q;

    // Factory method to create the logger singleton object
    // param timeKeeper  : reference to a time keeper for add timestamps
    // param nvmem       : reference to NV memory for store the log entries
    // param appCrc      : application's CRC
    // return Pointer to the created logger singleton
    static Logger* Init(TimeKeeper& timeKeeper, HAL::NVMEM& nvmem, uint32_t appCrc);

    // Get the created logger singleton object
    // return Pointer to the created logger singleton
    static Logger* GetLogger() { return _pLogger; }

    // Set the OS Fault
    void SetOSFault() { }

    // Send Lockout
    void SendLockOut() { }

    // Runtime control

    // Create a log entry
    // param level : log level of this entry
    // param code  : log code to identify the content of this entry
    // param logData : data to be attached to this entry
    // param logDataSize : size of the log data
    void Log(tlog_level_t level, tlog_literal_t code, void* logData, uint8_t logDataSize)
    {
        TLogEntry(nullptr, level, code, logData, logDataSize);
    }

    // Task loop
    void Loop() override { }

    // Flush all pending log entries.  This should be called before shut off the power
    void Flush();

    // Pop (retrieve a remove) a log page from the NV memory storage
    // param [out] pBuf : pointer to the output buffer
    // param  pMsg      : pointer to the message to indicate the retrieval is done
    // param  pQ        : pointer to the queue to receive the retrieved data
    void RequestPopPage(uint8_t* pBuf, RTOS::ShortMsg* pMsg, RTOS::MQ* pQ);

    // Service command handling

    // Process the general service command (available in all states)
    // param cmd  : reference to the received command
    // return true, the command is accepted by the task/class; false, the command is not accepted
    bool ProcessGeneralServiceCmd(IServiceCmd& cmd) override { return true; }

    // Help information this task/class provide to display when queried by "?" command
    // return the help information string
    const char* GetGeneralServiceHelp() override { return ""; }

    logger_setting_t _logSettings;

private:
    Logger(TimeKeeper& timeKeeper, uint32_t appCrc) :
        _logSettings()
    { }

    const static RTOS::Ev _evFlush;
    const static RTOS::Ev _evPopPage;
    static Logger* _pLogger;

    static uint32_t _GetTime();
    static void _TakeMutex();
    static void _ReleaseMutex();

    enum { LOGGER_PERIOD = 100 };
};

// These macro helps to define a unique event entry and log it
#define STRINGIZE_DETAIL(x) #x
#define STRINGIZE(x) STRINGIZE_DETAIL(x)

#define LOG(__level, __logMsg, __pData, __dataSize)								\
    {																			\
        static const char* __logLiteral =										\
             __FILE__ " : " STRINGIZE(__LINE__) " : " __logMsg;					\
        Logger::GetLogger()->Log(__level, __logLiteral, __pData, __dataSize);	\
    }

#define LOG_TRACE(__logMsg, __pData, __dataSize)            \
        LOG(TLOG_LEVEL_TRACE, __logMsg, __pData, __dataSize)

#define LOG_EVENT(__logMsg, __pData, __dataSize)            \
        LOG(TLOG_LEVEL_EVENT, __logMsg, __pData, __dataSize)

#define LOG_FAULT(__logMsg, __pData, __dataSize)            \
        LOG(TLOG_LEVEL_FAULT, __logMsg, __pData, __dataSize)

#define TLOG_LEVEL_SYS_EV(__logMsg, __pData, __dataSize)    \
        LOG(TLOG_LEVEL_SYS_EV, __logMsg, __pData, __dataSize)

#define LOG_OS_FLT_LOCKOUT() Logger::GetLogger()->SetOSFault();

#define LOG_FLT_LOCKOUT() Logger::GetLogger()->SendLockOut();

#endif // INCLUDE_LOGGER_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_HAL_FLASH_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_HAL_FLASH_H_

namespace HAL
{
    // HAL::Flash Class
    // Interface to Flash driver
    class IFlash
    {
    public:
        // Initialize the Flash access driver
        virtual void Init() = 0;

        // Erase content from Flash memory
        // param  startAddress : start address of the erased memory
        // param  size : size of the erased memory
        // return true, erase is completed; false, erase cannot be performed
        virtual bool Erase(uint32_t startAddress, uint32_t size) = 0;

        // Write content to Flash memory
        // param  startAddress : start address of the memory to be written to
        // param  size : size of the data to be written
        // param  data : pointer to the data to be written
        // return true, wrting is completed; false, wrting cannot be performed
        virtual bool Write(uint32_t startAddress, uint32_t size, const uint8_t* data) = 0;
    };

    // HAL::Flash Class
    // Implementation of the Flash driver
    class Flash : public IFlash
    {
    public:
        // Structure definition for flash sector information
        struct SectorInfo
        {
            uint32_t startAddress;   // Start address of this sector
            uint32_t endAddress;     // End address of this sector
            uint32_t sectorNumber;   // Sector number
            uint32_t bank;           // Memory bank number, 0 or 1
        };

        // Initialize the Flash access driver
        void Init() override { }

        // Erase content from Flash memory
        // param  startAddress : start address of the erased memory
        // param  size : size of the erased memory
        // return true, erase is completed; false, erase cannot be performed
        bool Erase(uint32_t startAddress, uint32_t size) override { return true; }

        // Write content to Flash memory
        // param  startAddress : start address of the memory to be written to
        // param  size : size of the data to be written
        // param  data : pointer to the data to be written
        // return true, wrting is completed; false, wrting cannot be performed
        bool Write(uint32_t startAddress, uint32_t size, const uint8_t* data) override { return true; }

    private:
        // The following members are used for System ROM IAP API
        uint32_t _command[5];
        uint32_t _result[4];
        uint32_t _startSectorNumber;
        uint32_t _endSectorNumber;
        uint32_t _bank;

        bool _Prepare();
        bool _GetSectorInfo(uint32_t address, SectorInfo const*& sectorInfo);
        bool _GetSectors(uint32_t startAddress, uint32_t endAddress, uint32_t& startSectorNumber, uint32_t& endSectorNumber, uint32_t& bank);
        bool _EraseSector(uint32_t startAddr, uint32_t size);
        bool _ErasePage(uint32_t startAddr, uint32_t size);
    };
}

#endif // INCLUDE_HAL_FLASH_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_OE_PINS_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_OE_PINS_H_

namespace OE
{
    class GPIO_VCC_12V0_EN : public Pin
    {
    public:
        GPIO_VCC_12V0_EN() : Pin(7, 2, 3, 10, FUNC_0, OUTPUT) { }
    };

    class GPIO_VCC_12V0_PG : public Pin
    {
    public:
        GPIO_VCC_12V0_PG() : Pin(5, 1, 2, 10, FUNC_0, INPUT) { }
    };

    class OPT_SW
    {
    public:
        OPT_SW() : sw
        {
            Pin(12, 1, 6, 0, Pin::FUNC_4, Pin::OUTPUT),
            Pin(12, 2, 6, 1, Pin::FUNC_4, Pin::OUTPUT)
        }
        { }

        Pin sw[2];
    };

    class LASER_CONTROL
    {
    public:
        LASER_CONTROL() :
            enable(13, 10, 6, 24, Pin::FUNC_4, Pin::OUTPUT),
            monitor(13, 11, 6, 25, Pin::FUNC_4, Pin::INPUT),
            statClear(13, 13, 6, 27, Pin::FUNC_4, Pin::OUTPUT),
            lowPowerStat(6, 11, 3, 7, Pin::FUNC_0, Pin::INPUT)
        { }

        Pin enable;
        Pin monitor;
        Pin statClear;
        Pin lowPowerStat;
    };

    class BAL_MUX
    {
    public:
        BAL_MUX() : mux
        {
            Pin(13, 13, 6, 27, Pin::FUNC_4, Pin::OUTPUT),
            Pin(13, 14, 6, 28, Pin::FUNC_4, Pin::OUTPUT)
        },
            enable(13, 15, 6, 29, Pin::FUNC_4, Pin::OUTPUT | Pin::REVERSED),
            sd(13, 16, 6, 30, Pin::FUNC_4, Pin::OUTPUT)
        { }

        Pin mux[2];
        Pin enable;
        Pin sd;
    };

    struct OePins
    {
        GPIO_CARD_UPDATE_        gpioCardUpdate;
        GPIO_PWR_24V_OK          gpioPwr24vOk;
        GPIO_PWR_5V_OK           gpioPwr5vOk;
        GPIO_PWR_BAT_OK          gpioPwrBatOk;
        GPIO_PWR_LINE_OK         gpioPwrLineOk;
        GPIO_PWR_CARD_OK         gpioPwrCardOk;
        GPIO_RUN_LED_0_          ledRun0;
        GPIO_RUN_LED_1_          ledRun1;
        GPIO_CARD_LED0           ledCard0;
        GPIO_CARD_LED1           ledCard1;
        GPIO_CARD_LED2           ledCard2;
        GPIO_CARD_LED3           ledCard3;
        GPIO_CARD_PRESENT1_      gpioCardPresent1;
        GPIO_CARD_PRESENT2_      gpioCardPresent2;
        GPIO_EM_DIAG_TEST        gpioEmDiagTest;
        CARD_ID                  cardId;
        PWR_STATE                pwrState;
        BOARD_ID                 boardId;
        BUILD_ID                 buildId;
        ILOCK_PINS               ilockPins;
        CAN_PINS                 canPins;
        GPIO_PWR_ON              gpioPwrOn;
        GPIO_VCC_12V0_EN         gpioVcc12vEn;
        GPIO_VCC_5V0_PG          gpioVcc5V0Pg;
        GPIO_VCC_3V3_PG          gpioVcc3V3Pg;
        GPIO_VCC_12V0_PG         gpioVcc12vPg;
        GPIO_VCC_5V0_EN          gpioVcc5v0En;
        GPIO_VCC_3V3_EN          gpioVcc3v3En;
        SPI_PINS                 spiPins;
        I2C1_PINS                i2c1Pins;
        UART1_PINS               uart1Pins;
        UART3_PINS               uart3Pins;
        STEPPER_CONTROL          stepperControl;
        OPT_SW                   optSw;
        LASER_CONTROL            laserControl;
        BAL_MUX                  balMux;
    };
}

#endif // INCLUDE_OE_PINS_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// INCLUDE_OE_PARAMETERS_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace OE
{
#define BGMAP_LINES			(2048)
#define DISMAP_LINES		(4096)
#define IDXMAP_LINES		(4096)
#define WTMAP_LINES			(4096)

#define BGMAP_ENTRY_LEN		(4) // BackGround Ch1 (16bit) + Ch2 (16bit)
#define DISMAP_ENTRY_LEN	(2) // real (8bit) + imag (8bit)
#define IDXMAP_ENTRY_LEN	(2) // index (16bit)
#define WTMAP_ENTRY_LEN		(4) // weight (32bit)

#define DISMAP_ADDR			(0x1B070000)
#define DISMAP_CRC			(EEPROM_CONFIG_ADDR + 0x10)
#define IDXMAP_ADDR			(0x1B072000)
#define IDXMAP_CRC			(EEPROM_CONFIG_ADDR + 0x14)
#define WTMAP_ADDR			(0x1B074000)
#define WTMAP_CRC			(EEPROM_CONFIG_ADDR + 0x18)
#define BGMAP_ADDR			(0x1B078000)
#define BGMAP_CRC			(EEPROM_CONFIG_ADDR + 0x1C)
#define PARAM_ADDR			(EEPROM_CONFIG_ADDR)
#define PARAM_CRC			(EEPROM_CONFIG_ADDR + 0x0C)

#define MAP_PRINT_RATE		(50)
}

#ifdef INCLUDE_OE_PARAMETERS_H_

namespace HAL
{
    class IFlash;
}

namespace OE
{
    // OE::MapFile class
    // This class provides methods to access and manipulate MAP files
    class MapFile
    {
    public:
        // MapFile Constructor
        // param lineLength:   Number of bytes of each line
        // param numOfLines:   Number of lines of the MAP file
        // param flasher:      Reference to Flash memory access object
        // param flashAddr:    Address to the MAP in FLASH memory
        // param pCrc:         Pointer to the CRC value stored in EEPROM
        MapFile(uint32_t lineLength, uint32_t numOfLines, HAL::IFlash& flasher, uint32_t flashAddr, uint32_t* pCrc);

        // Read a line of the MAP file to the buffer
        // param index:        Line index in the MAP file
        // param pBuf:         Pointer to the output buffer
        // return:             "true" on successful reading, "false" otherwise
        bool ReadLine(uint32_t index, void* pBuf);

        // Write a line to the MAP file
        // param index:        Line index in the MAP file
        // param pBuf:         Pointer to the buffer containing new line
        // return:             "true" on successful writing, "false" otherwise
        bool WriteLine(uint32_t index, const void* pBuf);

        // Read a block from the MAP file
        // param addr:         Address in the MAP file
        // param pBuf:         Pointer to the buffer containing the block of data read
        // param bytes:        Buffer size in bytes
        // return:             "true" on successful reading, "false" otherwise
        bool ReadBlock(uint32_t addr, void* pBuf, uint32_t bytes);

        // Write a block to the MAP file
        // param addr:         Address in the MAP file
        // param pBuf:         Pointer to the buffer containing the block of data to write
        // param bytes:        Buffer size in bytes
        // return:             "true" on successful writing, "false" otherwise
        bool WriteBlock(uint32_t addr, const void* pBuf, uint32_t bytes);

        // Compute and update the current MAP file CRC
        void UpdateCrc();

        // Update the EEPROM stored MAP file CRC value, if not matching with computed one
        void UpdateCrc_EEPROM();

        // Check the passed in CRC value against the computed and stored values and return it's status
        // param crc32:    32- bit CRC
        // return:         "true" if the passed in CRC matches the stored CRC, "false" otherwise
        bool CheckCrc(uint32_t crc32) const;

        // Verify the computed CRC against the stored values and return it's status
        // return:         "true" if the passed in CRC matches the stored CRC, "false" otherwise
        bool VerifyMapCrc();

        // Get the stored CRC value
        // return:         Stored CRC value
        uint32_t GetCrc() const;

        // Sync the RAM buffer MAP file to NVMEM
        void FlushRamBuffer();

    protected:
        enum { PAGE_SIZE = 4096 };

        const uint32_t _numOfLines;
        const uint32_t _lineLength;
        HAL::IFlash& _flasher;
        const uint32_t _flashAddr;
        uint32_t _pageStartForWrite;
        uint32_t* _pCrc;
        uint8_t _ramBuffer[PAGE_SIZE];

        uint32_t _CalcCrc() const;

        friend class OeParameters;

    private:
        uint32_t _tempCrc;
    };

    // OE::OeParameters Class
    // Optical Engine Parameters Task.  This task handles the multiple MAP files stored on the OE EM
    class OeParameters : public RTOS::Task<120>, public IServiceable
    {
    public:
        static const RTOS::LongMsg lmsgReadMap;     // Message to read the OE MAP file
        static const RTOS::LongMsg lmsgWriteMap;    // Message to write the OE MAP file
        const static RTOS::ShortMsg _msgDisEntry;   // Message to write the OE dispersion MAP data entry
        const static RTOS::ShortMsg _msgIdxEntry;   // Message to write the OE linearity MAP data entry
        const static RTOS::ShortMsg _msgWtEntry;    // Message to write the OE Weight MAP data entry
        const static RTOS::ShortMsg _msgBgEntry;    // Message to write the OE BackGround MAP data entry
        const static RTOS::ShortMsg _msgSetCoeff;   // Message to set the co-efficient data
        const static RTOS::ShortMsg _msgSetMduHome; // Message to set the MDU Home information
        const static RTOS::ShortMsg _msgSetCalHome;	// Message to set the Calibration Home information
        const static RTOS::LongMsg msgSetParams;	// Message to set the OE parameters
        const static RTOS::Ev evCkMapParamCRC;		// Update Delay line position event

        // Message queue for the OE Parameters task
        RTOS::MQ q;

        // Initialize the OE Parameters singleton class object
        // return: Pointer to the OE Parameters class object
        static OeParameters* Init();

        // Get the OE Parameters singleton class object
        // return: Pointer to the OE Parameters singleton class object
        static OeParameters* GetOeParameters() { return _pSelf; }

        // Set the passed in message queue reference as the OE Control queue pointer
        // param controlQ: Reference to the control queue
        void SetControlQ(RTOS::MQ& controlQ);

        // Set the passed in message queue reference to the OE communication queue pointer
        // param commQ: Reference to the comm queue
        void SetCommQ(RTOS::MQ& commQ);

        // Task loop
        // handles the event and messages for the queue
        void Loop() override;

        // Processes the service user general commands from service port
        // param cmd: Reference to the user input command
        // return: "true" if the command is processed. "false" otherwise
        bool ProcessGeneralServiceCmd(IServiceCmd& cmd) override;

        // Processes the service user general help request from the service port
        // return: String containing service help information
        const char* GetGeneralServiceHelp() override;

        // Processes the service user special commands from service port
        // param cmd: Reference to the user input command
        // return: "true" if the command is processed. "false" otherwise
        bool ProcessSpecialServiceCmd(IServiceCmd& cmd) override;

        // Processes the service user special help request from the service port
        // return: String containing service help information
        const char* GetSpecialServiceHelp() override;

        // Method called to start the OeParameters task
        void Enable();

        // Verify the computed CRC against the stored values and return it's status
        // return: "true" if the passed in crc matches the stored crc, othwise return false
        bool VerifyParamCrc();

    private:
        OeParameters();

        const static RTOS::Ev _evBgMap;
        const static RTOS::Ev _evDisMap;
        const static RTOS::Ev _evIdxMap;
        const static RTOS::Ev _evWtMap;
        const static RTOS::Ev _evBgMapCrc;
        const static RTOS::Ev _evDisMapCrc;
        const static RTOS::Ev _evIdxMapCrc;
        const static RTOS::Ev _evWtMapCrc;
        const static RTOS::Ev _evStreamTimer;
        const static RTOS::Ev _evUpdateBgCrc;
        const static RTOS::Ev _evUpdateDisCrc;
        const static RTOS::Ev _evUpdateIdxCrc;
        const static RTOS::Ev _evUpdateWtCrc;
        RTOS::MQ* _pControlQ;
        RTOS::MQ* _pCommQ;
        ServicePort _dumpPort;
        RTOS::Timer* _pTmr;
        const RTOS::Ev _evCRCUpdate;
        MapFile _disMap;
        MapFile _idxMap;
        MapFile _wtMap;
        MapFile _bgMap;
        HAL::Flash _flasher;
        MapFile* _pPrintingMap;
        RTOS::Timer _streamTimer;
        uint32_t _printIndex;
        volatile NXGen::OE_PARAMS* _pParam;     // Maps to EEPROM Memory
        NXGen::OE_PARAMS _tempParam;			// Temporary RAM data
        static OeParameters* _pSelf;

        void _StartMapPrint(MapFile& map);
        void _StopMapPrint();
        void _PrintLine();
        void _PrintTitle();
        void _PrintCrc(MapFile& map);
        void _UpdateMapCrc(MapFile& map);
        void _UpdateParamCrc();
        void _UpdateOEMapCrc();
    };
}

#endif // INCLUDE_OE_PARAMETERS_H_

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_WATCHDOG_H_
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_WATCHDOG_H_

// WatchDog Class
class WatchDog : public RTOS::Task<120>
{
public:
    // Initialize the singleton object
    // return: Pointer to the WatchDog class
    static WatchDog* Init()
    {
        WatchDog* _pWatchDog = new WatchDog();
        return _pWatchDog;
    }

    // Get the WatchDog singleton class object pointer
    // return: Pointer to WatchDog class object
    static WatchDog* GetWatchDog() { return _pWatchDog; }

    // SetCommQ Method
    // Method to set the communication queue reference
    // param q: Reference to the Queue class object for task communication
    void SetCommQ(RTOS::MQ& q) { }

    // Enable WatchDog
    // param NoOfTasks:  number of tasks to monitor
    // param exceptionTskBitMap:  Exception Task Bit Map. (Workaround - need to remove - TBD - TODO)
    // Return : "true" if success. "false" otherwise
    bool EnableWDTmr(uint8_t NoOfTasks, uint32_t exceptionTskBitMap) { return true; }

    // Debug mode on/ off
    // param TurnOn : if "true", turn no. Off otherwise
    void Debug(bool TurnOn) { }

    // Get Debug mode status
    // Return : "true" if Debug mode On. "false" otherwise
    bool GetDebug() { return true; }

    // Service WatchDog Timer
    // param TaskNo:  Task number that is currently serviced
    // Return: "true" if success. "false" otherwise
    bool ServiceWDTmr(uint8_t TaskNo) { return true; }

protected:
    // WatchDog Constructor
    WatchDog() :
        _SW_WatchDog_Tmr("watchdog", RTOS::Timer::PERIODICAL),
        _EM_taskBitmap(),
        _pCommQ(),
        _timerPeriod(),
        _taskBitmap()
    { }

    // Timer period in mSec
    enum _CONST
    {
        WATCHDOG_PERIOD = 10000
    };

    // OS_CFG_PRIO_MAX-1 is the lowest priority for the IDLE task. OS_CFG_PRIO_MAX is 64
    enum
    {
        WDT_TASK_PRI = 62
    };

    static WatchDog* _pWatchDog;	// Stored pointer to the singleton class object
    RTOS::Timer _SW_WatchDog_Tmr;	// WatchDog Timer instance
    RTOS::MQ* _pCommQ;				// Comm Queue for sending safe
    uint32_t _timerPeriod;          // Time Period
    uint32_t _taskBitmap;			// A bitmap showing tasks that were serviced by WatchDog
    uint32_t _EM_taskBitmap;		// Expected EM bitmap
    static bool _EM_Debug_On;		// Holds the Debug mode status

    // Callback method that is called by the periodically by the WatchDog Timer
    // param pParam:   Pointer to the object that has the members used
    // by the callback
    static void _WatchDogCallback(void* pParam) { }
};

#endif // INCLUDE_WATCHDOG_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_QEI_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_QEI_H_

namespace HAL
{
    // HAL::QEI Class
    // Quadrature Encoder Interface
    // This is the common base class to different implementation of the quadrature
    // encoder interfaces
    class QEI
    {
    public:

        virtual void Enable() { qei_enable = true; }
        virtual void Disable() { qei_enable = false; }
        virtual void Reset() { qei_enable = true; }
        virtual void SetSamplePeriod(uint32_t period) { _timerPeriod = period; }
        uint32_t HAL::QEI::GetSamplePeriod() { return _timerPeriod; }
        void SetReversed(bool isReversed) { _isReversed = true; }
        virtual uint32_t GetPosition() { _position = 33;  return _position; }
        virtual uint32_t GetVelocity() { _velocity = 111; return _velocity; }
        virtual void IntHandler() { }

        void HAL::QEI::SetCallback(callback_func_t callback, void* pObj)
        {
            _callback = callback;
            _callbackObj = pObj;
        }

        void* _callbackObj;
        callback_func_t _callback;

    protected:
        QEI() :
            _callbackObj(),
            _timerPeriod(),
            _isReversed(),
            _position(),
            _velocity()
        {
            q_initialized = true;
        }

        uint32_t _timerPeriod;
        bool _isReversed;
        int32_t _position;
        int32_t _velocity;
    };
}

#endif // INCLUDE_QEI_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	   INCLUDE_QEI_BUILTIN_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_QEI_BUILTIN_H_

namespace HAL
{
    // HAL::QEI_Builtin Class
    // Built-in Quadrature Encoder Interface (QEI) peripheral
    // This class uses the hardware built into the LPC4337 
    // processor chip to decode the QEI signals. All the QEI signals 
    // are generated by an external encoder chip and then connected to 
    // the LPC4337 to be decoded
    class QEI_Builtin : public QEI
    {
    public:
        // Instantiate a QEI_Builtin singleton class object
        // return: Pointer to the QEI singleton class object
        static QEI* Init() { }

        // Getter method to get the Pointer to QEI base class instantiated in init() method
        // return: Pointer to the base class QEI object previously instantiated in Init()
        static QEI* GetQei() { }

        // Enable the QEI
        void Enable() override;

        // Disable the QEI
        void Disable() override;

        // Resets all hardware counters of the QEI
        void Reset() override;

        // Set the sampling period to calculate and update the position and speed
        // The counters should all be asynchrounously edge detection, and will not
        // be affected by this timing, except possible overflow
        // param period:   Update period in milliseconds
        void SetSamplePeriod(uint32_t period) override;

        // QEI Interrupt service routine (ISR)
        void IntHandler() override;

    private:
        // QEI constructor
        QEI_Builtin() :
            _isEnabled()
        { }

        static QEI_Builtin* _pQei;
        bool _isEnabled;
    };
}

#endif // INCLUDE_QEI_BUILTIN_H_

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     INCLUDE_LPC_H_
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_LPC_H_

// LPC base addresses were originally fixed hardware addresses, which threw exceptions when unit tested
// This provides an array of 32 bit addresses in which to set flags

// LPC18xx/43xx Specific Interrupt Numbers
enum IRQn_Type
{
    DAC_IRQn = 0,	                // DAC
    M0APP_IRQn = 1,	                // M0APP Core interrupt
    DMA_IRQn = 2,	                // DMA
    RESERVED1_IRQn = 3,	            // EZH/EDM
    RESERVED2_IRQn = 4,
    ETHERNET_IRQn = 5,	            // ETHERNET
    SDIO_IRQn = 6,                  // SDIO
    LCD_IRQn = 7,	                // LCD
    USB0_IRQn = 8,	                // USB0
    USB1_IRQn = 9,	                // USB1
    SCT_IRQn = 10,	                // SCT
    RITIMER_IRQn = 11,	            // RITIMER
    TIMER0_IRQn = 12,	            // TIMER0
    TIMER1_IRQn = 13,	            // TIMER1
    TIMER2_IRQn = 14,	            // TIMER2
    TIMER3_IRQn = 15,	            // TIMER3
    MCPWM_IRQn = 16,	            // MCPWM
    ADC0_IRQn = 17,	                // ADC0
    I2C0_IRQn = 18,	                // I2C0
    I2C1_IRQn = 19,	                // I2C1
    SPI_INT_IRQn = 20,	            // SPI_INT
    ADC1_IRQn = 21,	                // ADC1
    SSP0_IRQn = 22,	                // SSP0
    SSP1_IRQn = 23,	                // SSP1
    USART0_IRQn = 24,	            // USART0
    UART1_IRQn = 25,	            // UART1
    USART2_IRQn = 26,	            // USART2
    USART3_IRQn = 27,	            // USART3
    I2S0_IRQn = 28,	                // I2S0
    I2S1_IRQn = 29,	                // I2S1
    RESERVED4_IRQn = 30,
    SGPIO_INT_IRQn = 31,            // SGPIO_IINT
    PIN_INT0_IRQn = 32,             // PIN_INT0
    PIN_INT1_IRQn = 33,             // PIN_INT1
    PIN_INT2_IRQn = 34,             // PIN_INT2
    PIN_INT3_IRQn = 35,             // PIN_INT3
    PIN_INT4_IRQn = 36,             // PIN_INT4
    PIN_INT5_IRQn = 37,             // PIN_INT5
    PIN_INT6_IRQn = 38,             // PIN_INT6
    PIN_INT7_IRQn = 39,             // PIN_INT7
    GINT0_IRQn = 40,                // GINT0
    GINT1_IRQn = 41,                // GINT1
    EVENTROUTER_IRQn = 42,          // EVENTROUTER
    C_CAN1_IRQn = 43,               // C_CAN1
    RESERVED6_IRQn = 44,
    ADCHS_IRQn = 45,                // ADCHS interrupt
    ATIMER_IRQn = 46,               // ATIMER
    RTC_IRQn = 47,                  // RTC
    RESERVED8_IRQn = 48,
    WWDT_IRQn = 49,                 // WWDT
    M0SUB_IRQn = 50,                // M0SUB core interrupt
    C_CAN0_IRQn = 51,               // C_CAN0
    QEI_IRQn = 52                   // QEI
};

// LPC_TIMER_T Structure
// 32-bit Standard timer register block structure
typedef struct
{
    uint32_t IR;                    // Interrupt Register. The IR can be written to clear interrupts. The IR can be read to identify which of eight possible interrupt sources are pending
    uint32_t TCR;                   // Timer Control Register. The TCR is used to control the Timer Counter functions. The Timer Counter can be disabled or reset through the TCR
    uint32_t TC;                    // Timer Counter. The 32 bit TC is incremented every PR+1 cycles of PCLK. The TC is controlled through the TCR
    uint32_t PR;                    // Prescale Register. The Prescale Counter (below) is equal to this value, the next clock increments the TC and clears the PC
    uint32_t PC;                    // Prescale Counter. The 32 bit PC is a counter which is incremented to the value stored in PR. When the value in PR is reached, the TC is incremented and the PC is cleared. The PC is observable and controllable through the bus interface
    uint32_t MCR;                   // Match Control Register. The MCR is used to control if an interrupt is generated and if the TC is reset when a Match occurs
    uint32_t MR[4];                 // Match Register. MR can be enabled through the MCR to reset the TC, stop both the TC and PC, and/or generate an interrupt every time MR matches the TC
    uint32_t CCR;                   // Capture Control Register. The CCR controls which edges of the capture inputs are used to load the Capture Registers and whether or not an interrupt is generated when a capture takes place
    uint32_t CR[4];                 // Capture Register. CR is loaded with the value of TC when there is an event on the CAPn.0 input
    uint32_t EMR;                   // External Match Register. The EMR controls the external match pins MATn.0-3 (MAT0.0-3 and MAT1.0-3 respectively)
    uint32_t RESERVED0[12];
    uint32_t CTCR;                  // Count Control Register. The CTCR selects between Timer and Counter mode, and in Counter mode selects the signal and edge(s) for counting
}
LPC_TIMER_T;

// GPIO_PORT Structure
// GPIO port register block structure
typedef struct
{
    uint8_t B[128][32];	            // Offset 0x0000: Byte pin registers ports 0 to n; pins PIOn_0 to PIOn_31
    uint32_t W[32][32];	            // Offset 0x1000: Word pin registers port 0 to n
    uint32_t DIR[32];               // Offset 0x2000: Direction registers port n
    uint32_t MASK[32];              // Offset 0x2080: Mask register port n
    uint32_t PIN[32];               // Offset 0x2100: Portpin register port n
    uint32_t MPIN[32];              // Offset 0x2180: Masked port register port n
    uint32_t SET[32];               // Offset 0x2200: Write: Set register for port n Read: output bits for port n
    uint32_t CLR[32];               // Offset 0x2280: Clear port n
    uint32_t NOT[32];               // Offset 0x2300: Toggle port n
}
LPC_GPIO_T;

// LPC_GIMA_T GIMA Structure
// Global Input Multiplexer Array (GIMA) register block structure
typedef struct
{
    uint32_t  CAP0_IN[4][4];		// Timer x CAP0_y capture input multiplexer (GIMA output ((x*4)+y))
    uint32_t  CTIN_IN[8];			// SCT CTIN_x capture input multiplexer (GIMA output (16+x))
    uint32_t  ADCHS_TRIGGER_IN;     // ADCHS trigger input multiplexer (GIMA output 24)
    uint32_t  EVENTROUTER_13_IN;	// Event router input 13 multiplexer (GIMA output 25)
    uint32_t  EVENTROUTER_14_IN;	// Event router input 14 multiplexer (GIMA output 26)
    uint32_t  EVENTROUTER_16_IN;	// Event router input 16 multiplexer (GIMA output 27)
    uint32_t  ADCSTART0_IN;		    // ADC start0 input multiplexer (GIMA output 28)
    uint32_t  ADCSTART1_IN;		    // ADC start1 input multiplexer (GIMA output 29)
}
LPC_GIMA_T;

// LPC_SCU_T structure
// System Control Unit register block
typedef struct
{
    uint32_t  SFSP[16][32];
    uint32_t  RESERVED0[256];
    uint32_t  SFSCLK[4];            // Pin configuration register for pins CLK0-3
    uint32_t  RESERVED16[28];
    uint32_t  SFSUSB;               // Pin configuration register for USB
    uint32_t  SFSI2C0;              // Pin configuration register for I2C0-bus pins
    uint32_t  ENAIO[3];             // Analog function select registerS
    uint32_t  RESERVED17[27];
    uint32_t  EMCDELAYCLK;          // EMC clock delay register
    uint32_t  RESERVED18[63];
    uint32_t  PINTSEL[2];           // Pin interrupt select register for pin int 0 to 3 index 0, 4 to 7 index 1
}
LPC_SCU_T;

// GPIO_GROUP_INTn Structure
// GPIO grouped interrupt register block structure
typedef struct
{
    uint32_t  CTRL;                 // GPIO grouped interrupt control register
    uint32_t  RESERVED0[7];			// Reserved blick of 7 bytes
    uint32_t  PORT_POL[8];          // GPIO grouped interrupt port polarity register
    uint32_t  PORT_ENA[8];          // GPIO grouped interrupt port m enable register
    uint32_t  RESERVED1[1000];		// Reserved block of 1000 bytes
}
LPC_GPIOGROUPINT_T;

// PIN_INT Structure
// LPC18xx/43xx Pin Interrupt and Pattern Match register block structure
typedef struct
{
    uint32_t ISEL;                  // Pin Interrupt Mode register
    uint32_t IENR;                  // Pin Interrupt Enable (Rising) register
    uint32_t SIENR;	                // Set Pin Interrupt Enable (Rising) register
    uint32_t CIENR;                 // Clear Pin Interrupt Enable (Rising) register
    uint32_t IENF;                  // Pin Interrupt Enable Falling Edge / Active Level register
    uint32_t SIENF;                 // Set Pin Interrupt Enable Falling Edge / Active Level register
    uint32_t CIENF;                 // Clear Pin Interrupt Enable Falling Edge / Active Level address
    uint32_t RISE;                  // Pin Interrupt Rising Edge register
    uint32_t FALL;                  // Pin Interrupt Falling Edge register
    uint32_t IST;                   // Pin Interrupt Status register
}
LPC_PIN_INT_T;

// SCU function and mode selection definitions
// See the User Manual for specific modes and functions supoprted by the
// various LPC18xx/43xx devices. Functionality can vary per device
#define SCU_MODE_PULLUP             (0x0 << 3)          // Enable pull-up resistor at pad
#define SCU_MODE_REPEATER           (0x1 << 3)          // Enable pull-down and pull-up resistor at resistor at pad (repeater mode)
#define SCU_MODE_INACT              (0x2 << 3)          // Disable pull-down and pull-up resistor at resistor at pad
#define SCU_MODE_PULLDOWN           (0x3 << 3)          // Enable pull-down resistor at pad
#define SCU_MODE_HIGHSPEEDSLEW_EN   (0x1 << 5)          // Enable high-speed slew
#define SCU_MODE_INBUFF_EN          (0x1 << 6)          // Enable Input buffer
#define SCU_MODE_ZIF_DIS            (0x1 << 7)          // Disable input glitch filter
#define SCU_MODE_4MA_DRIVESTR       (0x0 << 8)          // Normal drive: 4mA drive strength
#define SCU_MODE_8MA_DRIVESTR       (0x1 << 8)          // Medium drive: 8mA drive strength
#define SCU_MODE_14MA_DRIVESTR      (0x2 << 8)          // High drive: 14mA drive strength
#define SCU_MODE_20MA_DRIVESTR      (0x3 << 8)          // Ultra high- drive: 20mA drive strength
#define SCU_MODE_FUNC0               0x0                // Selects pin function 0
#define SCU_MODE_FUNC1               0x1                // Selects pin function 1
#define SCU_MODE_FUNC2               0x2                // Selects pin function 2
#define SCU_MODE_FUNC3               0x3                // Selects pin function 3
#define SCU_MODE_FUNC4               0x4                // Selects pin function 4
#define SCU_MODE_FUNC5               0x5                // Selects pin function 5
#define SCU_MODE_FUNC6               0x6                // Selects pin function 6
#define SCU_MODE_FUNC7               0x7                // Selects pin function 7

// SCU function and mode selection definitions (old)
// For backwards compatibility
#define MD_PUP						(0x0 << 3)		    // Enable pull-up resistor at pad
#define MD_BUK						(0x1 << 3)		    // Enable pull-down and pull-up resistor at resistor at pad (repeater mode)
#define MD_PLN						(0x2 << 3)		    // Disable pull-down and pull-up resistor at resistor at pad
#define MD_PDN						(0x3 << 3)		    // Enable pull-down resistor at pad
#define MD_EHS						(0x1 << 5)		    // Enable fast slew rate
#define MD_EZI						(0x1 << 6)		    // Input buffer enable
#define MD_ZI						(0x1 << 7)		    // Disable input glitch filter
#define MD_EHD0						(0x1 << 8)		    // EHD driver strength low bit
#define MD_EHD1						(0x1 << 8)		    // EHD driver strength high bit
#define MD_PLN_FAST					(MD_PLN | MD_EZI | MD_ZI | MD_EHS)
#define I2C0_STANDARD_FAST_MODE		(1 << 3 | 1 << 11)	// Pin configuration for STANDARD/FAST mode I2C
#define I2C0_FAST_MODE_PLUS			(2 << 1 | 1 << 3 | 1 << 7 | 1 << 10 | 1 << 11)	// Pin configuration for Fast-mode Plus I2C
#define FUNC0						0x0				    // Pin function 0
#define FUNC1						0x1				    // Pin function 1
#define FUNC2						0x2				    // Pin function 2
#define FUNC3						0x3				    // Pin function 3
#define FUNC4						0x4				    // Pin function 4
#define FUNC5						0x5				    // Pin function 5
#define FUNC6						0x6				    // Pin function 6
#define FUNC7						0x7				    // Pin function 7
#define PORT_OFFSET					0x80			    // Port offset definition
#define PIN_OFFSET					0x04			    // Pin offset definition

// Returns the SFSP register address in the SCU for a pin and port, recommend using (*(volatile int *) &LPC_SCU->SFSP[po][pi];)
#define LPC_SCU_PIN(LPC_SCU_BASE, po, pi) (*(volatile int *) ((LPC_SCU_BASE) + ((po) * 0x80) + ((pi) * 0x4))

// Returns the address in the SCU for a SFSCLK clock register, recommend using (*(volatile int *) &LPC_SCU->SFSCLK[c];)
#define LPC_SCU_CLK(LPC_SCU_BASE, c) (*(volatile int *) ((LPC_SCU_BASE) +0xC00 + ((c) * 0x4)))

// CCU clock config/status register pair
typedef struct
{
    uint32_t  CFG;  // CCU clock configuration register
    uint32_t  STAT; // CCU clock status register
}
CCU_CFGSTAT_T;

typedef enum CHIP_CCU_CLK
{
    // CCU1 clocks
    CLK_APB3_BUS,							// APB3 bus clock from base clock CLK_BASE_APB3
    CLK_APB3_I2C1,							// I2C1 register/perigheral clock from base clock CLK_BASE_APB3
    CLK_APB3_DAC,							// DAC peripheral clock from base clock CLK_BASE_APB3
    CLK_APB3_ADC0,							// ADC0 register/perigheral clock from base clock CLK_BASE_APB3
    CLK_APB3_ADC1,							// ADC1 register/perigheral clock from base clock CLK_BASE_APB3
    CLK_APB3_CAN0,							// CAN0 register/perigheral clock from base clock CLK_BASE_APB3
    CLK_APB1_BUS = 32,						// APB1 bus clock clock from base clock CLK_BASE_APB1
    CLK_APB1_MOTOCON,						// Motor controller register/perigheral clock from base clock CLK_BASE_APB1
    CLK_APB1_I2C0,							// I2C0 register/perigheral clock from base clock CLK_BASE_APB1
    CLK_APB1_I2S,							// I2S register/perigheral clock from base clock CLK_BASE_APB1
    CLK_APB1_CAN1,							// CAN1 register/perigheral clock from base clock CLK_BASE_APB1
    CLK_SPIFI = 64,							// SPIFI SCKI input clock from base clock CLK_BASE_SPIFI
    CLK_MX_BUS = 96,						// M3/M4 BUS core clock from base clock CLK_BASE_MX
    CLK_MX_SPIFI,							// SPIFI register clock from base clock CLK_BASE_MX
    CLK_MX_GPIO,							// GPIO register clock from base clock CLK_BASE_MX
    CLK_MX_LCD,								// LCD register clock from base clock CLK_BASE_MX
    CLK_MX_ETHERNET,						// ETHERNET register clock from base clock CLK_BASE_MX
    CLK_MX_USB0,							// USB0 register clock from base clock CLK_BASE_MX
    CLK_MX_EMC,								// EMC clock from base clock CLK_BASE_MX
    CLK_MX_SDIO,							// SDIO register clock from base clock CLK_BASE_MX
    CLK_MX_DMA,								// DMA register clock from base clock CLK_BASE_MX
    CLK_MX_MXCORE,							// M3/M4 CPU core clock from base clock CLK_BASE_MX
    RESERVED_ALIGN = CLK_MX_MXCORE + 3,
    CLK_MX_SCT,								// SCT register clock from base clock CLK_BASE_MX
    CLK_MX_USB1,							// USB1 register clock from base clock CLK_BASE_MX
    CLK_MX_EMC_DIV,							// ENC divider clock from base clock CLK_BASE_MX
    CLK_MX_FLASHA,							// FLASHA bank clock from base clock CLK_BASE_MX
    CLK_MX_FLASHB,							// FLASHB bank clock from base clock CLK_BASE_MX
    CLK_M4_M0APP,							// M0 app CPU core clock from base clock CLK_BASE_MX
    CLK_MX_ADCHS,							// ADCHS clock from base clock CLK_BASE_ADCHS
    CLK_MX_EEPROM,							// EEPROM clock from base clock CLK_BASE_MX
    CLK_MX_WWDT = 128,						// WWDT register clock from base clock CLK_BASE_MX
    CLK_MX_UART0,							// UART0 register clock from base clock CLK_BASE_MX
    CLK_MX_UART1,							// UART1 register clock from base clock CLK_BASE_MX
    CLK_MX_SSP0,							// SSP0 register clock from base clock CLK_BASE_MX
    CLK_MX_TIMER0,							// TIMER0 register/perigheral clock from base clock CLK_BASE_MX
    CLK_MX_TIMER1,							// TIMER1 register/perigheral clock from base clock CLK_BASE_MX
    CLK_MX_SCU,								// SCU register/perigheral clock from base clock CLK_BASE_MX
    CLK_MX_CREG,							// CREG clock from base clock CLK_BASE_MX
    CLK_MX_RITIMER = 160,					// RITIMER register/perigheral clock from base clock CLK_BASE_MX
    CLK_MX_UART2,							// UART3 register clock from base clock CLK_BASE_MX
    CLK_MX_UART3,							// UART4 register clock from base clock CLK_BASE_MX
    CLK_MX_TIMER2,							// TIMER2 register/perigheral clock from base clock CLK_BASE_MX
    CLK_MX_TIMER3,							// TIMER3 register/perigheral clock from base clock CLK_BASE_MX
    CLK_MX_SSP1,							// SSP1 register clock from base clock CLK_BASE_MX
    CLK_MX_QEI,								// QEI register/perigheral clock from base clock CLK_BASE_MX
    CLK_PERIPH_BUS = 192,					// Peripheral bus clock from base clock CLK_BASE_PERIPH
    CLK_RESERVED3,
    CLK_PERIPH_CORE,						// Peripheral core clock from base clock CLK_BASE_PERIPH
    CLK_PERIPH_SGPIO,						// SGPIO clock from base clock CLK_BASE_PERIPH
    CLK_USB0 = 224,							// USB0 clock from base clock CLK_BASE_USB0
    CLK_USB1 = 256,							// USB1 clock from base clock CLK_BASE_USB1
    CLK_SPI = 288,							// SPI clock from base clock CLK_BASE_SPI
    CLK_ADCHS = 320,						// ADCHS clock from base clock CLK_BASE_ADCHS
    CLK_CCU1_LAST,

    // CCU2 clocks
    CLK_CCU2_START,
    CLK_APLL = CLK_CCU2_START,				// Audio PLL clock from base clock CLK_BASE_APLL
    RESERVED_ALIGNB = CLK_CCU2_START + 31,
    CLK_APB2_UART3,							// UART3 clock from base clock CLK_BASE_UART3
    RESERVED_ALIGNC = CLK_CCU2_START + 63,
    CLK_APB2_UART2,							// UART2 clock from base clock CLK_BASE_UART2
    RESERVED_ALIGND = CLK_CCU2_START + 95,
    CLK_APB0_UART1,							// UART1 clock from base clock CLK_BASE_UART1
    RESERVED_ALIGNE = CLK_CCU2_START + 127,
    CLK_APB0_UART0,							// UART0 clock from base clock CLK_BASE_UART0
    RESERVED_ALIGNF = CLK_CCU2_START + 159,
    CLK_APB2_SSP1,							// SSP1 clock from base clock CLK_BASE_SSP1
    RESERVED_ALIGNG = CLK_CCU2_START + 191,
    CLK_APB0_SSP0,							// SSP0 clock from base clock CLK_BASE_SSP0
    RESERVED_ALIGNH = CLK_CCU2_START + 223,
    CLK_APB2_SDIO,							// SDIO clock from base clock CLK_BASE_SDIO
    CLK_CCU2_LAST
}
CHIP_CCU_CLK_T;

// (0x40051000) CCU1 Structure
// CCU1 register block structure
typedef struct
{
    uint32_t PM;                            // (0x40051000) CCU1 power mode register
    uint32_t BASE_STAT;                     // (0x40051004) CCU1 base clocks status register
    uint32_t RESERVED0[62];                 // Reserved 62 Bytes
    CCU_CFGSTAT_T CLKCCU[CLK_CCU1_LAST];    // (0x40051100) Start of CCU1 clock registers
}
LPC_CCU1_T;

typedef struct
{
    uint32_t ISER[1U];                      // Offset: 0x000 (R/W)  Interrupt Set Enable Register
    uint32_t RESERVED0[31U];
    uint32_t ICER[1U];                      // Offset: 0x080 (R/W)  Interrupt Clear Enable Register
    uint32_t RSERVED1[31U];
    uint32_t ISPR[1U];                      // Offset: 0x100 (R/W)  Interrupt Set Pending Register
    uint32_t RESERVED2[31U];
    uint32_t ICPR[1U];                      // Offset: 0x180 (R/W)  Interrupt Clear Pending Register
    uint32_t RESERVED3[31U];
    uint32_t RESERVED4[64U];
    uint32_t IP[8U];                        // Offset: 0x300 (R/W)  Interrupt Priority Register
}
NVIC_Type;

typedef struct
{
    uint32_t CPUID;						    // Offset: 0x000 (R/ )  CPUID Base Register
    uint32_t ICSR;						    // Offset: 0x004 (R/W)  Interrupt Control and State Register
    uint32_t RESERVED0;
    uint32_t AIRCR;						    // Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register
    uint32_t SCR;						    // Offset: 0x010 (R/W)  System Control Register
    uint32_t CCR;						    // Offset: 0x014 (R/W)  Configuration Control Register
    uint32_t RESERVED1;
    uint32_t SHP[2U];					    // Offset: 0x01C (R/W)  System Handlers Priority Registers. [0] is RESERVED
    uint32_t SHCSR;						    // Offset: 0x024 (R/W)  System Handler Control and State Register
}
SCB_Type;

// QEI Structure
typedef struct
{
    uint32_t CON;						    // Control register      
    uint32_t STAT;						    // Encoder status register
    uint32_t CONF;						    // Configuration register
    uint32_t POS;						    // Position register     
    uint32_t MAXPOS;					    // Maximum position register
    uint32_t CMPOS0;					    // position compare register 0
    uint32_t CMPOS1;					    // position compare register 1
    uint32_t CMPOS2;					    // position compare register 2
    uint32_t INXCNT;					    // Index count register  
    uint32_t INXCMP0;					    // Index compare register 0
    uint32_t LOAD;						    // Velocity timer reload register
    uint32_t TIME;						    // Velocity timer register
    uint32_t VEL;						    // Velocity counter register
    uint32_t CAP;						    // Velocity capture register
    uint32_t VELCOMP;					    // Velocity compare register
    uint32_t FILTERPHA;					    // Digital filter register on input phase A (QEI_A)
    uint32_t FILTERPHB;					    // Digital filter register on input phase B (QEI_B)
    uint32_t FILTERINX;					    // Digital filter register on input index (QEI_IDX)
    uint32_t WINDOW;					    // Index acceptance window register
    uint32_t INXCMP1;					    // Index compare register 1
    uint32_t INXCMP2;					    // Index compare register 2
    uint32_t RESERVED0[993];
    uint32_t IEC;						    // Interrupt enable clear register
    uint32_t IES;						    // Interrupt enable set register
    uint32_t INTSTAT;					    // Interrupt status register
    uint32_t IE;						    // Interrupt enable register
    uint32_t CLR;						    // Interrupt status clear register
    uint32_t SET;						    // Interrupt status set register
}
LPC_QEI_T;

// Interrupt Priorities are WORD accessible only under ARMv6M                  
// The following MACROS handle generation of the register offset and byte masks
#define _BIT_SHIFT(IRQn)         (  ((((uint32_t)(int32_t)(IRQn))         )      &  0x03UL) * 8UL)
#define _SHP_IDX(IRQn)           ( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >>    2UL)      )
#define _IP_IDX(IRQn)            (   (((uint32_t)(int32_t)(IRQn))                >>    2UL)      )

#define SCU_PINIO_FAST           ((0x2 << 3) | (0x1 << 5) | (0x1 << 6) | SCU_MODE_INBUFF_EN )

#define ADC_IRQ_PRIORITY     1UL
#define TIMER_IRQ_PRIORITY   1UL
#define GINT_IRQ_PRIORITY    1UL
#define PIN_INT_IRQ_PRIORITY 1UL
#define __NVIC_PRIO_BITS     2U

extern bool testTimerEnabled;
extern uint32_t TEST_NVIC_IRQ_FLAGS;

extern LPC_CCU1_T LPC_CCU1[sizeof(LPC_CCU1_T)];
extern LPC_QEI_T LPC_QEI[sizeof(LPC_QEI_T)];
extern NVIC_Type NVIC[sizeof(NVIC_Type)];
extern SCB_Type SCB[sizeof(SCB_Type)];
extern uint32_t NVIC_GetEnableIRQ(IRQn_Type);
extern uint32_t TEST_NVIC_IRQ_FLAGS;

// Memory mapping of Cortex-M0 Hardware
#define SCS_BASE            (0xE000E000UL)               // System Control Space Base Address
#define SysTick_BASE        (SCS_BASE +  0x0010UL)       // SysTick Base Address
#define NVIC_BASE           (SCS_BASE +  0x0100UL)       // NVIC Base Address
#define SCB_BASE            (SCS_BASE +  0x0D00UL)       // System Control Block Base Address

static inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
    if ((int32_t)(IRQn) >= 0)
    {
        NVIC->ISER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
    }
}

static inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
    if ((int32_t)(IRQn) >= 0)
    {
        NVIC->ICER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
    }
}

static inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn) { }

// NVIC_SetPriority Method
// Set Interrupt Priority
// Sets the priority of a device specific interrupt or a processor exception
// The interrupt number can be positive to specify a device specific interrupt,
// or negative to specify a processor exception.
// param [in]      IRQn  Interrupt number
// param [in]  priority  Priority to set
// note    The priority cannot be set for every processor exception
static inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
    // Simplify by loading test NVIC params instead of figuring out bit setting....
    // testNVIC.test_IRQn = IRQn;
    // testNVIC.test_Priority = priority;

    if ((int32_t)(IRQn) >= 0)
    {
        NVIC->IP[_IP_IDX(IRQn)] = ((uint32_t)(NVIC->IP[_IP_IDX(IRQn)] & ~(0xFFUL << _BIT_SHIFT(IRQn))) |
            (((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL) << _BIT_SHIFT(IRQn)));
    }
    else
    {
        SCB->SHP[_SHP_IDX(IRQn)] = ((uint32_t)(SCB->SHP[_SHP_IDX(IRQn)] & ~(0xFFUL << _BIT_SHIFT(IRQn))) |
            (((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL) << _BIT_SHIFT(IRQn)));
    }
}

#endif // INCLUDE_LPC_H_

#endif // _HEADER_BLOCKS_H_
