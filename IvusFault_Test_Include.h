// Copyright (c) 2019-2022 Terumo Heart, Inc. All Rights Reserved.
// Any use of this software without the express written 
// consent of Terumo Heart, Inc. is strictly prohibited.

#include "gtest/gtest_prod.h"
#include "IvusFault_Test.h"

#ifndef FRIEND_TEST_FIXTURE_IVUS_FAULT
#define FRIEND_TEST_FIXTURE_IVUS_FAULT \
FRIEND_TEST(IvusFault_Test, Test_Init); \
FRIEND_TEST(IvusFault_Test, Test_GetIvusFault); \
FRIEND_TEST(IvusFault_Test, Test_SetComm); \
FRIEND_TEST(IvusFault_Test, Test_IvusFault); \
FRIEND_TEST(IvusFault_Test, Test_HandleMq);
#endif // FRIEND_TEST_FIXTURE_IVUS_FAULT
