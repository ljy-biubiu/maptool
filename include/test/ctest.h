#ifndef __CTEST_H__
#define __CTEST_H__

#include <iostream>
#include <vector>
#include <stdio.h>
#include <string.h>

#include "test/hmapdata.h"
#include "test/routetest.h"
#include "test/attributetest.h"
#include "test/datareliabletest.h"
#include "dialog_widget/testResult_dialog.h"

using namespace cti::test;

class CTest
{
public:
  CTest();
  ~CTest();
  void test(const std::string filepath);

public:
  std::vector<std::string> resulterrs_;
  std::vector<std::string> dataresults_;
  std::vector<std::string> clearattresults_;

private:
  HmapData *hdata;
};

#endif