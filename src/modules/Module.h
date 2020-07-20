#pragma once

class Module
{
public:
  std::string name = "";

  virtual void setup() = 0;

  virtual void loop() = 0;

  virtual void handleMsg(std::string msg) = 0;
};
