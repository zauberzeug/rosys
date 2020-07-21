#pragma once

class Module
{
public:
  std::string name = "";

  virtual void setup(){};

  virtual void loop(){};

  virtual void handleMsg(std::string msg) = 0;
};
