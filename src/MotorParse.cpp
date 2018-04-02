// Copyright 2018 Toyota Research Institute.  All rights reserved.
#include <iostream>
#include <memory>
#include <regex>
#include <string>

#include "jaguar4x4_base/MotorParse.h"
#include "jaguar4x4_base/Utils.h"

static double adToTemperature(uint16_t adValue)
{
  static const double resTable[25] = {
    114660, 84510, 62927, 47077, 35563,
    27119, 20860, 16204, 12683, 10000,
    7942, 6327, 5074, 4103, 3336,
    2724, 2237, 1846, 1530, 1275,
    1068, 899.3, 760.7, 645.2, 549.4
  };
  static const double tempTable[25] = {
    -20, -15, -10, -5, 0,
    5, 10, 15, 20, 25,
    30, 35, 40, 45, 50,
    55, 60, 65, 70, 75,
    80, 85, 90, 95, 100
  };
  static const double FULLAD = 4095;

  // for new temperature sensor
  double tempM = 0;
  double k = (adValue / FULLAD);
  double resValue = 0;

  if (k != 0) {
    resValue = (10000 / k -10000);      //AD value to resistor
  } else {
    resValue = resTable[0];
  }

  int index = -1;
  if (resValue >= resTable[0]) {      //too lower
    tempM = -20;
  } else if (resValue <= resTable[24]) {
    tempM = 100;
  } else {
    for (int i = 0; i < 24; ++i) {
      if ((resValue <= resTable[i]) && (resValue >= resTable[i + 1])) {
        index = i;
        break;
      }
    }
    if (index >= 0) {
      tempM = tempTable[index] + (resValue - resTable[index]) / (resTable[index + 1] - resTable[index]) * (tempTable[index + 1] - tempTable[index]);
    } else {
      tempM = 0;
    }
  }

  return tempM;
}

MotorTempMsg::MotorTempMsg(uint16_t temp1, uint16_t temp2)
  : AbstractMotorMsg(AbstractMotorMsg::MessageType::motor_temperature),
    motor_temp_adc_1_(temp1), motor_temp_adc_2_(temp2)
{
  motor_temp_1_ = adToTemperature(temp1);
  motor_temp_2_ = adToTemperature(temp2);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorAmpMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("A=(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorAmpMsg>(str_to_d(sm[1]) / 10.0,
                                         str_to_d(sm[2]) / 10.0);
  }

  std::cerr << "Motor Amp Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorEncPosMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("C=(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorEncPosMsg>(std::stol(sm[1]),
                                            std::stol(sm[2]));
  }

  std::cerr << "Motor EncPos Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorEncPosDiffMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("CR=(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorEncPosDiffMsg>(std::stol(sm[1]),
                                                std::stol(sm[2]));
  }

  std::cerr << "Motor EncPosDiff Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnDigitalInputMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("D=(-?[0-9]*?)$"))) {
    return std::make_unique<MotorDigitalInputMsg>(std::stoul(sm[1]));
  }

  std::cerr << "Motor Mode Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnDigitalOutputMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("DO=(-?[0-9]*?)$"))) {
    return std::make_unique<MotorDigitalOutputMsg>(std::stoul(sm[1]));
  }

  std::cerr << "Motor Mode Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorPowerMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("P=(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorPowerMsg>(std::stol(sm[1]),
                                           std::stol(sm[2]));
  }

  std::cerr << "Motor Power Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorEncVelMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("S=(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorEncVelMsg>(std::stol(sm[1]),
                                            std::stol(sm[2]));
  }

  std::cerr << "Motor EncVel Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorBoardTempMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("T=(-?[0-9]*?):(-?[0-9]*?)$"))) {
      return std::make_unique<MotorBoardTempMsg>(str_to_d(sm[1]),
                                                 str_to_d(sm[2]));
  }

  std::cerr << "Motor BoardTemp Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorVoltageMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("V=(-?[0-9]*?):(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorVoltageMsg>(
                                             str_to_d(sm[1])/10.0,
                                             str_to_d(sm[2])/10.0,
                                             str_to_d(sm[3])/1000.0);
  }

  std::cerr << "Motor Voltage Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorTempMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm,
                       std::regex("AI=(-?[0-9]*?):(-?[0-9]*?):(-?[0-9]*?):(-?[0-9]*?)$"))) {
      return std::make_unique<MotorTempMsg>(std::stoul(sm[3]),
                                            std::stoul(sm[4]));
  }

  std::cerr << "Motor Temp Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorFlagsMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("FF=(-?[0-9]*?)$"))) {
    return std::make_unique<MotorFlagsMsg>(std::stoul(sm[1]));
  }

  std::cerr << "Motor Flags Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorModeMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("MMOD=(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorModeMsg>(std::stoul(sm[1]),
                                          std::stoul(sm[2]));
  }

  std::cerr << "Motor Mode Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

std::unique_ptr<AbstractMotorMsg> parseMotorMessage(const std::string& msg)
{
  std::smatch sm;

  // nothing before the "\r"
  if (msg.empty()) {
    return std::unique_ptr<AbstractMotorMsg>(nullptr);
  }

  if (startsWith(msg, "A=")) {
    return parseAndReturnMotorAmpMsg(msg);
  } else if (startsWith(msg, "C=")) {
    return parseAndReturnMotorEncPosMsg(msg);
  } else if (startsWith(msg, "CR=")) {
    return parseAndReturnMotorEncPosDiffMsg(msg);
  } else if (startsWith(msg, "D=")) {
    return parseAndReturnDigitalInputMsg(msg);
  } else if (startsWith(msg, "DO=")) {
    return parseAndReturnDigitalOutputMsg(msg);
  } else if (startsWith(msg, "P=")) {
    return parseAndReturnMotorPowerMsg(msg);
  } else if (startsWith(msg, "S=")) {
    return parseAndReturnMotorEncVelMsg(msg);
  } else if (startsWith(msg, "T=")) {
    return parseAndReturnMotorBoardTempMsg(msg);
  } else if (startsWith(msg, "V=")) {
    return parseAndReturnMotorVoltageMsg(msg);
  } else if (startsWith(msg, "+")) {
    // valid command received
    return std::make_unique<MotorCmdAcceptedMsg>();
  } else if (startsWith(msg, "-")) {
    // INvalid command received
    return std::make_unique<MotorCmdRejectedMsg>();
  } else if (startsWith(msg, "AI=")) {
    return parseAndReturnMotorTempMsg(msg);
  } else if (startsWith(msg, "FF=")) {
    return parseAndReturnMotorFlagsMsg(msg);
  } else if (startsWith(msg, "MMOD=")) {
    return parseAndReturnMotorModeMsg(msg);
  }

  std::cerr << "UNKNOWN MOTOR MESSAGE TYPE '"<< msg << "' (" << dumpHex(msg) << ")" << std::endl;

  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}
