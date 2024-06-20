#include "intentcode.h"
#include <unordered_map>

const std::string &enum_string(IntentCode intentcode) {
  static const std::unordered_map<IntentCode, std::string> intentcode_string = {
      {IntentCode::INTENT_DEFAULT, "INTENT_DEFAULT"},
      {IntentCode::NODE_CLE_OUTPUT, "NODE_CLE_OUTPUT"},
      {IntentCode::NODE_DEDICATED, "NODE_DEDICATED"},
      {IntentCode::NODE_DOUBLE, "NODE_DOUBLE"},
      {IntentCode::NODE_GLOBAL_BUFG, "NODE_GLOBAL_BUFG"},
      {IntentCode::NODE_GLOBAL_HDISTR, "NODE_GLOBAL_HDISTR"},
      {IntentCode::NODE_GLOBAL_HROUTE, "NODE_GLOBAL_HROUTE"},
      {IntentCode::NODE_GLOBAL_LEAF, "NODE_GLOBAL_LEAF"},
      {IntentCode::NODE_GLOBAL_VDISTR, "NODE_GLOBAL_VDISTR"},
      {IntentCode::NODE_GLOBAL_VROUTE, "NODE_GLOBAL_VROUTE"},
      {IntentCode::NODE_HLONG, "NODE_HLONG"},
      {IntentCode::NODE_HQUAD, "NODE_HQUAD"},
      {IntentCode::NODE_INT_INTERFACE, "NODE_INT_INTERFACE"},
      {IntentCode::NODE_LAGUNA_DATA, "NODE_LAGUNA_DATA"},
      {IntentCode::NODE_LAGUNA_OUTPUT, "NODE_LAGUNA_OUTPUT"},
      {IntentCode::NODE_LOCAL, "NODE_LOCAL"},
      {IntentCode::NODE_OPTDELAY, "NODE_OPTDELAY"},
      {IntentCode::NODE_OUTPUT, "NODE_OUTPUT"},
      {IntentCode::NODE_PINBOUNCE, "NODE_PINBOUNCE"},
      {IntentCode::NODE_PINFEED, "NODE_PINFEED"},
      {IntentCode::NODE_SINGLE, "NODE_SINGLE"},
      {IntentCode::NODE_VLONG, "NODE_VLONG"},
      {IntentCode::NODE_VQUAD, "NODE_VQUAD"},
      {IntentCode::NODE_HDOUBLE, "NODE_HDOUBLE"},
      {IntentCode::NODE_HSINGLE, "NODE_HSINGLE"},
      {IntentCode::NODE_VDOUBLE, "NODE_VDOUBLE"},
      {IntentCode::NODE_VSINGLE, "NODE_VSINGLE"},
  };
  return intentcode_string.at(intentcode);
}
