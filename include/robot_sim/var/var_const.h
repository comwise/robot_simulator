#ifndef __COMWISE__VAR_CONST__H__
#define __COMWISE__VAR_CONST__H__

#include <cstdint>
#include <string>

namespace robot {

//! error string
static const char* kReturnBoolString[] = {"error", "ok"};

//! topic key
static constexpr char kTfTopic[] = "tf_topic";

static constexpr char kConfigGetServiceName[] = "get_config";
static constexpr char kConfigSetServiceName[] = "set_config";

static constexpr char kOdomTopic[] = "odom";
static constexpr char kCmdVelTopic[] = "move_cmd";
static constexpr char kCmdFeedbackTopic[] = "move_feedback";

} // namespace robot

#endif // __COMWISE__VAR_CONST__H__
