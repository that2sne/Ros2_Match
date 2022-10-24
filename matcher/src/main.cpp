#include "matcher/match_node.hpp"
#include "matcher/parameter/match_parameter.hpp"
#include "matcher/parameter/node_parameter_factory.hpp"

void termination_handler(int signum)
{
  rclcpp::shutdown();

  exit(0);
}

using namespace ebase::fusion::matcher;
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // handle Ctrl-C
  if (signal(SIGINT, termination_handler) == SIG_IGN) signal(SIGINT, SIG_IGN);
  rclcpp::Node::SharedPtr match = std::make_shared<Matcher>();

  // To start the parameter service, must call the ParameterServiceInit function.
  NodeParameterFactory<MatchParameter>::get()->ParameterServiceInit(match);

  rclcpp::spin(match);
  rclcpp::shutdown();
  return 0;
}