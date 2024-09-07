#include "px4_behavior/bt_ros2_node.hpp"

#define INPUT_KEY_IF "if"
#define INPUT_KEY_ELSE "else"
#define INPUT_KEY_CHECK_WHILE_RUNNING "check_while_running"

using namespace BT;

namespace px4_behavior::ops_engine {

/**
 * Same as PreconditionNode but when the condition is true, this node returns RUNNING immediately and executes the tick
 * in the next iteration.
 */
class AsyncPrecondition : public DecoratorNode
{
   public:
    AsyncPrecondition(const std::string& name, const NodeConfig& config) : DecoratorNode(name, config)
    {
        loadExecutor();
    }

    virtual ~AsyncPrecondition() override = default;

    static PortsList providedPorts()
    {
        return {InputPort<std::string>(INPUT_KEY_IF,
                                       "If true, return RUNNING. Otherwise, return what's specified in argument '" +
                                           std::string(INPUT_KEY_ELSE) + "'"),
                InputPort<NodeStatus>(INPUT_KEY_ELSE,
                                      NodeStatus::SKIPPED,
                                      "Return status if condition is "
                                      "false"),
                InputPort<bool>(INPUT_KEY_CHECK_WHILE_RUNNING,
                                false,
                                "Check the condition in argument '" + std::string(INPUT_KEY_IF) +
                                    "' also while the child is RUNNING")};
    }

   private:
    virtual BT::NodeStatus tick() override
    {
        // Load the result of the condition
        loadExecutor();
        Ast::Environment env = {config().blackboard, config().enums};
        bool condition_result = _executor(env).cast<bool>();

        BT::NodeStatus else_return;
        if (!getInput(INPUT_KEY_ELSE, else_return)) {
            throw RuntimeError("Missing parameter [" + std::string(INPUT_KEY_ELSE) + "] in Precondition");
        }

        // Return RUNNING the first time the condition is true
        if (status() == NodeStatus::IDLE) {
            if (condition_result) {
                setStatus(NodeStatus::RUNNING);  // Change the current status
                return NodeStatus::RUNNING;
            };
            return else_return;
        }

        if (getInput<bool>(INPUT_KEY_CHECK_WHILE_RUNNING).value() && !condition_result) { return else_return; }

        // Propagate the tick
        auto const child_status = child_node_->executeTick();
        if (isStatusCompleted(child_status)) { resetChild(); }
        return child_status;
    }

    void loadExecutor()
    {
        std::string script;
        if (!getInput(INPUT_KEY_IF, script)) {
            throw RuntimeError("Missing parameter [" + std::string(INPUT_KEY_IF) + "] in Precondition");
        }
        if (script == _script) { return; }
        auto executor = ParseScript(script);
        if (!executor) { throw RuntimeError(executor.error()); }
        else {
            _executor = executor.value();
            _script = script;
        }
    }

    std::string _script;
    ScriptFunction _executor;
};

}  // namespace px4_behavior::ops_engine

#include "px4_behavior/register_behavior_tree_node_macro.hpp"
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::ops_engine::AsyncPrecondition)