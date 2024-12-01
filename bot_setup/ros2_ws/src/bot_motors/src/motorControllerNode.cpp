#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bot_interfaces/action/motors_instruct.hpp"
#include "bot_motors/motorDriver.hpp"

//L289N pins to GPIO pins
#define IN1 23
#define IN2 24
#define IN3 17
#define IN4 27
#define ENA 25
#define ENB 22

using motorDriver:: MotorDriver;
using motorDriver::MotorDirection;
using MotorsInstruct = bot_interfaces::action::MotorsInstruct;
using MotorsGoalHandle = rclcpp_action::ServerGoalHandle<MotorsInstruct>;
using namespace std::placeholders;

/**
 * @class MotorNode
 * @brief Represents an action server for robot's motor controller (L298N). 
 * 
 * This class node receives requests in the form of MotorsInstruct 
 * actions which contain instructions related to direction and motor efforts.
 */
class MotorControllerNode : public rclcpp::Node 
{
    public:
        MotorControllerNode() :
	    Node("Motors_Node"),
            leftMotors_(std::make_unique<MotorDriver>(
				IN3, IN4, ENB, "leftMotors")),
            rightMotors_(std::make_unique<MotorDriver>(
				IN1, IN2, ENA, "rightMotors"))
        {
		callback_group_1_ = this->create_callback_group(
            		rclcpp::CallbackGroupType::Reentrant);

		motor_contr_server_ = rclcpp_action::create_server<MotorsInstruct>(
				this,
				"motor_controller_actions",
				std::bind(&MotorControllerNode::goal_callback, this, _1, _2),
				std::bind(&MotorControllerNode::cancel_callback, this, _1),
				std::bind(&MotorControllerNode::handle_accepted_callback, this, 
					_1),
				rcl_action_server_get_default_options(),
				callback_group_1_
		);
		
		RCLCPP_INFO(this->get_logger(), "Motor controller node initialized");
        }

    private:
    	std::unique_ptr <MotorDriver> leftMotors_; 
    	std::unique_ptr <MotorDriver> rightMotors_; 

		std::mutex mutex_;

		std::shared_ptr<MotorsGoalHandle> current_handle_goal_;//motor instructs
		rclcpp_action::Server<MotorsInstruct>::SharedPtr motor_contr_server_;
		rclcpp::CallbackGroup::SharedPtr callback_group_1_;
	
	/**
	 * @brief Handle incoming motor controller goal inside MotorsInstruct
	 *
	 * A new motor instruction (goal) is accepted provided that a current
	 * goal isn't being executed, otherwise reject new goal.
	 */
	rclcpp_action::GoalResponse goal_callback(
		const rclcpp_action::GoalUUID &uuid, 
		std::shared_ptr<const MotorsInstruct::Goal> new_motors_goal)
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (current_handle_goal_) 
			{
				if (current_handle_goal_ -> is_active())
				{
					RCLCPP_INFO(this->get_logger(), 
						"[Motor Server]: A goal is active, rejected new goal");
					return rclcpp_action::GoalResponse::REJECT;
				}
			}
		}

		(void) uuid;
		(void) new_motors_goal;
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	/**
	 * @brief Handle a rejected motor controller goal request
	 * 
	 */
	rclcpp_action::CancelResponse cancel_callback(
		const std::shared_ptr<MotorsGoalHandle> goal_handle)
	{
		(void) goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}
	
	/**
	 * @brief Handle accepted motor goal sent from robot manager node
	 *@param new_goal_handle the latest accepted motor instructions
	 */
	void handle_accepted_callback(
		const std::shared_ptr<MotorsGoalHandle> new_goal_handle)
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			this -> current_handle_goal_ = new_goal_handle;
		}

		execute_motor_goal_(new_goal_handle);
		send_goal_result_(new_goal_handle);

	}

	/**
	 *@brief Set left and right motor efforts and directions
	 * 
	 * @param goal_handle the action containing motor joy-stick values
	 */
	void execute_motor_goal_(const std::shared_ptr<MotorsGoalHandle> 
		goal_handle)
	{
		double new_left_motors_effort, new_right_motors_effort;
		MotorDirection new_left_motors_dir, new_right_motors_dir;

		auto goal = goal_handle -> get_goal();
		new_left_motors_effort = goal -> left_motors_effort;
		new_right_motors_effort = goal -> right_motors_effort;

		get_new_motors_direction(new_left_motors_dir, new_right_motors_dir, 
			goal_handle);
		set_change_in_motors_direction(new_left_motors_dir, 
			new_right_motors_dir);

		// new_left_motors_effort = controllerInput_to_motorEffort(
		// 	abs(left_joy_y));
		// new_right_motors_effort = controllerInput_to_motorEffort(
		// 	abs(right_joy_y));

		leftMotors_->setEffortPercent(new_left_motors_effort);
		rightMotors_->setEffortPercent(new_right_motors_effort);
	}

	/**
	 * @brief Based on new motor instructions determine the direction for left &
	 *       right motors. (Utility func. for execute_motor_goal) 
	 * @param goal_handle contains the new motor instructions	
	 * @post the parameters for motors direction is set
	 */
	void get_new_motors_direction(
		MotorDirection& new_left_motors_dir, 
		MotorDirection& new_right_motors_dir, 
		const std::shared_ptr<MotorsGoalHandle> goal_handle)
	{
		auto goal = goal_handle -> get_goal();

		if (goal -> steer_right)
		{
			new_left_motors_dir = MotorDirection::Backward;
			new_right_motors_dir = MotorDirection::Forward;
		}
		else if (goal -> steer_left)
		{
			new_left_motors_dir = MotorDirection::Forward;
			new_right_motors_dir = MotorDirection::Backward;

		}
		else if (goal -> reverse)
		{
			new_left_motors_dir = MotorDirection::Backward;
			new_right_motors_dir = MotorDirection::Backward;

		}
		else //Forward
		{
			new_left_motors_dir = MotorDirection::Forward;
			new_right_motors_dir = MotorDirection::Forward;
		}
	}

	/**
	* @brief Set directions for left and right motors.
	*/
	void set_change_in_motors_direction(
		const MotorDirection new_left_motors_dir,
		const MotorDirection new_right_motors_dir)
	{
		if (leftMotors_->getDirection() != new_left_motors_dir)
		{
			leftMotors_->setDirection(new_left_motors_dir);
		}

		if (rightMotors_->getDirection() != new_right_motors_dir)
		{
			rightMotors_->setDirection(new_right_motors_dir);
		}
	}

	/**
	 *@brief Send back the result of accepted and executed motor goal
	 */
	void send_goal_result_(const std::shared_ptr<MotorsGoalHandle> goal_handle)
	{
		auto result = std::make_shared<MotorsInstruct::Result>();
		result->result = "Success";
		goal_handle->succeed(result);
	}
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto motor_controller_node = std::make_shared<MotorControllerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(motor_controller_node);
    executor.spin();

    rclcpp::spin(motor_controller_node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
