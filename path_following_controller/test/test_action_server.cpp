// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// 
// Apache License, Version 2.0
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.


#include "test_path_following_controller.hpp"

#include "rclcpp_action/client.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/create_client.hpp"

using namespace test_path_following_controller;
using namespace testing;
using namespace std;

using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

using std::placeholders::_1;
using std::placeholders::_2;

using ActionType = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;
using GoalOptions = rclcpp_action::Client<ActionType>::SendGoalOptions;

class ActionClientFixturePFC : public FixturePFC
{
public:
    void SetUp()
    {
        FixturePFC::SetUp();
        goal_options_.result_callback = 
            std::bind(&ActionClientFixturePFC::actionClientCallback, this, _1);
        goal_options_.feedback_callback = nullptr;

        // populate msgs
        full_msg = make_shared<JointTrajectory>();
        empty_msg = make_shared<JointTrajectory>();
        trivial_msg = make_shared<JointTrajectory>();

        full_msg->header.stamp = rclcpp::Time(0);
        trivial_msg->header.stamp = rclcpp::Time(0);

        for (size_t i=0; i<3; i++)
        {   
            auto value = static_cast<double>(i);
            JointTrajectoryPoint p;
            p.time_from_start = rclcpp::Duration::from_seconds(value);
            
            for(size_t j=0; j<DOF; j++)
                p.positions.push_back(value);

            full_msg->points.push_back(p);
            // trivial msg: one point, positions: {1, 1, 1}  
            if(i==1) trivial_msg->points.push_back(p);          
        }
    }

    void StartController(const std::vector<rclcpp::Parameter> & parameters = {}){
        InitializeConfigureActivatePFC(parameters);
    }

    void StopController(){
        DeactivatePFC();
        CleanUpPFC();
        rclcpp::shutdown();
    }

    void StartActionClient()
    {
        client_node_ = std::make_shared<rclcpp::Node>("trajectory_action_client");
        action_client_ = rclcpp_action::create_client<ActionType>(
            client_node_->get_node_base_interface(), client_node_->get_node_graph_interface(),
            client_node_->get_node_logging_interface(), client_node_->get_node_waitables_interface(),
            controller_name_ + "/follow_joint_trajectory");

        bool response = action_client_->wait_for_action_server(std::chrono::seconds(1));
        if(!response)
            throw std::runtime_error("Could not reach action server.");

        exe_.add_node(client_node_->get_node_base_interface());
        executor_future_handle_ = std::async(std::launch::async, [&](){ exe_.spin(); });
    }

    void actionClientCallback(const GoalHandle::WrappedResult & result)
    {
        action_handling_result_ = result.code;
        follow_traj_action_result_ = result.result->error_code;
        switch(result.code)
        {   
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(client_node_->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(client_node_->get_logger(), "Goal was canceled");
            default:
                RCLCPP_INFO(client_node_->get_logger(), "Unknown result code");
                return;
        }
    }

    shared_ptr<JointTrajectory> empty_msg;
        // trivial trajectory == one point trajectory
    shared_ptr<JointTrajectory> trivial_msg;
    shared_ptr<JointTrajectory> full_msg;

protected:

    std::shared_future<typename GoalHandle::SharedPtr> sendActionGoal(
        const std::vector<JointTrajectoryPoint> & points, double timeout, const GoalOptions & opt)
    {
        ActionType::Goal goal_msg;
        goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(timeout);
        goal_msg.trajectory.joint_names = joint_names_;
        goal_msg.trajectory.points = points;

        return action_client_->async_send_goal(goal_msg, opt);
    }

    std::shared_future<typename GoalHandle::SharedPtr> gh_future_;

    rclcpp::Node::SharedPtr client_node_;
    rclcpp_action::Client<ActionType>::SharedPtr action_client_;
    std::future<void> executor_future_handle_;
    GoalOptions goal_options_;

    rclcpp_action::ResultCode action_handling_result_ = rclcpp_action::ResultCode::UNKNOWN;
    int follow_traj_action_result_ = ActionType::Result::SUCCESSFUL;
};

TEST_F(ActionClientFixturePFC, multi_point_goal_suceeds)
{
    StartController();
        
    StartActionClient();

    UpdatePFC(rclcpp::Duration::from_seconds(0.1));
    
    gh_future_ = sendActionGoal(full_msg->points, 1.0, goal_options_);

    UpdatePFC(rclcpp::Duration::from_seconds(0.1));

    EXPECT_TRUE(gh_future_.get());
    ASSERT_EQ(action_handling_result_, rclcpp_action::ResultCode::SUCCEEDED);

    // expect the controller to be holding on last point
    auto current_state = controller_->get_state_feedback();
    EXPECT_THAT(current_state.positions,
        ElementsAreArray(full_msg->points[2].positions));

    StopController();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}