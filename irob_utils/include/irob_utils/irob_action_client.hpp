/*
 *  irob_action_client.hpp
 *
 *  Author(s): Tamas Levendovics
 *  Created on: 2017-07-25
 *
 *  Definition of an action client tailored to the needs of the framework.
 *
 */

#ifndef IROB_ACTION_CLIENT_HPP_
#define IROB_ACTION_CLIENT_HPP_

#include <iostream>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace saf {

typedef enum ActionState
{ACTIVE, DONE} ActionState;

template <class ActionSpec>
class IrobActionClient {
public:
    using GoalHandle = typename rclcpp_action::ClientGoalHandle<ActionSpec>;
    using Goal = typename ActionSpec::Goal;
    using Result = typename ActionSpec::Result;
    using Feedback = typename ActionSpec::Feedback;

protected:
    rclcpp::Node::SharedPtr node;
    typename rclcpp_action::Client<ActionSpec>::SharedPtr action_client;
    std::shared_ptr<Result> result;
    std::shared_ptr<Feedback> feedback;
    bool done = false;
    bool active = false;

public:
    IrobActionClient(rclcpp::Node::SharedPtr node, const std::string& action_name);

    void sendGoal(const Goal& goal);
    bool isDone();
    bool isActive();
    Feedback getFeedback();
    Result getResult();

private:
    void goalResponseCallback(const typename GoalHandle::SharedPtr & goal_handle);
    void feedbackCallback(typename GoalHandle::SharedPtr, const std::shared_ptr<const Feedback> feedback);
    void resultCallback(const typename GoalHandle::WrappedResult & result);
};

// Implementation

template <class ActionSpec>
IrobActionClient<ActionSpec>::IrobActionClient(rclcpp::Node::SharedPtr node, const std::string& action_name)
    : node(node) {
    action_client = rclcpp_action::create_client<ActionSpec>(node, action_name);
}

template <class ActionSpec>
void IrobActionClient<ActionSpec>::sendGoal(const Goal& goal) {
    if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available!");
        return;
    }

    auto goal_msg = std::make_shared<Goal>(goal);
    auto send_goal_options = typename rclcpp_action::Client<ActionSpec>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&IrobActionClient<ActionSpec>::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&IrobActionClient<ActionSpec>::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&IrobActionClient<ActionSpec>::resultCallback, this, std::placeholders::_1);

    action_client->async_send_goal(goal_msg, send_goal_options);
}

template <class ActionSpec>
void IrobActionClient<ActionSpec>::goalResponseCallback(const typename GoalHandle::SharedPtr & goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server.");
        return;
    }
    active = true;
}

template <class ActionSpec>
void IrobActionClient<ActionSpec>::feedbackCallback(typename GoalHandle::SharedPtr, const std::shared_ptr<const Feedback> feedback_msg) {
    feedback = std::make_shared<Feedback>(*feedback_msg);
}

template <class ActionSpec>
void IrobActionClient<ActionSpec>::resultCallback(const typename GoalHandle::WrappedResult & result_msg) {
    if (result_msg.code == rclcpp_action::ResultCode::SUCCEEDED) {
        result = std::make_shared<Result>(*result_msg.result);
    }
    done = true;
    active = false;
}

template <class ActionSpec>
bool IrobActionClient<ActionSpec>::isDone() {
    return done;
}

template <class ActionSpec>
bool IrobActionClient<ActionSpec>::isActive() {
    return active;
}

template <class ActionSpec>
typename IrobActionClient<ActionSpec>::Feedback IrobActionClient<ActionSpec>::getFeedback() {
    return *feedback;
}

template <class ActionSpec>
typename IrobActionClient<ActionSpec>::Result IrobActionClient<ActionSpec>::getResult() {
    return *result;
}

} // namespace saf

#endif