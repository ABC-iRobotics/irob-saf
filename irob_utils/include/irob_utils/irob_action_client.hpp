/*
 *  irob_action_client.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-25
 *
 *  Definition of an action client tailored to the needs of the framework.
 *
 */

#ifndef IROB_ACTION_CLIENT_HPP_
#define IROB_ACTION_CLIENT_HPP_

#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/action_definition.h>	


namespace saf {

typedef enum ActionState 
{ACTIVE, DONE} ActionState;

template <class ActionSpec>
class IrobActionClient: public actionlib::SimpleActionClient<ActionSpec> {
public:
  // Macro to define action data types
  ACTION_DEFINITION(ActionSpec);

protected:
  bool done = false;
  bool active = false;

  ResultConstPtr result;
  FeedbackConstPtr feedback;



public:

  IrobActionClient(const std::string&, bool=true);

  void sendGoal(const Goal&,
                typename actionlib::SimpleActionClient<ActionSpec>
                ::SimpleDoneCallback =
      typename actionlib::SimpleActionClient<ActionSpec>
      ::SimpleDoneCallback(),
                typename actionlib::SimpleActionClient<ActionSpec>
                ::SimpleActiveCallback =
      typename actionlib::SimpleActionClient<ActionSpec>
      ::SimpleActiveCallback(),
                typename actionlib::SimpleActionClient<ActionSpec>
                ::SimpleFeedbackCallback =
      typename actionlib::SimpleActionClient<ActionSpec>
      ::SimpleFeedbackCallback());

  void doneCB(const actionlib::SimpleClientGoalState&,
              const ResultConstPtr&);
  void activeCB();
  void feedbackCB(const FeedbackConstPtr&);

  bool isDone(bool = true);
  bool isActive(bool = true);
  Feedback getFeedback(bool = true);
  Result getResult(bool = true );

};

// Implementation

template <class ActionSpec>
IrobActionClient<ActionSpec>::IrobActionClient(
    const std::string &name,
    bool spin_thread /* = true */):
  actionlib::SimpleActionClient<ActionSpec>(name, spin_thread)
{
  // Constructor yet empty
}

template <class ActionSpec>
void IrobActionClient<ActionSpec>::sendGoal(const Goal& goal,
                                            typename actionlib::SimpleActionClient<ActionSpec>
                                            ::SimpleDoneCallback done_cb,
                                            typename actionlib::SimpleActionClient<ActionSpec>
                                            ::SimpleActiveCallback active_cb,
                                            typename actionlib::SimpleActionClient<ActionSpec>
                                            ::SimpleFeedbackCallback feedback_cb)
{
  // Reset states
  done = false;
  active = false;

  // Send goal
  actionlib::SimpleActionClient<ActionSpec>::sendGoal(
        goal,
        boost::bind(&IrobActionClient<ActionSpec>::doneCB, this, _1, _2),
        boost::bind(&IrobActionClient<ActionSpec>::activeCB, this),
        boost::bind(&IrobActionClient<ActionSpec>::feedbackCB, this, _1));
}

template <class ActionSpec>
void IrobActionClient<ActionSpec>::doneCB(
    const actionlib::SimpleClientGoalState& state,
    const ResultConstPtr& result)
{
  done = true;
  this -> result = result;
}

template <class ActionSpec>
void IrobActionClient<ActionSpec>::activeCB()
{
  active = true;
}

template <class ActionSpec>
void IrobActionClient<ActionSpec>::feedbackCB(
    const FeedbackConstPtr& feedback)
{
  this -> feedback = feedback;
}

template <class ActionSpec>
bool IrobActionClient<ActionSpec>::isDone(bool ros_spin /* = true */)
{
  // actionlib::SimpleClientGoalState:
  // True if in RECALLED, REJECTED, PREEMPTED, ABORTED,
  // SUCCEEDED, or LOST. False if ACTIVE or PENDING
  if (ros_spin)
    ros::spinOnce();
  return done;
}

template <class ActionSpec>
bool IrobActionClient<ActionSpec>::isActive(bool ros_spin /* = true */)
{
  if (ros_spin)
    ros::spinOnce();
  return active;
}


template <class ActionSpec>
typename IrobActionClient<ActionSpec>::Feedback
IrobActionClient<ActionSpec>::getFeedback(bool ros_spin /* = true */)
{
  if (ros_spin)
    ros::spinOnce();
  return *feedback;
}

template <class ActionSpec>
typename IrobActionClient<ActionSpec>::Result
IrobActionClient<ActionSpec>::getResult(bool ros_spin/*= true */)
{
  if (ros_spin)
    ros::spinOnce();
  return *result;
}

}
#endif
