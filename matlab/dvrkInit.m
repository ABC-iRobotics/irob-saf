function [targetsrv, donesrv, errpub, statussub] = dvrkInit()


targetsrv = rossvcserver('/dvrk_vision/movement_target','irob_dvrk_automation/TargetPose', @getTargetCallback)
donesrv = rossvcserver('/dvrk_vision/task_done', 'irob_dvrk_automation/BoolQuery', @isDoneCallback);
errpub = rospublisher('/dvrk_vision/error', 'std_msgs/String');

statussub = rossubscriber('/dvrk_vision/subtask_status', 'std_msgs/String');
pause(2) % Wait to ensure publisher is registered

% Start

sendDone( donepub, false );

status = receive(statussub);
disp(status.Data);


end

