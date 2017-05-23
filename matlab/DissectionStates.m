classdef DissectionStates
   enumeration
      done_dissection_group, at_tgt_dp, at_tgt_goal, at_distant_dp, at_distant_goal, abort
   end
   
   methods
      function next = next(obj, reqmsg, group_done)
          next = DissectionStates.abort;
          
          switch obj
            case DissectionStates.done_dissection_group             
                if reqmsg.target_type == reqmsg.DISSECTION
                    next = DissectionStates.at_tgt_dp;
                end
            case DissectionStates.at_tgt_dp
                 if reqmsg.target_type == reqmsg.DISSECTION
                    next = DissectionStates.at_tgt_goal;
                 end
                 
            case DissectionStates.at_tgt_goal
                 if reqmsg.target_type == reqmsg.DISTANT
                     if group_done
                        next = DissectionStates.at_distant_goal;
                     else
                        next = DissectionStates.at_distant_dp;
                     end
                 end
            case DissectionStates.at_distant_dp
                 if reqmsg.target_type == reqmsg.DISTANT
                    next = DissectionStates.done_dissection_group;
                 end
                 
            case DissectionStates.at_distant_goal
                 if reqmsg.target_type == reqmsg.DISSECTION
                    next = DissectionStates.at_tgt_dp;
                 end
                 
            otherwise
                next = DissectionStates.abort;
                warning('Unexpected query, do nothing...')
            end
      end
   end
   
end

