# irob_motion

Hierarchical motion package based on ROS actionlib, contains implementation of surgemes. Motion decomposition is based on [Vedula et al. "Analysis of the Structure of Surgical Activity for a Suturing and Knot-Tying Task"](http://journals.plos.org/plosone/article?id=10.1371/journal.pone.0149174).

## The list of parameterizable surgemes

- nav_to_pos (navigate freely to the target position)
- grasp (grasp object)
- cut (cut object using scrissors)
- release (release grasped object)
- push (push tissue with tool tip)
- dissect (dissect soft tissue using an opening motion)
- place (lace grasped object to position, jaws angles remain constant)
- manipulate (manipulate grasped soft tissue)
