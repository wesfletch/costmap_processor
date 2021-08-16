#  costmap_processor

A set of ROS nodes and basic Python utilities to process the inputs/outputs of a TraCR agent (though it could certainly be used for other purposes).

Required dependencies:

* [occupancy_grid_python](https://github.com/awesomebytes/occupancy_grid_python): used to subscribe to and manipulate OccupancyGrid messages for costmap_processor node. However, most of this functionality (minus the ability to subscribe to a message and update object data automatically) is available in the Costmap object of costmap_processor.
* [rospy_message_converter](https://github.com/uos/rospy_message_converter): used in the bridge_server node to JSON-serialize ROS messages for transport to Java client ROSBridge.

### Nodes

This ROS package contains a few different nodes to handle input/output for TraCR agents:

### CostmapProcessor

Designed to take in OccupancyGrid messages, segment out features, then output Contour messages (defined in msg) to be routed into the TraCR agent (after some additional Java processing). 

Frankly, this requires more work to be useful. A costmap contains only 2D cells with a "cost" value associated with each. If we take the naïve approach of ONLY looking at costmap data, we get ONLY occupancy information, with no insight into *what* the obstacle is or how long it's been there. Without that information, TraCR decision-making is limited. There needs to be some further sensor processing to determine what a feature is, then pass that semantic info into the TraCR agents.  

All that being said, the CostmapProcessor node does what it's meant to, for the most part: input costmap, output contours (basically just sets of points and a cost stamp).

### CostmapGenerator

This node is designed to take TraCR "decisions" (formatted as point-value pairs to add to OccupancyGrid) as inputs, then outputs a published OccupancyGrid message. It makes use of a Costmap object to hold/track data, rather than using the roscpp costmap_2d interface.

One of the features of the TraCR system at time-of-writing is that it can be a mostly drop-in solution for an existing autonomy stack. Being able to output TraCR decisions as costmaps is an important part of that, since most (if not all) autonomy stacks can use costmaps for path-planning. 

Like CostmapProcessor, this node does it's job: it takes in decisions on points and values, and publishes a properly marked costmap.

Future work:

* It might be nice to extend the Costmap object to a [layered costmap](https://ieeexplore.ieee.org/document/6942636). This is standard in ROS as of the Hydro (?) release, and having features on different layers based on type might be one method of providing semantic information to the system for closed-loop (back into TraCR) control.



### BridgeServer

<insert picture from powerpoint slides here>

A node capable of sending/receiving JSON-serialized ROS messages, used as half of a socket connection to a Java client using ROSBridge. The BridgeServer gets publish/subscribe requests from the Java-side of the connection, then creates rospy publishers and subscribers to match. Then, messages are passed back and forth via socket connection.

Future Work:

* Initially, a custom protocol was developed for this communication before being abandoned for simple "HERE'S THE WHOLE MESSAGE AT ONCE." However, Python seems to struggle with receiving large messages all at once as JSON, so it may be necessary to go back to a protocol that at least warns Python-side of the number of bytes it's about to receive.
* Currently, BridgeServer only supports connection at a time. Since the plan is to have multiple TraCR agents running simultaneously, it would likely be best to make multiple connections possible. Or maybe just use multiple BridgeServer nodes. I don't have all the answers.
* This socket connection needs a thorough Wireshark-ing under load. Need more information on the reliability of the connection.

