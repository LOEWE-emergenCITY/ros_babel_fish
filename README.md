# ROS Babel Fish

This library enables ROS2 nodes written in C++ to communicate using message types that are unknown at compile time.

You can subscribe and even publish any available message type.  
It also supports both providing and calling services and actions.
However, the type support for the used message, service or action needs to be available on the machine and in the environment the node is running in.

Please strongly consider whether you actually need this library because there may be a better solution.  
Possible use cases where you do need it are:

* UIs displaying the content of various at compile time unknown messages
* Plugins for (script) languages that can not access the C++ message definitions without modification
  * Spot for shameless self-advertising: Check out my [ROS2 QML plugin](https://github.com/StefanFabian/qml_ros2_plugin) which uses this library to allow subscribing, publishing and more directly in QML

## Scientific Works

If you are using this module in a scientific context, feel free to cite this paper:
```
@INPROCEEDINGS{fabian2021hri,
  author = {Stefan Fabian and Oskar von Stryk},
  title = {Open-Source Tools for Efficient ROS and ROS2-based 2D Human-Robot Interface Development},
  year = {2021},
  booktitle = {2021 European Conference on Mobile Robots (ECMR)},
}
```

## Installation

Clone this repo into the `src` folder of your colcon workspace, build and open a new terminal or source the workspace overlay again.
(Beginners: see [here](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) for an introduction to workspaces)

**Note:** Only foxy, rolling and ROS2 versions from jazzy upwards have full support for this library as the others are missing functionality in the core ROS packages and the backports have not been merged yet.
On the other versions, services and actions do not work. Subscribing and publishing works on all ROS2 versions.

## Examples

### Subscribing

```C++
using namespace ros_babel_fish; // Except Node all of the following classes are in that namespace
Node node;
BabelFish::SharedPtr fish = BabelFish::make_shared();
// Subscribe topic /pose
BabelFishSubscription::SharedPtr sub = fish->create_subscription( node, topic, 1, &callback );

/* ... */

void callback( const ros_babel_fish::CompoundMessage::SharedPtr &msg )
{
  std::cout << "Message received!" << std::endl;
  std::cout << "Datatype:" << msg->datatype() << std::endl; // geometry_msgs::msg::Pose
  std::cout << "Name:" << msg->name() << std::endl; // geometry_msgs/msg/Pose
  std::cout << "Message Content:" << std::endl;
  const CompoundMessage &compound = *msg;
  std::cout << "Position: " << compound["position"]["x"].value<double>() << ", " << compound["position"]["y"].value<double>() << ", "
            << compound["position"]["z"].value<double>() << std::endl;
  // Alternatively, you can use the new get<T> convenience method
  std::cout << "Orientation: " << compound["orientation"].get<double>("w") << ", " << compound["orientation"].get<double>("x") << ", "
            << compound["orientation"].get<double>("y") << ", " << compound["orientation"].get<double>("z") << std::endl;
};
```

### Publishing

```C++
using namespace ros_babel_fish; // Except Node all of the following classes are in that namespace
Node node;
BabelFish fish;
// Advertise a publisher on topic /pose
BabelFishPublisher::SharedPtr pub_pose = fish->create_publisher( node, "/pose", "geometry_msgs/msg/Pose", 1 );

CompoundMessage::SharedPtr message = fish->create_message_shared( "geometry_msgs/msg/Pose" );
auto &compound = *message;
compound["position"].as<CompoundMessage>()["x"].as<ValueMessage<double>>().setValue(1.1);
// or using convenience methods
compound["position"]["y"].as<ValueMessage<double>>().setValue(2.4);
// or using even more convenience methods
compound["position"]["z"] = 3.6;
// UPDATE: now also shorter with explicit type
compound["position"].set<double>("z", 3.6)
// Be careful with your types here. Casting to a wrong type will throw an exception!
// The as<ValueMessage<T>> method is also a bit faster because the convenience method
// will automatically convert to the right type and perform bound and compatibility checks.
// This makes it more robust but comes with a little overhead.
// Note that assigning a double to a float field will always throw an exception because
// the float may not have the required resolution.
// Assigning, e.g., an int to a uint8 field will only throw if the int is out of bounds (0-255)
// otherwise a warning will be printed because uint8 is not compatible with all possible values
// of int. This warning can be disabled as a compile option. 

compound["orientation"]["w"] = 0.384;
compound["orientation"]["x"] = -0.003;
compound["orientation"]["y"] = -0.876;
compound["orientation"]["z"] = 0.292;

pub_pose->publish( compound );
```

For more examples on how to use this library, check the examples folder.
