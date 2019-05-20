// Example plugin code poached from here: http://gazebosim.org/tutorials/?tut=plugins_hello_world
#include <gazebo/gazebo.hh>

namespace gazebo {
class WorldPluginTutorial : public WorldPlugin
{
public:
    WorldPluginTutorial()
      : WorldPlugin()
    {
        printf("Hello World!\n");
    }

public:
    void Load(physics::WorldPtr, sdf::ElementPtr)
    {
    }
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
