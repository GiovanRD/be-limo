// Gazebo GUI plugin hiding the green markers drawn by gzclient for every
// light source. The lights still illuminate the scene; only their markers
// in the Gazebo window are hidden. Camera sensors never show these markers.

#include <functional>

#include <gazebo/common/Events.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/rendering/rendering.hh>

namespace ensem_room207
{
class HideLightMarkers : public gazebo::GUIPlugin
{
public:
  HideLightMarkers()
  {
    // No widget: the plugin only acts on the rendering scene.
    this->resize(0, 0);
    this->preRenderConnection = gazebo::event::Events::ConnectPreRender(
      std::bind(&HideLightMarkers::OnPreRender, this));
  }

private:
  void OnPreRender()
  {
    auto scene = gazebo::rendering::get_scene();
    if (!scene) {
      return;
    }
    // Lights can be added after the plugin is loaded: hide new ones only.
    const unsigned int count = scene->LightCount();
    if (count == this->hiddenCount) {
      return;
    }
    for (unsigned int i = 0; i < count; ++i) {
      auto light = scene->LightByIndex(i);
      if (light) {
        light->ShowVisual(false);
      }
    }
    this->hiddenCount = count;
    gzmsg << "[hide_light_markers] Hid " << count << " light markers.\n";
  }

  gazebo::event::ConnectionPtr preRenderConnection;
  unsigned int hiddenCount = 0;
};

GZ_REGISTER_GUI_PLUGIN(HideLightMarkers)
}  // namespace ensem_room207
