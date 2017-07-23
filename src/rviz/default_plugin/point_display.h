#ifndef POINT_DISPLAY_H
#define POINT_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#endif

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rviz/message_filter_display.h>


namespace Ogre
{
    class SceneNode;
}

namespace rviz
{
    class ColorProperty;
    class FloatProperty;
    class IntProperty;
}

namespace rviz
{

    class PointStampedVisual;

    class PointStampedDisplay: public rviz::MessageFilterDisplay<geometry_msgs::msg::PointStamped>
    {
    Q_OBJECT
    public:
	// Constructor.  pluginlib::ClassLoader creates instances by calling
	// the default constructor, so make sure you have one.
	PointStampedDisplay();
	virtual ~PointStampedDisplay();

    protected:
	// Overrides of public virtual functions from the Display class.
	virtual void onInitialize();
	virtual void reset();

     private Q_SLOTS:
	// Helper function to apply color and alpha to all visuals.
	void updateColorAndAlpha();
        void updateHistoryLength();

	// Function to handle an incoming ROS message.

	/** @brief Get the frame for the given message. */
	virtual std::string getMsgFrame(const geometry_msgs::msg::PointStamped::SharedPtr msg);

	/** @brief Get the time stamp for the given message. */
	virtual tf2::TimePoint getMsgTime(const geometry_msgs::msg::PointStamped::SharedPtr msg);
	
    private:
	void processMessage( const geometry_msgs::msg::PointStamped::SharedPtr msg );

        // Storage for the list of visuals.  It is a circular buffer where
        // data gets popped from the front (oldest) and pushed to the back (newest)
        boost::circular_buffer<std::shared_ptr<PointStampedVisual> > visuals_;

	// Property objects for user-editable properties.
	rviz::ColorProperty *color_property_;
        rviz::FloatProperty *alpha_property_, *radius_property_;
	rviz::IntProperty *history_length_property_;

    };
} // end namespace rviz_plugin_tutorials

#endif // POINT_DISPLAY_H
