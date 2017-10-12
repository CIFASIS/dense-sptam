#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "denseInterface.hpp"

namespace dense
{

	class denseNodelet : public nodelet::Nodelet
	{
	public:
		void onInit() {
			NODELET_DEBUG("Initializing DENSE nodelet...");
			dense_interface_.reset(new dense::denseInterface(getNodeHandle(), getPrivateNodeHandle()));
		}

	private:
		std::unique_ptr<dense::denseInterface> dense_interface_;
	};
}

PLUGINLIB_EXPORT_CLASS(dense::denseNodelet, nodelet::Nodelet)
