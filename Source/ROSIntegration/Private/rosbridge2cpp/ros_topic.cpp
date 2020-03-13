#include "ros_topic.h"

namespace rosbridge2cpp {

	ROSCallbackHandle<FunVrROSPublishMsg> ROSTopic::Subscribe(FunVrROSPublishMsg callback)
	{
		++subscription_counter_;

		// Only send subscribe when this ROSTopic hasn't sent this command before
		if (subscribe_id_ == "") {
			subscribe_id_.append("subscribe:");
			subscribe_id_.append(topic_name_);
			subscribe_id_.append(":");
			subscribe_id_.append(std::to_string(++ros_.id_counter));

			ROSBridgeSubscribeMsg cmd(true);
			cmd.id_ = subscribe_id_;
			cmd.topic_ = topic_name_;
			cmd.type_ = message_type_;
			cmd.compression_ = compression_;
			cmd.throttle_rate_ = throttle_rate_;
			cmd.queue_length_ = queue_size_;

			if (!ros_.SendMessage(cmd))
			{
				subscribe_id_ = "";
				UE_LOG(LogROS, Warning, TEXT("[ROSTopic] Subscription failed, setting id to empty string."));
			}
		}

		if (subscribe_id_ != "")
		{
			// Register callback in ROSBridge
			ROSCallbackHandle<FunVrROSPublishMsg> handle(callback);
			ros_.RegisterTopicCallback(topic_name_, handle); // Register callback in ROSBridge
			return handle;
		}

		subscribe_id_ = "";
		return ROSCallbackHandle<FunVrROSPublishMsg>();
	}

	bool ROSTopic::Unsubscribe(const ROSCallbackHandle<FunVrROSPublishMsg>& callback_handle)
	{
		// We've no active subscription
		if (subscribe_id_ == "")
			return false;

		if (!ros_.UnregisterTopicCallback(topic_name_, callback_handle)) { // Unregister callback in ROSBridge
			// failed to unregister callback - maybe the method is different from already registered callbacks
			UE_LOG(LogROS, Error, TEXT("[ROSTopic] Passed unknown callback to ROSTopic::unsubscribe. This callback is not registered in the ROSBridge instance. Aborting..."));
			return false;
		}

		--subscription_counter_;

		if (subscription_counter_ > 0)
			return true;

		UE_LOG(LogROS, Warning, TEXT("[ROSTopic] No callbacks registered anymore - unsubscribe from topic"));

		// Handle unsubscription when no callback is registered anymore
		//rapidjson::Document cmd;
		//cmd.SetObject();

		ROSBridgeUnsubscribeMsg cmd(true);
		cmd.id_ = subscribe_id_;
		cmd.topic_ = topic_name_;

		if (ros_.SendMessage(cmd)) {
			subscribe_id_ = "";
			subscription_counter_ = 0; // shouldn't be necessary ...
			return true;
		}
		return false;
	}

	bool ROSTopic::Advertise()
	{
		if (is_advertised_) {
			UE_LOG(LogROS, Warning, TEXT("[ROSTopic]: Already advertising!"));
			return true;
		}

		advertise_id_ = "";
		advertise_id_.append("advertise:");
		advertise_id_.append(topic_name_);
		advertise_id_.append(":");
		advertise_id_.append(std::to_string(++ros_.id_counter));

		ROSBridgeAdvertiseMsg cmd(true);
		cmd.id_ = advertise_id_;
		cmd.topic_ = topic_name_;
		cmd.type_ = message_type_;
		cmd.latch_ = latch_;
		cmd.queue_size_ = queue_size_;

		is_advertised_ = ros_.SendMessage(cmd);
		return is_advertised_;
	}

	bool ROSTopic::Unadvertise()
	{
		if (!is_advertised_) {
			UE_LOG(LogROS, Warning, TEXT("[ROSTopic]: Already un-advertised!"));
			return true;
		}

		ROSBridgeUnadvertiseMsg cmd(true);
		cmd.id_ = advertise_id_;
		cmd.topic_ = topic_name_;

		is_advertised_ = !ros_.SendMessage(cmd);
		return !is_advertised_;
	}

	bool ROSTopic::Publish(rapidjson::Value &message)
	{
		if (!is_advertised_) {
			if (!Advertise()) {
				return false;
			}
		}

		std::string publish_id = GeneratePublishID();

		ROSBridgePublishMsg cmd(true);
		cmd.id_ = publish_id;
		cmd.topic_ = topic_name_;
		cmd.msg_json_ = message;
		cmd.latch_ = latch_;

		return ros_.QueueMessage(topic_name_, queue_size_, cmd);
	}

	bool ROSTopic::Publish(bson_t *message)
	{
		if (!is_advertised_) {
			if (!Advertise()) {
				// Destroy the message since it won't be GCed by UE4.
				UE_LOG(LogROS, Warning, TEXT("[ROSTopic]: Destroying BSON; not advertising."));
				bson_destroy(message);
				return false;
			}
		}

		assert(message);

		std::string publish_id = GeneratePublishID();

		ROSBridgePublishMsg cmd(true);
		cmd.id_ = publish_id;
		cmd.topic_ = topic_name_;
		cmd.msg_bson_ = message;
		cmd.latch_ = latch_;

		return ros_.QueueMessage(topic_name_, queue_size_, cmd);
	}

	std::string ROSTopic::GeneratePublishID()
	{
		std::string publish_id;
		publish_id.append("publish:");
		publish_id.append(topic_name_);
		publish_id.append(":");
		publish_id.append(std::to_string(++ros_.id_counter));
		return publish_id;
	}
}
