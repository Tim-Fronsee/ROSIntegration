
#include "ros_bridge.h"
#include "ros_topic.h"
#include <bson.h>

#include "Misc/ScopeTryLock.h"

namespace rosbridge2cpp {

	static const std::chrono::seconds SendThreadFreezeTimeout = std::chrono::seconds(5);
	unsigned long ROSCallbackHandle_id_counter = 1;

	ROSBridge::ROSBridge(ITransportLayer &transport) : ROSBridge(transport, false)
	{}

	ROSBridge::ROSBridge(ITransportLayer &transport, bool bson_only_mode) :
	transport_layer_(transport),
	bson_mode(bson_only_mode),
	running(true)
	{
		if (bson_mode) {
			auto fun = [this](bson_t &bson) { IncomingMessageCallback(bson); };

			transport_layer_.SetTransportMode(ITransportLayer::BSON);
			transport_layer_.RegisterIncomingMessageCallback(fun);
		}
		else {
			// JSON mode
			auto fun = [this](json &document) { IncomingMessageCallback(document); };
			transport_layer_.RegisterIncomingMessageCallback(fun);
		}

		Thread = FRunnableThread::Create(this, TEXT("ROS Bridge Thread"), 0, TPri_BelowNormal);
	}

	ROSBridge::~ROSBridge()
	{
		delete Thread;
	}

	uint32 ROSBridge::Run()
	{
		int num_retries = 5;

		while (running)
		{
			// First, check that the transport layer is healthy.
			if (num_retries <= 0)
			{
				UE_LOG(LogROS, Warning, TEXT("[ROSBridge]: Lost connection to ROSBridge!"));
				return 1;
			}

			// Don't try to send to an unhealthy transport layer or we may deadlock.
			if (!transport_layer_.IsHealthy()) {
				UE_LOG(LogROS, Display,
					TEXT("[ROSBridge]: Waiting for Connection...%d"), num_retries);
				num_retries--;
				FPlatformProcess::Sleep(1);
				continue;
			}

			// Cycle through each queue so every topic has a chance to publish.
			if (current_queue < publisher_queues.Num())
			{
				auto& queue = publisher_queues[current_queue];
				if (!queue->IsEmpty() && queue->Peek() != nullptr)
				{
					bson_t* msg;
					queue->Dequeue(msg);

					const uint8_t* bson_data = bson_get_data(msg);
					uint32_t bson_size = msg->len;
					if (!transport_layer_.SendMessage(bson_data, bson_size)) num_retries--;
					else num_retries = 5;
					bson_destroy(msg);
				}
				current_queue++;
			}
			else current_queue = 0;
		}

		return 0;
	}

	void ROSBridge::Exit()
	{
		running = false;
		Thread->WaitForCompletion();
		UE_LOG(LogROS, Display, TEXT("[ROSBridge]: Exited"));
	}

	void ROSBridge::Stop()
	{
		running = false;
		UE_LOG(LogROS, Display, TEXT("[ROSBridge]: Stopped"));
	}

	bool ROSBridge::SendMessage(std::string data) {
		return transport_layer_.SendMessage(data);
	}

	bool ROSBridge::SendMessage(json &data)
	{
		if (bson_only_mode()) {
			// going from JSON to BSON
			std::string str_repr = Helper::get_string_from_rapidjson(data);
			UE_LOG(LogROS, Display,
				TEXT("[ROSBridge] serializing from JSON to BSON for: %s"), *FString(str_repr.c_str()));

			bson_t bson;
			bson_error_t error;
			if (!bson_init_from_json(&bson, str_repr.c_str(), -1, &error)) {
				printf("bson_init_from_json() failed: %s\n", error.message);
				bson_destroy(&bson);
				return false;
			}
			const uint8_t *bson_data = bson_get_data(&bson);
			uint32_t bson_size = bson.len;
			bool retval = transport_layer_.SendMessage(bson_data, bson_size);
			bson_destroy(&bson);
			return retval;
		}
		else {
			std::string str_repr = Helper::get_string_from_rapidjson(data);
			return SendMessage(str_repr);
		}
	}

	bool ROSBridge::SendMessage(ROSBridgeMsg &msg)
	{
		if (bson_only_mode()) {
			bson_t message = BSON_INITIALIZER;
			msg.ToBSON(message);

			const uint8_t *bson_data = bson_get_data(&message);
			uint32_t bson_size = message.len;
			bool retval = transport_layer_.SendMessage(bson_data, bson_size);
			bson_destroy(&message); // TODO needed?
			return retval;
		}
		else {
			// Convert ROSBridgeMsg to JSON
			json alloc;
			json message = msg.ToJSON(alloc.GetAllocator());

			std::string str_repr = Helper::get_string_from_rapidjson(message);
			return SendMessage(str_repr);
		}
	}

	bool ROSBridge::QueueMessage(const std::string& topic_name, int queue_size, ROSBridgePublishMsg& msg)
	{
		assert(bson_mode); // queueing is not supported for json data

		if (!running)	return false;

		if (!publisher_topics.Find(FString(topic_name.c_str())))
		{
			publisher_topics.Add(FString(topic_name.c_str()), publisher_queues.Num());
			publisher_queues.Add(new TCircularQueue<bson_t*>(queue_size));
		}
		auto& queue = publisher_queues[publisher_topics[FString(topic_name.c_str())]];
		if (!queue->IsFull())
		{
			// Convert to BSON.
			bson_t* message = bson_new();
			bson_init(message);
			msg.ToBSON(*message);

			// Place message on the queue.
			queue->Enqueue(message);
			return true;
		}
		else UE_LOG(LogROS, Warning, TEXT("[ROSBridge]: Message Queue Full, Skipping..."));
		return false;
	}

	void ROSBridge::HandleIncomingPublishMessage(ROSBridgePublishMsg &data)
	{
		FScopeTryLock ScopeLock(&TopicsMutex);

		// Incoming topic message - dispatch to correct callback
		std::string &incoming_topic_name = data.topic_;
		if (registered_topic_callbacks_.find(incoming_topic_name) == registered_topic_callbacks_.end()) {
			std::cerr << "[ROSBridge] Received message for topic " << incoming_topic_name << " where no callback has been registered before" << std::endl;
			return;
		}

		if (bson_only_mode()) {
			if (!data.full_msg_bson_) {
				std::cerr << "[ROSBridge] Received message for topic " << incoming_topic_name << ", but full message field is missing. Aborting" << std::endl;
				return;
			}
		}
		else {
			if (data.msg_json_.IsNull()) {
				std::cerr << "[ROSBridge] Received message for topic " << incoming_topic_name << ", but 'msg' field is missing. Aborting" << std::endl;
				return;
			}
		}

		// Iterate over all registered callbacks for the given topic
		for (auto& topic_callback : registered_topic_callbacks_.find(incoming_topic_name)->second) {
			topic_callback.GetFunction()(data);
		}
		return;
	}

	void ROSBridge::HandleIncomingServiceResponseMessage(ROSBridgeServiceResponseMsg &data)
	{
		std::string &incoming_service_id = data.id_;

		auto service_response_callback_it = registered_service_callbacks_.find(incoming_service_id);

		if (service_response_callback_it == registered_service_callbacks_.end()) {
			std::cerr << "[ROSBridge] Received response for service id " << incoming_service_id << "where no callback has been registered before" << std::endl;
			return;
		}

		// Execute the callback for the given service id
		service_response_callback_it->second(data);

		// Delete the callback.
		// Every call_service will create a new id
		registered_service_callbacks_.erase(service_response_callback_it);

	}

	void ROSBridge::HandleIncomingServiceRequestMessage(ROSBridgeCallServiceMsg &data)
	{
		std::string &incoming_service = data.service_;

		if (bson_only_mode()) {
			auto service_request_callback_it = registered_service_request_callbacks_bson_.find(incoming_service);

			if (service_request_callback_it == registered_service_request_callbacks_bson_.end()) {
				std::cerr << "[ROSBridge] Received service request for service :" << incoming_service << " where no callback has been registered before" << std::endl;
				return;
			}
			service_request_callback_it->second(data);
		}
		else
		{
			auto service_request_callback_it = registered_service_request_callbacks_.find(incoming_service);

			if (service_request_callback_it == registered_service_request_callbacks_.end()) {
				std::cerr << "[ROSBridge] Received service request for service :" << incoming_service << " where no bson callback has been registered before" << std::endl;
				return;
			}
			rapidjson::Document response_allocator;

			// Execute the callback for the given service id
			service_request_callback_it->second(data, response_allocator.GetAllocator());
		}
	}

	void ROSBridge::IncomingMessageCallback(bson_t &bson)
	{
		// Check the message type and dispatch the message properly
		//
		// Incoming Topic messages
		bool key_found = false;

		if (Helper::get_utf8_by_key("op", bson, key_found) == "publish") {
			ROSBridgePublishMsg m;
			if (m.FromBSON(bson)) {
				HandleIncomingPublishMessage(m);
				return;
			}

			std::cerr << "Failed to parse publish message into class. Skipping message." << std::endl;
		}

		// Service responses for service we called earlier
		if (Helper::get_utf8_by_key("op", bson, key_found) == "service_response") {
			ROSBridgeServiceResponseMsg m;
			if (m.FromBSON(bson)) {
				HandleIncomingServiceResponseMessage(m);
				return;
			}
			std::cerr << "Failed to parse service_response message into class. Skipping message." << std::endl;
		}

		// Service Requests to a service that we advertised in ROSService
		if (Helper::get_utf8_by_key("op", bson, key_found) == "call_service") {
			ROSBridgeCallServiceMsg m;
			m.FromBSON(bson);
			HandleIncomingServiceRequestMessage(m);
		}
	}

	void ROSBridge::IncomingMessageCallback(json &data)
	{
		std::string str_repr = Helper::get_string_from_rapidjson(data);

		// Check the message type and dispatch the message properly
		//
		// Incoming Topic messages
		if (std::string(data["op"].GetString(), data["op"].GetStringLength()) == "publish") {
			ROSBridgePublishMsg m;
			if (m.FromJSON(data)) {
				HandleIncomingPublishMessage(m);
				return;
			}

			std::cerr << "Failed to parse publish message into class. Skipping message." << std::endl;
		}

		// Service responses for service we called earlier
		if (std::string(data["op"].GetString(), data["op"].GetStringLength()) == "service_response") {
			ROSBridgeServiceResponseMsg m;
			// m.FromJSON(data);
			if (m.FromJSON(data)) {
				HandleIncomingServiceResponseMessage(m);
				return;
			}
			std::cerr << "Failed to parse service_response message into class. Skipping message." << std::endl;
		}

		// Service Requests to a service that we advertised in ROSService
		if (std::string(data["op"].GetString(), data["op"].GetStringLength()) == "call_service") {
			ROSBridgeCallServiceMsg m;
			m.FromJSON(data);
			HandleIncomingServiceRequestMessage(m);
		}
	}

	bool ROSBridge::IsHealthy() const
	{
		return running;
	}

	void ROSBridge::RegisterTopicCallback(std::string topic_name, ROSCallbackHandle<FunVrROSPublishMsg>& callback_handle)
	{
		FScopeTryLock ScopeLock(&TopicsMutex);
		registered_topic_callbacks_[topic_name].push_back(callback_handle);
	}

	void ROSBridge::RegisterServiceCallback(std::string service_call_id, FunVrROSServiceResponseMsg fun)
	{
		registered_service_callbacks_[service_call_id] = fun;
	}

	void ROSBridge::RegisterServiceRequestCallback(std::string service_name, FunVrROSCallServiceMsgrROSServiceResponseMsgrAllocator fun)
	{
		registered_service_request_callbacks_[service_name] = fun;
	}

	void ROSBridge::RegisterServiceRequestCallback(std::string service_name, FunVrROSCallServiceMsgrROSServiceResponseMsg fun)
	{
		registered_service_request_callbacks_bson_[service_name] = fun;
	}

	bool ROSBridge::UnregisterTopicCallback(std::string topic_name, const ROSCallbackHandle<FunVrROSPublishMsg>& callback_handle)
	{
		FScopeTryLock ScopeLock(&TopicsMutex);
		if (registered_topic_callbacks_.find(topic_name) == registered_topic_callbacks_.end()) {
			std::cerr << "[ROSBridge] UnregisterTopicCallback called but given topic name '" << topic_name << "' not in map." << std::endl;
			return false;
		}

		std::list<ROSCallbackHandle<FunVrROSPublishMsg>> &r_list_of_callbacks = registered_topic_callbacks_.find(topic_name)->second;

		for (std::list<ROSCallbackHandle<FunVrROSPublishMsg>>::iterator topic_callback_it = r_list_of_callbacks.begin();
			topic_callback_it != r_list_of_callbacks.end();
			++topic_callback_it) {

			if (*topic_callback_it == callback_handle) {
				std::cout << "[ROSBridge] Found CB in UnregisterTopicCallback. Deleting it ... " << std::endl;
				r_list_of_callbacks.erase(topic_callback_it);
				return true;
			}
		}
		return false;
	}
}
