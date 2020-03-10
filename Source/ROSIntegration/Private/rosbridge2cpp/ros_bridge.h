#pragma once

#include <iostream>
#include <string>
#include <functional>
#include <unordered_map>
#include <list>
#include <queue>
#include <chrono>

#include <stdio.h>
#include "types.h"
#include "helper.h"
#include "spinlock.h"

#include "itransport_layer.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "messages/rosbridge_advertise_msg.h"
#include "messages/rosbridge_advertise_service_msg.h"
#include "messages/rosbridge_call_service_msg.h"
#include "messages/rosbridge_msg.h"
#include "messages/rosbridge_publish_msg.h"
#include "messages/rosbridge_service_response_msg.h"
#include "messages/rosbridge_subscribe_msg.h"
#include "messages/rosbridge_unadvertise_msg.h"
#include "messages/rosbridge_unadvertise_service_msg.h"
#include "messages/rosbridge_unsubscribe_msg.h"

#include "Containers/Queue.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"

using json = rapidjson::Document;

namespace rosbridge2cpp {

	/**
	 * The main rosbridge2cpp class that connects to the rosbridge server.
	 * The library is inspired by [roslibjs](http://wiki.ros.org/roslibjs),
	 * which is a feature-rich client-side implementation of the rosbridge protocol in java script.
	 */
	class ROSBridge : public FRunnable {

	public:
		ROSBridge(ITransportLayer &transport);
		ROSBridge(ITransportLayer &transport, bool bson_only_mode);

		~ROSBridge();

		// FRunnable
		virtual uint32 Run() override;
		virtual void Exit() override;
		virtual void Stop() override;

		bool IsHealthy() const;

		// Send arbitrary string-data over the given TransportLayer
		bool SendMessage(std::string data);

		// Send json data over the transport layer,
		// by serializing it and using
		// ROSBridge::send_message(std::string data)
		bool SendMessage(json &data);

		bool SendMessage(ROSBridgeMsg &msg);

		bool QueueMessage(const std::string& topic_name, int queue_size, ROSBridgePublishMsg& msg);


		// Registration function for topic callbacks.
		// This method should ONLY be called by ROSTopic instances.
		// It will pass the received data to the registered std::function.
		//
		// Please note:
		// _If you register more than one callback for the
		// same topic, the old one get's overwritten_
		void RegisterTopicCallback(std::string topic_name, ROSCallbackHandle<FunVrROSPublishMsg>& callback_handle);

		// This method should ONLY be called by ROSTopic instances.
		// If you call this on your own, the housekeeping in ROSTopic
		// might fail which leads to missing unsubscribe messages etc.
		//
		// @return true, if the passed callback has been found and removed. false otherwise.
		bool UnregisterTopicCallback(std::string topic_name, const ROSCallbackHandle<FunVrROSPublishMsg>& callback_handle);

		// Register the callback for a service call.
		// This callback will be executed when we receive the response for a particular Service Request
		void RegisterServiceCallback(std::string service_call_id, FunVrROSServiceResponseMsg fun);

		// Register the callback that shall be executed,
		// whenever we receive a request for a service that
		// this ROSBridge has advertised via a ROSService.
		void RegisterServiceRequestCallback(std::string service_name, FunVrROSCallServiceMsgrROSServiceResponseMsgrAllocator fun);
		void RegisterServiceRequestCallback(std::string service_name, FunVrROSCallServiceMsgrROSServiceResponseMsg fun);

		// An ID Counter that will be used to generate increasing
		// IDs for service/topic etc. messages
		long id_counter = 0;

		// Returns true if the bson only mode is activated
		bool bson_only_mode() {
			return bson_mode;
		}

		// Enable the BSON only mode.
		// All communications with the rosbridge server
		// will be in BSON, instead of JSON
		void enable_bson_mode() { bson_mode = true; }

	private:
		// Callback function for the used ITransportLayer.
		// It receives the received json that was contained
		// in the incoming ROSBridge packet
		//
		// @pre This method assumes a valid json variable
		void IncomingMessageCallback(json &data);

		void IncomingMessageCallback(bson_t &bson);

		// Handler Method for reply packet
		void HandleIncomingPublishMessage(ROSBridgePublishMsg &data);

		// Handler Method for reply packet
		void HandleIncomingServiceResponseMessage(ROSBridgeServiceResponseMsg &data);

		// Handler Method for reply packet
		void HandleIncomingServiceRequestMessage(ROSBridgeCallServiceMsg &data);

		ITransportLayer &transport_layer_;
		std::unordered_map<std::string, std::list<ROSCallbackHandle<FunVrROSPublishMsg>>> registered_topic_callbacks_;
		std::unordered_map<std::string, FunVrROSServiceResponseMsg> registered_service_callbacks_;
		std::unordered_map<std::string, FunVrROSCallServiceMsgrROSServiceResponseMsgrAllocator> registered_service_request_callbacks_;
		std::unordered_map<std::string, FunVrROSCallServiceMsgrROSServiceResponseMsg> registered_service_request_callbacks_bson_;
		bool bson_mode, running;

		// UE4 Thread Safe vars
		FRunnableThread * Thread;
		FCriticalSection QueueMutex, TopicsMutex, TransportMutex;
		TQueue<bson_t*, EQueueMode::Mpsc> messages;

		uint32 _queue_max = 1024;
		uint32 _queue_size = 0;
	};
}
